/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Matthias Grob	<maetugr@gmail.com>
 * @author Beat Küng		<beat-kueng@gmx.net>
 *
 */

#include "mc_rate_control.hpp"

#include <lib/ecl/geo/geo.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <uORB/topics/vehicle_rates.h>
#include <lib/work_queue/WorkItem.hpp>

#define TPA_RATE_LOWER_LIMIT 0.05f

#define AXIS_INDEX_ROLL 0
#define AXIS_INDEX_PITCH 1
#define AXIS_INDEX_YAW 2
#define AXIS_COUNT 3

using namespace matrix;


int MulticopterRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter attitude and rate controller. It takes attitude
setpoints (`vehicle_attitude_setpoint`) or rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has two loops: a P loop for angular error and a PID loop for angular rate error.

Publication documenting the implemented Quaternion Attitude Control:
Nonlinear Quadrocopter Attitude Control (2013)
by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

### Implementation
To reduce control latency, the module directly polls on the gyro topic published by the IMU driver.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

MulticopterRateControl::MulticopterRateControl() :
	ModuleParams(nullptr),
	WorkItem(px4::wq_configurations[px4::rate_ctrl]),
	_loop_perf(perf_alloc(PC_ELAPSED, "mc_rate_control"))
{
	_vehicle_status.is_rotary_wing = true;

	/* initialize quaternions in messages to be valid */
	_rates_prev.zero();
	_rates_prev_filtered.zero();
	_rates_sp.zero();
	_rates_int.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();

	parameters_updated();

	_task_start = hrt_absolute_time();
}

MulticopterRateControl::~MulticopterRateControl()
{
	orb_unsubscribe(_v_rates_sp_sub);
	orb_unsubscribe(_v_control_mode_sub);
	orb_unsubscribe(_params_sub);
	orb_unsubscribe(_manual_control_sp_sub);
	orb_unsubscribe(_vehicle_status_sub);
	orb_unsubscribe(_motor_limits_sub);
	orb_unsubscribe(_battery_status_sub);
	//orb_unsubscribe(_vehicle_rates_sub);
	orb_unsubscribe(_vehicle_land_detected_sub);
	orb_unsubscribe(_landing_gear_sub);
}

void MulticopterRateControl::init()
{
	if (!_inited) {
		/*
		 * do subscriptions
		 */
		_v_rates_sub = orb_subscribe(ORB_ID(vehicle_rates));
		_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
		_v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
		_params_sub = orb_subscribe(ORB_ID(parameter_update));
		_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
		_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
		_motor_limits_sub = orb_subscribe(ORB_ID(multirotor_motor_limits));
		_battery_status_sub = orb_subscribe(ORB_ID(battery_status));
		_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
		_landing_gear_sub = orb_subscribe(ORB_ID(landing_gear));

		_inited = true;
	}
}

void
MulticopterRateControl::parameters_updated()
{
	/* Store some of the parameters in a more convenient way & precompute often-used values */

	/* roll gains */
	_rate_p(0) = _roll_rate_p.get();
	_rate_i(0) = _roll_rate_i.get();
	_rate_int_lim(0) = _roll_rate_integ_lim.get();
	_rate_d(0) = _roll_rate_d.get();
	_rate_ff(0) = _roll_rate_ff.get();

	/* pitch gains */
	_rate_p(1) = _pitch_rate_p.get();
	_rate_i(1) = _pitch_rate_i.get();
	_rate_int_lim(1) = _pitch_rate_integ_lim.get();
	_rate_d(1) = _pitch_rate_d.get();
	_rate_ff(1) = _pitch_rate_ff.get();

	/* yaw gains */
	_rate_p(2) = _yaw_rate_p.get();
	_rate_i(2) = _yaw_rate_i.get();
	_rate_int_lim(2) = _yaw_rate_integ_lim.get();
	_rate_d(2) = _yaw_rate_d.get();
	_rate_ff(2) = _yaw_rate_ff.get();

	if (fabsf(_lp_filters_d.get_cutoff_freq() - _d_term_cutoff_freq.get()) > 0.01f) {
		_lp_filters_d.set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
		_lp_filters_d.reset(_rates_prev);
	}

	/* angular rate limits */
	_mc_rate_max(0) = math::radians(_roll_rate_max.get());
	_mc_rate_max(1) = math::radians(_pitch_rate_max.get());
	_mc_rate_max(2) = math::radians(_yaw_rate_max.get());

	/* auto angular rate limits */
	_auto_rate_max(0) = math::radians(_roll_rate_max.get());
	_auto_rate_max(1) = math::radians(_pitch_rate_max.get());
	_auto_rate_max(2) = math::radians(_yaw_auto_max.get());

	/* manual rate control acro mode rate limits and expo */
	_acro_rate_max(0) = math::radians(_acro_roll_max.get());
	_acro_rate_max(1) = math::radians(_acro_pitch_max.get());
	_acro_rate_max(2) = math::radians(_acro_yaw_max.get());

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);
}

void
MulticopterRateControl::parameter_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		updateParams();
		parameters_updated();
	}
}

void
MulticopterRateControl::vehicle_control_mode_poll()
{
	bool updated;

	/* Check if vehicle control mode has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
	}
}

bool
MulticopterRateControl::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
		return true;
	}
	return false;
}

bool
MulticopterRateControl::vehicle_rates_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
		return true;
	}
	return false;
}

void
MulticopterRateControl::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (_actuators_id == nullptr) {
			if (_vehicle_status.is_vtol) {
				_actuators_id = ORB_ID(actuator_controls_virtual_mc);

			} else {
				_actuators_id = ORB_ID(actuator_controls_0);
			}
		}
	}
}

void
MulticopterRateControl::vehicle_motor_limits_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_motor_limits_sub, &updated);

	if (updated) {
		multirotor_motor_limits_s motor_limits = {};
		orb_copy(ORB_ID(multirotor_motor_limits), _motor_limits_sub, &motor_limits);

		_saturation_status.value = motor_limits.saturation_status;
	}
}

void
MulticopterRateControl::battery_status_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_battery_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
	}
}

void
MulticopterRateControl::vehicle_land_detected_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_vehicle_land_detected_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
	}

}

void
MulticopterRateControl::landing_gear_state_poll()
{
	bool updated;
	orb_check(_landing_gear_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(landing_gear), _landing_gear_sub, &_landing_gear);
	}
}

float
MulticopterRateControl::get_landing_gear_state()
{
	// Only switch the landing gear up if we are not landed and if
	// the user switched from gear down to gear up.
	// If the user had the switch in the gear up position and took off ignore it
	// until he toggles the switch to avoid retracting the gear immediately on takeoff.
	if (_vehicle_land_detected.landed) {
		_gear_state_initialized = false;
	}
	float landing_gear = landing_gear_s::GEAR_DOWN; // default to down
	if (_manual_control_sp.gear_switch == manual_control_setpoint_s::SWITCH_POS_ON && _gear_state_initialized) {
		landing_gear = landing_gear_s::GEAR_UP;

	} else if (_manual_control_sp.gear_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
		// Switching the gear off does put it into a safe defined state
		_gear_state_initialized = true;
	}

	return landing_gear;
}

/*
 * Throttle PID attenuation
 * Function visualization available here https://www.desmos.com/calculator/gn4mfoddje
 * Input: 'tpa_breakpoint', 'tpa_rate', '_thrust_sp'
 * Output: 'pidAttenuationPerAxis' vector
 */
Vector3f
MulticopterRateControl::pid_attenuations(float tpa_breakpoint, float tpa_rate)
{
	/* throttle pid attenuation factor */
	float tpa = 1.0f - tpa_rate * (fabsf(_thrust_sp) - tpa_breakpoint) / (1.0f - tpa_breakpoint);
	tpa = fmaxf(TPA_RATE_LOWER_LIMIT, fminf(1.0f, tpa));

	Vector3f pidAttenuationPerAxis;
	pidAttenuationPerAxis(AXIS_INDEX_ROLL) = tpa;
	pidAttenuationPerAxis(AXIS_INDEX_PITCH) = tpa;
	pidAttenuationPerAxis(AXIS_INDEX_YAW) = 1.0;

	return pidAttenuationPerAxis;
}

/*
 * Attitude rates controller.
 * Input: '_rates_sp' vector, '_thrust_sp'
 * Output: '_att_control' vector
 */
void
MulticopterRateControl::control_attitude_rates(float dt, const Vector3f& rates)
{
	/* reset integral if disarmed */
	if (!_v_control_mode.flag_armed || !_vehicle_status.is_rotary_wing) {
		_rates_int.zero();
	}

	Vector3f rates_p_scaled = _rate_p.emult(pid_attenuations(_tpa_breakpoint_p.get(), _tpa_rate_p.get()));
	Vector3f rates_i_scaled = _rate_i.emult(pid_attenuations(_tpa_breakpoint_i.get(), _tpa_rate_i.get()));
	Vector3f rates_d_scaled = _rate_d.emult(pid_attenuations(_tpa_breakpoint_d.get(), _tpa_rate_d.get()));

	/* angular rates error */
	Vector3f rates_err = _rates_sp - rates;

	/* apply low-pass filtering to the rates for D-term */
	Vector3f rates_filtered(_lp_filters_d.apply(rates));

	_att_control = rates_p_scaled.emult(rates_err) +
		       _rates_int -
		       rates_d_scaled.emult(rates_filtered - _rates_prev_filtered) / dt +
		       _rate_ff.emult(_rates_sp);

	_rates_prev = rates;
	_rates_prev_filtered = rates_filtered;

	/* update integral only if we are not landed */
	if (!_vehicle_land_detected.maybe_landed && !_vehicle_land_detected.landed) {
		for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
			// Check for positive control saturation
			bool positive_saturation =
				((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_pos) ||
				((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_pos) ||
				((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_pos);

			// Check for negative control saturation
			bool negative_saturation =
				((i == AXIS_INDEX_ROLL) && _saturation_status.flags.roll_neg) ||
				((i == AXIS_INDEX_PITCH) && _saturation_status.flags.pitch_neg) ||
				((i == AXIS_INDEX_YAW) && _saturation_status.flags.yaw_neg);

			// prevent further positive control saturation
			if (positive_saturation) {
				rates_err(i) = math::min(rates_err(i), 0.0f);

			}

			// prevent further negative control saturation
			if (negative_saturation) {
				rates_err(i) = math::max(rates_err(i), 0.0f);

			}

			// Perform the integration using a first order method and do not propagate the result if out of range or invalid
			float rate_i = _rates_int(i) + rates_i_scaled(i) * rates_err(i) * dt;

			if (PX4_ISFINITE(rate_i) && rate_i > -_rate_int_lim(i) && rate_i < _rate_int_lim(i)) {
				_rates_int(i) = rate_i;

			}
		}
	}

	/* explicitly limit the integrator state */
	for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
		_rates_int(i) = math::constrain(_rates_int(i), -_rate_int_lim(i), _rate_int_lim(i));
	}
}

void
MulticopterRateControl::publish_rates_setpoint()
{
	_v_rates_sp.roll = _rates_sp(0);
	_v_rates_sp.pitch = _rates_sp(1);
	_v_rates_sp.yaw = _rates_sp(2);
	_v_rates_sp.thrust_body[0] = 0.0f;
	_v_rates_sp.thrust_body[1] = 0.0f;
	_v_rates_sp.thrust_body[2] = -_thrust_sp;
	_v_rates_sp.timestamp = hrt_absolute_time();
	orb_publish_auto(ORB_ID(vehicle_rates_setpoint), &_v_rates_sp_pub, &_v_rates_sp, nullptr, ORB_PRIO_DEFAULT);
}

void
MulticopterRateControl::publish_rate_controller_status()
{
	rate_ctrl_status_s rate_ctrl_status;
	rate_ctrl_status.timestamp = hrt_absolute_time();
	rate_ctrl_status.rollspeed = _rates_prev(0);
	rate_ctrl_status.pitchspeed = _rates_prev(1);
	rate_ctrl_status.yawspeed = _rates_prev(2);
	rate_ctrl_status.rollspeed_integ = _rates_int(0);
	rate_ctrl_status.pitchspeed_integ = _rates_int(1);
	rate_ctrl_status.yawspeed_integ = _rates_int(2);
	orb_publish_auto(ORB_ID(rate_ctrl_status), &_controller_status_pub, &rate_ctrl_status, nullptr, ORB_PRIO_DEFAULT);
}

void
MulticopterRateControl::publish_actuator_controls()
{
	_actuators.control[0] = (PX4_ISFINITE(_att_control(0))) ? _att_control(0) : 0.0f;
	_actuators.control[1] = (PX4_ISFINITE(_att_control(1))) ? _att_control(1) : 0.0f;
	_actuators.control[2] = (PX4_ISFINITE(_att_control(2))) ? _att_control(2) : 0.0f;
	_actuators.control[3] = (PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;
	_actuators.control[7] = get_landing_gear_state();
	_actuators.timestamp = hrt_absolute_time();

	/* scale effort by battery status */
	if (_bat_scale_en.get() && _battery_status.scale > 0.0f) {
		for (int i = 0; i < 4; i++) {
			_actuators.control[i] *= _battery_status.scale;
		}
	}

	if (!_actuators_0_circuit_breaker_enabled) {
		orb_publish_auto(_actuators_id, &_actuators_0_pub, &_actuators, nullptr, ORB_PRIO_DEFAULT);
	}

	// TODO: fix landing gear pub
	// orb_publish_auto(ORB_ID(landing_gear), &_landing_gear_pub, &attitude_setpoint, nullptr, ORB_PRIO_DEFAULT);
}

void
MulticopterRateControl::Run()
{
	init();
	parameter_update_poll();

	float dt_accumulator = 0.f;
	int loop_counter = 0;

	perf_begin(_loop_perf);

	/* run controller on gyro changes */
	const hrt_abstime now = hrt_absolute_time();
	/* guard against too small (< 0.2ms) and too large (> 20ms) dt's */
	const float dt = math::constrain((now - _last_run) / 1e6f, 0.0002f, 0.02f);
	_last_run = now;

	/* copy gyro data */
	vehicle_rates_s vrates;
	orb_copy(ORB_ID(vehicle_rates), _v_rates_sub, &vrates);

	_actuators.timestamp_sample = vrates.timestamp;

	Vector3f rates{vrates.roll, vrates.pitch, vrates.yaw};

	/* check for updates in other topics */
	vehicle_control_mode_poll();
	vehicle_status_poll();
	vehicle_motor_limits_poll();
	battery_status_poll();
	vehicle_land_detected_poll();
	landing_gear_state_poll();
	const bool manual_control_updated = vehicle_manual_poll();

	/* Check if we are in rattitude mode and the pilot is above the threshold on pitch
	 * or roll (yaw can rotate 360 in normal att control). If both are true don't
	 * even bother running the attitude controllers */
	if (_v_control_mode.flag_control_rattitude_enabled) {

		_v_control_mode.flag_control_attitude_enabled =
				fabsf(_manual_control_sp.y) <= _rattitude_thres.get() &&
				fabsf(_manual_control_sp.x) <= _rattitude_thres.get();
	}

	/* attitude controller disabled, poll rates setpoint topic */
	if (_v_control_mode.flag_control_manual_enabled && _vehicle_status.is_rotary_wing) {
		if (manual_control_updated) {
			/* manual rates control - ACRO mode */
			Vector3f man_rate_sp(
					math::superexpo(_manual_control_sp.y, _acro_expo_rp.get(), _acro_superexpo_rp.get()),
					math::superexpo(-_manual_control_sp.x, _acro_expo_rp.get(), _acro_superexpo_rp.get()),
					math::superexpo(_manual_control_sp.r, _acro_expo_y.get(), _acro_superexpo_y.get()));

			_rates_sp = man_rate_sp.emult(_acro_rate_max);
			_thrust_sp = _manual_control_sp.z;
			publish_rates_setpoint();
		}

	} else {
		/* attitude controller disabled, poll rates setpoint topic */
		if (vehicle_rates_setpoint_poll()) {
			_rates_sp(0) = _v_rates_sp.roll;
			_rates_sp(1) = _v_rates_sp.pitch;
			_rates_sp(2) = _v_rates_sp.yaw;
			_thrust_sp = -_v_rates_sp.thrust_body[2];
		}
	}

	/* run the rate controller immediately after a gyro update */
	if (!_v_control_mode.flag_control_termination_enabled) {
		if (_v_control_mode.flag_control_rates_enabled) {
			control_attitude_rates(dt, rates);

			publish_actuator_controls();
			publish_rate_controller_status();
		}
	} else {
		if (!_vehicle_status.is_vtol) {
			_rates_sp.zero();
			_rates_int.zero();
			_thrust_sp = 0.0f;
			_att_control.zero();
			publish_actuator_controls();
		}
	}

	/* calculate loop update rate while disarmed or at least a few times (updating the filter is expensive) */
	if (!_v_control_mode.flag_armed || (now - _task_start) < 3300000) {
		dt_accumulator += dt;
		++loop_counter;

		if (dt_accumulator > 1.f) {
			const float loop_update_rate = (float)loop_counter / dt_accumulator;
			_loop_update_rate_hz = _loop_update_rate_hz * 0.5f + loop_update_rate * 0.5f;
			dt_accumulator = 0;
			loop_counter = 0;
			_lp_filters_d.set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
		}
	}

	perf_end(_loop_perf);
}

int MulticopterRateControl::task_spawn(int argc, char *argv[])
{
	MulticopterRateControl *instance = new MulticopterRateControl();

	int ret = orb_register_work_callback(ORB_ID(vehicle_rates), 0, instance);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return ret;
}

int MulticopterRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int mc_rate_control_main(int argc, char *argv[])
{
	return MulticopterRateControl::main(argc, argv);
}
