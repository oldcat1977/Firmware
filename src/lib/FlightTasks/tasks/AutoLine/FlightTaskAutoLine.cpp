/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file FlightAutoLine.cpp
 */

#include "FlightTaskAutoLine.hpp"
#include <mathlib/mathlib.h>

using namespace matrix;

static constexpr float SIGMA_NORM		=	0.001f;

bool FlightTaskAutoLine::activate()
{
	bool ret = FlightTaskAutoMapper::activate();

	for (int i = 0; i < 3; ++i) {
		_smoothing[i].reset(0.f, _velocity(i), _position(i));
	}

	return ret;
}

void FlightTaskAutoLine::_generateSetpoints()
{
	if (!PX4_ISFINITE(_yaw_setpoint)) {
		// no valid heading -> set heading along track
		_generateHeadingAlongTrack();
	}

	_generateAltitudeSetpoints();
	_generateXYsetpoints();
	_generateTrajectory();
}

void FlightTaskAutoLine::_generateHeadingAlongTrack()
{
	Vector2f prev_to_dest(_target - _prev_wp);
	_compute_heading_from_2D_vector(_yaw_setpoint, prev_to_dest);

}

void FlightTaskAutoLine::_generateXYsetpoints()
{
	// Get various path specific vectors. */
	Vector3f pos_traj;
	pos_traj(0) = _smoothing[0].getCurrentPosition();
	pos_traj(1) = _smoothing[1].getCurrentPosition();
	pos_traj(2) = _smoothing[2].getCurrentPosition();
	Vector2f u_prev_to_dest = Vector2f(_target - _prev_wp).unit_or_zero();
	Vector2f prev_to_pos(pos_traj - _prev_wp);
	Vector2f closest_pt = Vector2f(_prev_wp) + u_prev_to_dest * (prev_to_pos * u_prev_to_dest);
	Vector2f closest_to_dest(_target - pos_traj);
	Vector2f u_closest_to_dest(closest_to_dest.unit_or_zero());
	//Vector2f prev_to_dest(_target - _prev_wp);
	float speed_sp_track = _mc_cruise_speed;
	//float speed_sp_prev_track = math::max(Vector2f(_velocity_setpoint) * u_prev_to_dest, 0.0f);

	speed_sp_track = closest_to_dest.length() * 0.3f;
	speed_sp_track = math::constrain(speed_sp_track, 0.0f, _mc_cruise_speed);

	_position_setpoint(0) = closest_pt(0);
	_position_setpoint(1) = closest_pt(1);
	Vector2f velocity_sp_xy = u_closest_to_dest * speed_sp_track;
	_velocity_setpoint(0) = velocity_sp_xy(0);
	_velocity_setpoint(1) = velocity_sp_xy(1);

	_velocity_setpoint(0) =  _velocity_setpoint(0) + (_position_setpoint(0) - _smoothing[0].getCurrentPosition()) * 0.3f; // Along-track setpoint + cross-track P controller
	_velocity_setpoint(1) =  _velocity_setpoint(1) + (_position_setpoint(1) - _smoothing[1].getCurrentPosition()) * 0.3f;
}

void FlightTaskAutoLine::_generateAltitudeSetpoints()
{
	_velocity_setpoint(2) = (_target(2) - _smoothing[2].getCurrentPosition()) * 0.3f; // Generate a velocity targer for the trajectory using a stupid P loop
}

void FlightTaskAutoLine::_generateTrajectory()
{
	_smoothing[0].setMaxAccel(MPC_ACC_HOR_MAX.get());
	_smoothing[1].setMaxAccel(MPC_ACC_HOR_MAX.get());
	_smoothing[0].setMaxVel(_constraints.speed_xy);
	_smoothing[1].setMaxVel(_constraints.speed_xy);
	_smoothing[0].setMaxJerk(MPC_JERK_MAX.get());
	_smoothing[1].setMaxJerk(MPC_JERK_MAX.get());
	_smoothing[2].setMaxJerk(MPC_JERK_MAX.get());

	if (_velocity_setpoint(2) < 0.f) { // up
		_smoothing[2].setMaxAccel(MPC_ACC_UP_MAX.get());
		//_smoothing[2].setMaxVel(_constraints.speed_up);
		_smoothing[2].setMaxVel(10.f);

	} else { // down
		_smoothing[2].setMaxAccel(MPC_ACC_DOWN_MAX.get());
		//_smoothing[2].setMaxVel(_constraints.speed_down);
		_smoothing[2].setMaxVel(10.f);
	}
	for (int i = 0; i < 3; ++i) {
		_smoothing[i].updateDurations(_deltatime, _velocity_setpoint(i));
	}

	VelocitySmoothing::timeSynchronization(_smoothing, 2); // Synchronize x and y only

	Vector3f accel_sp_smooth;

	for (int i = 0; i < 3; ++i) {
		_smoothing[i].integrate(accel_sp_smooth(i), _velocity_setpoint(i), _position_setpoint(i));
	}
}
