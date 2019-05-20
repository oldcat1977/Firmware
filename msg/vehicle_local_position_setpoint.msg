# Local position setpoint in NED frame
# setting something to NaN means the state should not be controlled

uint64 timestamp	# time since system start (microseconds)

float32[3] position	# in meters NED
float32[3] velocity	# in meters/second
float32[3] acceleration	# in meters/second^2
float32[3] jerk		# in meters/second^3
float32[3] thrust	# normalized thrust vector in NED
float32 yaw		# in radians NED -PI..+PI
float32 yawspeed	# in radians/sec

# TOPICS vehicle_local_position_setpoint trajectory_setpoint
