# offboard setpoints
uint8 TYPE_LOITER=0	# loiter setpoint
uint8 TYPE_TAKEOFF=1	# takeoff setpoint
uint8 TYPE_LAND=2	# land setpoint, altitude must be ignored, descend until landing
uint8 TYPE_IDLE=3	# do nothing, switch off motors or keep at idle speed (MC)
uint8 TYPE_STREAM=4   # offboard setpoints are continuously applied

uint8 VELOCITY_FRAME_LOCAL_NED = 1 # MAV_FRAME_LOCAL_NED
uint8 VELOCITY_FRAME_BODY_NED = 8 # MAV_FRAME_BODY_NED

px4/vehicle_local_position_setpoint setpoint # position/velocity/acceleration/thrust setpoints
uint8 velocity_frame		# to set velocity setpoints in NED or body
uint8 type			# setpoint type to adjust behavior of position controller

# TOPICS offboard_setpoints
