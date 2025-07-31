ros2 topic pub --once --qos-durability transient_local /velocity_modifier/control robot_interfaces/msg/ModifierControl 'command_type: 1
linear_value: 0.35
angular_value: 0.5
' 



# Constants defining the type of modification
uint8 TYPE_UNSPECIFIED = 0
uint8 TYPE_SPEED_LIMIT = 1
uint8 TYPE_SPEED_SCALE = 2

# The type of modification to apply
uint8 command_type

# For TYPE_SPEED_LIMIT: The maximum linear velocity (m/s).
# For TYPE_SPEED_SCALE: The multiplicative scale factor (e.g., 0.5 for 50%).
float32 linear_value

# For TYPE_SPEED_LIMIT: The maximum angular velocity (rad/s).
# For TYPE_SPEED_SCALE: This field is unused.
float32 angular_value