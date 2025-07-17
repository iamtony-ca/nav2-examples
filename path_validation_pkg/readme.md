

ros2 launch path_validation_pkg path_validator_rviz_launch.py

ros2 run path_validation_pkg path_validator


```
ros2 topic pub --once /validate_path_request geometry_msgs/msg/PoseArray "
header:
  frame_id: 'map'
poses:
- position:
    x: 1.0
    y: 1.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.382 
    w: 0.924 
- position:
    x: -1.5
    y: -1.5
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
"
```