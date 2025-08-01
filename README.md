tf2_web_republisher 
===================

The tf2_web_republisher package can be used to throttle and precompute [tf transform] information to be sent via the [rosbridge_suite] to a [ros3djs] web client. The tf2_web_republisher is developed as part of the Robot Web Tools effort.

## Action interface
The ROS node starts a [ROS action](https://docs.ros.org/en/rolling/Concepts/About-Actionlib.html) server that allows clients to request transforms for a set of source frames relative to a target frame. The action server will compute the transforms at a specified rate and return them as feedback.

```
# goal
string[] source_frames
string target_frame
float32 angular_thres
float32 trans_thres
float32 rate
---
# result
---
# feedback
geometry_msgs/TransformStamped[] transforms
```

- `source_frames` - list of source TF frames to be transformed
- `target_frame` - target TF frame to which the source frames are transformed
- `angular_thres` - update threshold for angular changes in radians
- `trans_thres` - update threshold for translational changes in meters
- `rate` - maximum update rate of geometry_msgs/msg/TransformStamped messages on the feedback channel. Each feedback message contains a stamped transform for each specified source_frame.

## License
tf2_web_republisher is released with a BSD license. For full terms and conditions, see the [LICENSE](LICENSE) file.

## Authors
See the [AUTHORS](AUTHORS.md) file for a full list of contributors.

[tf transform]: https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Tf2.html#tf2
[rosbridge_suite]: https://github.com/RobotWebTools/rosbridge_suite
[ros3djs]: https://github.com/RobotWebTools/ros3djs
