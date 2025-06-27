Here is a sample command to piblish desired position values to robot
```shell
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  header: {
    stamp: {sec: 0, nanosec: 0}
  },
  joint_names: ['link_1_to_link_2', 'link_2_to_link_3', 'link_3_to_link_4'],
  points: [
    {
      positions: [0.5, 0.5, -0.5],
      velocities: [],
      accelerations: [],
      effort: [],
      time_from_start: {sec: 3, nanosec: 0}
    }
  ]
}"

```