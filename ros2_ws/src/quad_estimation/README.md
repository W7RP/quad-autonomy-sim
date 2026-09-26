# quad_estimation (Phase 2, stub)

A hand-rolled C++20 error-state EKF fusing IMU + optical flow/range (then visual
odometry), validated against Gazebo ground truth. It has bounded-time callbacks, no
heap allocation in the hot path, and an explicit callback-group and executor design.

Not a ROS package yet (no `package.xml`), so colcon skips it.
Design brief: [docs/phase2_state_estimation.md](../../../docs/phase2_state_estimation.md).
