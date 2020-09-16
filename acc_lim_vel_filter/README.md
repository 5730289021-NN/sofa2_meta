# acc_lim_vel_filter

## General description of the package

<!--- protected region package description begin -->
Acceleration Limit Velocity Filter
<!--- protected region package description end -->

<!--- todo How to handle the image generation -->
<!--- <img src="./model/acc_lim_vel_filter.png" width="300px" />-->

## Node: acc_lim_vel_filter

Update frequency: 20 Hz.

<!--- protected region acc_lim_vel_filter begin -->
<!--- protected region acc_lim_vel_filter end -->

### Static Parameters

All static parameters can be set through the command line:

```shell
rosrun acc_lim_vel_filter acc_lim_vel_filter [param_name]:=[new_value]
```

`acc_lin_lim` *(double, default: 0.6)*
<!--- protected region param acc_lin_lim begin -->
Limit Linear Acceleration
<!--- protected region param acc_lin_lim end -->
`dec_lin_lim` *(double, default: 0.6)*
<!--- protected region param dec_lin_lim begin -->
Limit Linear Deceleration
<!--- protected region param dec_lin_lim end -->
`acc_ang_lim` *(double, default: 0.6)*
<!--- protected region param acc_ang_lim begin -->
Limit Angular Acceleration
<!--- protected region param acc_ang_lim end -->
`dec_ang_lim` *(double, default: 0.6)*
<!--- protected region param dec_ang_lim begin -->
Limit Angular Deceleration
<!--- protected region param dec_ang_lim end -->
`goal_dist_thres` *(double, default: 1.5)*
<!--- protected region param goal_dist_thres begin -->
Goal Distance Threshold
<!--- protected region param goal_dist_thres end -->
`auto_min_vel` *(double, default: 0.1)*
<!--- protected region param auto_min_vel begin -->
Autonomous Minimum Velocity
<!--- protected region param auto_min_vel end -->
`Kp` *(double, default: 0.1)*
<!--- protected region param Kp begin -->
Displacement Gain which product of Kp * goal_dist_thres should less than 1
<!--- protected region param Kp end -->

### Published Topics

A topic can be remapped from the command line:

```shell
rosrun acc_lim_vel_filter acc_lim_vel_filter [old_name]:=[new_name]
```

`vel_out` *(geometry_msgs::Twist)*
<!--- protected region publisher vel_out begin -->
Output Filtered Velocity
<!--- protected region publisher vel_out end -->

### Subscribed Topics

A topic can be remapped from the command line:

```shell
rosrun acc_lim_vel_filter acc_lim_vel_filter [old_name]:=[new_name]
```

`vel_in` *(geometry_msgs::Twist)*
<!--- protected region subscriber vel_in begin -->
Input Velocity
<!--- protected region subscriber vel_in end -->
`amcl_pose` *(geometry_msgs::PoseWithCovarianceStamped)*
<!--- protected region subscriber amcl_pose begin -->
Current Pose
<!--- protected region subscriber amcl_pose end -->
`move_base_goal` *(move_base_msgs::MoveBaseActionGoal)*
<!--- protected region subscriber move_base_goal begin -->
move_base Goal Pose
<!--- protected region subscriber move_base_goal end -->
`control_mode` *(std_msgs::String)*
<!--- protected region subscriber control_mode begin -->
Control Mode
<!--- protected region subscriber control_mode end -->

---

*Package generated with the [ROS Package Generator](https://github.com/tecnalia-advancedmanufacturing-robotics/ros_pkg_gen).*
