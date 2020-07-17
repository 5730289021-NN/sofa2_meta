# costmap_based_vel_filter

## General description of the package

<!--- protected region package description begin -->
Velocity Filter based on provided costmap
<!--- protected region package description end -->

<!--- todo How to handle the image generation -->
<!--- <img src="./model/costmap_based_vel_filter.png" width="300px" />-->

## Node: costmap_based_vel_filter

Update frequency: 10 Hz.

<!--- protected region costmap_based_vel_filter begin -->
<!--- protected region costmap_based_vel_filter end -->

### Static Parameters

All static parameters can be set through the command line:

```shell
rosrun costmap_based_vel_filter costmap_based_vel_filter [param_name]:=[new_value]
```

`radius` *(double, default: 0.5)*
<!--- protected region param radius begin -->
Robot Radius
<!--- protected region param radius end -->
`forecasting_time` *(double, default: 0.5)*
<!--- protected region param forecasting_time begin -->
Forecasting Time
<!--- protected region param forecasting_time end -->
`h` *(int, default: 5)*
<!--- protected region param h begin -->
Horizon Number(Amount of sample within forecast time)
<!--- protected region param h end -->
`polyno` *(int, default: 5)*
<!--- protected region param polyno begin -->
Polygon Number
<!--- protected region param polyno end -->
`max_cost_threshold` *(int, default: 85)*
<!--- protected region param max_cost_threshold begin -->
Maximum Cost Threshold (cost > max_cost ? cost=255:cost)
<!--- protected region param max_cost_threshold end -->
`Kc` *(double, default: 1.5)*
<!--- protected region param Kc begin -->
Cost Multiplier [vel_out = vel_in * (1 - max(bound(Kc * cost[i], 255)) / 255)]
<!--- protected region param Kc end -->

### Subscribed Topics

A topic can be remapped from the command line:

```shell
rosrun costmap_based_vel_filter costmap_based_vel_filter [old_name]:=[new_name]
```

`amcl_pose` *(geometry_msgs::PoseWithCovarianceStamped)*
<!--- protected region subscriber amcl_pose begin -->
Current Pose of the Robot
<!--- protected region subscriber amcl_pose end -->
`costmap` *(nav_msgs::OccupancyGrid)*
<!--- protected region subscriber costmap begin -->
Rolling Window Costmap
<!--- protected region subscriber costmap end -->
`move_base_status` *(actionlib_msgs::GoalStatusArray)*
<!--- protected region subscriber move_base_status begin -->
movebase status which will disable this node when status is active
<!--- protected region subscriber move_base_status end -->
`force_control` *(std_msgs::Bool)*
<!--- protected region subscriber force_control begin -->
Force Control
<!--- protected region subscriber force_control end -->

### Direct Publishers

These publishers are not handled through the update loop.
Their publication frequency is thus unknown

`pose_array` *(geometry_msgs::PoseArray)*
<!--- protected region direct publisher pose_array begin -->
Array of Simulated Bound
<!--- protected region direct publisher pose_array end -->
`vel_out` *(geometry_msgs::Twist)*
<!--- protected region direct publisher vel_out begin -->
Filtered Velocity
<!--- protected region direct publisher vel_out end -->

### Direct Subscribers

These subscribers are not handled through the update loop.
The subscription handler is triggered as soon as a message arrives.

`vel_in` *(geometry_msgs::Twist)*
<!--- protected region direct subscriber vel_in begin -->
Input Velocity
<!--- protected region direct subscriber vel_in end -->

---

*Package generated with the [ROS Package Generator](https://github.com/tecnalia-advancedmanufacturing-robotics/ros_pkg_gen).*
