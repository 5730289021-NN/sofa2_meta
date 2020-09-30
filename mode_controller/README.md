# mode_controller

## General description of the package

<!--- protected region package description begin -->
Custom Mode Controller for SOFA Robot
<!--- protected region package description end -->

<!--- todo How to handle the image generation -->
<!--- <img src="./model/mode_controller.png" width="300px" />-->

## Node: mode_controller

Update frequency: 10 Hz.

<!--- protected region mode_controller begin -->
<!--- protected region mode_controller end -->

### Static Parameters

All static parameters can be set through the command line:

```shell
rosrun mode_controller mode_controller [param_name]:=[new_value]
```

`follow_me_status_param` *(string, default: "/follow_me_status_param")*
<!--- protected region param follow_me_status_param begin -->
follow_me Status Parameter
<!--- protected region param follow_me_status_param end -->
`safe_btn` *(int, default: 0)*
<!--- protected region param safe_btn begin -->
Manual/Override(Safe) Control Button
<!--- protected region param safe_btn end -->
`cancel_btn` *(int, default: 3)*
<!--- protected region param cancel_btn begin -->
Cancel Control Button
<!--- protected region param cancel_btn end -->
`raw_btn` *(int, default: 1)*
<!--- protected region param raw_btn begin -->
Force/Force_Override(Raw) Control Button
<!--- protected region param raw_btn end -->

### Published Topics

A topic can be remapped from the command line:

```shell
rosrun mode_controller mode_controller [old_name]:=[new_name]
```

`control_mode` *(std_msgs::String)*
<!--- protected region publisher control_mode begin -->
Current Control Mode
<!--- protected region publisher control_mode end -->

### Subscribed Topics

A topic can be remapped from the command line:

```shell
rosrun mode_controller mode_controller [old_name]:=[new_name]
```

`move_base_status` *(actionlib_msgs::GoalStatusArray)*
<!--- protected region subscriber move_base_status begin -->
move_base Status
<!--- protected region subscriber move_base_status end -->
`joy` *(sensor_msgs::Joy)*
<!--- protected region subscriber joy begin -->
Joy
<!--- protected region subscriber joy end -->

### Direct Publishers

These publishers are not handled through the update loop.
Their publication frequency is thus unknown

`move_base_cancel` *(actionlib_msgs::GoalID)*
<!--- protected region direct publisher move_base_cancel begin -->
move_base Cancel
<!--- protected region direct publisher move_base_cancel end -->
`follow_me_enable` *(std_msgs::Bool)*
<!--- protected region direct publisher follow_me_enable begin -->
follow_me Enable
<!--- protected region direct publisher follow_me_enable end -->
`joy_final_vel` *(geometry_msgs::Twist)*
<!--- protected region direct publisher joy_final_vel begin -->
Output Final Joy Velocity
<!--- protected region direct publisher joy_final_vel end -->

### Direct Subscribers

These subscribers are not handled through the update loop.
The subscription handler is triggered as soon as a message arrives.

`joy_raw_vel` *(geometry_msgs::Twist)*
<!--- protected region direct subscriber joy_raw_vel begin -->
Input Raw Joy Velocity
<!--- protected region direct subscriber joy_raw_vel end -->
`joy_safe_vel` *(geometry_msgs::Twist)*
<!--- protected region direct subscriber joy_safe_vel begin -->
Input Safe Joy Velocity
<!--- protected region direct subscriber joy_safe_vel end -->

---

*Package generated with the [ROS Package Generator](https://github.com/tecnalia-advancedmanufacturing-robotics/ros_pkg_gen).*
