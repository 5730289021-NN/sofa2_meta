# roboteq_tcp_driver

## General description of the package

<!--- protected region package description begin -->
ROS Roboteq Driver
<!--- protected region package description end -->

<!--- todo How to handle the image generation -->
<!--- <img src="./model/driver.png" width="300px" />-->

## Node: driver

Update frequency: 20 Hz.

This node is using `\tf` to broadcast transforms.

<!--- protected region driver begin -->
<!--- protected region driver end -->

### Static Parameters

All static parameters can be set through the command line:

```shell
rosrun roboteq_tcp_driver driver [param_name]:=[new_value]
```

`ip_addr` *(string, default: "192.168.200.171")*
<!--- protected region param ip_addr begin -->
IP Address
<!--- protected region param ip_addr end -->
`port_num` *(int, default: 9001)*
<!--- protected region param port_num begin -->
Port Number
<!--- protected region param port_num end -->
`wheel_circ` *(double, default: 0.47124)*
<!--- protected region param wheel_circ begin -->
Circumstance of the wheel (2*PI*radius)
<!--- protected region param wheel_circ end -->
`track_width` *(double, default: 0.34)*
<!--- protected region param track_width begin -->
Distance between center of two wheel
<!--- protected region param track_width end -->
`cpr` *(double, default: 200000)*
<!--- protected region param cpr begin -->
Number of counts per wheel revolution (ppr*gear_ratio)
<!--- protected region param cpr end -->
`gear_ratio` *(double, default: 20)*
<!--- protected region param gear_ratio begin -->
Gear Ratio
<!--- protected region param gear_ratio end -->
`invert_mul` *(int, default: -1)*
<!--- protected region param invert_mul begin -->
Multiplier for robot that need invert driving
<!--- protected region param invert_mul end -->

### Published Topics

A topic can be remapped from the command line:

```shell
rosrun roboteq_tcp_driver driver [old_name]:=[new_name]
```

`odom` *(nav_msgs::Odometry)*
<!--- protected region publisher odom begin -->
Wheel Odometry
<!--- protected region publisher odom end -->
`queryResult` *(std_msgs::String)*
<!--- protected region publisher queryResult begin -->
Direct Query Result
<!--- protected region publisher queryResult end -->

### Subscribed Topics

A topic can be remapped from the command line:

```shell
rosrun roboteq_tcp_driver driver [old_name]:=[new_name]
```

`queryCommand` *(std_msgs::String)*
<!--- protected region subscriber queryCommand begin -->
Direct Query Command
<!--- protected region subscriber queryCommand end -->
`cmd_vel` *(geometry_msgs::Twist)*
<!--- protected region subscriber cmd_vel begin -->
The command of linear.x and angular.z
<!--- protected region subscriber cmd_vel end -->

---

*Package generated with the [ROS Package Generator](https://github.com/tecnalia-advancedmanufacturing-robotics/ros_pkg_gen).*
