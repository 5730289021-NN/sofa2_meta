# system_checker

## General description of the package

<!--- protected region package description begin -->
System Checker
<!--- protected region package description end -->

<!--- todo How to handle the image generation -->
<!--- <img src="./model/system_checker.png" width="300px" />-->

## Node: system_checker

Update frequency: 1 Hz.

<!--- protected region system_checker begin -->
<!--- protected region system_checker end -->

### Subscribed Topics

A topic can be remapped from the command line:

```shell
rosrun system_checker system_checker [old_name]:=[new_name]
```

`move_base_status` *(actionlib_msgs::GoalStatusArray)*
<!--- protected region subscriber move_base_status begin -->
move base status
<!--- protected region subscriber move_base_status end -->
`scan_1` *(sensor_msgs::LaserScan)*
<!--- protected region subscriber scan_1 begin -->
Lidar 1 Topic
<!--- protected region subscriber scan_1 end -->
`scan_2` *(sensor_msgs::LaserScan)*
<!--- protected region subscriber scan_2 begin -->
Lidar 2 Topic
<!--- protected region subscriber scan_2 end -->
`scan_fusion` *(sensor_msgs::LaserScan)*
<!--- protected region subscriber scan_fusion begin -->
Lidar 3 Topic
<!--- protected region subscriber scan_fusion end -->

---

*Package generated with the [ROS Package Generator](https://github.com/tecnalia-advancedmanufacturing-robotics/ros_pkg_gen).*
