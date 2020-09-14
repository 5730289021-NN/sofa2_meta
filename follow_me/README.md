# follow_me

## General description of the package

<!--- protected region package description begin -->
follow_me
<!--- protected region package description end -->

<!--- todo How to handle the image generation -->
<!--- <img src="./model/follow_me.png" width="300px" />-->

## Node: follow_me

Update frequency: 20 Hz.

<!--- protected region follow_me begin -->
<!--- protected region follow_me end -->

### Static Parameters

All static parameters can be set through the command line:

```shell
rosrun follow_me follow_me [param_name]:=[new_value]
```

`Kp_x` *(double, default: 1)*
<!--- protected region param Kp_x begin -->
Gain Multiplier feedback(dist to vx) for x-direction
<!--- protected region param Kp_x end -->
`Kp_a` *(double, default: 1)*
<!--- protected region param Kp_a begin -->
Gain Multiplier feedback(pixel to wz) for theta-direction
<!--- protected region param Kp_a end -->
`vx_max` *(double, default: 0.5)*
<!--- protected region param vx_max begin -->
Maximum x-direction velocity
<!--- protected region param vx_max end -->
`wz_max` *(double, default: 0.6)*
<!--- protected region param wz_max begin -->
Maximum theta-direction velocity
<!--- protected region param wz_max end -->
`min_dist_fol` *(double, default: 1.5)*
<!--- protected region param min_dist_fol begin -->
Minimum follow distance(meter)
<!--- protected region param min_dist_fol end -->
`max_dist_fol` *(double, default: 4.0)*
<!--- protected region param max_dist_fol begin -->
Maximum follow distance(meter)
<!--- protected region param max_dist_fol end -->
`prob_thres` *(double, default: 0.9)*
<!--- protected region param prob_thres begin -->
Probability Threshold to follow
<!--- protected region param prob_thres end -->

### Published Topics

A topic can be remapped from the command line:

```shell
rosrun follow_me follow_me [old_name]:=[new_name]
```

`cmd_vel` *(geometry_msgs::Twist)*
<!--- protected region publisher cmd_vel begin -->
Robot Command Velocity
<!--- protected region publisher cmd_vel end -->
`status` *(std_msgs::String)*
<!--- protected region publisher status begin -->
follow_me status
<!--- protected region publisher status end -->

### Subscribed Topics

A topic can be remapped from the command line:

```shell
rosrun follow_me follow_me [old_name]:=[new_name]
```

`detected_people` *(object_msgs::ObjectsInBoxes)*
<!--- protected region detected_people begin -->
<!--- protected region detected_people end -->
`depth_image` *(sensor_msgs::Image)*
<!--- protected region depth_image begin -->
<!--- protected region depth_image end -->

### Services proposed

Any service name can be adjusted using the ROS remapping functionality:

```shell
rosrun follow_me follow_me [old_name]:=[new_name]
```

`command` *(std_srvs::SetBool)*
<!--- protected region service server command begin -->
Enable follow_me
<!--- protected region service server command end -->

---

*Package generated with the [ROS Package Generator](https://github.com/tecnalia-advancedmanufacturing-robotics/ros_pkg_gen).*
