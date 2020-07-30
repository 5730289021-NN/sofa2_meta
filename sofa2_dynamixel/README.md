# sofa2_dynamixel

## General description of the package

<!--- protected region package description begin -->
A Dynamixel Controller for SOFA-2 Robot
<!--- protected region package description end -->

<!--- todo How to handle the image generation -->
<!--- <img src="./model/dynamixel_controller.png" width="300px" />-->

## Node: dynamixel_controller

Update frequency: 20 Hz.

<!--- protected region dynamixel_controller begin -->
<!--- protected region dynamixel_controller end -->

### Static Parameters

All static parameters can be set through the command line:

```shell
rosrun sofa2_dynamixel dynamixel_controller [param_name]:=[new_value]
```

`hpai` *(int, default: 4)*
<!--- protected region param hpai begin -->
Head Pitch Axis Index
<!--- protected region param hpai end -->
`hyai` *(int, default: 3)*
<!--- protected region param hyai begin -->
Head Yaw Axis Index
<!--- protected region param hyai end -->
`cpuai` *(int, default: 2)*
<!--- protected region param cpuai begin -->
Camera Pitch UP Axis Index
<!--- protected region param cpuai end -->
`cpdai` *(int, default: 5)*
<!--- protected region param cpdai begin -->
Camera Pitch DOWN Axis Index
<!--- protected region param cpdai end -->

### Published Topics

A topic can be remapped from the command line:

```shell
rosrun sofa2_dynamixel dynamixel_controller [old_name]:=[new_name]
```

`joint_state` *(sensor_msgs::JointState)*
<!--- protected region publisher joint_state begin -->
Dynamixel Joint State
<!--- protected region publisher joint_state end -->

### Subscribed Topics

A topic can be remapped from the command line:

```shell
rosrun sofa2_dynamixel dynamixel_controller [old_name]:=[new_name]
```

`joy` *(sensor_msgs::Joy)*
<!--- protected region joy begin -->
<!--- protected region joy end -->

---

*Package generated with the [ROS Package Generator](https://github.com/tecnalia-advancedmanufacturing-robotics/ros_pkg_gen).*
