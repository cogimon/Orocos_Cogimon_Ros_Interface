# Orocos_Cogimon_Ros_Interface

This repository provides an interface to communicate  with Coman in the gazebo/Orocos environment by using Ros topics.  Fethures of this commit: 1- Controlling dual robotic arm. 2- Controlling the position of an object.

If you want to control Coman, you can checkout [this](https://github.com/cogimon/Orocos_Cogimon_Ros_Interface/tree/b7c195a874ea28af50a5fe3c7d46bfd57ddca725).

---

#### Requirements:

OS: Ubuntu 14.04 or 16.04
ROS compatibility: Indigo, Kinetic

| Dependencies  |
| ------------- |
| [Orocos-ROS integration](https://github.com/orocos/rtt_ros_integration)         |
| [CoSimA-environment](http://cogimon.github.io/runtime/gettingstarted.html)  |
| [CoSimA-utilities](https://github.com/cogimon/cosima-utilities) |


## Set-up:

As the Orocos-Ros integration package is completely independent from the Coman-cogimon enviroment, we need to make sure that both packages are installed correctly and they are fully functional. 
Let's assume that the  Coman-enviroment package is installed here:
```
/opt/cogimon
```
and the Orocos-Ros integration package is installed here:
```
/home/joshua/catkin_ws/src
```
and
```
/home/joshua/ws/underlay_isolated
```

open the '.bashrc' and add the following lines:

```
source /opt/ros/kinetic/setup.bash
export OROCOS_TARGET=xenomai # OROCOS_TARGET=gnulinux if you are not using the realtime kernel
source /opt/cogimon/cogimon-minimal-nightly/bin/setup-cogimon-env.sh
source /home/joshua/ws/underlay_isolated/install_isolated/setup.bash
source /home/joshua/catkin_ws/devel/setup.bash
export RTT_COMPONENT_PATH=$RTT_COMPONENT_PATH:/opt/cogimon/cogimon-minimal-nightly/lib/orocos
```
If  both packages are installed correctly, their examples should run without any problem.

## Run:

1- In Terminal 1:

```
roscore
```

2- In Terminal 2:
```
rsb0.14 server
```

3- In Terminal 3
```
deployer-xenomai -s /home/joshua/catkin_ws/src/Orocos_Cogimon_Ros_Interface/Integration_IJRR.ops
```
Important Note: There are some lines in ```Integration_IJRR.ops``` which need to be changed. 

4- In Terminal 4

```
gzclient
```

## Test:

In Terminal 5
```
rostopic list 
```

should result in
```
/KUKA/Left/Dq/out
/KUKA/Left/T/out
/KUKA/Left/in
/KUKA/Left/q/out
/KUKA/Right/Dq/out
/KUKA/Right/T/out
/KUKA/Right/in
/KUKA/Right/q/out
/Object/Pos/out
/Object/Position/in
/Object/Vel/out
/rosout
/rosout_agg
```

