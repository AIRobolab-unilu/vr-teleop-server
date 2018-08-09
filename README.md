# vr_teleop_server

This repository permits to run a ROS bridge websocket server to communicate to a Unity program. Here is the Unity program : https://github.com/AIRobolab-unilu/vr_teleop
## User

To use this repository, the robot needs to have [ROS](http://www.ros.org/). if you want to run a dialog and have a motivational component, here is another repository : https://github.com/AIRobolab-unilu/qt_dialog.To run this program, you need to clone it, compile it with catkin and then you can launch it with

```bash
roslaunch vr_teleop_server run.launch
```

## Developer
This quick tutorial assumes that the you know how to code on ROS.If you want to use this code with QTrobot, this is straightforward. If you don't, you will need to change things in the code. The file data/motors.json contains the values for the motors. "min" is the lowest value for the motor (in degree) and "max" is the maximum value. "home" is the position of the motor at the default state.The service "IncrementMotor" aims to control the motors. The string needs to be the name of the motor (neck_h for instance) and the value will be added to the current value of the motor.

The aim of the "ros_side.py" file is to interacts directly with the Unity side, by sending the command to the correct topic in the correct protocol or by receiving and aggregate the different status values to send them to Unity.