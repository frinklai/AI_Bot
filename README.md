# About AI_Bot
This control system of AI Bot(8-DOF) made by TKU ICLab.

# AI_Bot's functions
1. Vision servo using yolo_V3
2. Adaptive redundancy control (null space control).

# Illustration about siang_lin's adaptive 3 finger demo (2nd year final demo)

```roslaunch strategy Vision_servo_setup.launch```      //open pointgrey camera and yolov3 

```roslaunch manipulator_h_manager left_arm.launch```   // enable arm 

```roslaunch comm_stm32 ui_gripper_control.launch```    // enable serial communication and ui web socket 

```rosrun strategy vision_servo_use_gripper.py```       // run strategy 




  
