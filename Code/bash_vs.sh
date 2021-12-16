#!/bin/bash

echo 'Starting autonomous driving'

gnome-terminal --working-directory='/home/mscv_gr1@CO-ROBOT03' --tab -- bash -c "echo 'roscore';
 roscore;
 exec bash"

sleep 2

gnome-terminal --working-directory='/home/mscv_gr1@CO-ROBOT03' --tab -- bash -c "echo 'Launch camera'; roslaunch ueye_cam rgb8.launch';
exec bash"

sleep 2

gnome-terminal --working-directory='/home/mscv_gr1@CO-ROBOT03' --tab -- bash -c "echo 'Image view camera image';
 rosrun image_view image_view image:=/camera/image_raw;
 exec bash"

sleep 2

gnome-terminal --working-directory='/home/mscv_gr1@CO-ROBOT03' --tab -- bash -c "echo 'Connecting to turtlebot with password to launch the bringup';
echo 'Copy Paste the following line:';
echo 'roslaunch turtlebot3_bringup turtlebot3_robot.launch';
sshpass -p 'napelturbot' ssh ubuntu@192.168.0.200;
exec bash"

sleep 7

gnome-terminal --working-directory='/home/mscv_gr1@CO-ROBOT03' --tab -- bash -c "echo 'Press k to use keyboard, press j to use controller.';
read varname;
if $varname = "k""

then gnome-terminal --working-directory='/home/mscv_gr1@CO-ROBOT03' --tab -- bash -c "echo 'Teleop for joystick';
echo 'plug the usb of the controller';
 roslaunch teleop_twist_joy logitech.launch;
 exec bash"

sleep 2
elif
gnome-terminal --working-directory='/home/mscv_gr1@CO-ROBOT03' --tab -- bash -c "echo 'Teleop with keyboard';
  roslaunch turtlebot3_teleop keyboard_teleop.launch;
  exec bash"

sleep 2

gnome-terminal --working-directory='/home/mscv_gr1@CO-ROBOT03' --tab -- bash -c "echo 'Image registration';
 rosrun image_view video_recorder image:=/camera/rgb/image_raw
 exec bash"

sleep 2

gnome-terminal --working-directory='/home/mscv_gr1@CO-ROBOT03' --tab -- bash -c "echo 'Graphe'; mv /Download/output.avi /Desktop/VS_Project
 python aruco_pose_estimation.py;
 exec bash"








