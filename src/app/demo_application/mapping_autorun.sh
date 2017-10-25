#!/bin/bash
source /home/px/demo_application/bashrc.sh;

#terminator -T "sgbm" -x ssh -t nvidia@tegra2 /home/nvidia/demo_application/launch_sgbm.sh

roslaunch chisel_ros omni.launch & 
sleep 1; rosrun point_cloud_pub color &
sleep 1; rosservice call /Chisel/Reset &
exec bash;

