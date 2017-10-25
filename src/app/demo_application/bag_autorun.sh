#!/bin/bash
source /home/px/demo_application/bashrc.sh;
rosparam set use_sim_time true &
rosservice call /Chisel/Reset &
sleep 1; rosbag play 2017-08-13-20-01-39.bag --clock
exec bash;
