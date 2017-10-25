#!/bin/bash

source /home/px/demo_application/bashrc.sh;

# roscore & sensor & ctrl & rviz & VINS+mapping@TX1
roslaunch djiros djirostrigger.launch &
sleep 3; roslaunch vins_estimator px2.launch &
sleep 3; roslaunch machine_defined ctrl_md.launch | grep WARN
sleep 3; rosservice call /Chisel/Reset &
exec bash;
