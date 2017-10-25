#!/bin/bash
source /home/uav/demo_application/bashrc.sh;
roslaunch ces_planner planning_core.launch &
sleep 2; 
exec bash; 

