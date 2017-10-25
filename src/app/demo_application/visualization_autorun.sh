#!/bin/bash

source /home/px/demo_application/bashrc.sh;
roscore &
#sleep 2; #ssh -t ubuntu@tegra1 "echo 1 | sudo -S date --set \"$(date -u)\"" &
#sleep 1; #ssh -t ubuntu@tegra2 "echo 1 | sudo -S date --set \"$(date -u)\"" &
sleep 1; roslaunch vins_estimator vins_rviz.launch &
exec bash;
