#!/bin/bash

source /home/px/demo_application/bashrc.sh;
cd ~/bag/; rosbag record rosout /pg_17221110/image_raw /pg_17221121/image_raw /djiros/imu;
exec bash;
