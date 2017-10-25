#!/bin/bash
terminator -T "shutdown" -x ssh -t ubuntu@tegra-ubuntu /home/ubuntu/demo_application/run_chicken_blood.sh
sleep 1; echo 666;
