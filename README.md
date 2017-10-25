# ELEC5660_lab_code

## get code

`cd`

`git clone https://github.com/gaowenliang/ELEC5660_lab_code.git`


## install DJI sdk

`mkdir ws`

`cd ws/src/djiros/OSDK/Onboard-SDK-3.3/build/`

`cmake  ..`

`make -j4`

`sudo make install`

## install others

`cd`

`cd ws`

`catkin_make`

## get application

`cd`

`cd ws/src/app/demo_application`

`chmod +x hover.sh`

`chmod +x hover.desktop.desktop`

`cp hover.desktop.desktop ~/Desktop/`

# have fun!
