# ELEC5660_lab_code

## get code

`cd`

`git clone https://github.com/gaowenliang/ELEC5660_lab_code.git`


## install DJI sdk

`cd`

`cd ELEC5660_lab_code/src/djiros/OSDK/Onboard-SDK-3.3/`

`mkdir build`

`cd build`

`cmake  ..`

`make -j4`

`sudo make install`

`sudo apt-get install libarmadillo-dev`

## install others

`cd`

`cd ELEC5660_lab_code`

`catkin_make`

after all errors are debuged, run:

`echo "source /home/nvidia/ELEC5660_lab_code/devel/setup.bash" >> ~/.bashrc`

`source ~/.bashrc`


## get application

`cd`

`cd ELEC5660_lab_code/src/app/demo_application`

`chmod +x hover.sh`

`cd apps`

`chmod +x hover.desktop.desktop`

`cp hover.desktop.desktop ~/Desktop/`

# have fun!
