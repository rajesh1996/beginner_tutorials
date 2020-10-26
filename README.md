## ROS - Beginner Tutorials
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Author
Rajeshwar N S

## Overview
Create a ROS package with simple subscriber-publisher nodes in C++

## Dependencies
* Ubuntu-18.04
* ROS Melodic

## Create catkin workspace
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

## Build Instructions
* Clone the below repoitory in catkin _ws
```
$ cd ~/catkin_ws/src
$ https://github.com/rajesh1996/beginner_tutorials.git
```
* Source the directory and then build it
```
$ cd ..
$ source ~/catkin_ws/devel/setup.bash
$ catkin_make
```

## Run Instructions
* Start the ros master
```
$ roscore
```
* In a new terminal source the dir again and run the talker node to publish the message

```
$ source ~/catkin_ws/devel/setup.bash
$ rosrun beginner_tutorials talker
```
* In another terminal source the dir again and run the listener node to subscribe to the message published
```
$ source ~/catkin_ws/devel/setup.bash
$ rosrun beginner_tutorials listener
```
* Ctrl + C in each window to terminate the program

## Run cppcheck

To detect bugs and perform static code analysis, cppcheck is used. It can be installed with the following command:
```
$ sudo apt-get install cppcheck
```
To run cppcheck, run the following command in the `beginner_tutorials` folder:
```
$ cppcheck --enable=all --std=c++11 -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" )
```

## Run cpplint

To check if the Google style guide is followed cpplint is used:
```
$ sudo apt-get install cpplint
```

To run the cpplint, follow the following commands in the `beginner_tutorials` directory:
```
$ cpplint $( find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./build/" -e "^./vendor/" -e "^./data" )
```




