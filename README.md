## ROS - Beginner Tutorials
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://github.com/rajesh1996/beginner_tutorials/blob/master/LICENSE)

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
* In a new terminal source the dir again and run the launch file to run the publisher and subscriber

```
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch beginner_tutorials beg_nodes.launch freq_rate:=10

```
* In another terminal source the dir again and run the service call to publish new message 
```
$ source ~/catkin_ws/devel/setup.bash
$ rosservice call /changeString "Hello"

```
* To see message logs, run the rqt console by typing in a new terminal
```
rqt_console

```
* Ctrl + C in each window to terminate the program

## Inspect TF Frames
* The talker publishes tf topic of a static frame /talk
* To check tf tree, run the following
```
source ~/catkin_ws/devel/setup.bash
rosrun rqt_tf_tree rqt_tf_tree
```
* To echo the values, run the following,
```
source ~/catkin_ws/devel/setup.bash
rosrun tf tf_echo /world /talk
```
* To download the broadcaster diagram, run the following. This will generate the frames.pdf in the workspace. A sample could be seen in results dir
```
source ~/catkin_ws/devel/setup.bash
rosrun tf view_frames
```

# Run Unit Tests
## Using Launch
* Start the roscore and run the following in a new terminal
```
source ~/catkin_ws/devel/setup.bash
rostest beginner_tutorials test.launch
```
## Using catkin
```
source ~/catkin_ws/devel/setup.bash
catkin_make run_tests_beginner_tutorials
```
## Run testTalker node
* Start the roscore and in a new terminal run the publisher node and again open a new terminal and run the following
```
source ~/catkin_ws/devel/setup.bash
rosrun beginner_tutorials testTalker
```

## Record ROSbag
* start roscore and type the following in a new terminal
```
source ~/catkin_ws/devel/setup.bash
roslaunch beginner_tutorials beg_nodes.launch start_record:=true
```
* This command will ouput a bag file containing the messages in the results dir

## Play rosbag file
* Start roscore and in a new terminal, run the listener
```
source ~/catkin_ws/devel/setup.bash
rosrun beginner_tutorials listener
```
* In an new terminal, play the stored rosbag in the results dir. Now we will be able to see messages in the listener terminal
```
source ~/catkin_ws/devel/setup.bash
cd catkin_ws/src/beginner_tutorials/results
rosbag play ros_bag.bag
```
## Run cppcheck
To run cppcheck, run the following command in the `beginner_tutorials` folder:
```
$ cppcheck --enable=all --std=c++11 -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" )
```

## Run cpplint
To run the cpplint, follow the following commands in the `beginner_tutorials` directory:
```
$ cpplint $( find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./build/" -e "^./vendor/" -e "^./data" )
```




