<h1 align="center"> ROS Publisher/Subscriber Beginner Tutorials </h1>
<p align="center">
<a href='https://github.com/rohit517/beginner_tutorials/blob/master/LICENSE'><img src='https://img.shields.io/badge/license-MIT-blue.svg'/></a>
</p>

## Overview

ROS Package for publisher and subscriber nodes based on [ROS tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29) 

## Dependencies
For running the above package we must have the following dependencies:

- Ubuntu 16.04
- ROS Kinetic

ROS Kinetic can be installed by following the instructions given [here](http://wiki.ros.org/kinetic/Installation). 

## Build instructions

We first need to create a catkin workspace. In your directory, run the following commands
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
We then source the setup.bash file from the devel folder.
```
source devel/setup.bash
```
We now clone the beginner_tutorials package in the src folder and build the package.
```
cd src/
git clone --single-branch -b Week11_HW https://github.com/rohit517/beginner_tutorials.git
cd ..
catkin_make
```

## Demo using rosrun 

Run the following commands in the terminal opened in your catkin workspace. First we need to start roscore. If you already have
roscore running, you may skip this command.
```
roscore
```

To launch the publisher node, open a new terminal in your catkin workspace and run the following commands
```
source devel/setup.bash
rosrun beginner_tutorials talker
```

The output on the terminal will be similar to 
```
[ INFO] [1540878831.864738212]: Welcome to ENPM808X 0
[ INFO] [1540878831.964824548]: Welcome to ENPM808X 1
```

To launch the subscriber node, open a new terminal in your catkin workspace and run the following commands
```
source devel/setup.bash
rosrun beginner_tutorials listener
```

The output on the terminal will be similar to 
```
[ INFO] [1540878910.158962521]: I heard: [Welcome to ENPM808X 0]
[ INFO] [1540878910.258683733]: I heard: [Welcome to ENPM808X 1]

```
To close the nodes, press CTRL+C in the terminal.


## Run demo using launch file

To run the demo using the launch file, open a terminal and run the following commands. This will start the talker and listener node with the talker node publishing at 10Hz by default.
```
cd catkin_ws
source devel/setup.bash
roslaunch beginner_tutorials beginner_tutorial.launch
```

Alternatively, you can launch the nodes by specifying the frequency as follows.
```
roslaunch beginner_tutorials beginner_tutorial.launch frequency:=5
```
Here we pass the frequency as 5Hz. Setting the value to a negative number will not start the nodes and a zero value will start the nodes at default frequency of 10Hz. To close the nodes, press CTRL+C in the terminal.


## Changing output text using service

Once you start the nodes using the launch file as shown above, we can update the string being published by the ModifyText service. To check if the service is running, run the following command. 
```
rosservice list
```

You should see the following output. /ModifyText is the sevice we need to call to update the text.
```
/ModifyText
/listener/get_loggers
/listener/set_logger_level
/rosout/get_loggers
/rosout/set_logger_level
/talker_node/get_loggers
/talker_node/set_logger_level
```

Run the following command to change the output text. Here we update the text to 'Updated Message' which can be seen on the listener terminal. Note that an empty string will raise an error and will not update the string.
```
rosservice call /ModifyText "Updated Message"
```

## Logging

To see the output in rqt_console, run the following command. We can filter output based on the logger level.
```
rosrun rqt_console rqt_console
```

## Inspecting TF Frames

The talker node broadcasts a tf frame named /talk with parent as /world. This frame has a non-zero translation and rotation with respect to the world frame. This can be verified using tf_echo and rqt_tf_tree. 

To use tf_echo, start roscore and the talker node and run the following command in terminal
```
rosrun tf tf_echo /world /talk
```
A sample output can be seen below. The translation and rotation change with time. 
```
At time 1542141785.075
- Translation: [1.696, 3.000, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.656, 0.755]
            in RPY (radian) [0.000, -0.000, 1.431]
            in RPY (degree) [0.000, -0.000, 82.001]
At time 1542141785.775
- Translation: [-1.535, 3.000, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.954, 0.301]
            in RPY (radian) [0.000, -0.000, 2.530]
            in RPY (degree) [0.000, -0.000, 144.969]
At time 1542141786.775
```

While this is running, we can use rqt_tf_tree to visualize the tree of frames. Run the following command to visualize rqt_tf_tree
```
rosrun rqt_tf_tree rqt_tf_tree
```
This visualization can be saved to a pdf by running the following command. To visualize the frames, one can also use rviz. The programs can be stopped by pressing Ctrl+C in terminal. 
```
rosrun tf view_frames
```

## ROSTEST

The tests for talker node are written in gtest and rostest. To build the tests, run the following commands
```
cd ~/catkin_ws
source devel/setup.bash
catkin_make run_tests_beginner_tutorials
```

The tests can be run by the entering the following commands in the terminal
```
cd ~/catkin_ws
source devel/setup.bash
rostest beginner_tutorials talkerTest.launch
```
A sample output can be seen below
```
... logging to /home/rohit/.ros/log/rostest-rohit-Alienware-17-R4-8930.log
[ROSUNIT] Outputting test results to /home/rohit/.ros/test_results/beginner_tutorials/rostest-test_talkerTest.xml
[Testcase: testtalkerTest] ... ok

[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-talkerTest/testModifyTextServiceInit][passed]
[beginner_tutorials.rosunit-talkerTest/testModifyTextServiceCall][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0

rostest log file is in /home/rohit/.ros/log/rostest-rohit-Alienware-17-R4-8930.log
```
## Recording bag file with launch file

The launch file is updated to add a node for rosbag to record all the topics being published into the results folder. File named talker.bag can be found inside the results folder with a ~15 sec recording. 

To record using rosbag, first we need to start roscore if not already running
```
roscore
```
Once roscore is running, open a new terminal and enter the following commands to launch the beginner_tutorial.launch file. The argument record is used to choose whether to record a bagfile or not. Default is false (record:=false). 
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch beginner_tutorials beginner_tutorial.launch record:=true
```
This will record all the topics and store the bag file in the results folder. 

### Inspect bag file
To inspect the bagfile created in the above step run the following commands
```
cd ~/catkin_ws/src/beginner_tutorials/results
rosbag info talker.bag
```
A sample output can be seen below
```
path:        talker.bag
version:     2.0
duration:    17.4s
start:       Nov 13 2018 14:19:12.73 (1542136752.73)
end:         Nov 13 2018 14:19:30.11 (1542136770.11)
size:        197.4 KB
messages:    930
compression: none [1/1 chunks]
types:       rosgraph_msgs/Log  [acffd30cd6b6de30f120938c17c593fb]
             std_msgs/String    [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage [94810edda583a504dfda3829e70d7eec]
topics:      /chatter      153 msgs    : std_msgs/String   
             /rosout       313 msgs    : rosgraph_msgs/Log  (3 connections)
             /rosout_agg   311 msgs    : rosgraph_msgs/Log 
             /tf           153 msgs    : tf2_msgs/TFMessage
```

### Play bag file with listener node
The rosbag generated, can be played with the listener node running. We can see the output in the terminal where listener node is running once we start the rosbag. Follow the steps below. 
Start roscore if already not running
```
roscore
```
Open a new terminal where we start the listener node
```
cd ~/catkin_ws
source devel/setup.bash
rosrun beginner_tutorials listener
```
Next, play the rosbag file.
```
cd ~/catkin_ws/src/beginner_tutorials/results
rosbag play talker.bag
```
We can see the output on the listener node that are recorded in the bagfile.

## License
```
MIT License

Copyright (c) 2018 Rohit

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
