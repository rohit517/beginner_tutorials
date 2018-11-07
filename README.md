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
git clone --single-branch -b Week10_HW https://github.com/rohit517/beginner_tutorials.git
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
