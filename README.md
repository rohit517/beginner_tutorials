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
git clone --recursive https://github.com/rohit517/beginner_tutorials.git
cd ..
catkin_make
```

## Demo

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
[ INFO] [1540878832.064775503]: Welcome to ENPM808X 2
[ INFO] [1540878832.164805350]: Welcome to ENPM808X 3
[ INFO] [1540878832.264797381]: Welcome to ENPM808X 4
[ INFO] [1540878832.364780430]: Welcome to ENPM808X 5
[ INFO] [1540878832.464758091]: Welcome to ENPM808X 6
[ INFO] [1540878832.564772969]: Welcome to ENPM808X 7
[ INFO] [1540878832.664750234]: Welcome to ENPM808X 8
[ INFO] [1540878832.764749461]: Welcome to ENPM808X 9
```

To launch the subscriber node, open a new terminal in your catkin workspace and run the following commands
```
source devel/setup.bash
rosrun beginner_tutorials listener
```

The output on the terminal will be similar to 
```
[ INFO] [1540878910.158962521]: I heard: [Welcome to ENPM808X 3]
[ INFO] [1540878910.258683733]: I heard: [Welcome to ENPM808X 4]
[ INFO] [1540878910.358709019]: I heard: [Welcome to ENPM808X 5]
[ INFO] [1540878910.458697160]: I heard: [Welcome to ENPM808X 6]
[ INFO] [1540878910.558673154]: I heard: [Welcome to ENPM808X 7]
[ INFO] [1540878910.658629519]: I heard: [Welcome to ENPM808X 8]
[ INFO] [1540878910.758689818]: I heard: [Welcome to ENPM808X 9]
[ INFO] [1540878910.858700028]: I heard: [Welcome to ENPM808X 10]
[ INFO] [1540878910.958850812]: I heard: [Welcome to ENPM808X 11]
[ INFO] [1540878911.058708591]: I heard: [Welcome to ENPM808X 12]
```

To close the nodes, press CTRL+C in the terminal.


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
