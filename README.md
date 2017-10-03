# ifttt-ros
A framework for integrating ROS systems with IFTTT.

## Vision
ROS offers many powerful packages that enable users to develop advanced robots without expert knowledge of kinematics, motion planning, computer vision etc. There are also packages available to allow access to robotic systems from browser-based applications and mobile devices without implementing the ROS communication protocols yourself. However, until now there has not been much in the way of packages that can be used to connect a ROS-based robot to web services or IoT devices with the same level of ease. This project aims to exploit the existing IFTTT infrastructure, including [services especially for Makers](https://platform.ifttt.com/maker) to make this right!

The project is planned to consist of components that you can plug into any current or new system to translate events and actions between IFTTT and ROS, as well as a [rosnodejs](https://github.com/RethinkRobotics-opensource/rosnodejs)-based test server.

## Cloning
When cloning this repo directly into a catkin workspace, set the name of the resulting directory to ifttt as follows:

`git clone https://github.com/JamesGiller/ifttt-ros.git ifttt`

This is because I have named the package "ifttt" in the configuration files (CMakeLists.txt, package.xml).

## Dependencie
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- [Node.js](https://nodejs.org/en/download/package-manager/#debian-and-ubuntu-based-linux-distributions) (v8.6.0)
- [NPM](https://docs.npmjs.com/getting-started/installing-node) (5.4.2)

Also check dependencies of [rosnodejs](http://wiki.ros.org/rosnodejs#Installation) relating to ROS.

## Building
1. Install dependencies from package.json

`npm install`

2. Build with catkin

`catkin build`

## Testing
To run the tests, you must install the source of Google Mock.
This can be done on Ubuntu 16.04 by executing the following:

`sudo apt install google-mock`

See [the catkin docs](http://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html#building-and-running-tests) for how to run tests.