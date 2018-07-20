# ifttt-ros
A framework for integrating ROS systems with IFTTT.

## Vision
ROS offers many powerful packages that enable users to develop advanced robots without expert knowledge of kinematics, motion planning, computer vision etc. There are also packages available to allow access to robotic systems from browser-based applications and mobile devices without implementing the ROS communication protocols yourself. However, until now there has not been much in the way of packages that can be used to connect a ROS-based robot to web services or IoT devices with the same level of ease. This project aims to exploit the existing IFTTT infrastructure, including [services especially for Makers](https://platform.ifttt.com/maker) to make this right!

The project is planned to consist of components that you can plug into any current or new system to translate events and actions between IFTTT and ROS, as well as a [rosnodejs](https://github.com/RethinkRobotics-opensource/rosnodejs)-based test server.

## Current Status
### Supported Integrations
Making use of the [IFTTT Webhooks service](https://ifttt.com/maker_webhooks), the following integrations with the IFTTT platform are currently supported:

- ROS topics --> IFTTT triggers
- Web requests --> ROS action requests

This project provides a C++ class `TriggerEventPublisher` to enable users to translate any ROS message to a `TriggerEvent` message that the rosnodejs-based test server will listen for. Users can also try the [topic_tools transform](https://wiki.ros.org/topic_tools/transform) command to translate messages in Python.
The test server will automatically make a web request to the Webhooks service that will cause an IFTTT trigger to fire.

When made available at a public URL (e.g. by using [ngrok](https://ngrok.com/)), the test server will listen for web requests to `/ifttt-ros/v0/actions/<action_name>` endpoints, which it will convert to ROS action requests. The IFTTT Webhooks service can be used to make a request in response to a trigger.

## Upcoming Improvements
- Send fields of ROS topic messages to populate IFTTT trigger fields.
- Provide a documented working example application.

## Cloning
When cloning this repo directly into a catkin workspace, set the name of the resulting directory to ifttt as follows:

`git clone https://github.com/JamesGiller/ifttt-ros.git ifttt`

This is because the package is named "ifttt" in the configuration files (CMakeLists.txt, package.xml).

## Dependencies
This project has been developed and tested on Ubuntu 16.04

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
