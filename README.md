# UVCRobot


[![pipeline status](https://gitlab.com/jeferson.lima/uvc_robot/badges/master/pipeline.svg)](https://gitlab.com/jeferson.lima/uvc_robot/pipelines)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)  

## Overview

The purpose of project is to develop UVC-light Disinfection Robot in a Hospital Environment. This Project was inspired by the [UVD ROBOTS](http://www.uvd-robots.com/).


# Installation

First, install the latest version of JetPack on your Jetson.

## jetson-inference
These ROS nodes use the DNN objects from the jetson-inference project (aka Hello AI World). To build and install jetson-inference, see this page or run the commands below:

    $ mkdir ~/Libraries
    $ cd ~/Libraries
    $ git clone --recursive https://github.com/dusty-nv/jetson-inference
    $ cd ~/Libraries/jetson-inference
    $ mkdir build && cd build
    $ cmake ../
    $ make -j$(nproc)
    $ sudo make install
    $ sudo ldconfig

## ROS

Install the `ros-melodic-ros-base`:

* [ROS Install Instructions](http://wiki.ros.org/melodic/Installation/Ubuntu)


Next install:

    $$ sudo apt-get install ros-melodic-image-transport ros-melodic-vision-msgs ros-melodic-rosserial-python


navigate to rules.d directory

    $ cd /etc/udev/rules.d
    $ sudo touch my-newrule.rules
    $ sudo vim my-newrule.rules

add the following `KERNEL=="ttyACM0", MODE="0666"`

## ros_deep_learning
Next, navigate into your catkin_ws/src directory and clone ros_deep_learning:

    $ cd ~/catkin_ws/src
    $ git clone https://github.com/dusty-nv/ros_deep_learning

# Usage
This code is currently tested and working with ROS Melodic and [Ubuntu 18.04 LTS](https://downloads.ubiquityrobotics.com/pi.html)  on the Jetson Nano.

WIP 

    $ roslaunch uvcrobot uvcrobot.launch


# Simulation
WIP

# Bugs & Feature Requests
Please report bugs and request features using the [issues](https://gitlab.com/jeferson.lima/uvc_robot/-/issues)