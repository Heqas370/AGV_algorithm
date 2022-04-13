# Table of contents

* [Introduction](#Introduction)
* [Components](#Components)
* [Technologies](#Technologies)
* [Before_you_started](#Before_you_started)
* [Hardware](#Hardware)

# Introduction

Portenta-Sensors is a project creating by students at Silesian University of Technology. During the semester we are creating AGV (Automated guided vehicle). For my part of this project
I am creating a software for an Arduino Portenta H7 Lite connected which will allow to read all the data from the available sensors. Here I will be explaining the process of creating my
code and also I will show how to connect everything to have an easy startup.

# Components
* Arduino Portenta H7 Lite Connected
* Arduino Portenta Breakout
* Pololu VL53L1X
* TFMini Plus
* Nvidia AGX Xavier 

# Technologies

* Arduino IDE(C++) - the software where I am writing the code for Arduino Portenta
* Python - Creating a program for connecting Arduino with Xavier
* Linux(Ubuntu) - operating system for ROS(Robot Operating System)
* ROS - software for reading the data directly from AGV

# Before_you_started

Before you started reading everything in this repository, you need to understand a few things. Firstable you have to know the basics of Linux(cd, ls, etc.) and ROS (rostopics,
rosnodes, etc.). Very important is to understand the portenta breakout library for example how to activate each periphery.

# Hardware

* TFMini Plus - those sensors are connecting by UART bus. Of course their structure allow to connect the to I2C bus, but we will use I2C for Pololu. While we connecting 
TFMini Plus we have to remember about one thing. Portenta breakout has only 3.3 V pins, but the lidar need 5 V. That is why we have to use the logic converter.
