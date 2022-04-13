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

* TFMini Plus - those sensors are connecting via UART bus. Of course their structure allow to connect the to I2C bus, but we will use I2C for Pololu. While we connecting 
TFMini Plus we have to remember about one thing. Portenta breakout has only 3.3 V pins, but the lidar needs 5 V. That is why we have to use the logic converter.

* Pololu VL53L1X - they are also a lidar but with two diferrences. Firstable TFMini has range of 12 metres and resolution equaled 1 cm. Pololu has range of 4 m and resolution equaeled 1 mm. As I wrote before pololu are connected by I2C, but we have only three I2C on breakout. That is why we have to connect four sensors to one I2C.
To change an address of each sensors by using the software we have to use the XSHUT on pololu. XSHUT pins has to be connected to GPIO on breakout. Very important thing about it is that they have a nominal voltaged equaled 2.8 Voltage. For proper working the XSHUT pins must be connected as an input.

* Arduino Portenta - 
