# Table of contents

* [Introduction](#Introduction)
* [Components](#Components)
* [Technologies](#Technologies)
* [Before_you_started](#Before_you_started)
* [Hardware](#Hardware)
* [Software](#Software)

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

* Pololu GB37-100 - Motors which are resposible for moving the AGV. Eash motors has got an encoders for measuring the current speed of the platform.

* DFROBOT DRI0041 - dual motor driver for controlling the motors

* Grove - IMU 10DOF v2.0 - 3 dimension inertial measurement unit

* Pololu VL53L1X - they are also a lidar but with two diferrences. Firstable TFMini has range of 12 metres and resolution equaled 1 cm. Pololu has range of 4 m and resolution equaeled 1 mm. As I wrote before pololu are connected by I2C, but we have only three I2C on breakout. That is why we have to connect four sensors to one I2C.
To change an address of each sensors by using the software we have to use the XSHUT on pololu. XSHUT pins has to be connected to GPIO on breakout. Very important thing about it is that they have a nominal voltaged equaled 2.8 Voltage. For proper working the XSHUT pins must be connected as an input.

* Arduino Portenta - microcontroller which will operate the software. We decided to choose Portenta rather than the previous option which was Arduino Mega and STM32.

* Nvidia Xavier - micro computer for collecting data for our sensors. Of course in the future we will also receive data from cameras and motors. With Nvidia we can easily connect to it by out laptop. To do that we have to be in the network and we are connecting by using an SSH, so we have to know the password.

