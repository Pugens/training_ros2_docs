<!-- omit from toc -->
# SAM XL training on ROS 2
***ROS 2 TRAINING FOR DESIGNERS, MECHANICAL ENGINEERS AND HOBBYIST***

**Table of content**
- [Introduction](#introduction)
  - [Motivation of this training](#motivation-of-this-training)
- [Installation](#installation)
- [Chapters](#chapters)
  - [1 - Visualise a simple robot](#1---visualise-a-simple-robot)
  - [2 - Creating a simple python package](#2---creating-a-simple-python-package)
  - [3 - Add an end effector to your robot](#3---add-an-end-effector-to-your-robot)
  - [4 - Check a robot workspace using Reach (WIP)](#4---check-a-robot-workspace-using-reach-wip)
  - [5 - Use Moveit2 to plan your motions](#5---use-moveit2-to-plan-your-motions)
  - [6 - Simulate a robot in Gazebo (WIP)](#6---simulate-a-robot-in-gazebo-wip)
- [Acknowledgment](#acknowledgment)

## Introduction
Here you will learn basics about **ROS 2**, the "Robot Operating System" (version 2), and why ROS became such a fundamental part of the robotics development, research and even some industrial automation ecosystems like mobile robots in logistics.

Basic concepts of ROS and how to use it will be explained, with a focus on demonstrating its **effectiveness when designing, testing and controlling a new, one of a kind custom robot**, like the **[Gantry Robot @SAMXL!](https://samxl.tudelftcampus.nl)**

### Motivation of this training
ROS has some nice built-in capabilities when it is about visualisation and simulation of robotic platforms. Nowadays, this is something readily available  in most CAD software (see RoboDK and the likes). So why learn ROS?

The motivation lays in the fact that ROS packages let you build and compile your own custom software, continuing from the code used to design and test your custom robot, without the need to re-invent the wheel and create all the tooling at disposal within it, and also being capable of reusing again and again the code your write once both for the simulation and the control in real-time of your robot.
**For further reading about ROS back story, origins and motivations, go to the document [here >](docs/ROS%20back%20story.md)**

Most ROS tutorials start with the very basics, only showing how to use ROS, with limited details on the actual tooling at disposal (urdf robot descriptions, built-in libraries, linking 3rd party installed packages, etc..) and trying to skip to the most interesting parts from the software engineer POV.

Some tutorials come with a fully fledged simulation setup for you with a few comand lines, automatically installing and importing all the necessary libraries and packages, and skiping any explanation and details of ROS inner workings.

This leaves software beginners with more questions to be answered on their own, going online and looking for information in poorly mantained and lacking documentation webpages, or putting together the different pieces from several different examples.

**This training aims to present some slightly more advanced ROS topics from the start, but working them out backwards, to explain in details all the building blocks and featrues necessary when building your very own robot design!**

*Even if you have already a robot and planning on re-using the given setups provided by the OEMs (e.g. URs, KUKA and ABB robots), this training might still help you understand what you are working with!*

For missing explanations more specific to ROS 2 configuration, networking and ecosystem, it is of course suggested to visit the official documentation at **https://docs.ros.org**

## Installation 
To get a working development playground to complete this training successfully on your computer using Docker, follow the instructions **[here >](docs/Installation.md)**

## Chapters
### 1 - Visualise a simple robot
Showing how ROS 2 can help you visualise and jog a simple 3 axis gantry robot for quick design checks.\
**[Go to chapter >](docs/1%20-%20visualise%20robot.md)**

### 2 - Creating a simple python package
Basic explanation of ROS 2 utilities to create integrated python packages and demo on how to use them on your robot.\
**[Go to chapter >](docs/2%20-%20create%20package.md)**

### 3 - Add an end effector to your robot
How make your robot description more flexible and complex with modular and re-usable macros.\
**[Go to chapter >](docs/3%20-%20robot%20end%20effector.md)**

### 4 - Check a robot workspace using Reach (WIP)
How Reach can help you test the robot workspace and the reachability of your targets in space.\
~~**[Go to chapter >]()**~~

### 5 - Use Moveit2 to plan your motions
How Moveit2 can help you create and test the movements of your robot.\
**[Go to chapter >](docs/5%20-%20moveit2%20setup.md)**

### 6 - Simulate a robot in Gazebo (WIP)
Using ROS 2 control and Gazebo to simulate the robot with accurate physics.\
~~**[Go to chapter >]()**~~

## Acknowledgment
This project has received funding from the European Union’s Horizon 2020 research and innovation programme under grant agreement No 958303

Official Penelope EU project website: https://penelope-project.eu

<p float="left">
  <img src="docs/media/samxl-logo-fc.svg" width="200" />
  <img src="docs/media/penelope.png" width="170" /> 
</p>