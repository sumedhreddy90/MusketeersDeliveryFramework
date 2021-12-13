# Musketeers: Autonomous pickup and delivery fleet

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

![example workflow](https://github.com/sumedhreddy90/MusketeersDeliveryFramework/actions/workflows/muskyCI.yml/badge.svg)


## Overview
The rapid rise of e-commerce services in the last decade, and its forecast for the future, have proved that the need for delivery services will be increasing, which needs to be addressed by improving the delivery infrastructure. Developments in technology have now made it possible to provide an autonomous solution, which can revolutionize the industry by
creating an autonomous delivery network that can accomplish
this task without the need for human intervention. \
In this project, we propose a fleet of autonomous delivery
robots, called Musky, that can provide transportation services
for various kinds of small to medium-sized goods(<10 lbs)
for short-range hauling(<5 Miles) using Multi Fleet GPS Waypoint Navigation.

### UML
[Quad Chart](https://drive.google.com/file/d/112d7ZOfGakJW6sQtDZE99uixeMIY7aPG/view?usp=sharing) \
[Project Proposal](https://drive.google.com/file/d/1M1FodIeb_yLv4JwdbofX3ztwyTm6bGPM/view?usp=sharing) \
[Activity Diagram](https://drive.google.com/file/d/1Tu6yAAuHe9edw5sUqZ6b6rdyhWe9690b/view?usp=sharing) \
[Class Diagram](https://drive.google.com/file/d/1F0wCk2moPsRFCpEKdf5eaZrsnagD0Mt8/view?usp=sharing)


### AIP Document 
[![Doc Link](https://img.shields.io/badge/Doc-link-blue)](https://docs.google.com/spreadsheets/d/1ULTNvGetic1YF69uuCX7t-CtU-yicrW0z6M5p8Y3zxk/edit?usp=sharing)

### Sprint Planning 
[![Doc Link](https://img.shields.io/badge/Doc-link-blue)](https://docs.google.com/document/d/1PARTqCdQkq0k0sgHWpRNl_M1ng2w7DdYFu71zwEIHzk/edit?usp=sharing)

### Phase 2 Progress
* Spawning Multiple huskies on UMD Map near Mckeldin Library
* Creating a UMD world and Integrating with project 
  - Download UMD campus map from Open Street maps
  - OSM extractor to convert 3D OSM map to OBJ format
  - Convert OBJ to STL file conversion
  - Feed custom map to Gazebo
* Working on GPS Way point navigation for Multiple Huskies
* Creating Stub Classes and Test cases stubs
* Integrating Github Actions for CI and coverage

### Screenshots of our Output
[Gazebo: Spawning Fleet of Huskies in temporary Dock Station]file:///home/starfleeet-robotics/Pictures/Gazebo.png![image](https://user-images.githubusercontent.com/24978535/144908953-9a8ab76a-c516-43da-9176-961469c7d72a.png)

[Rviz: Spawning Fleet of Huskies in temporary Dock Station]file:///home/starfleeet-robotics/Pictures/RViz.png![image](https://user-images.githubusercontent.com/24978535/144908977-e60ed050-e89e-47d9-bfc4-a8884cad6ed5.png)

[UMD Campus Map Mckeldin Library]file:///home/starfleeet-robotics/Pictures/UMD.png![image](https://user-images.githubusercontent.com/24978535/144909022-c8d16ee7-5cba-4e44-b92f-40f51a340c25.png)

## Assumptions
* The streets do not have any
traffic or traffic signals, hence the robot can navigate freely
on the pedestrian paths, except for a few obstacles along the
way.
* The obstacles present on the
pedestrian paths are of dimensions such that they do not block
the complete pathway and will allow the robot to maneuver
around it.
* The paths are even and do
not contain extreme slopes.

## Project tested on

* Ubuntu 20.04 using Noetic 
* Ubuntu 18.04 using Melodic

## Technologies
*Programing language*: C++

*Build system*: cmake

*Testing Library*: Google Test, Google Mock

*Continuous Integration*: Travis CI, Coverall

*Other*: ROS, Gazebo, RViz, Clearpath Husky, Kivy(For GUI)

## Algorithms

- husky_navigation package, which uses gmapping, move_base and frontier_exploration [BSD-3-Clause License].
- GPS Waypoint Navigation

## Risk and Mitigations

- **Risk 1**: Error in Path planning Algorithm due to unknown
constraints which might lead to delay in delivery time or failed delivery.
Mitigation: This can be mitigated by improving the path
planning algorithm by feeding the planning data to supervised
learning ML algorithms and training the path planner to predict
obstacles/possible unknown constraints.
- **Risk 2**: Unexpected shutdown of ROS Master might lead to
the shutdown of all the available agents/Robots.
Mitigation: This risk can be mitigated by migrating the whole
project to ROS2 or by integrating a ROS1 bridge with ROS2.
- **Risk 3**: One of the sensors required for path planning might
malfunction leading to erratic behavior.
Mitigation: If such a condition occurs, the robot will be made
to override the autonomous navigation signals and a human
operator will manually control the robot.
