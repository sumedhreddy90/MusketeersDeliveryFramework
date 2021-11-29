# Musketeers: Autonomous pickup and delivery fleet

[![GitHub license](https://badgen.net/github/license/Naereen/Strapdown.js)](LICENSE.md)

## Overview
The rapid rise of e-commerce services in the last decade, and its forecast for the future, have proved that the need for delivery services will be increasing, which needs to be addressed by improving the delivery infrastructure. Developments in technology have now made it possible to provide an autonomous solution, which can revolutionize the industry by
creating an autonomous delivery network that can accomplish
this task without the need for human intervention. \
In this project, we propose a fleet of autonomous delivery
robots, called Musky, that can provide transportation services
for various kinds of small to medium-sized goods(<10 lbs)
for short-range hauling(<5 Miles).

### UML
[Quad Chart](https://drive.google.com/file/d/112d7ZOfGakJW6sQtDZE99uixeMIY7aPG/view?usp=sharing) \
[Project Proposal](https://drive.google.com/file/d/1M1FodIeb_yLv4JwdbofX3ztwyTm6bGPM/view?usp=sharing) \
[Activity Diagram](https://drive.google.com/file/d/1Tu6yAAuHe9edw5sUqZ6b6rdyhWe9690b/view?usp=sharing) \
[Class Diagram](https://drive.google.com/file/d/1F0wCk2moPsRFCpEKdf5eaZrsnagD0Mt8/view?usp=sharing)


### AIP Document 
[![Documentation Status](https://readthedocs.org/projects/ansicolortags/badge/?version=latest)](https://docs.google.com/spreadsheets/d/1ULTNvGetic1YF69uuCX7t-CtU-yicrW0z6M5p8Y3zxk/edit?usp=sharing)

## Assumptions
* It is assumed that the streets do not have any
traffic or traffic signals, hence the robot can navigate freely
on the pedestrian paths, except for a few obstacles along the
way.
* It is assumed that the obstacles present on the
pedestrian paths are of dimensions such that they do not block
the complete pathway and will allow the robot to maneuver
around it.
* It is assumed that the paths are even and do
not contain extreme slopes.

## Technologies
*Programing language*: C++

*Build system*: cmake

*Testing Library*: Google Test, Google Mock

*Continuous Integration*: Travis CI, Coverall

*Other*: ROS, Gazebo, RViz, Clearpath Husky

## Algorithms

- husky_navigation package, which uses gmapping, move_base and frontier_exploration [BSD-3-Clause License].

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
