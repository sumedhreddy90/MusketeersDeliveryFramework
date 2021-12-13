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

### Repository

This repository contains a multi-robot autonomous delivery framework, that works with husky robots from ClearPath robotics, in a simulation environment. The robots called "Musky" can be controlled via command line inputs, where they can be made to traverse to a preset location to pick up an order from a restaurant and will deliver it to a preset delivery location.

### UML
[Quad Chart](https://drive.google.com/file/d/112d7ZOfGakJW6sQtDZE99uixeMIY7aPG/view?usp=sharing) \
[Project Proposal](https://drive.google.com/file/d/1M1FodIeb_yLv4JwdbofX3ztwyTm6bGPM/view?usp=sharing) \
[Activity Diagram](https://drive.google.com/file/d/1Tu6yAAuHe9edw5sUqZ6b6rdyhWe9690b/view?usp=sharing) \
[Class Diagram](https://drive.google.com/file/d/1F0wCk2moPsRFCpEKdf5eaZrsnagD0Mt8/view?usp=sharing)


### AIP Document 
[![Doc Link](https://img.shields.io/badge/Doc-link-blue)](https://docs.google.com/spreadsheets/d/1ULTNvGetic1YF69uuCX7t-CtU-yicrW0z6M5p8Y3zxk/edit?usp=sharing)

### Sprint Planning 
[![Doc Link](https://img.shields.io/badge/Doc-link-blue)](https://docs.google.com/document/d/1PARTqCdQkq0k0sgHWpRNl_M1ng2w7DdYFu71zwEIHzk/edit?usp=sharing)

### Phase 3 Progress
* Generating a new world file, as localization is problematic in UMD world
* Generated map for the above world using Gmapping and used the same for localization 
* Created new UML diagram to reflect the change in code structure
* Created Stub Classes and Test cases stubs for the new UML
* Created header and source files according to the stubs which passed the tests
* Created a command line interface for controlling the fleet
* Created a Graphical User Interface to communicate and control the robots
* Created and released the package for the fleet system

### Presentaion Links 
[Presentation PPT](https://drive.google.com/file/d/1DYCjtVKh_N355wW-lCqWUxPlOrV-pOAf/view?usp=sharing) \
[Presentation Demo](https://youtu.be/GT90a_htJZU) 
### Demo Video links
[Demo Musky Single Robot](https://youtu.be/gXPuU7SFJIQ) \
[Demo Multiple Musky Robot](https://youtu.be/AbQy4rUTcHE) \
[Demo Multiple Musky Robot with obstacle avoidance](https://youtu.be/RAuY9NLsP7U) \
+ 

### Old World Files
Gazebo: Spawning Fleet of Huskies in temporary Dock Station ![image](https://user-images.githubusercontent.com/24978535/144908953-9a8ab76a-c516-43da-9176-961469c7d72a.png)

Rviz: Spawning Fleet of Huskies in temporary Dock Station ![image](https://user-images.githubusercontent.com/24978535/144908977-e60ed050-e89e-47d9-bfc4-a8884cad6ed5.png)

UMD Campus Map Mckeldin Library ![image](https://user-images.githubusercontent.com/24978535/144909022-c8d16ee7-5cba-4e44-b92f-40f51a340c25.png)

### New World File
![WhatsApp Image 2021-12-13 at 1 38 50 PM](https://user-images.githubusercontent.com/24701030/145869779-927f3504-0829-4aa2-a879-3d29cdf2373f.jpeg)

### Generated MAP for Localization
![WhatsApp Image 2021-12-13 at 1 38 50 PM (1)](https://user-images.githubusercontent.com/24701030/145869826-e1cd3ff5-e092-478b-8c75-7b2f60e4a9a5.jpeg)
![WhatsApp Image 2021-12-13 at 1 38 50 PM (2)](https://user-images.githubusercontent.com/24701030/145869830-365af42b-4755-4056-8bdf-7190cc029418.jpeg)


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

## Technologies
*Programing language*: C++

*Build system*: cmake

*Testing Library*: Google Test

*Continuous Integration*: GitHub Actions CI, Coverall

*Other*: ROS, Gazebo, RViz, Clearpath Husky



## Algorithms

- husky_navigation package, which uses gmapping, move_base and frontier_exploration [BSD-3-Clause License].
- CLI11 package for parsing command line input [BSD-3-Clause License].

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

### Setbacks
* Due to narrow pathways, the robots are unable to find paths as the cost map generated have high padding. This can be mitigated by using smaller robots or by generating different paths. 

## Running the code

To run the code, first open a terminal and download the repository

```
git clone https://github.com/sumedhreddy90/MusketeersDeliveryFramework.git
```

Then, add the downloaded repository into your ROS workspace's src folder (eg. catkin_ws/src)

After adding the repository, change the directory of the repository to the workspace repository

```
cd ~/catkin_ws
```
To build the changes use
```
catkin_make
```
To source the build files use
```
source devel/setup.bash
```
Launch files are provided that will spawn multiple robots in a gazebo environment and will simultaneously open rviz. To use the launch file, run the following command in the same terminal
```
roslaunch musky_nav multi_husky_delivery.launch 
```

Wait for all the windows to open fully, which can be verified by checking the `ROS_INFO` message in the terminal where the above launch file was launched. It should display `Recovery behavior will clear layer 'obstacles'`

After getting the above message, open a new terminal and navigate to the ROS workspace, using 
```
cd ~/catkin_ws
```

and source the build files
```
source devel/setup.bash
```

To run the ROS node that controls the muskies, run the following code
```
rosrun musky_nav musk_init __name:=node1 1
```
Where `__name:=node1` is used to create the instance of the node with a specified name, and `1` specifies the robot ID. The general command line code would be
```
rosrun musky_nav musk_init __name:=<node name> <Robot ID>
```

After the above command, the terminal will display all the options available for navigation
```
Where do you want the robot to go?

1 = Musketeers_Base-Station : 1 
2 = Musketeers_Base-Station : 2
3 = Musketeers_Base-Station : 3
4 = Musketeers_Base-Station : 4
5 = Chipotle
6 = Sub-way
7 = Taco-bell
8 = Panda Express
9 = Iribe
10 = Mckeldin Library
11 = J.M Patterson Hall
12 = Domain
```
Select one of the above mentioned options and the robot will plan the trajectory and will move accordingly

The bases are configured to cater to only the closes restaurants, hence, selection of the pickup location can be done as follows

![](https://user-images.githubusercontent.com/24701030/145853502-14e9387f-e29f-46b6-a16c-5e1e00173fd2.png)

The robots can also be controlled via a GUI as follows

```
rosrun musky_nav musketeers_gui.py
```

#### GUI 
![WhatsApp Image 2021-12-13 at 11 00 18 AM](https://user-images.githubusercontent.com/24701030/145869304-9392391b-51a4-4114-93a0-f74b49ca2876.jpeg)

