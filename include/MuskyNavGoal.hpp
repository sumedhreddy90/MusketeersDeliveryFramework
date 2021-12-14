/*
 *
 * @author Sumedh Koppula, Pratik Acharya, Rahul Karanam
 * @date 06th December 2021
 * @copyright Copyright [2021] [Sumedh Koppula, Pratik Acharya, Rahul Karanam]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.'
 *
 *
 * @file MuskyNavGoal.hpp
 * @brief This files defines the MuskNavGoal Class
 *
 * @section DESCRIPTION
 *
 * Header file for Naviagting muksy robots to different goal positions in the
 * predefined map.
 */

#ifndef INCLUDE_MUSK_NAV_MUSKYNAVGOAL_HPP_
#define INCLUDE_MUSK_NAV_MUSKYNAVGOAL_HPP_

// ROS Header Files
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

#include <../CLI11/include/CLI/CLI.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "geometry_msgs/Twist.h"

// created a type definition for a client called MoveBaseClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

/**
 *  @brief Class MuskNavGoal
 *
 *  The following class MuskNavGoal implements the navigation stack
 *  for the musky robots to navigate to different base locations or
 *  goal positions.
 */
class MuskyNavGoal {
 private:
  ros::NodeHandle n;  // Node handler object for ROS
  tf2::Quaternion
      my_quat_from_euler;  /// ROS tf2 transform for converting the RPY to
                           /// Quaternion using tf2/Quaternion

  bool client_status = true;  /// flag to check whether action client has been
                              /// established
  bool status = true;  /// flag to navigate the robot to different locations
  int location;        /// goal selection variable
  std::string bot;     // string variable for movebaseclient

 public:
  int musky_id;  // The Robot_ID to which we will send commands.
  /**
   *   @brief  function to concatenate the musky_id and the our base_id.
   *           This can be used to launch multiple robots.
   *
   *   @param  musky_id is a integer variable
   *
   *   @return std::string bot
   */
  std::string concatMuskyId(int musky_id);
  /**
   *   @brief  Default constructor for MuskNavGoal.This will setup our
   *           velocity node by subscribing to it.
   *
   *   @param  n - ROS nodehandle object
   *
   *   @return void
   */
  explicit MuskyNavGoal(ros::NodeHandle n, int musky_id);

  /**
   *   @brief  Destructor for MuskNavGoal
   *
   *   @param  none
   *
   *   @return void
   */
  ~MuskyNavGoal();
  /**
   *   @brief  function to perform all the navigation tasks assigned
   *
   *   @param  none
   *
   *   @return void
   */
  void muskySendGoal(int musky_id);
};

#endif  //  INCLUDE_MUSKY_NAV_MUSKNAVGOAL_HPP_
