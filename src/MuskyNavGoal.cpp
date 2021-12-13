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
 * @file MuskyNavGoal.cpp
 * @brief This files defines the MuskNavGoal Class method.
 * 
 * @section DESCRIPTION
 * 
 * Source file for Naviagting muksy robots to different goal positions in the predefined map.
 * First, the user selects the musky_robot using musky_id and chooses the goal location from 
 * the predefined set of goal poses.The musky will then be navigated using the move_base commands
 * provided using this class.
 */

#include "MuskyNavGoal.hpp"


MuskyNavGoal::MuskyNavGoal(ros::NodeHandle node , int musky_id) {}

MuskyNavGoal::~MuskyNavGoal() {}

std::string MuskyNavGoal::concatMuskyId(int musky_id) {}

void MuskyNavGoal::muskySendGoal(int musky_id) {}