/*
 * 
 * @author Sumedh Koppula, Pratik Acharya, Rahul Karanam
 * @date 12th December 2021
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
 * @file main.cpp
 * @brief This files will run all the test cases i.e unit tests and rostests.
 * 
 * 
 * 
 *  
 */
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "MuskyNavGoal.hpp"

std::shared_ptr<ros::NodeHandle> node1;

int main(int argc, char** argv) {
  ros::init(argc, argv, "musketeersTests");
  node1.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}