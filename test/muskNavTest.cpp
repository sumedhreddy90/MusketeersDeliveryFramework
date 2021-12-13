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
 * @file muskNavTest.cpp
 * @brief This files tests the MuskyNavGoal Class
 * 
 */
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "MuskyNavGoal.hpp"


/**
 * @brief      This tests the object instantiation for the MuskyNavGoal Class
 * @param          muskNavTest gtest framework
 * @param          muskNavTest Name of the test
 * @return     none
 */
TEST(muskNavTest, muskNavTest) {
  ros::NodeHandle nh;
  int musky_id=1;
  // Object of type Navigation created
  MuskyNavGoal musky(nh,musky_id);
  EXPECT_NO_FATAL_FAILURE(MuskyNavGoal musky(nh,musky_id));
}

/**
 * @brief      Tests the laserCallback method of the class ObstacleDetector
 * @param      VelocityPubTest     gtest framework
 * @param      CommandVelTest   Name of the test
 * @return     none
 */
TEST(VelocityPubTest , CommandVelTest) {
  // Object of type MuskyNavGoal is being instantiated
  // Create ros node
  ros::NodeHandle nh;
  // Create a ros publisher for sending velocity commands Note:testing for musky robot if hsk01
  ros::Publisher pub =
  nh.advertise<geometry_msgs::Twist>("/hsk01/cmd_vel", 1000);
  ros::WallDuration(25).sleep();
  ros::spinOnce();

  // Expect the collision flag to be false
  EXPECT_EQ(pub.getNumSubscribers(), 0);
}

/**
 * @brief      Tests the concatMuskyId of the class MuskyNavGoal
 * @param      ConcatMuskTest      gtest framework
 * @param      concatTest    Name of the test
 * @return     none
 */
TEST(ConcatMuskTest, concatTest) {
  ros::NodeHandle nh;
  int musky_id=1;
  // Object of type Navigation created
  MuskyNavGoal musky(nh,musky_id);
  // Object of type MuskyNavGoal is being instantiated
  EXPECT_EQ(musky.concatMuskyId(2), "/hsk02/move_base");
}


/**
 * @brief Test to check if roscore is running
 */
TEST(TESTSuite, master_test) {
  EXPECT_TRUE(ros::master::check());
}
