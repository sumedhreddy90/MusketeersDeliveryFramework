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
 * @file main.cpp
 * @brief This files defines the all the classes.
 *
 */
#include <ros/ros.h>

#include "MuskyNavGoal.hpp"

int main(int argc, char** argv) {
  // Using command line parser to take the robot id
  CLI::App fleet{"Welcome to Musketeers Fleet Hub"};
  // Connect to ROS
  ros::init(argc, argv, "musk_init");
  // ros node handling object nh
  ros::NodeHandle nh;
  int musky_id;
  // float x,y,roll,pitch,yaw;
  // Adding an option to the fleet parser i.e muskyid to choose from 20 agents.
  fleet.add_option("musk_id", musky_id, "Musk_ID (1-20)");
  // parsing the data from the cmd line
  CLI11_PARSE(fleet, argc, argv);
  // Instantiating our MuskyNavGoal Class by creating a musky instance
  MuskyNavGoal musky(nh, musky_id);

  ros::Rate loop_rate(10);

  ros::spinOnce();
  // After taking the commands from the user we then send our musky
  // robot to the desired location based upon user selection.
  while (ros::ok()) {
    ros::spinOnce();
    // we then call our muskySendGoal to send musky robot with
    // muskyid to desired goal position.
    musky.muskySendGoal(musky_id);
    loop_rate.sleep();
  }
  return 0;
}
