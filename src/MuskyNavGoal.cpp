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
 * Source file for Naviagting muksy robots to different goal positions in the
 * predefined map. First, the user selects the musky_robot using musky_id and
 * chooses the goal location from the predefined set of goal poses.The musky
 * will then be navigated using the move_base commands provided using this
 * class.
 */
#include "MuskyNavGoal.hpp"
/**
 * @fn MuskyNavGoal::MuskyNavGoal(ros::NodeHandle node , int musky_id)
 * @brief This is our constructor for MuskyNavGoal Class. It initializes the
 *        velocity of the the husky by publishing to the /cmd_vel topic
 *        of the musky robot.
 *
 * @param ros::NodeHandle node ros node handler object
 *        int musky_id    is the Robot id musky_id
 * @return None
 */
MuskyNavGoal::MuskyNavGoal(ros::NodeHandle node, int musky_id) {
  // Publishing the twist messages to the robot given by the musky_id
  ros::Publisher velocityPublisher = node.advertise<geometry_msgs::Twist>(
      "/hsk0" + std::to_string(musky_id) + "/cmd_vel", 1000);
  // Declare and initialize twist
  geometry_msgs::Twist twist;
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;
  // publish the initialised twist
  velocityPublisher.publish(twist);
}
/**
 * @fn MuskyNavGoal::~MuskyNavGoal()
 * @brief Destructor for the MuskyNavGoal
 *
 * @param none
 * @return none
 */
MuskyNavGoal::~MuskyNavGoal() {}
/**
 * @fn std::string MuskyNavGoal::concatMuskyId(int musky_id)
 * @brief This Function will concatenate the musky id with move_base
 *        message which will be later used in MoveBaseClient.
 * @param int musky_id    is the Robot id musky_id
 * @return bot <std::double> is the concatenated string
 */
std::string MuskyNavGoal::concatMuskyId(int musky_id) {
  // Concatenating inorder to call multiple musky robots.
  std::string bot = "/hsk0" + std::to_string(musky_id) + "/move_base";
  return bot;
}
/**
 * @fn void MuskyNavGoal::muskySendGoal(int musky_id)
 * @brief This Function will navigate the robot to the user
 *        selection.
 *
 * @param int musky_id    is the Robot id musky_id
 * @return None
 */
void MuskyNavGoal::muskySendGoal(int musky_id) {
  //
  MoveBaseClient ac(concatMuskyId(musky_id), true);

  // Wait for the action server to come up so that we can begin processing
  // goals.
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  // cmdline options to interact with the commands
  char choice_to_continue = 'Y';

  while (status) {
    // Get the input from the user for where the Musky to go?
    std::cout << "\nWhere do you want the robot to go?" << std::endl;
    std::cout << "\n1 = Musketeers_Base-Station : 1 " << std::endl;
    std::cout << "2 = Musketeers_Base-Station : 2" << std::endl;
    std::cout << "3 = Musketeers_Base-Station : 3" << std::endl;
    std::cout << "4 = Musketeers_Base-Station : 4" << std::endl;
    std::cout << "5 = Chipotle" << std::endl;
    std::cout << "6 = Sub-way" << std::endl;
    std::cout << "7 = Taco-bell" << std::endl;
    std::cout << "8 = Panda Express" << std::endl;
    std::cout << "9 = Iribe" << std::endl;
    std::cout << "10 = Mckeldin Library" << std::endl;
    std::cout << "11 = J.M Patterson Hall" << std::endl;
    std::cout << "12 = Domain" << std::endl;
    std::cout << "\nEnter a number to choose your location: ";
    std::cin >> location;

    // Create a new goal to send to move_base
    move_base_msgs::MoveBaseGoal goal;

    // Send a goal to the robot
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    bool valid_selection = true;

    // Now the user will select the location based upon the input.
    // We have predefined locations for the goal locations
    switch (location) {
      case 1:
        std::cout << "\n Base Location: Musketeers_Base-Station : 1\n"
                  << std::endl;
        goal.target_pose.pose.position.x = -17.5;
        goal.target_pose.pose.position.y = 21.5;
        break;
      case 2:
        std::cout << "\n Base Location: Musketeers_Base-Station : 2\n"
                  << std::endl;
        goal.target_pose.pose.position.x = 5.5;
        goal.target_pose.pose.position.y = 21.5;
        break;
      case 3:
        std::cout << "\n Base Location: Musketeers_Base-Station : 3\n"
                  << std::endl;
        goal.target_pose.pose.position.x = -17.5;
        goal.target_pose.pose.position.y = -3.5;
        break;
      case 4:
        std::cout << "\n Base Location: Musketeers_Base-Station : 4\n"
                  << std::endl;
        goal.target_pose.pose.position.x = 5.5;
        goal.target_pose.pose.position.y = -5.5;
        break;
      case 5:
        std::cout << "\n Restaurant Location: Chipotle\n" << std::endl;
        goal.target_pose.pose.position.x = -11.5;
        goal.target_pose.pose.position.y = 10;
        my_quat_from_euler.setRPY(0.0, 0.0, -3.14);
        // Converting the goal pose from RPY to quaternion using the
        // my_quat_from_euler.
        goal.target_pose.pose.orientation.x = my_quat_from_euler.x();
        goal.target_pose.pose.orientation.y = my_quat_from_euler.y();
        goal.target_pose.pose.orientation.z = my_quat_from_euler.z();
        goal.target_pose.pose.orientation.w = my_quat_from_euler.w();
        break;
      case 6:
        std::cout << "\n Restaurant Location: Sub-way\n" << std::endl;
        goal.target_pose.pose.position.x = 11.5;
        goal.target_pose.pose.position.y = 10;
        my_quat_from_euler.setRPY(0.0, 0.0, -3.14);
        goal.target_pose.pose.orientation.x = my_quat_from_euler.x();
        goal.target_pose.pose.orientation.y = my_quat_from_euler.y();
        goal.target_pose.pose.orientation.z = my_quat_from_euler.z();
        goal.target_pose.pose.orientation.w = my_quat_from_euler.w();
        break;
      case 7:
        std::cout << "\n Restaurant Location: Taco-bell\n" << std::endl;
        goal.target_pose.pose.position.x = -11.5;
        goal.target_pose.pose.position.y = -15;
        my_quat_from_euler.setRPY(0.0, 0.0, -3.14);
        goal.target_pose.pose.orientation.x = my_quat_from_euler.x();
        goal.target_pose.pose.orientation.y = my_quat_from_euler.y();
        goal.target_pose.pose.orientation.z = my_quat_from_euler.z();
        goal.target_pose.pose.orientation.w = my_quat_from_euler.w();
        break;
      case 8:
        std::cout << "\n Restaurant Location: Panda Express\n" << std::endl;
        goal.target_pose.pose.position.x = 11.5;
        goal.target_pose.pose.position.y = -15;
        my_quat_from_euler.setRPY(0.0, 0.0, -3.14);
        goal.target_pose.pose.orientation.x = my_quat_from_euler.x();
        goal.target_pose.pose.orientation.y = my_quat_from_euler.y();
        goal.target_pose.pose.orientation.z = my_quat_from_euler.z();
        goal.target_pose.pose.orientation.w = my_quat_from_euler.w();
        break;
      case 9:
        std::cout << "\n Goal Location: Iribe\n" << std::endl;
        goal.target_pose.pose.position.x = -11.5;
        goal.target_pose.pose.position.y = 2.2;
        my_quat_from_euler.setRPY(0.0, 0.0, -3.14);
        goal.target_pose.pose.orientation.x = my_quat_from_euler.x();
        goal.target_pose.pose.orientation.y = my_quat_from_euler.y();
        goal.target_pose.pose.orientation.z = my_quat_from_euler.z();
        goal.target_pose.pose.orientation.w = my_quat_from_euler.w();
        break;
      case 10:
        std::cout << "\nGoal Location: Mckeldin Library\n" << std::endl;
        goal.target_pose.pose.position.x = 5;
        goal.target_pose.pose.position.y = 11.3;
        my_quat_from_euler.setRPY(0.0, 0.0, -3.14);
        goal.target_pose.pose.orientation.x = my_quat_from_euler.x();
        goal.target_pose.pose.orientation.y = my_quat_from_euler.y();
        goal.target_pose.pose.orientation.z = my_quat_from_euler.z();
        goal.target_pose.pose.orientation.w = my_quat_from_euler.w();
        break;
      case 11:
        std::cout << "\nGoal Location: J.M Patterson Hall\n" << std::endl;
        goal.target_pose.pose.position.x = -19.2;
        goal.target_pose.pose.position.y = -21;
        my_quat_from_euler.setRPY(0.0, 0.0, -3.14);
        goal.target_pose.pose.orientation.x = my_quat_from_euler.x();
        goal.target_pose.pose.orientation.y = my_quat_from_euler.y();
        goal.target_pose.pose.orientation.z = my_quat_from_euler.z();
        goal.target_pose.pose.orientation.w = my_quat_from_euler.w();
        break;
      case 12:
        std::cout << "\nGoal Location: Domain\n" << std::endl;
        goal.target_pose.pose.position.x = 16;
        goal.target_pose.pose.position.y = -21;
        my_quat_from_euler.setRPY(0.0, 0.0, -3.14);
        goal.target_pose.pose.orientation.x = my_quat_from_euler.x();
        goal.target_pose.pose.orientation.y = my_quat_from_euler.y();
        goal.target_pose.pose.orientation.z = my_quat_from_euler.z();
        goal.target_pose.pose.orientation.w = my_quat_from_euler.w();
        break;
      default:
        std::cout << "\nInvalid selection/location to send your musky Please "
                     "try again.\n"
                  << std::endl;
        valid_selection = false;
    }

    // Go back to beginning if the selection is invalid.
    if (!valid_selection) {
      continue;
    }

    ROS_INFO("Musky is heading to you with your order");
    ac.sendGoal(goal);

    // Wait until the robot reaches the goal
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("  Musky has arrived the restaurant. ");
    else
      ROS_INFO(" Sorry !! Musky couldn't deliver your order at this location");

    // Ask the user if he wants to continue giving goals
    do {
      std::cout << "\n Do you want to deliver (Y) or pick-up (N) ? (Y/N)"
                << std::endl;
      std::cin >> choice_to_continue;
      choice_to_continue =
          tolower(choice_to_continue);  // Changing to lower case
    } while (choice_to_continue != 'n' && choice_to_continue != 'y');
    // If the order is pickup then we terminate this process
    if (choice_to_continue == 'n') {
      status = false;
    }
  }
}
