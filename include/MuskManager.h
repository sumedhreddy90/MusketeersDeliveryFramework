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
 * @file MuskManager.h
 * @brief This files defines the MuskManager Class
 */

#indef MUSKY_NAV_INCLUDE_MUSKMANAGER_H_
#define MUSKY_NAV_INCLUDE_MUSKMANAGER_H_

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>

class MuskManager {
    private:
    // Variable of type ros::NodeHandle
    ros::NodeHandle nh;

    // Variable of type ros::Subscriber
    ros::Subscriber sub;

    // Variable of type int defining the husky robot id
    int musky_id;

    public:

        /**
        * @brief Constructs the MuskManager Class
        * 
        */
        explicit MuskManager();

        /**
        * @brief Destructor of the MuskManager Class
        * 
        */
        ~MuskManager();
        double location;

        double bs1,bs2,bs3;

        double x_start,y_start,x_goal,y_goal,order_id;
        
        void get_all_musky_list()

        double get_closest_musky(double location)

        bool get_musky_status( int muskyid)

        double get_distance(std::vector<double> muskypose, double location)

        cal_path_plan(double x_start,double y_start,double x_goal,double y_goal, int muskyid)

        void assign_job_musky(int muskyid)

        assign_musky(int muskyid,double location)

        void get_order_status(int muskyid,int order_id)

        void get_goal_pose(double location)

        move_back_to_base(int muskyid, int base_id)

};

#endif //MUSKY_NAV_INCLUDE_MUSKMANAGER_H_