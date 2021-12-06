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
 * @file DockingStation.h
 * @brief This files defines the MuskManager Class
 */

#indef MUSKY_NAV_INCLUDE_DOCKINGSTATION_H_
#define MUSKY_NAV_INCLUDE_DOCKINGSTATION_H_

#include <iostream>
#include <ros/ros.h>

class DockingStation {
    private:
    // Variable of type ros::NodeHandle
    ros::NodeHandle nh;

    // Variable of type ros::Subscriber
    ros::Subscriber sub;

    // Variable of type int defining the husky station id
    int station_id;
    float origin;
    int rows,columns;
    double x_offset,y_offset;
    float cell_theta;

    public: 

    void commission();

    get_attributes();

    set_attributes( int station_id, float origin, rows, columns, x offset, y_offset, cell_theta);

};

#endif
