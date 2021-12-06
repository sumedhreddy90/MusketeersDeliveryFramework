/*
 * @file Musky.h
 * @author Sumedh Koppula, Pratik Acharya, Rahul Karanam
 * @date 06th December 2021
 * @copyright Copyright [2021] [Sumedh Koppula, Pratik Acharya, Rahul Karanam]
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.'
 * @brief file to create musky nodes
 */

#include <string>

class Musky {
 private:
  int docking_station;
  int id;
  int type;
  std::string scan_topic;
  std::string name;
  std::string namespace;
  std::string tf_prefix;
  std::string base_frame;
  std::string odom_frame;
  int station_id;
  bool assigned;
  int cell_id;
  int cell_origin;

 public:
  Musky(int robot_id);

  bool assign_cell(int docking_station);

  void launch(roslaunch::parent.ROSLaunchParent nav_package,
              roslaunch::parent.ORSLaunchParent uuid);
};
