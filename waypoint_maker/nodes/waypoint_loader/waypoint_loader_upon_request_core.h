/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef WAYPOINT_LOADER_UPON_REQUEST_CORE_H
#define WAYPOINT_LOADER_UPON_REQUEST_CORE_H

// ROS includes
#include <ros/ros.h>

// C++ includes
#include <iostream>
#include <fstream>
#include <vector>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <unordered_map>

#include "autoware_msgs/LaneArray.h"
#include <autoware_msgs/String.h>

namespace waypoint_maker
{
const std::string MULTI_LANE_CSV = "/tmp/driving_lane.csv";

enum class FileFormat : int32_t
{
  ver1,  // x,y,z,(velocity)
  ver2,  // x,y,z,yaw,(velocity)
  ver3,  // first line consists on explanation of values

  unknown = -1,
};

typedef std::underlying_type<FileFormat>::type FileFormatInteger;

inline double kmph2mps(double velocity_kmph)
{
  return (velocity_kmph * 1000) / (60 * 60);
}
inline double mps2kmph(double velocity_mps)
{
  return (velocity_mps * 60 * 60) / 1000;
}

class WaypointLoaderUponRequestNode
{
public:
  WaypointLoaderUponRequestNode();
  ~WaypointLoaderUponRequestNode() = default;
  void run();

private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher & subscriber
  ros::Publisher lane_pub_;
  ros::Subscriber config_sub_;

  // Service server.
  ros::ServiceServer service_;

  // Service func
  bool serveLanePublishRequest(autoware_msgs::String::Request &req, autoware_msgs::String::Request &res);

  // functions
  void createLaneWaypoint(const std::string& file_path, autoware_msgs::Lane* lane);
  void createLaneArray(const std::vector<std::string>& paths, autoware_msgs::LaneArray* lane_array);

  FileFormat checkFileFormat(const char* filename);
  bool verifyFileConsistency(const char* filename);
  void loadWaypointsForVer3(const char* filename, std::vector<autoware_msgs::Waypoint>* wps);
  void parseWaypointForVer3(const std::string& line, const std::vector<std::string>& contents,
                            autoware_msgs::Waypoint* wp);
};

const std::string addFileSuffix(std::string file_path, std::string suffix);
void parseColumns(const std::string& line, std::vector<std::string>* columns);
size_t countColumns(const std::string& line);
}
#endif  // WAYPOINT_LOADER_UPON_REQUEST_CORE_H
