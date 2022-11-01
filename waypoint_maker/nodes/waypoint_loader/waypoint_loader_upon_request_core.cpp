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

#include "waypoint_loader_upon_request_core.h"

// Google
#include <glog/logging.h>

namespace waypoint_maker
{
// Constructor
WaypointLoaderUponRequestNode::WaypointLoaderUponRequestNode() 
  : private_nh_("~") {

  // setup publisher
  lane_pub_ = nh_.advertise<autoware_msgs::LaneArray>("/based/lane_waypoints_raw", 10, true);

  // Set up service server.
  service_ = nh_.advertiseService("lane_load_from_csv", &WaypointLoaderUponRequestNode::serveLanePublishRequest, this);
}

bool WaypointLoaderUponRequestNode::serveLanePublishRequest(autoware_msgs::String::Request &req, autoware_msgs::String::Request &res) {

  std::vector<std::string> columns;
  parseColumns(req.str, &columns);
  autoware_msgs::LaneArray lane_array;
  createLaneArray(columns, &lane_array);
  lane_pub_.publish(lane_array);

  return true;
}

void WaypointLoaderUponRequestNode::run()
{
  ros::spin();
}

void WaypointLoaderUponRequestNode::createLaneArray(const std::vector<std::string>& paths, autoware_msgs::LaneArray* lane_array)
{
  for (const auto& el : paths)
  {
    autoware_msgs::Lane lane;
    createLaneWaypoint(el, &lane);
    lane_array->lanes.emplace_back(lane);
  }
}

void WaypointLoaderUponRequestNode::createLaneWaypoint(const std::string& file_path, autoware_msgs::Lane* lane)
{
  if (!verifyFileConsistency(file_path.c_str()))
  {
    ROS_ERROR("lane data is something wrong...");
    return;
  }

  ROS_INFO("lane data is valid. publishing...");
  FileFormat format = checkFileFormat(file_path.c_str());
  CHECK(format == FileFormat::ver3);

  std::vector<autoware_msgs::Waypoint> wps;
  loadWaypointsForVer3(file_path.c_str(), &wps);
  lane->header.frame_id = "/map";
  lane->header.stamp = ros::Time(0);
  lane->waypoints = wps;
}

void WaypointLoaderUponRequestNode::loadWaypointsForVer3(const char* filename, std::vector<autoware_msgs::Waypoint>* wps)
{
  std::ifstream ifs(filename);

  if (!ifs)
  {
    return;
  }

  std::string line;
  std::getline(ifs, line);  // get first line
  std::vector<std::string> contents;
  parseColumns(line, &contents);

  // std::getline(ifs, line);  // remove second line
  while (std::getline(ifs, line))
  {
    autoware_msgs::Waypoint wp;
    parseWaypointForVer3(line, contents, &wp);
    wps->emplace_back(wp);
  }
}

void WaypointLoaderUponRequestNode::parseWaypointForVer3(const std::string& line, const std::vector<std::string>& contents,
                                              autoware_msgs::Waypoint* wp)
{
  std::vector<std::string> columns;
  parseColumns(line, &columns);
  std::unordered_map<std::string, std::string> map;
  for (size_t i = 0; i < contents.size(); i++)
  {
    map[contents.at(i)] = columns.at(i);
  }

  wp->pose.pose.position.x = std::stod(map["x"]);
  wp->pose.pose.position.y = std::stod(map["y"]);
  wp->pose.pose.position.z = std::stod(map["z"]);
  wp->pose.pose.orientation = tf::createQuaternionMsgFromYaw(std::stod(map["yaw"]));
  wp->twist.twist.linear.x = kmph2mps(std::stod(map["velocity"]));
  wp->change_flag = std::stoi(map["change_flag"]);
  wp->wpstate.steering_state = (map.find("steering_flag") != map.end()) ? std::stoi(map["steering_flag"]) : 0;
  wp->wpstate.accel_state = (map.find("accel_flag") != map.end()) ? std::stoi(map["accel_flag"]) : 0;
  wp->wpstate.stop_state = (map.find("stop_flag") != map.end()) ? std::stoi(map["stop_flag"]) : 0;
  wp->wpstate.event_state = (map.find("event_flag") != map.end()) ? std::stoi(map["event_flag"]) : 0;
}

FileFormat WaypointLoaderUponRequestNode::checkFileFormat(const char* filename)
{
  std::ifstream ifs(filename);

  if (!ifs)
  {
    return FileFormat::unknown;
  }

  // get first line
  std::string line;
  std::getline(ifs, line);

  // parse first line
  std::vector<std::string> parsed_columns;
  parseColumns(line, &parsed_columns);

  // check if first element in the first column does not include digit
  if (!std::any_of(parsed_columns.at(0).cbegin(), parsed_columns.at(0).cend(), isdigit))
  {
    return FileFormat::ver3;
  }

  // if element consists only digit
  int num_of_columns = countColumns(line);
  ROS_INFO("columns size: %d", num_of_columns);

  if (num_of_columns == 3) {
    return FileFormat::ver1;
  } else if (num_of_columns == 4){
    return FileFormat::ver2;
  } else {
    return FileFormat::unknown;
  }
}

bool WaypointLoaderUponRequestNode::verifyFileConsistency(const char* filename)
{
  ROS_INFO("verify...");
  std::ifstream ifs(filename);

  if (!ifs)
  {
    return false;
  }

  FileFormat format = checkFileFormat(filename);
  ROS_INFO("format: %d", static_cast<int>(format));
  if (format == FileFormat::unknown)
  {
    ROS_ERROR("unknown file format");
    return false;
  }

  std::string line;
  std::getline(ifs, line);  // remove first line

  size_t ncol;
  if (format == FileFormat::ver1) {
    ncol = 4;
  } else if (format == FileFormat::ver2) {
    ncol = 5;
  } else {
    ncol = countColumns(line);
  }

  while (std::getline(ifs, line))  // search from second line
  {
    if (countColumns(line) != ncol)
    {
      return false;
    }
  }
  return true;
}

void parseColumns(const std::string& line, std::vector<std::string>* columns)
{
  std::istringstream ss(line);
  std::string column;
  while (std::getline(ss, column, ','))
  {
    while (1)
    {
      auto res = std::find(column.begin(), column.end(), ' ');
      if (res == column.end())
      {
        break;
      }
      column.erase(res);
    }
    if (!column.empty())
    {
      columns->emplace_back(column);
    }
  }
}

size_t countColumns(const std::string& line)
{
  std::istringstream ss(line);
  size_t ncol = 0;

  std::string column;
  while (std::getline(ss, column, ','))
  {
    ++ncol;
  }

  return ncol;
}

}  // waypoint_maker
