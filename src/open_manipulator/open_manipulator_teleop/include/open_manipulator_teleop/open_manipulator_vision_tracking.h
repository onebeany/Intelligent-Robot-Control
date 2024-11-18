/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#ifndef OPEN_MANIPULATOR_VISION_TRACKING_H_
#define OPEN_MANIPULATOR_VISION_TRACKING_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "open_manipulator_msgs/SetJointPosition.h"
#include <vision/frameInfo.h>

#define NUM_OF_JOINT 4
#define JOINT_DELTA 0.05
#define PATH_TIME 0.5
#define INIT_JOINT1 0.0
#define INIT_JOINT2 0.0
#define INIT_JOINT3 -1.5
#define INIT_JOINT4 1.5

class OpenManipulatorTeleop
{
 public:
  OpenManipulatorTeleop();
  ~OpenManipulatorTeleop();

  void printJointStates();

 private:
  /*****************************************************************************
  ** ROS NodeHandle
  *****************************************************************************/
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  /*****************************************************************************
  ** Variables
  *****************************************************************************/
  std::vector<double> present_joint_angle_;
  open_manipulator_msgs::SetJointPosition srv_;

  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void initSubscriber();
  void initClient();

  /*****************************************************************************
  ** ROS Subscribers and Callback Functions
  *****************************************************************************/
  ros::Subscriber joint_states_sub_;
  ros::Subscriber face_info_sub_;

  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void faceInfoCallback(const vision::frameInfo::ConstPtr &msg);
  void callServiceWithTimeout();


  /*****************************************************************************
  ** ROS Clients
  *****************************************************************************/
  ros::ServiceClient goal_joint_space_path_client_;
  ros::ServiceClient goal_joint_space_path_from_present_client_;

  bool setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
};

#endif //OPEN_MANIPULATOR_VISION_TRACKING_H_
