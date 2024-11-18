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

#include "open_manipulator_teleop/open_manipulator_vision_tracking.h"

OpenManipulatorTeleop::OpenManipulatorTeleop()
: node_handle_(""),
  priv_node_handle_("~")
{
  present_joint_angle_.resize(NUM_OF_JOINT);

  initSubscriber();
  initClient();

  // 초기 포즈 설정
  std::vector<std::string> joint_name{"joint1", "joint2", "joint3", "joint4"};
  std::vector<double> joint_angle{INIT_JOINT1, INIT_JOINT2, INIT_JOINT3, INIT_JOINT4};
  
  // 충분한 이동 시간 설정
  double init_path_time = 2.0;
  setJointSpacePath(joint_name, joint_angle, init_path_time);
  
  ROS_INFO("OpenManipulator vision tracking start");
}

OpenManipulatorTeleop::~OpenManipulatorTeleop()
{
  
  ROS_INFO("Terminate OpenManipulator vision tracking");
  ros::shutdown();
}

void OpenManipulatorTeleop::initClient()
{
  goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
  goal_joint_space_path_from_present_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path_from_present");
}

void OpenManipulatorTeleop::initSubscriber()
{
  joint_states_sub_ = node_handle_.subscribe("joint_states", 10, &OpenManipulatorTeleop::jointStatesCallback, this);
  face_info_sub_ = node_handle_.subscribe<vision::frameInfo>("face_info", 10, &OpenManipulatorTeleop::faceInfoCallback, this);
}

void OpenManipulatorTeleop::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);
  for (std::vector<int>::size_type i = 0; i < msg->name.size(); i ++)
  {
    if (!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;
}

void OpenManipulatorTeleop::faceInfoCallback(const vision::frameInfo::ConstPtr &msg)
{
  if(!msg || !msg->isDetected) return;

  ROS_INFO("Face detected: error yaw: %.3f, error pitch: %.3f", msg->error_yaw, msg->error_pitch);
  std::vector<std::string> joint_name{"joint1", "joint2", "joint3", "joint4"};
  std::vector<double> joint_angle(NUM_OF_JOINT, 0.0);

  const double SCALE_FACTOR = 4.0;
  const double threshold = 0.1; // threshold for guaranting the error radian is not too small

  if(msg->error_yaw != 0.0){ 
    joint_angle[0] = msg->error_yaw;
    if(msg->error_yaw > threshold) joint_angle[0] /= SCALE_FACTOR;
  }
  if(msg->error_pitch != 0.0){
    joint_angle[3] = -msg->error_pitch;
    if(msg->error_pitch > threshold) joint_angle[3] /= SCALE_FACTOR;
  }

  double path_time = (std::abs(msg->error_yaw) / 0.05 * 0.1 + 
                     std::abs(msg->error_pitch) / 0.05 * 0.1) / 2;

  setJointSpacePathFromPresent(joint_name, joint_angle, path_time);
}

bool OpenManipulatorTeleop::setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  // 이전 요청이 처리 중인지 확인
  if (goal_joint_space_path_from_present_client_.isValid())
  {
    ROS_WARN("Previous service request is still being processed");
    //return false;
  }

  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if (goal_joint_space_path_from_present_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if (goal_joint_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

void OpenManipulatorTeleop::printJointStates()
{
  ROS_INFO("Joint Angles: J1: %.3f, J2: %.3f, J3: %.3f, J4: %.3f",
           present_joint_angle_[0],
           present_joint_angle_[1],
           present_joint_angle_[2],
           present_joint_angle_[3]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "open_manipulator_vision_tracking");
  OpenManipulatorTeleop openManipulatorTeleop;

  ros::Rate loop_rate(2); // Reduced rate to 2Hz for readable output
  while (ros::ok())
  {
    ros::spinOnce();
    openManipulatorTeleop.printJointStates();
    loop_rate.sleep();
  }

  return 0;
}
