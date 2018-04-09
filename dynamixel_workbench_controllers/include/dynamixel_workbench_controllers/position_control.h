/*******************************************************************************
 * Copyright 2016 ROBOTIS CO., LTD.
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

/* Authors: Taehoon Lim (Darby) */

#ifndef DYNAMIXEL_WORKBENCH_POSITION_CONTROL_H
#define DYNAMIXEL_WORKBENCH_POSITION_CONTROL_H

#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include "message_header.h"

#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/JointCommand.h>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_msgs/JointState.h>

class PositionControl {
private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher dynamixel_state_list_pub_;

  // ROS Topic Subscriber

  // ROS Service Server
  ros::ServiceServer joint_command_server_;
  ros::Subscriber joint_sub_[16];
  std::vector<ros::Publisher> joint_pubs_;
  ros::Publisher joint_states_pub_;

  // ROS Service Client

  // Dynamixel Workbench Parameters
  DynamixelWorkbench *dxl_wb_;
  uint8_t dxl_id_[16];
  uint8_t dxl_cnt_;

public:
  PositionControl();
  ~PositionControl();
  void controlLoop(void);

private:
  void initMsg();

  void initPublisher();
  void dynamixelStatePublish();

  void initServer();
  bool jointCommandMsgCallback(
      dynamixel_workbench_msgs::JointCommand::Request &req,
      dynamixel_workbench_msgs::JointCommand::Response &res);
  void joint_command_sub_callback(const uint8_t id,
                                  const std_msgs::Float64::ConstPtr &msg);
};

#endif // DYNAMIXEL_WORKBENCH_POSITION_CONTROL_H
