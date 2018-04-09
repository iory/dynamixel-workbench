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

#include "dynamixel_workbench_controllers/position_control.h"

PositionControl::PositionControl() : node_handle_("") {
  std::string device_name =
      node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  uint32_t dxl_baud_rate = node_handle_.param<int>("baud_rate", 57600);

  uint8_t scan_range = node_handle_.param<int>("scan_range", 200);

  uint32_t profile_velocity = node_handle_.param<int>("profile_velocity", 200);
  uint32_t profile_acceleration =
      node_handle_.param<int>("profile_acceleration", 50);

  dxl_wb_ = new DynamixelWorkbench;

  dxl_wb_->begin(device_name.c_str(), dxl_baud_rate);

  if (dxl_wb_->scan(dxl_id_, &dxl_cnt_, scan_range) != true) {
    ROS_ERROR("Not found Motors, Please check scan range and baud rate");
    ros::shutdown();
    return;
  }

  initMsg();

  for (int index = 0; index < dxl_cnt_; index++)
    dxl_wb_->jointMode(dxl_id_[index], profile_velocity, profile_acceleration);

  initPublisher();
  initServer();

  // if not torque enable, reboot.
  for (int index = 0; index < dxl_cnt_; index++) {
      if (dxl_wb_->itemRead(dxl_id_[index], "Torque_Enable") == 0) {
          printf("Rebooting id = [%d]\n", dxl_id_[index]);
          dxl_wb_->reboot(dxl_id_[index]);
      }
  }
}

PositionControl::~PositionControl() {
  for (int index = 0; index < dxl_cnt_; index++)
    dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);

  ros::shutdown();
}

void PositionControl::initMsg() {
  printf("---------------------------------------------------------------------"
         "--\n");
  printf("  dynamixel_workbench controller; position control example           "
         "  \n");
  printf("---------------------------------------------------------------------"
         "--\n");
  printf("\n");

  for (int index = 0; index < dxl_cnt_; index++) {
    printf("MODEL   : %s\n", dxl_wb_->getModelName(dxl_id_[index]));
    printf("ID      : %d\n", dxl_id_[index]);
    printf("\n");
  }
  printf("---------------------------------------------------------------------"
         "--\n");
}

void PositionControl::initPublisher() {
  dynamixel_state_list_pub_ =
      node_handle_.advertise<dynamixel_workbench_msgs::DynamixelStateList>(
          "dynamixel_state", 10);

  joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>
      ("joint_states", 10);

  for (int i = 0; i < dxl_cnt_; ++i) {
    joint_sub_[i] = node_handle_.subscribe<std_msgs::Float64>(
        "arm_" + std::to_string(dxl_id_[i]) + "/command", 1,
        boost::bind(&PositionControl::joint_command_sub_callback, this,
                    dxl_id_[i], _1));

    std::ostringstream os;
    os << unsigned(dxl_id_[i]);
    joint_pubs_.push_back(node_handle_.advertise<dynamixel_msgs::JointState>
                          ("arm_" + os.str() + "/state", 1));
  }
}

void PositionControl::initServer() {
  joint_command_server_ = node_handle_.advertiseService(
      "joint_command", &PositionControl::jointCommandMsgCallback, this);
}

void PositionControl::dynamixelStatePublish() {
  dynamixel_workbench_msgs::DynamixelState dynamixel_state[dxl_cnt_];
  dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list;
  dynamixel_msgs::JointState joint_state;
  sensor_msgs::JointState joint_state_org;
  int reverse[] = {1, 1, 0, 1, 1, 1, 1, 1};
  for (int index = 0; index < dxl_cnt_; index++) {
    dynamixel_state[index].model_name =
        std::string(dxl_wb_->getModelName(dxl_id_[index]));
    dynamixel_state[index].id = dxl_id_[index];
    dynamixel_state[index].torque_enable =
        dxl_wb_->itemRead(dxl_id_[index], "Torque_Enable");
    dynamixel_state[index].present_position =
        dxl_wb_->itemRead(dxl_id_[index], "Present_Position");
    dynamixel_state[index].present_velocity =
        dxl_wb_->itemRead(dxl_id_[index], "Present_Velocity");
    dynamixel_state[index].goal_position =
        dxl_wb_->itemRead(dxl_id_[index], "Goal_Position");
    dynamixel_state[index].goal_velocity =
        dxl_wb_->itemRead(dxl_id_[index], "Goal_Velocity");
    dynamixel_state[index].moving = dxl_wb_->itemRead(dxl_id_[index], "Moving");

    dynamixel_state_list.dynamixel_state.push_back(dynamixel_state[index]);

    std::ostringstream os;
    os << unsigned(dxl_id_[index]);
    joint_state.name = os.str();
    joint_state.current_pos = dxl_wb_->convertValue2Radian(dxl_id_[index],
                                                           dynamixel_state[index].present_position);
    joint_state.velocity = dxl_wb_->convertValue2Radian(dxl_id_[index],
                                                        dynamixel_state[index].present_velocity);
    joint_state.is_moving = dynamixel_state[index].moving;
    joint_pubs_[index].publish(joint_state);

    int rev;
    os.str("");
    if (index == 2) {
        continue;
    } else if (index > 2) {
        os << (index - 1);
        if (index - 1 == 6) {
            rev = 0;
        } else {
            rev = reverse[index - 1];
        }
    } else {
        os << index;
        rev = reverse[index];
    }
    joint_state_org.name.push_back("JOINT" + os.str());
    if (rev) {
        joint_state_org.position.push_back(-joint_state.current_pos);
    } else {
        joint_state_org.position.push_back(joint_state.current_pos);
    }
    if (index == dxl_cnt_ - 1) {
        os.str("");
        os << index;
        joint_state_org.name.push_back("JOINT" + os.str());
        joint_state_org.position.push_back(joint_state.current_pos);
    }
  }
  joint_state_org.header.stamp = ros::Time::now();
  dynamixel_state_list_pub_.publish(dynamixel_state_list);
  joint_states_pub_.publish(joint_state_org);
}

void PositionControl::controlLoop() { dynamixelStatePublish(); }

bool PositionControl::jointCommandMsgCallback(
    dynamixel_workbench_msgs::JointCommand::Request &req,
    dynamixel_workbench_msgs::JointCommand::Response &res) {
  int32_t goal_position = 0;
  int32_t present_position = 0;

  if (req.unit == "rad") {
    goal_position = dxl_wb_->convertRadian2Value(req.id, req.goal_position);
  } else if (req.unit == "raw") {
    goal_position = req.goal_position;
  } else if (req.unit == "deg") {
    goal_position = dxl_wb_->convertRadian2Value(
        req.id, req.goal_position / 180.0 * 3.141592653589793);
  } else {
    goal_position = req.goal_position;
  }

  bool ret = dxl_wb_->goalPosition(req.id, goal_position);

  res.result = ret;
}

void PositionControl::joint_command_sub_callback(
    const uint8_t id, const std_msgs::Float64::ConstPtr &msg) {
  int32_t goal_position =
      dxl_wb_->convertRadian2Value(id, msg->data / 180.0 * 3.141592653589793);
  bool ret = dxl_wb_->goalPosition(id, goal_position);
}

int main(int argc, char **argv) {
  // Init ROS node
  ros::init(argc, argv, "position_control");
  PositionControl pos_ctrl;

  ros::Rate loop_rate(1000);

  while (ros::ok()) {
    pos_ctrl.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
