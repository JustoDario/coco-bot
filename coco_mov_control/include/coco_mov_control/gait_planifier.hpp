// Copyright 2025 Justo Dario Valverde <justodariovalverde@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef COCO_MOV_CONTROL__GAIT_PLANIFIER_HPP_
#define COCO_MOV_CONTROL__GAIT_PLANIFIER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "control_msgs/msg/joint_tolerance.hpp"

namespace coco_mov_control
{

using namespace std::chrono_literals;

class GaitPlanifier : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using JointTrajectory = trajectory_msgs::msg::JointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
  using Twist = geometry_msgs::msg::Twist;

  GaitPlanifier();
  
  void send_request(JointTrajectory & gait);
  bool is_action_finished() {return action_finished_;}
  bool is_result_success()  {return action_succeeded_;}

protected:
  virtual void goal_response_callback(const GoalHandleFollowJointTrajectory::SharedPtr & goal_handle);
  virtual void result_callback(const GoalHandleFollowJointTrajectory::WrappedResult & wrapped_result);

private:
  std::array<float, 3> get_leg_angles(const std::array<float, 3>& foot_goal_position);
  std::vector<std::array<float, 12>> calculate_joint_positions(const std::vector<std::array<float, 3>>& gait_points_per_leg);
  std::vector<std::array<float, 12>> spin_joint_positions(const std::vector<std::array<float, 3>>& gait_right_legs, const std::vector<std::array<float, 3>>& gait_left_legs, bool right_spin);
  std::vector<std::array<float, 12>> curve_joint_positions();  

  void twist_callback(const Twist::ConstSharedPtr & twist);
  void control_cycle();
  JointTrajectory get_joint_trajectory(const Twist & twist);
  float get_step_time(std::vector<std::array<float, 12>> trajectory_points, float vel);
  
  //Values obtained through experimentation with coco -> may change with better gaits
  const float max_vel_ = 0.4;
  const float std_vel_ = 0.25;
  const float min_vel_ = 0.15;
  const float std_real_step_time = 0.4;//secs

  static const int STANDBY = 0;
  static const int WALKING = 1;
  static const int EXECUTING = 2;
  int state_;
  bool action_finished_ {true};
  bool action_succeeded_ {true};
  std::vector<std::array<float, 12>> default_forward_;
  std::vector<std::array<float, 12>> default_backward_;
  std::vector<std::array<float, 12>> default_left_;
  std::vector<std::array<float, 12>> default_right_;
  std::vector<std::array<float, 12>> default_standby_;
  std::vector<std::array<float, 12>> default_right_spin_;
  std::vector<std::array<float, 12>> default_left_spin_;
  std::vector<std::array<float, 12>> sit_;


//Emotes
  std::vector<std::array<float, 12>> stretch_;
  std::vector<std::array<float, 12>> jump_;

  std::vector<std::string> joint_names_;
  float curve_amplitude_; //future implementation
  float jump_intensity_;  //future implementation 
  size_t ngait_points_;
  const size_t njoints_ = 12;

  JointTrajectory current_trajectory_;
  Twist current_twist_;
  Twist standby_twist_;

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::Subscription<Twist>::SharedPtr twist_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_twist_;


  // rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Clock system_clock_{RCL_SYSTEM_TIME};

};
}



#endif // COCO_MOV_CONTROL__GAIT_PLANIFIER_HPP_
