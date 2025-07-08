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

namespace coco_mov_control
{

using namespace std::chrono_literals;

class GaitPlanifier : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using JointTrajectory = trajectory_msgs::msg::JointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FolloJointTrajectory>;
  using Twist = geometry_msgs::msg::Twist;

  GaitPlanifier();
  
  void send_request(FollowJointTrajectory::Goal goal);
  bool is_action_finished() {return action_finished_;}
  bool is_result_success()  {return action_succeded_;}

protected:
  virtual void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle);
  virtual void feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  virtual void result_callback(const GoalHandleNavigateToPose::WrappedResult & result);

private:
  void twist_callback(Twist::UniquePtr twist);
  void control_cycle();
  JointTrajectory getJointTrajectory(Twist::UniquePtr twist);
  static const int STANDBY = 0;
  static const int WALKING = 1;
  static const int LAST_MOV_FINISHED = 2;
  int state_;
  bool action_finished_ {false};
  bool action_succeded_ {false};

  std::vector<std::string> joint_names_;
  JointTrajectory forward_;
  JointTrajectory backwards_;
  JointTrajectory sideways_;
  JointTrajectory combined_;
  JointTrajectory spin_;

  JointTrajectory curve_;  //future implementation
  JointTrajectory jump_;  //future implementation
  float curve_amplitude_; //future implementation
  float jump_intensity_;  //future implementation 

  float forward_factor_; //maps between (backward)-1 and 1(forward)
  float side_factor_;  //maps between (right)-1 and 1(left)
  float spin_factor_;  //maps between (right)-1 and 1(left)

  Twist current_twist_; 
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::Subscription<Twist>::SharedPtr twist_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  }
}



#endif // COCO_MOV_CONTROL__GAIT_PLANIFIER_HPP_
