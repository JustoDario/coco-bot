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
#include <algorithm>
#include "coco_mov_control/gait_planifier.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace
{ //Falta revisar si para los servos que estan en posicion contraria el angulo se invierte
  //En el hardware interface se ajustara eso
  //De momento para rpobar todos los gaits son hacia delante default
  std::vector<std::vector<float>> DEFAULT_FORWARD_GAIT = {
  {1.5708,1.5708,1.5708,1.5421,0.8948,1.2368,1.5421,0.8948,1.2368,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5449,0.8210,1.3958,1.5449,0.8210,1.3958,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5449,0.7021,1.4155,1.5449,0.7021,1.4155,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5421,0.7640,1.2574,1.5421,0.7640,1.2574,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5421,0.8303,1.2445,1.5421,0.8303,1.2445,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5421,0.8071,1.2484,1.5421,0.8071,1.2484,1.5708,1.5708,1.5708},
  {1.5421,0.8948,1.2368,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5421,0.8948,1.2368},
  {1.5449,0.8210,1.3958,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5449,0.8210,1.3958},
  {1.5449,0.7021,1.4155,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5449,0.7021,1.4155},
  {1.5421,0.7640,1.2574,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5421,0.7640,1.2574},
  {1.5421,0.8303,1.2445,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5421,0.8303,1.2445},
  {1.5421,0.8071,1.2484,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5421,0.8071,1.2484}
  };
  std::vector<std::vector<float>> DEFAULT_BACKWARD_GAIT = {
  {1.5708,1.5708,1.5708,1.5421,0.8948,1.2368,1.5421,0.8948,1.2368,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5449,0.8210,1.3958,1.5449,0.8210,1.3958,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5449,0.7021,1.4155,1.5449,0.7021,1.4155,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5421,0.7640,1.2574,1.5421,0.7640,1.2574,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5421,0.8303,1.2445,1.5421,0.8303,1.2445,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5421,0.8071,1.2484,1.5421,0.8071,1.2484,1.5708,1.5708,1.5708},
  {1.5421,0.8948,1.2368,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5421,0.8948,1.2368},
  {1.5449,0.8210,1.3958,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5449,0.8210,1.3958},
  {1.5449,0.7021,1.4155,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5449,0.7021,1.4155},
  {1.5421,0.7640,1.2574,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5421,0.7640,1.2574},
  {1.5421,0.8303,1.2445,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5421,0.8303,1.2445},
  {1.5421,0.8071,1.2484,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5421,0.8071,1.2484}
  };
  std::vector<std::vector<float>> DEFAULT_LEFT_GAIT = {
  {1.5708,1.5708,1.5708,1.5421,0.8948,1.2368,1.5421,0.8948,1.2368,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5449,0.8210,1.3958,1.5449,0.8210,1.3958,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5449,0.7021,1.4155,1.5449,0.7021,1.4155,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5421,0.7640,1.2574,1.5421,0.7640,1.2574,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5421,0.8303,1.2445,1.5421,0.8303,1.2445,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5421,0.8071,1.2484,1.5421,0.8071,1.2484,1.5708,1.5708,1.5708},
  {1.5421,0.8948,1.2368,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5421,0.8948,1.2368},
  {1.5449,0.8210,1.3958,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5449,0.8210,1.3958},
  {1.5449,0.7021,1.4155,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5449,0.7021,1.4155},
  {1.5421,0.7640,1.2574,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5421,0.7640,1.2574},
  {1.5421,0.8303,1.2445,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5421,0.8303,1.2445},
  {1.5421,0.8071,1.2484,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5421,0.8071,1.2484}
  };
  std::vector<std::vector<float>> DEFAULT_RIGHT_GAIT = {
  {1.5708,1.5708,1.5708,1.5421,0.8948,1.2368,1.5421,0.8948,1.2368,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5449,0.8210,1.3958,1.5449,0.8210,1.3958,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5449,0.7021,1.4155,1.5449,0.7021,1.4155,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5421,0.7640,1.2574,1.5421,0.7640,1.2574,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5421,0.8303,1.2445,1.5421,0.8303,1.2445,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5421,0.8071,1.2484,1.5421,0.8071,1.2484,1.5708,1.5708,1.5708},
  {1.5421,0.8948,1.2368,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5421,0.8948,1.2368},
  {1.5449,0.8210,1.3958,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5449,0.8210,1.3958},
  {1.5449,0.7021,1.4155,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5449,0.7021,1.4155},
  {1.5421,0.7640,1.2574,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5421,0.7640,1.2574},
  {1.5421,0.8303,1.2445,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5421,0.8303,1.2445},
  {1.5421,0.8071,1.2484,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5421,0.8071,1.2484}
  };
  std::vector<std::vector<float>> DEFAULT_RIGHT_SPIN = {
  {1.5708,1.5708,1.5708,1.5421,0.8948,1.2368,1.5421,0.8948,1.2368,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5449,0.8210,1.3958,1.5449,0.8210,1.3958,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5449,0.7021,1.4155,1.5449,0.7021,1.4155,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5421,0.7640,1.2574,1.5421,0.7640,1.2574,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5421,0.8303,1.2445,1.5421,0.8303,1.2445,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5421,0.8071,1.2484,1.5421,0.8071,1.2484,1.5708,1.5708,1.5708},
  {1.5421,0.8948,1.2368,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5421,0.8948,1.2368},
  {1.5449,0.8210,1.3958,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5449,0.8210,1.3958},
  {1.5449,0.7021,1.4155,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5449,0.7021,1.4155},
  {1.5421,0.7640,1.2574,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5421,0.7640,1.2574},
  {1.5421,0.8303,1.2445,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5421,0.8303,1.2445},
  {1.5421,0.8071,1.2484,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5421,0.8071,1.2484}
  };
  std::vector<std::vector<float>> DEFAULT_LEFT_SPIN = {
  {1.5708,1.5708,1.5708,1.5421,0.8948,1.2368,1.5421,0.8948,1.2368,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5449,0.8210,1.3958,1.5449,0.8210,1.3958,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5449,0.7021,1.4155,1.5449,0.7021,1.4155,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5421,0.7640,1.2574,1.5421,0.7640,1.2574,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5421,0.8303,1.2445,1.5421,0.8303,1.2445,1.5708,1.5708,1.5708},
  {1.5708,1.5708,1.5708,1.5421,0.8071,1.2484,1.5421,0.8071,1.2484,1.5708,1.5708,1.5708},
  {1.5421,0.8948,1.2368,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5421,0.8948,1.2368},
  {1.5449,0.8210,1.3958,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5449,0.8210,1.3958},
  {1.5449,0.7021,1.4155,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5449,0.7021,1.4155},
  {1.5421,0.7640,1.2574,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5421,0.7640,1.2574},
  {1.5421,0.8303,1.2445,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5421,0.8303,1.2445},
  {1.5421,0.8071,1.2484,1.5708,1.5708,1.5708,1.5708,1.5708,1.5708,1.5421,0.8071,1.2484}
  };
  std::vector<std::vector<float>> DEFAULT_STANDBY = {
    {1.5708,0.6807,1.2902,1.5708,0.6807,1.2902,1.5708,0.6807,1.2902,1.5708,0.6807,1.2902}
  };
}
namespace coco_mov_control
{
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

GaitPlanifier::GaitPlanifier()
: Node("gait_planifier")
{
  action_client_ = rclcpp_action::create_client<coco_mov_control::GaitPlanifier::FollowJointTrajectory>(
  this, "/joint_trajectory_controller/follow_joint_trajectory");
  bool sim_time = this->get_parameter_or("use_sim_time", false);
  if (sim_time) {
    RCLCPP_WARN(this->get_logger(), "use_sim_time está en true, pero este nodo debe usar wall time.");
  }
  twist_sub_ = create_subscription<Twist>(
  "cmd_vel", 1, std::bind(&GaitPlanifier::twist_callback, this, _1));
  timer_ = create_wall_timer(
    50ms, std::bind(&GaitPlanifier::control_cycle, this));
  
  joint_names_ = {"lf_coax_joint","lf_femur_joint","lf_tibia_joint",
                  "rf_coax_joint","rf_femur_joint","rf_tibia_joint",
                  "lb_coax_joint","lb_femur_joint","lb_tibia_joint",
                  "rb_coax_joint","rb_femur_joint","rb_tibia_joint"};
  state_ = STANDBY;
  standby_twist_.linear.x = 0.0;
  standby_twist_.linear.y = 0.0;
  standby_twist_.angular.z = 0.0;

}
float
GaitPlanifier::get_step_time(std::vector<std::vector<float>> trajectory_points, float vel)
{
  //This will clamp the velocity received and map it to values that coco can achive
// Clamp velocity to [0, 1.5]
  float clamped_vel = std::clamp(vel, 0.0f, 1.5f);
  
  // Map absolute velocity from [0, 1.5] to [min_vel_, max_vel_] = [0.15, 0.4]
  float mapped_vel = min_vel_ + (clamped_vel / 1.5f) * (max_vel_ - min_vel_);
  
  // Calculate the ratio compared to standard velocity (0.25 m/s)
  float vel_ratio = std_vel_ / mapped_vel;
  
  // Calculate time per point
  // If we have 12 points at 0.25 m/s, each point takes 0.083 secs
  // So base_time_per_point = std_real_step_time / standard_trajectory_size
  // But we need to adjust based on actual trajectory size and velocity
  float base_time_per_point = std_real_step_time / trajectory_points.size(); // 0.083 secs for 12 points at std_vel_
  float time_per_point = base_time_per_point * vel_ratio;
  
  return time_per_point;
}


JointTrajectory 
GaitPlanifier::get_joint_trajectory(const Twist & twist)
{
  /*The servo angles in rads are obtained by using coco_ik.py given
    the desired position x,y,z in wich we want the foot to be placed
    ,for each keyframe.More info about keyframes i README.md
  */
  JointTrajectory result_gait;
  result_gait.header.stamp.sec = 0;
  result_gait.header.stamp.nanosec = 0;
  result_gait.header.frame_id = "";
  result_gait.joint_names = joint_names_;
  std::vector<std::vector<float>> joint_positions;
  float step_time;
  float movement_velocity;

  if(twist.linear.x != 0 && twist.linear.y == 0 && twist.angular.z == 0) {
    if(twist.linear.x > 0) {
      joint_positions = DEFAULT_FORWARD_GAIT;
    }
    else {
      joint_positions = DEFAULT_BACKWARD_GAIT;
    }
    movement_velocity = std::abs(twist.linear.x);
  }
  else if(twist.linear.x == 0 && twist.linear.y != 0 && twist.angular.z == 0) {
    if(twist.linear.y > 0) {
      joint_positions = DEFAULT_LEFT_GAIT;
    }
    else {
      joint_positions = DEFAULT_RIGHT_GAIT;
    }
    movement_velocity = std::abs(twist.linear.y);
  }
  else if(twist.linear.x == 0 && twist.linear.y == 0 && twist.angular.z != 0) {
    if(twist.angular.z > 0) {
      joint_positions = DEFAULT_LEFT_SPIN;
    }
    else {
      joint_positions = DEFAULT_RIGHT_SPIN;
    }
    movement_velocity = std::abs(twist.angular.z);
  }
  else if(twist.linear.x != 0 && twist.linear.y == 0 && twist.angular.z != 0) {
    joint_positions = DEFAULT_STANDBY;
    //Not yet supported but will in the future make a specific gait for this case
  }
  else if((twist.linear.x == 0 && twist.linear.y == 0 && twist.angular.z == 0)) {
    joint_positions = DEFAULT_STANDBY;
  }
  else if(twist.linear.x != 0 && twist.linear.y != 0 && twist.angular.z == 0) {
    joint_positions = DEFAULT_STANDBY;
    //Not yet supported but will in the future make a specific gait for this case
  }
  else {
    //Not supported
    joint_positions = DEFAULT_STANDBY;
  }
  ngait_points_ = joint_positions.size();
  if(joint_positions != DEFAULT_STANDBY) {
    result_gait.points.resize(ngait_points_);
    step_time = get_step_time(joint_positions, movement_velocity);
    for (size_t i = 0; i < ngait_points_; i++) {
          result_gait.points[i].positions.resize(njoints_);
          for(size_t j = 0; j < njoints_ ; j++) {
            result_gait.points[i].positions[j] = joint_positions[i][j];
          }
          // Times depending on velocity received
          result_gait.points[i].time_from_start.sec = 0;
          result_gait.points[i].time_from_start.nanosec =  static_cast<uint32_t>(step_time * 1e9) * (i + 1);;
    }
  }
  else {
    result_gait.points.resize(1);
    result_gait.points[0].positions.resize(njoints_);
    for(size_t i; i < njoints_ ; i++) {
      result_gait.points[0].positions[i] = joint_positions[0][i];
    }
    result_gait.points[0].time_from_start.sec = 0;
    result_gait.points[0].time_from_start.nanosec = 125000000;
  }

  return result_gait;
}

void
GaitPlanifier::twist_callback(const Twist::ConstSharedPtr & twist)
{
  if(current_twist_.linear.x != twist->linear.x &&
    current_twist_.linear.y != twist->linear.y &&
    current_twist_.angular.z != twist->angular.z){

    current_twist_ =  *twist;
    current_trajectory_ = get_joint_trajectory(current_twist_);
  }
  last_twist_ = this->now();
  state_ = WALKING;
}


void
GaitPlanifier::send_request(JointTrajectory& gait)
{
  auto goal = GaitPlanifier::FollowJointTrajectory::Goal();
  goal.trajectory = gait;
  for (const auto & name : gait.joint_names) {
    control_msgs::msg::JointTolerance jt;
    jt.name = name;
    jt.position = -1.0;
    jt.velocity = -1.0;
    jt.acceleration = -1.0;

    goal.path_tolerance.push_back(jt);
    goal.goal_tolerance.push_back(jt);
  }
  goal.goal_time_tolerance = rclcpp::Duration::from_seconds(1.0);
  action_finished_ = false;
  action_succeeded_ = false;
  state_ = WALKING;
  if (!action_client_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto send_goal_options =
    rclcpp_action::Client<GaitPlanifier::FollowJointTrajectory>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    std::bind(&GaitPlanifier::goal_response_callback, this, _1);
  send_goal_options.feedback_callback = nullptr;
  send_goal_options.result_callback =
    std::bind(&GaitPlanifier::result_callback, this, _1);

  action_client_->async_send_goal(goal, send_goal_options);
}

void
GaitPlanifier::goal_response_callback(const GoalHandleFollowJointTrajectory::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
  }
}

void
GaitPlanifier::result_callback(const GoalHandleFollowJointTrajectory::WrappedResult & wrapped_result)
{
  {
  // Check communication with action server
  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Action ended(ROS2): SUCCEEDED");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "Action ended(ROS2): ABORTED");
      action_finished_ = true;
      action_succeeded_ = false;
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(get_logger(), "Action ended(ROS2): CANCELED");
      action_finished_ = true;
      action_succeeded_ = false;
      return;
    default:
      RCLCPP_ERROR(get_logger(), "Action ended(ROS2): Unknown code");
      action_finished_ = true;
      action_succeeded_ = false;
      return;
  }

  // Check specific state of FollowJointTrajectory
  const auto & result = wrapped_result.result;
  switch (result->error_code) {
    case control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL:
      RCLCPP_INFO(get_logger(), "Trajectory followed succesfully");
      action_succeeded_ = true;
      break;
    case control_msgs::action::FollowJointTrajectory::Result::INVALID_GOAL:
      RCLCPP_ERROR(get_logger(), "Error: INVALID_GOAL — %s", result->error_string.c_str());
      action_succeeded_ = false;
      break;
    case control_msgs::action::FollowJointTrajectory::Result::INVALID_JOINTS:
      RCLCPP_ERROR(get_logger(), "Error: INVALID_JOINTS — %s", result->error_string.c_str());
      action_succeeded_ = false;
      break;
    case control_msgs::action::FollowJointTrajectory::Result::OLD_HEADER_TIMESTAMP:
      RCLCPP_ERROR(get_logger(), "Error: OLD_HEADER_TIMESTAMP — %s", result->error_string.c_str());
      action_succeeded_ = false;
      break;
    case control_msgs::action::FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED:
      RCLCPP_ERROR(get_logger(), "Error: PATH_TOLERANCE_VIOLATED — %s", result->error_string.c_str());
      action_succeeded_ = false;
      break;
    case control_msgs::action::FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED:
      RCLCPP_ERROR(get_logger(), "Error: GOAL_TOLERANCE_VIOLATED — %s", result->error_string.c_str());
      action_succeeded_ = false;
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Error desconocido en ejecución de trayectoria: %d — %s",
                   result->error_code, result->error_string.c_str());
      action_succeeded_ = false;
      break;
  }
  action_finished_ = true;
  }
}


void
GaitPlanifier::control_cycle()
{
  if(state_ != STANDBY && (this->now() - last_twist_ )> 1s) {
    state_ = STANDBY;
  }
  if (!action_finished_) {
    return;
  }
  switch (state_) {
    case STANDBY:
      // In standby and no movement required -> send standby_pos
      current_trajectory_ = get_joint_trajectory(standby_twist_);
      send_request(current_trajectory_);
      state_ = EXECUTING;
      break;

    case WALKING:
      // New twist and anterior has finished -> send current_twist trajectory
      current_trajectory_ = get_joint_trajectory(current_twist_);
      send_request(current_trajectory_);
      state_ = EXECUTING;
      break;

    case EXECUTING:
      // Waiting action_finished
      break;
  }
  }
}
