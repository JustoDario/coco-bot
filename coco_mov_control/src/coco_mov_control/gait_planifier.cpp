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
#include <cmath>
#include "coco_mov_control/gait_planifier.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace
{
  std::vector<std::array<float, 3>> DEFAULT_FORWARD_GAIT = {
    {7, 160.0, 40.0},
    {7, 180.0, 40.0},
    {42, 180.0, 40.0},
    {42.0, 160.0, 40.0},
    {7.0, 160.0, 40.0}

  };
  std::vector<std::array<float, 3>> DEFAULT_BACKWARD_GAIT = {
    DEFAULT_FORWARD_GAIT[3],
    DEFAULT_FORWARD_GAIT[2],
    DEFAULT_FORWARD_GAIT[1],
    DEFAULT_FORWARD_GAIT[0],
  };
  std::vector<std::array<float, 3>> DEFAULT_LEFT_GAIT = {
    {18.0, 150.0, 36.0},
    {18.0, 142.0, 27.0},
    {18.0, 135.0, 21.0},
    {18.0, 142.0, 15.0},
    {18.0, 150.0, 6.0},
    {18.0, 150.0, 36.0}
  };
  std::vector<std::array<float, 3>> DEFAULT_RIGHT_GAIT = {
    {18.0, 150.0, 36.0},
    {18.0, 142.0, 45.0},
    {18.0, 135.0, 51.0},
    {18.0, 142.0, 57.0},
    {18.0, 150.0, 66.0}
  };
  std::vector<std::array<float, 3>> DEFAULT_RIGHT_SPIN ;
  std::vector<std::array<float, 3>> DEFAULT_LEFT_SPIN;
  std::vector<std::array<float, 3>> DEFAULT_STANDBY = {{7.0, 160.0, 40.0}}; 
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
  timer_ = this->create_wall_timer(
    10ms, std::bind(&GaitPlanifier::control_cycle, this));
  joint_names_ = {"lf_coax_joint","lf_femur_joint","lf_tibia_joint",
                  "rf_coax_joint","rf_femur_joint","rf_tibia_joint",
                  "lb_coax_joint","lb_femur_joint","lb_tibia_joint",
                  "rb_coax_joint","rb_femur_joint","rb_tibia_joint"};
  state_ = STANDBY;
  standby_twist_.linear.x = 0.0;
  standby_twist_.linear.y = 0.0;
  standby_twist_.angular.z = 0.0;
  default_forward_ = calculate_joint_positions(DEFAULT_FORWARD_GAIT);
  default_backward_ = calculate_joint_positions(DEFAULT_BACKWARD_GAIT);
  default_left_ = calculate_joint_positions(DEFAULT_LEFT_GAIT);
  default_right_ = calculate_joint_positions(DEFAULT_RIGHT_GAIT);
  default_standby_ = calculate_joint_positions(DEFAULT_STANDBY);

}
float
GaitPlanifier::get_step_time(std::vector<std::array<float, 12>> trajectory_points, float vel)
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
std::array<float, 3>
GaitPlanifier::get_leg_angles(const std::array<float, 3>& foot_goal_position)
{
  /* This function uses inverse kinematics, receibes the desired positioning
  of the foot of one leg, and calculates the necesary angles in rads for
  coxa femur and tibia.
  More details in project readme
  */
  float x = foot_goal_position[0];  //forward/backward separation
  float y = foot_goal_position[1];  //distance between foot and coxa(altitude)
  float z = foot_goal_position[2];  //lateral distance between foot and coxa
  // Calculate coxa angle
  float parte1_coxa = std::atan(z / y);
  float arg_raiz_coxa = (z * z + y * y) - 1600.0f;
  float parte2_coxa = std::atan(std::sqrt(arg_raiz_coxa) / 40.0f);
  float coxa_rad = parte1_coxa + parte2_coxa;

  // Calculate tibia angle
  float arg_acos_tibia = ((z * z + y * y) - 1600.0f + x * x - 28800.0f) / -28800.0f;
  float tibia_rad = std::acos(arg_acos_tibia);

  // Calculate femur angle
  float arg_raiz_femur1 = ((z * z + y * y) - 1600.0f);
  float parte1_femur = std::atan(x / std::sqrt(arg_raiz_femur1));

  float arg_raiz_femur2 = arg_raiz_femur1 + x * x;
  float arg_asin_femur = (120.0f * std::sin(tibia_rad)) / std::sqrt(arg_raiz_femur2);
  float parte2_femur = std::asin(arg_asin_femur);

  float femur_rad = parte1_femur + parte2_femur;

  return { coxa_rad, femur_rad, tibia_rad };

}
std::vector<std::array<float, 12>>
GaitPlanifier::calculate_joint_positions(const std::vector<std::array<float, 3>>& gait_steps_per_leg)
{
  size_t n_steps = gait_steps_per_leg.size();
  std::vector<std::array<float, 12>> result_positions;
  result_positions.resize(n_steps);
  /* for each step, legs are ordered front left, front right, back left, back right
  and right now only gait_type supported is intercalated (first 2 opossing legs,then the other 2);
  */ 
  for(size_t i = 0; i < n_steps; i++)
  {
    std::array<float, 3> moving_legs_angles = get_leg_angles(gait_steps_per_leg[i]);
    std::array<float, 3> standby_legs_angles = get_leg_angles(DEFAULT_STANDBY[0]);
    if(i%2 == 0){
      result_positions[i][0] = moving_legs_angles[0];
      result_positions[i][1] = moving_legs_angles[1];
      result_positions[i][2] = moving_legs_angles[2];
      result_positions[i][3] = standby_legs_angles[0];
      result_positions[i][4] = standby_legs_angles[1];
      result_positions[i][5] = standby_legs_angles[2];
      result_positions[i][6] = standby_legs_angles[0];
      result_positions[i][7] = standby_legs_angles[1];
      result_positions[i][8] = standby_legs_angles[2];
      result_positions[i][9] = moving_legs_angles[0];
      result_positions[i][10] = moving_legs_angles[1];
      result_positions[i][11] = moving_legs_angles[2];
    }
    else  {
      result_positions[i][0] = standby_legs_angles[0];
      result_positions[i][1] = standby_legs_angles[1];
      result_positions[i][2] = standby_legs_angles[2];
      result_positions[i][3] = moving_legs_angles[0];
      result_positions[i][4] = moving_legs_angles[1];
      result_positions[i][5] = moving_legs_angles[2];
      result_positions[i][6] = moving_legs_angles[0];
      result_positions[i][7] = moving_legs_angles[1];
      result_positions[i][8] = moving_legs_angles[2];
      result_positions[i][9] = standby_legs_angles[0];
      result_positions[i][10] = standby_legs_angles[1];
      result_positions[i][11] = standby_legs_angles[2];
    }
  }
  return result_positions;
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
  std::vector<std::array<float ,12>> joint_positions;
  float step_time;
  float movement_velocity;

  if(twist.linear.x != 0 && twist.linear.y == 0 && twist.angular.z == 0) {
    if(twist.linear.x > 0) {
      joint_positions = default_forward_;
    }
    else {
      joint_positions = default_backward_;
    }
    movement_velocity = std::abs(twist.linear.x);
  }
  else if(twist.linear.x == 0 && twist.linear.y != 0 && twist.angular.z == 0) {
    if(twist.linear.y > 0) {
      joint_positions = default_left_;
    }
    else {
      joint_positions = default_right_;
    }
    movement_velocity = std::abs(twist.linear.y);
  }
  else if(twist.linear.x == 0 && twist.linear.y == 0 && twist.angular.z != 0) {
    if(twist.angular.z > 0) {
      joint_positions = calculate_joint_positions(DEFAULT_LEFT_SPIN);
    }
    else {
      joint_positions = calculate_joint_positions(DEFAULT_RIGHT_SPIN);;
    }
    movement_velocity = std::abs(twist.angular.z);
  }
  else if(twist.linear.x != 0 && twist.linear.y == 0 && twist.angular.z != 0) {
    joint_positions = default_standby_;
    //Not yet supported but will in the future make a specific gait for this case
  }
  else if((twist.linear.x == 0 && twist.linear.y == 0 && twist.angular.z == 0)) {
    joint_positions = default_standby_;
  }
  else if(twist.linear.x != 0 && twist.linear.y != 0 && twist.angular.z == 0) {
    joint_positions = default_standby_;
    //Not yet supported but will in the future make a specific gait for this case
  }
  else {
    //Not supported
    joint_positions = default_standby_;
  }
  ngait_points_ = joint_positions.size();
  //Aqui puedo quitar el if y hacer que este trozo de funcion valga para ambos casos?
  if(joint_positions != default_standby_) {
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
    for(size_t i = 0; i < njoints_ ; i++) {
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
  if(current_twist_.linear.x != twist->linear.x ||
    current_twist_.linear.y != twist->linear.y ||
    current_twist_.angular.z != twist->angular.z){

    current_twist_ =  *twist;
    current_trajectory_ = get_joint_trajectory(current_twist_);
  }

  //last_twist_ = steady_clock_.now();
  last_twist_ = system_clock_.now();
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
  /*
  if((steady_clock_.now() - last_twist_) > std::chrono::seconds(1)) {
    state_ = STANDBY;
  }
 */ 
 
 if((system_clock_.now() - last_twist_) > 1s) {
    state_ = STANDBY;
  }
  
  if (!action_finished_) {
    return;
  }
  switch (state_) {
    case STANDBY:
      // In standby and no movement required -> send standby_pos
      if(current_twist_ != standby_twist_){
        RCLCPP_INFO(get_logger(), "outdated vel,proceding to standby");
        current_trajectory_ = get_joint_trajectory(standby_twist_);
        current_twist_ = standby_twist_;
        send_request(current_trajectory_);
        state_ = EXECUTING;
      }
      break;

    case WALKING:
      // New twist and anterior has finished -> send current_twist trajectory
      RCLCPP_INFO(get_logger(), "Sending action");
      send_request(current_trajectory_);
      state_ = EXECUTING;
      break;

    case EXECUTING:
      // Waiting action_finished
      break;
  }
  }
}
