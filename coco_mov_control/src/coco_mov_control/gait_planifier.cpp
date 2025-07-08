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

#include "coco_mov_control/gait_planifier.hpp"

namespace
{ //Falta revisar si para los servos que estan en posicion contraria el angulo se invierte
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
  DEFAULT_BACKWARD_GAIT
  DEFAULT_LEFT_GAIT
  DEFAULLT_RIGHT_GAIT
  DEFAULT_RIGHT_SPIN
  DEFAULT_LEFT_SPIN
  DEFAULT_STANDBY
}
namespace coco_mov_control
{
using std::placeholders::_1;
using std::placeholders::_2;

GaitPlanifier::GaitPlanifier()
: Node("gait_planifier")
{
  action_client_ = rclcpp_action::create_client<coco_mov_control::GaitPlanifier::FollowJointTrajectory>(
  this, "follow_joint_trajectory");
    
  twist_sub_ = create_subscription<Twist>(
  "cmd_vel", 1, std::bind(&GaitPlanifier::twist_callback, this, _1));
  joint_names_ = {"lf_coax_joint","lf_femur_joint","lf_tibia_joint",
                  "rf_coax_joint","rf_femur_joint","rf_tibia_joint",
                  "lb_coax_joint","lb_femur_joint","lb_tibia_joint",
                  "rb_coax_joint","rb_femur_joint","rb_tibia_joint"};
  state_ = STANDBY;
}
JointTrajectory 
GaitPlanifier::get_joint_trajectory(Twist::UniquePtr twist)
{
  /*The servo angles in rads are obtained by using coco_ik.py given
    the desired position x,y,z in wich we want the foot to be placed
    ,for each keyframe.More info about keyframes i README.md
  */
  JointTrajectory result_gait;
  result_gait.header.stamp = this->get_clock()->now();
  result_gait.header.frame_id = "";
  result_gait.joint_names = joint_names_;
  result_gait.points.resize(12);
  std::vector<std::vector<float>> joint_positions;

  if(twist->linear.x != 0 && twist->linear.y == 0 && twist->angular.z == 0) {
    if(twist->linear.x > 0) {
      joint_positions = DEFAULT_FORWARD_GAIT;
    }
    else {
      joint_positions = DEFAULT_BACKWARD_GAIT;
    }
  }
  else if(twist->linear.x == 0 && twist->linear.y != 0 && twist->angular.z == 0) {
    if(twist->linear.y > 0) {
      joint_positions = DEFAULT_LEFT_GAIT;
    }
    else {
      joint_positions = DEFAULT_RIGHT_GAIT;
    }
  }
  else if(twist->linear.x == 0 && twist->linear.y == 0 && twist->angular.z != 0) {
    if(twist->angular.z > 0) {
      joint_positions = DEFAULT_LEFT_SPIN;
    }
    else {
      joint_positions = DEFAULT_RIGHT_SPIN;
    }
  }
  else if(twist->linear.x != 0 && twist->linear.y == 0 && twist->angular.z != 0) {
    joint_positions = DEFAULT_STANDBY;
    //Not yet supported but will in the future make a specific gait for this case
  }
  else if(twist->linear.x != 0 && twist->linear.y != 0 && twist->angular.z == 0) {
    joint_positions = DEFAULT_STANDBY;
    //Not yet supported but will in the future make a specific gait for this case
  }
  else {
    //Not supported
    return nullptr;
  }

  for (int i = 0; i < 12; i++) {
        result_gait.points[i].positions.resize(12);
        
        // Asignar posiciones desde tu matriz
        for (int j = 0; j < 12; j++) {
            result_gait.points[i].positions[j] = joint_positions[i][j];
        }
        
        // Tiempo: 0.125, 0.250, 0.375, ..., 1.000 segundos
        result_gait.points[i].time_from_start.sec = 0;
        result_gait.points[i].time_from_start.nanosec = (i + 1) * 125000000;
  }

  return result_gait;
}
}