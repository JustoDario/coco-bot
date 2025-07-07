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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace coco_mov_control
{
  class GaitPlanifier : public rclcpp::Node
  {
    public:
      GaitPlanifier();
    private:
    
  }
}



#endif // COCO_MOV_CONTROL__GAIT_PLANIFIER_HPP_
