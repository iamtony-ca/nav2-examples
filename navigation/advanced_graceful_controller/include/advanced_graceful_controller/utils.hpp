// Copyright (c) 2023 Alberto J. Tudela Roldán
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

#ifndef ADVANCED_GRACEFUL_CONTROLLER__UTILS_HPP_
#define ADVANCED_GRACEFUL_CONTROLLER__UTILS_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace advanced_graceful_controller
{
/**
   * @brief Create a flat circle marker of radius slowdown_radius around the motion target for
   * debugging / visualization porpuses.
   *
   * @param motion_target Motion target
   * @param slowdown_radius Radius of the slowdown circle
   * @return visualization_msgs::msg::Marker Slowdown marker
   */
visualization_msgs::msg::Marker createSlowdownMarker(
  const geometry_msgs::msg::PoseStamped & motion_target, const double & slowdown_radius);

}  // namespace advanced_graceful_controller

#endif  // ADVANCED_GRACEFUL_CONTROLLER__UTILS_HPP_
