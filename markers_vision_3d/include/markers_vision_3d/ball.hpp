// Copyright 2022 Kenji Brameld
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

#ifndef MARKERS_3D_VISION__BALL_HPP_
#define MARKERS_3D_VISION__BALL_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "soccer_vision_3d_msgs/msg/ball_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace markers_vision_3d
{

class Ball : public rclcpp::Node
{
public:
  Ball(const rclcpp::NodeOptions & options);

private:
  visualization_msgs::msg::Marker convert(
    int markerId, std::string frameId,
    soccer_vision_3d_msgs::msg::Ball & ball);

  visualization_msgs::msg::Marker createDeleteAllActionMarker();

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::Subscription<soccer_vision_3d_msgs::msg::BallArray>::SharedPtr subscriber_;
};

}  // namespace markers_vision_3d

#endif  // MARKERS_3D_VISION__BALL_HPP_
