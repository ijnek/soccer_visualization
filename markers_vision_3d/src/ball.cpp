// Copyright 2021 Kenji Brameld
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

#include <memory>
#include "markers_vision_3d/ball.hpp"

#define VISUALISATION_BALL_DIAMETER 0.1  // m

namespace markers_vision_3d
{

Ball::Ball(const rclcpp::NodeOptions & options)
: Node("Ball", options)
{
  publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "visualization/balls", 10);
  subscriber_ = this->create_subscription<soccer_vision_3d_msgs::msg::BallArray>(
    "vision/balls", 1,
    [this](soccer_vision_3d_msgs::msg::BallArray::SharedPtr balls) {
      visualization_msgs::msg::MarkerArray markerArray;
      markerArray.markers.push_back(createDeleteAllActionMarker());
      for (unsigned i = 0; i < balls->balls.size(); ++i) {
        markerArray.markers.push_back(convert(i, balls->header.frame_id, balls->balls.at(i)));
      }
      publisher_->publish(markerArray);
    });
}

visualization_msgs::msg::Marker Ball::convert(
  int markerId, std::string frameId,
  soccer_vision_3d_msgs::msg::Ball & ball)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frameId;
  marker.ns = "";
  marker.id = markerId;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = ball.center.x;
  marker.pose.position.y = ball.center.y;
  marker.pose.position.z = ball.center.z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = VISUALISATION_BALL_DIAMETER;
  marker.scale.y = VISUALISATION_BALL_DIAMETER;
  marker.scale.z = VISUALISATION_BALL_DIAMETER;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  return marker;
}

visualization_msgs::msg::Marker Ball::createDeleteAllActionMarker()
{
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}

}  // namespace markers_vision_3d

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(markers_vision_3d::Ball)
