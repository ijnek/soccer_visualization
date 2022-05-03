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
#include "markers_vision_3d/marking_array.hpp"

namespace markers_vision_3d
{

MarkingArray::MarkingArray(const rclcpp::NodeOptions & options)
: Node("MarkingArray", options)
{
  publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "visualization/markings", 10);
  subscriber_ = this->create_subscription<soccer_vision_3d_msgs::msg::MarkingArray>(
    "vision/markings", 1,
    [this](soccer_vision_3d_msgs::msg::MarkingArray::SharedPtr markingArray) {
      visualization_msgs::msg::MarkerArray markerArray;
      markerArray.markers.push_back(createDeleteAllActionMarker());
      for (unsigned i = 0; i < markingArray->segments.size(); ++i) {
        markerArray.markers.push_back(
          convert(i, markingArray->header.frame_id, markingArray->segments.at(i)));
      }
      publisher_->publish(markerArray);
    });
}

visualization_msgs::msg::Marker MarkingArray::convert(
  int markerId, std::string frameId,
  soccer_vision_3d_msgs::msg::MarkingSegment & markingSegment)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frameId;
  marker.ns = "";
  marker.id = markerId;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.points.push_back(markingSegment.start);
  marker.points.push_back(markingSegment.end);

  marker.scale.x = 0.05;    // See SPL rule book, line width must be 50mm
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  return marker;
}

visualization_msgs::msg::Marker MarkingArray::createDeleteAllActionMarker()
{
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}

}  // namespace markers_vision_3d

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(markers_vision_3d::MarkingArray)
