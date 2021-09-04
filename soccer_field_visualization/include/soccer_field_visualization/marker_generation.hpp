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

#ifndef SOCCER_FIELD_VISUALIZATION__MARKER_GENERATION_HPP_
#define SOCCER_FIELD_VISUALIZATION__MARKER_GENERATION_HPP_

#include "soccer_field_msgs/msg/field.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace soccer_field_visualization
{

namespace marker_generation
{

visualization_msgs::msg::MarkerArray createMarkerArray(soccer_field_msgs::msg::Field field);
visualization_msgs::msg::Marker createMarkerFieldOfPlay(geometry_msgs::msg::Polygon fop);
visualization_msgs::msg::Marker createMarker(soccer_field_msgs::msg::LineMarking lineMarking);
visualization_msgs::msg::Marker createMarker(soccer_field_msgs::msg::ArcMarking arcMarking);
visualization_msgs::msg::Marker createMarker(soccer_field_msgs::msg::SpotMarking spotMarking);
visualization_msgs::msg::Marker createDeleteAllActionMarker();

}  // marker_generation

}  // namespace soccer_field_visualization

#endif  // SOCCER_FIELD_VISUALIZATION__MARKER_GENERATION_HPP_
