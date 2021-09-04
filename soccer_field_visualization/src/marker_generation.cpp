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

#include <tuple>
#include <cmath>
#include <iostream>

#include "soccer_field_visualization/marker_generation.hpp"

namespace soccer_field_visualization
{

namespace marker_generation
{

visualization_msgs::msg::MarkerArray createMarkerArray(soccer_field_msgs::msg::Field field)
{
  visualization_msgs::msg::MarkerArray markerArray;

  int id = 0;

  // Delete all markers, we have a new field.
  markerArray.markers.push_back(createDeleteAllActionMarker());

  // Convert field of play
  {
    auto marker = createMarkerFieldOfPlay(field.field_of_play);
    marker.id = id++;
    markerArray.markers.push_back(marker);
  }

  // Convert markings
  for (auto lineMarking : field.markings.lines) {
    auto marker = createMarker(lineMarking);
    marker.id = id++;
    markerArray.markers.push_back(marker);
  }

  for (auto arcMarking : field.markings.arcs) {
    auto marker = createMarker(arcMarking);
    marker.id = id++;
    markerArray.markers.push_back(marker);
  }
  
  for (auto spotMarking : field.markings.spots) {
    auto marker = createMarker(spotMarking);
    marker.id = id++;
    markerArray.markers.push_back(marker);
  }


  return markerArray;
}

visualization_msgs::msg::Marker createMarkerFieldOfPlay(geometry_msgs::msg::Polygon fop)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.ns = "";
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  for (auto surfacePoint : fop.points) {
    geometry_msgs::msg::Point point;
    point.x = surfacePoint.x;
    point.y = surfacePoint.y;
    point.z = surfacePoint.z;
    marker.points.push_back(point);
  }

  // Add first point again, so line strip encloses the polygon.
  if (fop.points.size() > 0) {
    geometry_msgs::msg::Point point;
    point.x = fop.points.at(0).x;
    point.y = fop.points.at(0).y;
    point.z = fop.points.at(0).z;
    marker.points.push_back(point);
  }

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.10;  // something wide enough to be able to see
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  return marker;
}

visualization_msgs::msg::Marker createMarker(soccer_field_msgs::msg::LineMarking lineMarking)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.ns = "";
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.points.push_back(lineMarking.start);
  marker.points.push_back(lineMarking.end);

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = lineMarking.line_width;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  return marker;
}

visualization_msgs::msg::Marker createMarker(soccer_field_msgs::msg::ArcMarking arcMarking)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.ns = "";
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  const static float maxAngleBetweenPoints = 0.2;
  int numPoints = (arcMarking.heading_end - arcMarking.heading_start) / maxAngleBetweenPoints + 1;
  float anglePerPoint = (arcMarking.heading_end - arcMarking.heading_start) / numPoints;

  for (auto [i, angle] = std::tuple{0, arcMarking.heading_start}; i < numPoints + 1;
    i++, angle += anglePerPoint)
  {
    geometry_msgs::msg::Point point;
    point.x = arcMarking.center.x + arcMarking.radius * cos(angle);
    point.y = arcMarking.center.y + arcMarking.radius * sin(angle);
    point.z = arcMarking.center.z;
    marker.points.push_back(point);
  }

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = arcMarking.line_width;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  return marker;
}

visualization_msgs::msg::Marker createMarker(soccer_field_msgs::msg::SpotMarking spotMarking)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.ns = "";
  marker.type = visualization_msgs::msg::Marker::CYLINDER;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position = spotMarking.center;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = spotMarking.radius * 2;
  marker.scale.y = spotMarking.radius * 2;
  marker.scale.z = 0.0001;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  return marker;
}

visualization_msgs::msg::Marker createDeleteAllActionMarker()
{
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}

}  // namespace marker_generation

}  // namespace soccer_field_visualization
