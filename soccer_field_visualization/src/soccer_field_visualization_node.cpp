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

#include "rclcpp/rclcpp.hpp"
#include "soccer_field_msgs/msg/field.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "soccer_field_visualization/marker_generation.hpp"

namespace soccer_field_visualization
{

class SoccerFieldVisualizatonNode : public rclcpp::Node
{
public:
  SoccerFieldVisualizatonNode()
  : Node("SoccerFieldVisualizatonNode")
  {
    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "visualization/field", rclcpp::QoS(1).transient_local());

    subscription_ = this->create_subscription<soccer_field_msgs::msg::Field>(
      "field", rclcpp::QoS(1).transient_local(),
      [this](soccer_field_msgs::msg::Field::SharedPtr field) {
        RCLCPP_DEBUG(get_logger(), "Received Field msg");
        auto markerArray = marker_generation::createMarkerArray(*field);
        publisher_->publish(markerArray);
        RCLCPP_DEBUG(get_logger(), "Published Marker Array");
      });
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::Subscription<soccer_field_msgs::msg::Field>::SharedPtr subscription_;
};

}  // namespace soccer_field_visualization

int
main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<soccer_field_visualization::SoccerFieldVisualizatonNode>());
  rclcpp::shutdown();
  return 0;
}
