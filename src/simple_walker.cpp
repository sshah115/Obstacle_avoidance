// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

using SCAN = sensor_msgs::msg::LaserScan;
using TWIST = geometry_msgs::msg::Twist;

/**
 * @brief Class definition to make turtlebot walk while
 * avoiding obstacles in vicinity.
 */
class Turtlewalk : public rclcpp::Node {
 public:
  Turtlewalk() : Node("walker") {
    auto callback = std::bind(&Turtlewalk::scan_callback, this, _1);

    scan_data_sub = this->create_subscription<SCAN>("scan", 10, callback);
    twist_vel_pub = this->create_publisher<TWIST>("cmd_vel", 10);
  }

 private:
    /**
     * @brief callback function to get data from
     * scan result and publish command accordingly.
     * @param result 
     */
  void scan_callback(const SCAN &result) {
    if (result.header.stamp.sec == 0) {

      return;

    }
    auto scanResult = result.ranges;
    for (int i = 330; i < 330 + 60; i++) {

      if (scanResult[i % 360] < 0.8) {

        turtle_move(0.0, 0.2);

      } else {

        turtle_move(0.2, 0.0);
      }
    }
  }
    /**
     * @brief Moving the turtlebot in desired direction as
     * per scan data
     * @param xMove 
     * @param zMove 
     */
  void turtle_move(float xMove, float zMove) {

    auto moveDir = TWIST();
    moveDir.linear.x = xMove;
    moveDir.angular.z = -zMove;
    twist_vel_pub->publish(moveDir);
  }

  rclcpp::Subscription<SCAN>::SharedPtr scan_data_sub;
  rclcpp::Publisher<TWIST>::SharedPtr twist_vel_pub;
};
/**
 * @brief main function.
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtlewalk>());
  rclcpp::shutdown();
  return 0;

}