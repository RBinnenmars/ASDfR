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
// limitations under the License.wewewe

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "std_msgs/msg/string.hpp"
#include "asdfr_interfaces/msg/point2.hpp" // 2D point (x and y coordinates)

#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"

using std::placeholders::_1;

using namespace std::chrono_literals;

class BrightnessSetpoint : public rclcpp::Node
{
public:
  BrightnessSetpoint() : Node("brightness_setpoint")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
     "moving_camera_output", 10, std::bind(&BrightnessSetpoint::image_callback, this, _1));
    publisher_ = this->create_publisher<asdfr_interfaces::msg::Point2>("setpoint", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&BrightnessSetpoint::timer_callback, this));

    // parameter to set the brightness threshold while starting the node
    BrightnessThreshold = this->declare_parameter("threshold",128);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Convert the ROS image message to an OpenCV image
    cv::Mat cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    cv::cvtColor(cv_image, cv_image, cv::COLOR_BGR2GRAY);
    cv::threshold(cv_image,cv_image,BrightnessThreshold,255,cv::THRESH_BINARY);
    cv::Moments moments = cv::moments(cv_image, true);

    // Calculate the middle of the binary image, COG of white pixels
    if (moments.m00 != 0){
      double i_width = cv_image.size[1];  // 320
      double i_height = cv_image.size[0]; // 240

      // Calculate the middle of the binary image, COG of white pixels with bottem left beign -x,-y
      cx = (moments.m10 / moments.m00)/i_width-0.5;
      cy = -(moments.m01 / moments.m00)/i_height+0.5;
    }else{
        cx = 0.0;
        cy = 0.0;
    } 
    RCLCPP_INFO(this->get_logger(), "The COG is (%.0f,%.0f)", cx,cy);
    
    // Display the OpenCV image
    // cv::imshow("Received Image", cv_image);
    // cv::waitKey(1);
  }

  void timer_callback()
  {
    auto message = asdfr_interfaces::msg::Point2();
    message.x = cx;
    message.y = cy;

    RCLCPP_INFO(this->get_logger(), "Publishing: x='%.1f', y='%.1f'", cx, cy);
    publisher_->publish(message);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<asdfr_interfaces::msg::Point2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int BrightnessThreshold;
  int Assignment;
  double cx;
  double cy;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BrightnessSetpoint>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

