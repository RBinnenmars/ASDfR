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

#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"

using std::placeholders::_1;

class MovingCameraOutputSubscriber : public rclcpp::Node
{
public:
  MovingCameraOutputSubscriber() : Node("moving_output")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "moving_camera_output", 10, std::bind(&MovingCameraOutputSubscriber::image_callback, this, _1));
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Convert the ROS image message to an OpenCV image
    cv::Mat cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    
    RCLCPP_INFO(this->get_logger(), "Recieved an image");
    // Display the OpenCV image
    cv::imshow("Received Image", cv_image);
    cv::waitKey(1);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MovingCameraOutputSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

