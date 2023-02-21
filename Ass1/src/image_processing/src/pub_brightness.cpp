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

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber() : Node("image_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image", 10, std::bind(&ImageSubscriber::image_callback, this, _1));
      BrightnessThreshold = this->declare_parameter("threshold",128);
      Assignment = this->declare_parameter("assignment",2);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Convert the ROS image message to an OpenCV image
    cv::Mat cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    
    cv::cvtColor(cv_image, cv_image, cv::COLOR_BGR2GRAY);

    if (Assignment == 4)
    {
      cv::threshold(cv_image,cv_image,BrightnessThreshold,255,cv::THRESH_BINARY);
      cv::Moments moments = cv::moments(cv_image, true);
      double i_width = cv_image.size[1];  // 320
      double i_height = cv_image.size[0]; // 240

      // Calculate the middle of the binary image, COG of white pixels with bottem left beign -x,-y
      double cx = (moments.m10 / moments.m00)-i_width/2;
      double cy = -(moments.m01 / moments.m00)+i_height/2;
      RCLCPP_INFO(this->get_logger(), "The COG is (%.0f,%.0f), Width is (%.0f,%.0f)", cx,cy,i_width,i_height);
    }
    else{ // Section for Assigment 1.1.2 and 1.1.3
      double brightness = cv::mean(cv_image)[0];
      if (brightness < BrightnessThreshold)
      {
          BrightnessBool = false;
      }
      else
      {
          BrightnessBool = true;
      }
      RCLCPP_INFO(this->get_logger(), "Image brightness is:%f, and bool is: %d", brightness,BrightnessBool);
    }
    // Display the OpenCV image
    cv::imshow("Received Image", cv_image);
    cv::waitKey(1);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  bool BrightnessBool = false;
  int BrightnessThreshold;
  int Assignment;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

