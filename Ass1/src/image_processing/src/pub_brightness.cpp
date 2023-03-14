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

#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"

using std::placeholders::_1;

using namespace std::chrono_literals;
class PublishBrightness : public rclcpp::Node
{
public:
  PublishBrightness() : Node("pub_brightness")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image", 10, std::bind(&PublishBrightness::image_callback, this, _1));
    bright_publisher_ = this->create_publisher<std_msgs::msg::String>("bright", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&PublishBrightness::timer_callback, this));
    BrightnessThreshold = this->declare_parameter("threshold",128);
  }

private:
   void timer_callback()
   {
      auto message = std_msgs::msg::String();
      if (BrightnessBool)
      {
        message.data = "The image is Bright";
      }else{
        message.data = "The image is NOT Bright";
      }

      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      bright_publisher_->publish(message);
   }


  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    BrightnessThreshold = this->get_parameter("threshold").get_parameter_value().get<int>();

    // Convert the ROS image message to an OpenCV image
    cv::Mat cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    
    cv::cvtColor(cv_image, cv_image, cv::COLOR_BGR2GRAY);

    double brightness = cv::mean(cv_image)[0];
    if (brightness < BrightnessThreshold)
    {
        BrightnessBool = false;
    }
    else
    {
        BrightnessBool = true;
    }
    RCLCPP_INFO(this->get_logger(), "Image brightness is:%f, the threshold is: %d and bool is: %d", brightness,BrightnessThreshold,BrightnessBool);
  
    // Display the OpenCV image?
    cv::imshow("Received Image", cv_image);
    cv::waitKey(1);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bright_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool BrightnessBool = false;
  int BrightnessThreshold;
  int Assignment;
  double cx;
  double cy;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PublishBrightness>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

