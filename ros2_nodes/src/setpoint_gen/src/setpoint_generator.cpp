#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "asdfr_interfaces/msg/point2.hpp" // 2D point (x and y coordinates)

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class setpointPublisher : public rclcpp::Node
{
  public:
    setpointPublisher()
    : Node("setpoint_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<asdfr_interfaces::msg::Point2>("setpoint", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&setpointPublisher::timer_callback, this));
    }

  private:
    
    const double msg_x[5] = {-0.8, 0.2, -0.4, 0, 0.4};
    const double msg_y[5] = {0.4, -0.8, 0.2, -0.4, 0};
    int i = 0;
    
    void timer_callback()
    {
      auto message = asdfr_interfaces::msg::Point2();
      // Rotate between 5 desired angles every publish
      i = ++i % 5;
      message.x = msg_x[i];
      message.y = msg_y[i];
    
      RCLCPP_INFO(this->get_logger(), "Publishing: x='%.1f', y='%.1f'", message.x, message.y);
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<asdfr_interfaces::msg::Point2>::SharedPtr publisher_;
    size_t count_;

    //void setpoint_topic_callback(const asdfr_interfaces::msg::Point2::SharedPtr msg) 
    ///{
    //  RCLCPP_INFO(this->get_logger(), "Received setpoint: [%f, %f]", msg->x, msg->y);
    //  dynamics_simulation_.set_des(msg->x, msg->y);
    //}
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<setpointPublisher>());
  rclcpp::shutdown();
  return 0;
}