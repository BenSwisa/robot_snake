#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;
//==============================================================================
//Ben Swisa
//bensw@post.bgu.ac.il
//==============================================================================

class Node2 : public rclcpp::Node
{
  public:
    Node2()
    : Node("minimal_publisher"), count_(0)
    { // servo_cmd_topic
      publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("joint_cmd_topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&Node2::timer_callback, this));
      
    }

  private:
    void timer_callback()
    {
  
      auto message = std_msgs::msg::Float32MultiArray();
      message.data = {0,-10,0,0,0,0};
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data[0]);
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    size_t count_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Node2>());
  rclcpp::shutdown();
  return 0;
}