#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

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
      this->declare_parameter("joint__angle_cmd",std::vector<double>({0.0})); 
      joint__angle_cmd = this->get_parameter("joint__angle_cmd").as_double_array();

      publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("joint_cmd_topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&Node2::timer_callback, this));
      
    }

  private:
    void timer_callback()
    {
  
      auto message = std_msgs::msg::Float32MultiArray();
      message.data = {0,-10,0,0,0,0};
      for (int i=0;i<6;i++){
        message.data[i]=(float)(joint__angle_cmd[i]);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", joint__angle_cmd[i]);
      }
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data[0]);
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    size_t count_;
    std::vector<double>  joint__angle_cmd;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Node2>());
  rclcpp::shutdown();
  return 0;
}