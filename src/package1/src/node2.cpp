//==============================================================================
//Ben Swisa
//bensw@post.bgu.ac.il
//==============================================================================
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;


class Node2 : public rclcpp::Node
{
  public:
    Node2()
    : Node("node2"), count_(0)
    { // servo_cmd_topic
      this->declare_parameter("joint_angle_cmd",std::vector<double>({0.0})); 
      // joint_angle_cmd_ = this->get_parameter("joint_angle_cmd").as_double_array();
      publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("joint_cmd_topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&Node2::timer_callback, this));
      
    }

  private:
    void timer_callback()
    {
      joint_angle_cmd_ = this->get_parameter("joint_angle_cmd").as_double_array();
      auto message = std_msgs::msg::Float32MultiArray();
      message.data = {0,-10,0,0,0,0};
      for (int i=0;i<6;i++){
        message.data[i]=(float)(joint_angle_cmd_[i]);
      }
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    size_t count_;
    std::vector<double>  joint_angle_cmd_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Node2>());
  rclcpp::shutdown();
  return 0;
}