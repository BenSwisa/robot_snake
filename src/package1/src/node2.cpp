//==============================================================================
//Ben Swisa
//bensw@post.bgu.ac.il
//==============================================================================
// this node publishes the dersired joint angles 
//============================================================================
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
      publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("joint_cmd_topic", 10); //publisher to joint_cmd_topic
      timer_ = this->create_wall_timer(
      500ms, std::bind(&Node2::timer_callback, this)); //timer to publish the message
      
    }

  private:
    void timer_callback()
    {
      joint_angle_cmd_ = this->get_parameter("joint_angle_cmd").as_double_array();
      auto message = std_msgs::msg::Float32MultiArray();
      message.data = {0,0,0,0,0,0}; //random data to innitialize msg size
      for (int i=0;i<6;i++){
        message.data[i]=(float)(joint_angle_cmd_[i]);
      }
      publisher_->publish(message);
    }

    // ---------- constants ------------------
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    size_t count_;
    std::vector<double>  joint_angle_cmd_;

};

int main(int argc, char * argv[]) // main function to spin the node
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Node2>());
  rclcpp::shutdown();
  return 0;
}