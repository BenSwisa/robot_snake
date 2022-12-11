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
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

// temporary becouse parameters change - consider malloc
#define enc_per_joint_ 2 
#define strings_per_joint_ 3
#define linked_joints_ 3
 
class Node1 : public rclcpp::Node 
{
public:
    Node1() : Node("node1") 
    {
      get_parameters(); // set and get the parameters
     RCLCPP_INFO(this->get_logger(), "Node1 has been started.");
     // subscriber to encoder current values
     joint_val_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "joint_val_topic", 10, std::bind(&Node1::joint_val_callback, this, _1));
      // sbuscriber to tension values
    tension_val_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "tension_val_topic", 10, std::bind(&Node1::tension_val_callback, this, _1));
      // subscriber to joint angles requested
    joint_cmd_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "joint_cmd_topic", 10, std::bind(&Node1::joint_cmd_callback, this, _1));
      // pwm command publisher to motors
     motor_cmd_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("motor_cmd_topic",10);
    controller_ = this->create_wall_timer(std::chrono::milliseconds((int)(1/controller_freq_*1000)), std::bind(&Node1::controller, this)); //publish motor cmd every second  
     
    }

 
private:

    void joint_val_callback(const std_msgs::msg::Float32MultiArray msg){
      for(int i=0;i<6;i++){
        current_enc_val[i]=msg.data[i];
        }
    }

    void joint_cmd_callback(const std_msgs::msg::Float32MultiArray msg){ 

        for(int i=0;i<6;i++){
        joint_cmd_val[i]=msg.data[i];
        }
        
    }

  // =====================[CONTROLLER]===========================
    void controller(){
      motor_cmd_val[0]=(50*(joint_cmd_val[1]-current_enc_val[1]));
      //  RCLCPP_INFO(this->get_logger(), " %f  \n",motor_cmd_val[0]);
       if(motor_cmd_val[0]>100)
        motor_cmd_val[0]=100;
       else
        if(motor_cmd_val[0]<-100)
        motor_cmd_val[0]=-100;
      auto message = std_msgs::msg::Int32MultiArray();
        message.data={0,0,0,0,0,0,0,0,0,0,0,0};  
        message.data[0]=(int32_t)motor_cmd_val[0];
        motor_cmd_publisher_->publish(message);

    }
    
    void tension_val_callback(const std_msgs::msg::Float32MultiArray msg){
    //  RCLCPP_INFO(this->get_logger(), " %f , %f \n",msg.data[0],msg.data[1]);
      for(int i=0;i<12;i++){
        current_tension_val[i]=msg.data[i];
        }
    }
    
  // =====================[SET PARAMETERS]===========================
    void get_parameters(){ 

      this->declare_parameter("max_pwm",0);
      this->declare_parameter("enc_y",0); 
      this->declare_parameter("enc_z",1);  
      this->declare_parameter("controller_freq",100); 
      this->declare_parameter("kp",std::vector<double>({0.0})); 
      this->declare_parameter("ki",std::vector<double>({0.0})); 
      this->declare_parameter("kd",std::vector<double>({0.0})); 
      controller_freq_ = this->get_parameter("controller_freq").as_int();
      enc_y_ = this->get_parameter("enc_y" ).as_int();
      enc_z_ = this->get_parameter("enc_z" ).as_int();
      max_pwm_ = this->get_parameter("max_pwm").as_int();
      kp_ = this->get_parameter("kp").as_double_array();
      ki_ = this->get_parameter("ki").as_double_array();
      kd_ = this->get_parameter("kd").as_double_array();
      RCLCPP_INFO(this->get_logger(), " %f  \n",kd_[0]);
  }

// =========================[CONSTANTS]=========================================
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_val_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr tension_val_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_cmd_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr motor_cmd_publisher_;
    rclcpp::TimerBase::SharedPtr controller_;
    // int  enc_per_joint_=2;
    // int  linked_joints_=2;
    // int  strings_per_joint_=2;
    int controller_freq_,max_pwm_,enc_z_,enc_y_;
    float joint_cmd_val[enc_per_joint_*linked_joints_]={0}; // [joint*enc_linked] ******* positive val in the direction of pulling the string 
    float current_enc_val[enc_per_joint_*linked_joints_]={0};
    float last_enc_val[enc_per_joint_*linked_joints_]={0};
    float current_tension_val[strings_per_joint_*(linked_joints_+1)]={0}; // adding 1 for now need to be corrected
    float last_tension_val[strings_per_joint_*(linked_joints_+1)]={0};// adding 1 for now need to be corrected
    float motor_cmd_val[strings_per_joint_*(linked_joints_+1)]={0};// adding 1 for now need to be corrected
    std::vector<double>  kp_;
    std::vector<double>  ki_;
    std::vector<double> kd_;
    

};
 
//==========================[MAIN]===========================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Node1>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


