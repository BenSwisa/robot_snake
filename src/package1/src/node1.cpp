//==============================================================================
//Ben Swisa
//bensw@post.bgu.ac.il
//==============================================================================
/*TODO:
[ ]check if pid on string 2 is working
[x]check if mirco ros nodes are stiil running , if not ,shut dowm with output
[x]check the new motor micro_controller node
[ ]change tension vals in mc node
*/
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
//TODO change to parameters
#define LINKED_MOTORS 3 
#define MAX_TIME_BETWEEN_CALLBACKS 5 // in seconds
#define CONSTANT_TENSION_ON_STRING_2 0.8
#define TENSION_MAX_IMPULSE 10
 

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

  /*=====================[JOINT VAL CALLBACK]===========================
    this function gets the current angles from the joint_val_topic and inserting them into the
    array current_enc_val[]
  */
    void joint_val_callback(const std_msgs::msg::Float32MultiArray msg){
      for(int i=0;i<enc_per_joint_*linked_joints_;i++){
        current_enc_val[i]=msg.data[i];
        }
        joint_time_between_callbacks=0;
    }
  
  /*=====================[JOINT CMD CALLBACK]===========================
    this function gets the wanted angles from the joint_cmd_topic and inserting them into the
    array joint_cmd_val[]
  */
    void joint_cmd_callback(const std_msgs::msg::Float32MultiArray msg){ 

        for(int i=0;i<6;i++){
        joint_cmd_val[i]=msg.data[i];
        }
        
    }

  /* =====================[CONTROLLER]===========================
    ***** this function is the pid controller *****
  */
    void controller(){
      
      // checking for response from encoder values and tension values if no response for some time than stop the node
        if(joint_time_between_callbacks>1000*MAX_TIME_BETWEEN_CALLBACKS*controller_freq_||
           tension_time_between_callbacks>1000*MAX_TIME_BETWEEN_CALLBACKS*controller_freq_){
            RCLCPP_ERROR(this->get_logger(), "NO REPONSE FROM MICRO CONTROLLERS");
            rclcpp::shutdown();
        } // FIXME not working properly   
        joint_time_between_callbacks++;
        tension_time_between_callbacks++;     

      for(int i=0;i<LINKED_MOTORS;i++){ //loop for motor 1 & 3
        last_error[i]=error[i];
        switch(i%3){
          case 0: error[i]=joint_cmd_val[enc_z_]-current_enc_val[enc_z_];
                  break;
          case 1: error[i]=CONSTANT_TENSION_ON_STRING_2-current_tension_val[i];
                  break;
          case 2: error[i]=joint_cmd_val[enc_y_]-current_enc_val[enc_y_];
                  break;
          default: break;
        }
        if(isErrorSameSign(error[i],last_error[i])==false) //if error changed sign integrator should be 0 so we wont get windup
          error_sum_[i]=0;
        if(saturation_flag[i]==false) //while we are in saturation dont sum the integrator so we wont get windup
          error_sum_[i]+=error[i];
        motor_cmd_val[i]=kp_[i]*(error[i])+ // pid controll to publish the right motor cmd
                        kd_[i]*(error[i]-last_error[i])+ 
                        ki_[i]*error_sum_[i];
        if(motor_cmd_val[i]>max_pwm_){ //motor command limitation
          motor_cmd_val[i]=max_pwm_;
          saturation_flag[i]=1;
        }
        else
          if(motor_cmd_val[i]<-max_pwm_){
            motor_cmd_val[i]=-max_pwm_;
            saturation_flag[i]=1;
          }
          else // we are not saturated
            saturation_flag[i]=0;
      }

      //----- INITIALIZE MSG ------
      auto message = std_msgs::msg::Int32MultiArray(); 
      message.data={0,0,0,0,0,0,0,0,0,0,0,0};  
      for(int i=0;i<3;i++)
       message.data[i]=(int32_t)motor_cmd_val[i];
       
      //  -----TENSION CHECK-------
       for(int i=0;i<LINKED_MOTORS;i++){
         if(current_tension_val[i]>4 ){
          RCLCPP_ERROR(this->get_logger(), " TENSION TOO HIGH %f \n",current_tension_val[0]);
          for(int j=0;j<LINKED_MOTORS;j++)
            message.data[j]=-50;
        }
       }  
      
        motor_cmd_publisher_->publish(message); //publish pwm output to motors

    }

/*=====================[isErrorSameSign]===========================
    this function gets the current and last error and checks if the error flipped direction
    for instance if the robot was before the desired angle and then passed the angle  
  */
    int isErrorSameSign(float error,float last_error){
     if(error>0 && last_error>0)
      return 1;
     if(error<0 && last_error<0)
      return 1;
    return 0;
    }
  
  /*=====================[TENSION CALLBACK]===========================
    this function gets the tension from the tension_val_topic and inserting them into the
    array current_tension_val[] 
  */
    void tension_val_callback(const std_msgs::msg::Float32MultiArray msg){ 
    //  RCLCPP_INFO(this->get_logger(), " %f , %f, %f \n",msg.data[0],msg.data[1],msg.data[2]); 
      for(int i=0;i<strings_per_joint_*(linked_joints_+1);i++){
        if(abs(msg.data[i]-current_tension_val[i])<TENSION_MAX_IMPULSE) // tension gauge has some errors and and can return a big impulse 
        current_tension_val[i]=msg.data[i];
        } 
        tension_time_between_callbacks=0;
    } // TODO set the new tension constants to get better values
  
  /* =====================[GET PARAMETERS]===========================
  this function declares the parameters and then taking their value from a yaml file
  - max pwm: is the pwm limit to send to the motors
  - enc_y/z: spot in the array that represents enc values rotating around y/z axis 
  --   axes are :
          y
          ^
          |
          -----> x : the direction for going forward along the snake
  -controller_freq : frequency to run the controller
  - kp,ki,kd : arrays of constants for pid controll. position is as the same as for the motor_cmd array 
  */
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

/* =========================[CONSTANTS]=========================================
enc_axis : in the controller func you need to know witch axis you are getting info from -> enc_y/z
error: wanted position - current position
error_sum_: suming the error values for integral controller
joint_cmd_val: aray of angles to get to
current_enc_val: array of angle of the cuurent position
motor_cmd_val: pwm outputs for the motors
saturation_flag: when saturated we need to stop summing the error  
*/
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_val_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr tension_val_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_cmd_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr motor_cmd_publisher_;
    rclcpp::TimerBase::SharedPtr controller_;
    int controller_freq_,max_pwm_,enc_z_,enc_y_,enc_axis;
    int joint_time_between_callbacks=0;
    int tension_time_between_callbacks=0;
    float error[strings_per_joint_*(linked_joints_+1)]={0}; 
    float last_error[strings_per_joint_*(linked_joints_+1)]={0};
    float joint_cmd_val[enc_per_joint_*linked_joints_]={0}; // [joint*enc_linked] ******* if looking at the arm from the front positive enc val is up and right
    float current_enc_val[enc_per_joint_*linked_joints_]={0};
    float last_enc_val[enc_per_joint_*linked_joints_]={0};
    float current_tension_val[strings_per_joint_*(linked_joints_+1)]={0}; // adding 1 for now need to be corrected
    float last_tension_val[strings_per_joint_*(linked_joints_+1)]={0};// adding 1 for now need to be corrected
    float motor_cmd_val[strings_per_joint_*(linked_joints_+1)]={0};// adding 1 for now need to be corrected
    float error_sum_[strings_per_joint_*(linked_joints_+1)]={0};
    int saturation_flag[strings_per_joint_*(linked_joints_+1)]={0};
    std::vector<double>  kp_;
    std::vector<double>  ki_;
    std::vector<double> kd_;

};
 

void shutdown_callback()
{
       RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node1 has been stopped.");
} 

//==========================[MAIN]===========================
int main(int argc, char **argv)
{
    rclcpp::on_shutdown(shutdown_callback);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Node1>(); 
    rclcpp::spin(node);
    rclcpp::shutdown(); 

 
    return 0;
}


