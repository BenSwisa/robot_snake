

//================[INCLUDES]================

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <Arduino.h>

#include <std_msgs/msg/int32_multi_array.h>
//================[DEFINE]================

#define N_links 4
#define PRINT 0
#define Direction 0  //  0 for pull or 1 for release

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define ESTABLISH_RECCONECTIONS 0
#define MAX_TIME_BETWEEN_CALLBACKS 2
//================[FUNC DECLARTIONS]================

void set_motor_pwm(const void * msgin);
void error_loop();


//================[PARAM DECLARTIONS]================
//-------ROS----------
rcl_subscription_t subscriber;
std_msgs__msg__Int32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
//--------------------
byte motors_PWM_pin[N_links*3] = {2,3,4,5,6,7,8,9,10,29,30,35};    
byte motors_DIR_pin[N_links*3] = {0,1,11,12,24,25,26,27,28,31,32,33};
int time_from_last_callback=0;


//===========[SETUP]================================================

void setup() {

  // --------- initialize msg memory -------- 
  static int32_t memory[N_links*3]; 
  msg.data.capacity = N_links*3;
  msg.data.data = memory;
  msg.data.size = N_links*3;
  // --------------------------------------
  
  for (int i=0; i<N_links*3; i++)
  {
    pinMode(motors_PWM_pin[i], OUTPUT);
    pinMode(motors_DIR_pin[i], OUTPUT);
  }
  
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "motor_controller_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "motor_cmd_topic"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &set_motor_pwm, ON_NEW_DATA));
}


//===========[LOOP]==========================
void loop() {

  if(!ESTABLISH_RECCONECTIONS){
    if(time_from_last_callback>MAX_TIME_BETWEEN_CALLBACKS){ 
              for (int i=0; i<N_links*3; i++)
                analogWrite(motors_PWM_pin[i],0);
            }//if nothing was published to the topic for 1 sec than the controller isnt working->stop motors from spinning
            time_from_last_callback++; 
 
  }

   delay(100);        
   RCCHECK( rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100))); 
  
}


//===================[FUNCTIONS]===============

void set_motor_pwm(const void * msgin){ //topic callback function
  time_from_last_callback=0;
  const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;
  int motor_pwm = 0;
  bool motor_dir = 0;
  for (int i=0; i<N_links*3; i++){
    motor_pwm = msg->data.data[i];
    if (motor_pwm>0)
      motor_dir = 1;
    else
      motor_dir = 0;
      
    digitalWrite(motors_DIR_pin[i], motor_dir);
    analogWrite(motors_PWM_pin[i],  abs(motor_pwm));
  }
}



void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}
