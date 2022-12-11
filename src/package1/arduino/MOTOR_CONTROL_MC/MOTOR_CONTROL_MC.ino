

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

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\


//================[FUNC DECLARTIONS]================

void set_motor_pwm(const void * msgin);
void error_loop();
bool create_entities();
void destroy_entities();


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

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

//===========[SETUP]================================================

void setup() {

  static int32_t memory[N_links*3]; 
  msg.data.capacity = N_links*3;
  msg.data.data = memory;
  msg.data.size = N_links*3;
  
  
  for (int i=0; i<N_links*3; i++)
  {
    pinMode(motors_PWM_pin[i], OUTPUT);
    pinMode(motors_DIR_pin[i], OUTPUT);
  }
  
//  set_microros_transports();
//  
//  pinMode(LED_PIN, OUTPUT);
//  digitalWrite(LED_PIN, HIGH);  
//  
//  delay(2000);
//
//  allocator = rcl_get_default_allocator();
//
//  //create init_options
//  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
//
//  // create node
//  RCCHECK(rclc_node_init_default(&node, "motor_controller_node", "", &support));
//
//  // create subscriber
//  RCCHECK(rclc_subscription_init_default(
//    &subscriber,
//    &node,
//    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
//    "motor_cmd_topic"));
//
//  // create executor
//  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
//  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &set_motor_pwm, ON_NEW_DATA));

  state = WAITING_AGENT;
}


//===========[LOOP]==========================
void loop() {
   
 switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        delay(1);
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, 1);
  } else {
    digitalWrite(LED_PIN, 0);
  }



  delay(1);
}


//===================[FUNCTIONS]===============

void set_motor_pwm(const void * msgin){ //topic callback function
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

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher, &node);
//  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

bool create_entities()
{
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
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  return true;
  
}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}
