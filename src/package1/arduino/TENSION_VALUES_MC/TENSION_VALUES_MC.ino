//==============================================================================
//Ben Swisa
//bensw@post.bgu.ac.il
//==============================================================================

//=====[ INCULDE ]==========================================

#include "HX711-multi.h"
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <math.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/multi_array_dimension.h>
#include <std_msgs/msg/multi_array_layout.h>



//=====[ Constants ]========================================
#define PRINT 0
#define N_joints 4
#define CLK 11      // clock pin to the load cell amp
#define DOUT1 A0    // data pin to the first lca
#define DOUT2 A1    // data pin to the second lca
#define DOUT3 A2    // data pin to the third lca
#define DOUT4 A3    // data pin to the third lca
#define DOUT5 A4    // data pin to the third lca
#define DOUT6 A5    // data pin to the third lca
#define DOUT7 A6    // data pin to the third lca
#define DOUT8 A7    // data pin to the third lca
#define DOUT9 A8    // data pin to the third lca
#define DOUT10 A9    // data pin to the third lca
#define DOUT11 A12
#define DOUT12 A13
#define DOUT13 A14
#define DOUT14 A15
#define DOUT15 A16
#define DOUT16 A17
#define DOUT17 A18
#define DOUT18 A19
#define DOUT19 A20
#define DOUT20 25
#define BOOT_MESSAGE "MIT_ML_SCALE V0.8"
#define d_count 100
#define LED_PIN 13
#define N_tensions 12


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

//=====[ Function declaraion ]===========================================================
void error_loop();
void publish_msg();

//=====[ VARIABLES ]=========================================

byte DOUTS[N_tensions] = {DOUT1, DOUT2, DOUT3, DOUT4, DOUT5, DOUT6, DOUT7, DOUT8, DOUT9, DOUT10, DOUT11, DOUT12};
int k;
long int result[N_tensions];
float arr[N_tensions]={0};
HX711MULTI scales(N_joints*3, DOUTS, CLK);

//corret values
//
//   double P[12][3] =  {{1.726E-11, 5.0222E-06, 0.71137},
//                    {9.3052E-12, 1.1501E-05,1.5 -1.2368},
//                    {8.5654E-12, 1.403E-05,2.5 -2.1273},
//                    {-1.0379E-12, 2.3016E-05, -6.1681},
//                    {6.8068E-12, 1.5016E-05, -1.0202},
//                    {5.4685E-12, 1.6701E-05, -2.0988},
//                    {7.5975E-12, 1.3514E-05,1.5 -1.5006},
//                    {1.814E-11, 1.3612E-05, 1.1696},
//                    {6.8773E-12, 1.1903E-05, -1.2288},
//                    {7.8529E-12, 1.2574E-05, -1.9809},
//                    {5.9427E-12, 1.2468E-05, -1.4726},
//                    {5.0598E-12, 1.6667E-05, -2.2929}}; 

// ------ corrected while motor 7 and 1 are switched -----------

double P[12][3] =  {{7.5975E-12, 1.3514E-05,1.5 -1.5006},
                    {9.3052E-12, 1.1501E-05,1.5 -1.2368},
                    {8.5654E-12, 1.403E-05,2.5 -2.1273},
                    {-1.0379E-12, 2.3016E-05, -6.1681},
                    {6.8068E-12, 1.5016E-05, -1.0202},
                    {5.4685E-12, 1.6701E-05, -2.0988},
                    {1.726E-11, 5.0222E-06, 0.71137},
                    {1.814E-11, 1.3612E-05, 1.1696},
                    {6.8773E-12, 1.1903E-05, -1.2288},
                    {7.8529E-12, 1.2574E-05, -1.9809},
                    {5.9427E-12, 1.2468E-05, -1.4726},
                    {5.0598E-12, 1.6667E-05, -2.2929}};  

//----------------------------------------------------------------                                                           
rcl_publisher_t publisher;
std_msgs__msg__Float32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

//=====[ SETUP ]================================================================

void setup() {

  
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "tension_reader_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "tension_val_topic"));

  //---create msg memory----
   static float memory[N_tensions]; 
   msg.data.capacity = N_tensions;
   msg.data.data = memory;
   msg.data.size = N_tensions;

 

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

}

//=====[ LOOP ]================================================================
void loop() {
  delay(1);
  publish_msg();
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

//=====[ FUNCTIONS ]================================================================
void publish_msg()
{  
 
    scales.readRaw(result);             // Read raw data from all the tension-meters
    
  for (int i=0; i<(N_joints*3); i++)  // --  Convert RAW data to Kg
    msg.data.data[i] = P[i][0]*pow(result[i],2) + P[i][1]*result[i] + P[i][2];

   //----- publish the msg -------

   RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL)); 
}



void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}
