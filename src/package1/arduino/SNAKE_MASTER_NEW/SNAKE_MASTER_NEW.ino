//==============================================================================
//Ben Swisa
//bensw@post.bgu.ac.il
//==============================================================================

//=====[ INCULDE ]==============================================================
#include "RLSencoder2.h" //encoder
#include <Arduino.h>
#include <i2c_driver.h> //teensey wire
#include <i2c_driver_wire.h>

#include <micro_ros_arduino.h> //micro ros
#include <stdio.h>
#include <stdlib.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/multi_array_dimension.h>
#include <std_msgs/msg/multi_array_layout.h>

//============[ constants ]=================
#define PPR17 131072.0  //2^17
#define N_links 3 //as big as the number of slave MC connected + master (N_links=2 means that one slave is connected and so on)
#define N_enc_joint  2 // number of encoders for each slave/master
#define PRINT 0
#define LED_PIN 13
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

//=====[ Function declaraion ]================================================================
float wrapTo180(float val);
void Request_Event(); // request data function
int I2C_ClearBus();
void error_loop();
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void publish_msg();
bool create_entities();
void destroy_entities();

//=====[ VARIABLES ]============================================================
RLSencoder2 enc;
std_msgs__msg__Float32MultiArray msg_sensors;
rcl_publisher_t publisher;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

int T;
int count=0;

float arr[N_links*N_enc_joint] = {0};
uint8_t slave_add[N_links] = {00,100}; //
float joint_offset[N_links*N_enc_joint] = {166.772,289.385,-34.900,100.68,0,0}; //determined at test, each joint will have a different offset
int test;
union u_tag {
  byte b[4];
  float fval;
} u;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

//=====[ SETUP ]================================================================
void setup() {
  
  Serial.begin(115200); 
  while (!Serial);  
  Serial.println("Start"); 
  //--for encoder reading--
  Serial2.begin(115200);  // Encoder 2 - Y axis
  Serial3.begin(115200);  // Encoder 1 - Z axis
  
  //-------- wire setup -------------- 
  Wire.begin();
  Wire.setTimeout(250000); // 250ms default timeout
  
  //------ micro ros setup ---------
  
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
//  digitalWrite(LED_PIN, HIGH); 
  state = WAITING_AGENT;

}

//=====[ LOOP ]================================================================
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
        Request_Event(); // get the data from the encs and insert to arr[]
        publish_msg();
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
//  Request_Event(); // get the data from the encs and insert to arr[]
//  publish_msg();
//  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

//=====================================================================


//-------------- TIMER FUNCTION -------------


void publish_msg(){
  
   //---create msg memory----
   static float memory[N_links*N_enc_joint]; 
   msg_sensors.data.capacity = N_links*N_enc_joint;
   msg_sensors.data.data = memory;
   msg_sensors.data.size = N_links*N_enc_joint;

   //----- copy the data from arr to the msg -----
   for (int i=0;i<N_links*N_enc_joint;i++)
   msg_sensors.data.data[i]=arr[i];  
    
  
    //----- publish the msg -------

    RCSOFTCHECK(rcl_publish(&publisher, &msg_sensors, NULL));
}

//--------=== request data function ===---------
void Request_Event(){
  
  enc.get_pos2(); // manually takes self position from encoders
  arr[0] = enc.AllowAccess_Y(); //placing it at the begining of the array
  arr[1] = enc.AllowAccess_Z();
  arr[0] = wrapTo180(joint_offset[0]-arr[0]); //manually decreceasing the offset values
  arr[1] = joint_offset[1]-arr[1]; 
  //---------- getting data from slaves ----------
  for (int joint_i = 1; joint_i < N_links; joint_i++) {
    int counter=0;
    test = Wire.requestFrom(slave_add[joint_i], sizeof(float)*N_enc_joint); //requesting data from all slave devices
    //delay(4);
    while(Wire.available()<8 && counter<3){
      counter++;
        delay(1);
        Serial.println("wait for bytes");
        }
    if(counter >= 3){
      I2C_ClearBus();
      Wire.begin();
      Serial.println("bus reset");
      counter =0;
    }
      
//    Serial.print("Available bytes: ");
//    Serial.println(Wire.available());
  if (Wire.available()==8)
    for(int j=0 ; j<N_enc_joint ;j++){ //this loop takes in two, 4 byte data streams from slaves
      for(int i=0 ; i<4 ;i++)
        u.b[i]=Wire.read();
      arr[joint_i*2+j] = (joint_offset[joint_i*2+j]- u.fval  );      //^^ adding values from 'u' into arr[] at appropriate location, plus decreacing the appropriate Offset    
      }
  }
  //-----------PRINT--------------
  if (PRINT==1) {
    for (int i = 0; i < N_links*N_enc_joint; i++) {
      Serial.print("val ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(arr[i], 3);
      Serial.print("\t");
    }
    Serial.print("\n");
  }  
} 

//-----------------------------------------------------------------

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "snake_master_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "joint_val_topic"));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  return true;
  
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

float wrapTo180(float val) {
  return (val > 180) ? val - 360.0 : val;
}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}


//------ === I2c comunication reset === -------
int I2C_ClearBus() {
#if defined(TWCR) && defined(TWEN)
  TWCR &= ~(_BV(TWEN)); //Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
#endif
  pinMode(SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(SCL, INPUT_PULLUP);

//  delay(2500);  // Wait 2.5 secs. This is strictly only necessary on the first power
  // up of the DS3231 module to allow it to initialize properly,
  // but is also assists in reliable programming of FioV3 boards as it gives the
  // IDE a chance to start uploaded the program
  // before existing sketch confuses the IDE by sending Serial data.

  boolean SCL_LOW = (digitalRead(SCL) == LOW); // Check is SCL is Low.
  if (SCL_LOW) { //If it is held low Arduno cannot become the I2C master. 
    return 1; //I2C bus error. Could not clear SCL clock line held low
  }

  boolean SDA_LOW = (digitalRead(SDA) == LOW);  // vi. Check SDA input.
  int clockCount = 20; // > 2x9 clock

  while (SDA_LOW && (clockCount > 0)) { //  vii. If SDA is Low,
    clockCount--;
  // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(SCL, INPUT); // release SCL pullup so that when made output it will be LOW
    pinMode(SCL, OUTPUT); // then clock SCL Low
    delayMicroseconds(10); //  for >5us
    pinMode(SCL, INPUT); // release SCL LOW
    pinMode(SCL, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5us
    // The >5us is so that even the slowest I2C devices are handled.
    SCL_LOW = (digitalRead(SCL) == LOW); // Check if SCL is Low.
    int counter = 20;
    while (SCL_LOW && (counter > 0)) {  //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(2);
      SCL_LOW = (digitalRead(SCL) == LOW);
    }
    if (SCL_LOW) { // still low after 2 sec error
      return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    SDA_LOW = (digitalRead(SDA) == LOW); //   and check SDA input again and loop
  }
  if (SDA_LOW) { // still low
    return 3; // I2C bus error. Could not clear. SDA data line held low
  }

  // else pull SDA line low for Start or Repeated Start
  pinMode(SDA, INPUT); // remove pullup.
  pinMode(SDA, OUTPUT);  // and then make it LOW i.e. send an I2C Start or Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
  delayMicroseconds(10); // wait >5us
  pinMode(SDA, INPUT); // remove output low
  pinMode(SDA, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10); // x. wait >5us
  pinMode(SDA, INPUT); // and reset pins as tri-state inputs which is the default state on reset
  pinMode(SCL, INPUT);
  return 0; // all ok
}
