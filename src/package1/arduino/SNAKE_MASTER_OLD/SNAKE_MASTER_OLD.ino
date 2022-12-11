

//==============================================================================
//Ben Swisa
//bensw@post.bgu.ac.il
//==============================================================================

//=====[ INCULDE ]==============================================================
#include "RLSencoder2.h"
#include <Arduino.h>
#include <i2c_driver.h>
#include <i2c_driver_wire.h>

//============[ constants ]=================
#define PPR17 131072.0  //2^17
#define N_links 2 //as big as the number of slave MC connected (N_links=2 means that one slave is connected and so on)
#define N_enc_joint  2 // number of encoders for each slave
#define PRINT 1

//=====[ Function declaraion ]================================================================
float wrapTo180(float val);
void Request_Event(); // request data function
int I2C_ClearBus();
//=====[ VARIABLES ]============================================================
//-------ros--------
//ros::NodeHandle nh;
//std_msgs::Float32MultiArray joint_ang;
//ros::Publisher pub_joints("/robot_snake_1/joint_val", &joint_ang);
//-----------------
RLSencoder2 enc;

int T;
int count=0;

float arr[N_links*N_enc_joint] = {0};
uint8_t slave_add[N_links] = {00,100}; //
float joint_offset[N_links*N_enc_joint] = {0}; //determined at test, each joint will have a different offset
int test;
union u_tag {
  byte b[4];
  float fval;
} u;

//=====[ SETUP ]================================================================
void setup() {
  //------------------
  //enc.begin();
  //enc.set_read();
  //enc.start_response();
  //nh.getHardware()->setBaud(115200);
  //nh.initNode();
  //nh.advertise(pub_joints);
  //joint_ang.data_length = N_links;
  //------------------
  Serial.begin(115200); 
  while (!Serial);  
  Serial.println("Start"); 
  //--for encoder reading--
  Serial2.begin(115200);  // Encoder 2 - Y axis
  Serial3.begin(115200);  // Encoder 1 - Z axis
  //-------- wire setup -------------- 
  Wire.begin();
  
  Wire.setTimeout(250000); // 250ms default timeout
  //-------------------------
//  T = millis(); // FOR HERTZ
}

//=====[ LOOP ]================================================================
void loop() {
//  if(count==100){  // FOR HERTZ
//    Serial.print(millis()-T);
//    Serial.print("\n");
//    T = millis();
//    count=0;
//  }
  count++;
  Request_Event();
  delay(5);
}

//=====[ Functions ]========================================
float wrapTo180(float val) {
  return (val > 180) ? val - 360.0 : val;
}

//--------=== request data function ===---------
void Request_Event(){
  int counter=0;
  enc.get_pos2(); // manually takes self position from encoders
  arr[0] = enc.AllowAccess_Y(); //placing it at the begining of the array
  arr[1] = enc.AllowAccess_Z();
  arr[0] -= joint_offset[0]; //manually decreceasing the offset values
  arr[1] -= joint_offset[1]; 
  //---------- getting data from slaves ----------
  for (int joint_i = 1; joint_i < N_links; joint_i++) {
    test = Wire.requestFrom(slave_add[joint_i], sizeof(float)*N_enc_joint); //requesting data from all slave devices
    //delay(4);
    while(Wire.available()<8 && counter<3){
      counter++;
        delay(1);
        Serial.println("wair for bytes");
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
      arr[joint_i*2+j] = wrapTo180(u.fval - joint_offset[joint_i*2+j]);      //^^ adding values from 'u' into arr[] at appropriate location, plus decreacing the appropriate Offset    
      }
  }
  //-----------======PRINT=====------------
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
//--------------ros------------------
//  joint_ang.data = arr;
//  pub_joints.publish(&joint_ang);
//  nh.spinOnce();
//-------------------------------
}


///////////////////////////////////////////////////////////////
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
