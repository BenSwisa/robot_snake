//==============================================================================
//Ben swisa
//bensw@post.bgu.ac.il
//==============================================================================

//=====[ INCULDE ]==============================================================
#include <Arduino.h>
#include <i2c_driver.h>
#include <i2c_driver_wire.h>
#include "RLSencoder2.h"

//============[ constants ]==================================================
#define PPR17 131072.0  //2^17
#define N_enc_joint  2  // number of encoders for each slave
#define TEST_MODE 0 //for manual testing (direct USB connection to MC)

//=====[ Function declaraion ]========================================
void requestEvent();
void my_blink();

//=====[ VARIABLES ]============================================================
RLSencoder2 enc;
uint8_t slave_1 = 101;
float value[2]={0};

//=====[ SETUP ]================================================================
void setup() {
//------- encoder setting -------
//  enc.reset();
//  enc.set_read(); delay(5);
//  enc.start_response(); delay(5);
  enc.begin(); delay(5);
//-------------------------------
  Wire.begin(slave_1);
  delay(10);
  //----for encoder reading ----
  if(TEST_MODE){Serial.begin(115200); while(!Serial);}
  //Serial2.begin(115200);  // Encoder 2 - Y axis
  //Serial3.begin(115200);  // Encoder 1 - Z axis
  //-----------------------
  if(!TEST_MODE)
       Wire.onRequest(requestEvent); //on request from the master send data
}

//=====[ LOOP ]================================================================
void loop() {
  if(TEST_MODE){       
   value[0] = enc.AllowAccess_Y();
   value[1] = enc.AllowAccess_Z();
   }  
   Serial.print(value[0],3); //
   Serial.print(',');
   Serial.print(value[1],3); //
   Serial.print("\n");
//---------------
   enc.get_pos2();//constantly updating axis positions of joints to two global variabels in class
   delay(5);
}

//----- send 2 encoders values to master -----
void requestEvent(){
  
  float value1[2]={0};
  //--- allowing access for the slave to the constantly updated private gloabal variables used by get_pos2 
  value1[0]= enc.AllowAccess_Y(); 
  value1[1]= enc.AllowAccess_Z(); //the normal way to get the accesss
  //---- sends Y axis location then Z axis location that are stored in value[]
  for(int j = 0 ; j <= 1 ; j++){
   byte *data = (byte*)&value1[j]; //sending location to master byte by byte 
    for(int i=0; i<4; i++)
      Wire.write(data[i]);
  }
}







 
