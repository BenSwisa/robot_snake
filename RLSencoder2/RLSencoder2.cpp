#include "RLSencoder2.h"
#define onBoardLED 13
#define PPR17 131072.0		// 17Bits 2^17
#define PPR18 262144.0		// 18Bits 2^18



// Constructor
RLSencoder2::RLSencoder2() {
	//pinMode(onBoardLED, OUTPUT);
}

//Methods
void RLSencoder2::blink(int N) {
	pinMode(13, OUTPUT);
	for (int i = 0; i < N; i++) {
		digitalWrite(onBoardLED, HIGH);
		delay(200);
		digitalWrite(onBoardLED, LOW);
		delay(200);
	}
}
void RLSencoder2::begin(int baudRate) {
	Serial2.begin(115200);
	Serial3.begin(115200);
	delay(10);
	while (!Serial2);
	while (!Serial3);
	//Serial.begin(baudRate);
	//while(!Serial);
	//Serial.println("-->Encoder Started.");
}

void RLSencoder2::get_status() {
	Serial2.flush();
	Serial2.write(0x69); delay(5);
	byte b1 = Serial2.read();
	byte b2 = Serial2.read();
	Serial.print("--> Encoder status: ");
	Serial.print(b1);
	Serial.print(":");
	Serial.print(b2);
	Serial.println(";");
}

void RLSencoder2::unlock() {
	Serial2.write(0xCD); delay(5);
	Serial2.write(0xEF); delay(5);
	Serial2.write(0x89); delay(5);
	Serial2.write(0xAB); delay(5);
	/*/// <summary>
	/// //////---------------------------------------------------------------------------------------
	/// </summary>
	Serial3.write(0xCD); delay(5);
	Serial3.write(0xEF); delay(5);
	Serial3.write(0x89); delay(5);
	Serial3.write(0xAB); delay(5);
	/// //////------------------------------------------- */
}

void RLSencoder2::calibrate() {
	// Start calibration sequence
	Serial.println("--> Starting calibration sequence: ");
	RLSencoder2::unlock();
	Serial2.write(0x41); delay(5);
	Serial.println("--> calibration sequence Ended.");
	RLSencoder2::get_status();
}

void RLSencoder2::set_read() {
	RLSencoder2::unlock();
	Serial2.write(0x54); delay(5);		// Continues Read = 'T'
	Serial2.write(0x01); delay(5);		// Auto-start enable = 1
	Serial2.write(0x33); delay(5);		// Short response = '3'
	Serial2.write(0x00); delay(5);		// Period (250 micro-sec) (byte 8)
	Serial2.write(0xFA); delay(5);		// Period (250 micro-sec) (byte 9)
	Serial.println("--> Encoder Set to Continues read mode '1'.\n\tDont forget to start using 'start_response()'");
	RLSencoder2::save_conf();
	//RLSencoder2::get_status();
	/*///-------------------------------------------------------------------------------------------------------------------------------------
	Serial3.write(0x54); delay(5);		// Continues Read = 'T'
	Serial3.write(0x01); delay(5);		// Auto-start enable = 1
	Serial3.write(0x33); delay(5);		// Short response = '3'
	Serial3.write(0x00); delay(5);		// Period (250 micro-sec) (byte 8)
	Serial3.write(0xFA); delay(5);		// Period (250 micro-sec) (byte 9)
	//------------------------- */
}

void RLSencoder2::start_response() {
	//RLSencoder2::unlock();
	Serial2.write(0x53); delay(5);		// Continues Read = 'S'
	Serial3.write(0x53); delay(5);		// Continues Read = 'S'
	Serial.println("--> Encoder Set to start continues read.");
	RLSencoder2::save_conf();
	//RLSencoder2::get_status();
}

void RLSencoder2::stop_response() {
	//RLSencoder2::unlock();
	Serial2.write(0x50); delay(5);		// Continues Read = 'P'
	Serial.println("--> Encoder Set to stop continues read.");
	RLSencoder2::save_conf();
	//RLSencoder2::get_status();
}

void RLSencoder2::reset() {
	RLSencoder2::unlock();
	Serial2.write(0x72); delay(5);		// reset configuration parameters = 'r'
	Serial.println("--> Encoder reset to factory settings.");
}
int RLSencoder2::Encoder_Pos() {
	return Serial2.parseInt();
}
/*
float RLSencoder2::get_pos(){
	byte b1 = Serial2.read();
	byte b2 = Serial2.read();
	byte b3 = Serial2.read();
	uint32_t pos = b3 + (b2<<8) + (b1<<16);

	pos = (pos>>6);
	float pos_ang = pos*360.0/PPR18;
	return pos_ang;
}
*/
uint32_t RLSencoder2::get_raw_data() {
	byte b1 = Serial2.read();
	byte b2 = Serial2.read();
	byte b3 = Serial2.read();
	uint32_t pos = b3 + (b2 << 8) + (b1 << 16);
	return (pos >> 7);
}

void RLSencoder2::save_conf() {
	RLSencoder2::unlock();
	Serial2.write(0x63); delay(5);		// save configuration parameters = 'c'
	Serial.println("--> Encoders Configurations saved.\nYou have to reset the Encoder.");
	/*//-------------------------------------------------------------------------------------------
	Serial3.write(0x63); delay(5);
	*/
}

/* brief: reads the position (Y,Z axis) from the encoders, translates it into the relevant 17 bits then stores it in given variables
   param: two float type variables */
void RLSencoder2::get_pos(float* Y_axis_val, float* Z_axis_val) {
	byte b2[3] = { 0 };
	byte b3[3] = { 0 };
	long pos;
	// float angleOffset[]={147.64, 41.04}; //adjusted at test, WORKS ONLY WITH 1 SLAVE ATM

	Serial2.flush(); //reads Y axis values (24 bits)
	if (Serial2.available())
		for (int i = 0; i < 3; i++)
			b2[i] = Serial2.read();

	Serial3.flush(); //reads Z axis values (24 bits)
	if (Serial3.available())
		for (int i = 0; i < 3; i++)
			b3[i] = Serial3.read();

	// makes sure only relevant 17 bits are stored at 'pos' variable
	pos = b2[0];
	for (int i = 1; i < 3; i++) {
		pos = (pos << 8);
		pos = pos | b2[i];
	}
	pos = (pos >> 7);

	//converting 17 bit info stored on 'pos' into a coherent angle mesurment and changes the value of entered variable
	*Y_axis_val = (float)(pos * 360.0 / PPR17)/* - angleOffset[Offset_loc]*/;
	if (*Y_axis_val > 180)  //rounds readings because joints cant move more than 180 deg'
		*Y_axis_val -= 360;

	pos = b3[0];
	for (int i = 1; i < 3; i++) {
		pos = (pos << 8);
		pos = pos | b3[i];
	}
	pos = (pos >> 7);

	*Z_axis_val = (float)(pos * 360.0 / PPR17)/* - angleOffset[Offset_loc+1]*/;
	if (*Z_axis_val > 180)
		*Z_axis_val -= 360;

	*Z_axis_val *= -1;

}

/* brief: reads the position (Y,Z axis) from the encoders, translates it into the relevant 17 bits then stores it in global variables declared in class */
void RLSencoder2::get_pos2() {
	byte b2[3] = { 0 };
	byte b3[3] = { 0 };
	long pos;
	// float angleOffset[]={147.64, 41.04}; //adjusted at test, WORKS ONLY WITH 1 SLAVE ATM

	Serial2.flush(); //reads Y axis values (24 bits)
	if (Serial2.available())
		for (int i = 0; i < 3; i++)
			b2[i] = Serial2.read();

	Serial3.flush(); //reads Z axis values (24 bits)
	if (Serial3.available())
		for (int i = 0; i < 3; i++)
			b3[i] = Serial3.read();

	// makes sure only relevant 17 bits are stored at 'pos' variable
	pos = b2[0];
	for (int i = 1; i < 3; i++) {
		pos = (pos << 8);
		pos = pos | b2[i];
	}
	pos = (pos >> 7);

	//converting 17 bit info stored on 'pos' into a coherent angle mesurment and changes the value of entered variable
	Enc_angle[0] = (float)(pos * 360.0 / PPR17);
	if (Enc_angle[0] > 180)  //rounds readings because joints cant move more than 180 deg'
		Enc_angle[0] -= 360;
	 
	pos = b3[0];
	for (int i = 1; i < 3; i++) {
		pos = (pos << 8);
		pos = pos | b3[i];
	}
	pos = (pos >> 7);

	Enc_angle[1] = (float)(pos * 360.0 / PPR17);
	/*if (Enc_angle[1] > 180)
		Enc_angle[1] -= 360;

	Enc_angle[1] *= -1;
	*/
}
//alows access to local private variable that holds the data read from encoder (getpose2) 
float RLSencoder2::AllowAccess_Y() {
	return(Enc_angle[0]);
}

float RLSencoder2::AllowAccess_Z() {
	return(Enc_angle[1]);
}
