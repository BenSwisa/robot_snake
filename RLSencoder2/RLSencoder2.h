#ifndef tl
#define tl

#if (arduino >=100)
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif


class RLSencoder2 {
	public:
		// Constructor
		RLSencoder2(); 
		
		// Methods
		void begin(int baudRate=115200);
		void blink(int N);
		void get_status();
		void unlock();
		void calibrate();
		void set_read();
		void start_response();
		void stop_response();
		void reset();
		int  Encoder_Pos();
		void save_conf();
		//float get_pos();
		uint32_t get_raw_data();
		void get_pos(float* Y_axis_val, float* Z_axis_val);
		void get_pos2();
		float AllowAccess_Y();
		float AllowAccess_Z();
	private:
		float Enc_angle[2] = { 0 }; //[0] var will hold Y axis, [1] var will hold Z axis
	
};
#endif