#include <Servo.h>
#include <TimerOne.h>
#include <Wire.h>
#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SparkFunLSM9DS1.h>
#include "GY_85.h"
#include <PID_v1.h>


//-------ESCs signals------------
#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1148

//-------- radio controller defines ------------
#define CH1 2
#define CH2 4
#define CH3 8
#define CH4 12



//-----------------------------------------------


// ----------radio controller variables-------------
int throttle; // motor velocity
int roll; // left right
int pitch; // forward backward
int yaw; // turn clockwise counterclockwise

//-------IMU variables------
GY_85 GY85; 
int ax,ay,az;
float ax_g, ay_g, az_g;
float A = 0.1;
float filtered_roll = 0;
float filtered_pitch = 0;

//---------------ESCs variables--------------------
float esc1_vel,esc2_vel,esc3_vel,esc4_vel;
Servo esc1, esc2, esc3, esc4;

//---------------PID variables----------------------
double Setpoint, Input, Output;
PID x_axis(&Input, &Output, &Setpoint,50,0,0, DIRECT);


//-----------------------------------------------------

void attitude(float ax, float ay, float az)
{
  float measured_roll = atan2(ay, az);
  float measured_pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  measured_pitch *= 180.0 / PI;
  measured_roll  *= 180.0 / PI;
  
  filtered_roll = A*measured_roll + (1-A)*filtered_roll;
  filtered_pitch = A*measured_pitch + (1-A)*filtered_pitch;
  
  //Serial.print("Roll: ");
  //Serial.println(filtered_roll);  
  //Serial.print("      ");
  //Serial.print("Pitch: ");
  //Serial.print(filtered_pitch);  
  //Serial.println();  
}

void setup()
{
    Serial.begin(9600);
    pinMode(2,INPUT);

    // --------esc/pwm config-----------
    esc1.attach(3);
  	esc2.attach(5);
  	esc3.attach(6);
  	esc4.attach(9);

  	/*
    esc1.writeMicroseconds(MAX_SIGNAL); 
  	esc2.writeMicroseconds(MAX_SIGNAL);
  	esc3.writeMicroseconds(MAX_SIGNAL);
  	esc4.writeMicroseconds(MAX_SIGNAL);

  	delay(3);

  	esc1.writeMicroseconds(MIN_SIGNAL); 
  	esc2.writeMicroseconds(MIN_SIGNAL);
  	esc3.writeMicroseconds(MIN_SIGNAL);
  	esc4.writeMicroseconds(MIN_SIGNAL);

  	delay(3);
  	*/

    //------------IMU init-------------
    Wire.begin();
    delay(10);
    Serial.begin(9600);
    delay(10);
    GY85.init();
    delay(10);

  	//------------PID config------------
  	Setpoint = 0;
    x_axis.SetMode(AUTOMATIC);
    x_axis.SetSampleTime(10);
    x_axis.SetOutputLimits(-1000,1000);
    //----------------------------------


}




void loop()
{
	/*if (Serial.available())
	{
		data = Serial.read();

		switch(data)
		{
			case 48 : 
				esc4.writeMicroseconds(MIN_SIGNAL);
				esc3.writeMicroseconds(MIN_SIGNAL);
				esc2.writeMicroseconds(MIN_SIGNAL);
				esc1.writeMicroseconds(MIN_SIGNAL);
				Serial.println("0");
			break;

			case 49 :
				esc4.writeMicroseconds(1200);
				esc3.writeMicroseconds(1200);
				esc2.writeMicroseconds(1200);
				esc1.writeMicroseconds(1200);
				Serial.println("1");
			break;

			case 50 :
				esc4.writeMicroseconds(1400);
				esc3.writeMicroseconds(1400);
				esc2.writeMicroseconds(1400);
				esc1.writeMicroseconds(1400);
				Serial.println("2");
			break;

			case 51 :
				esc4.writeMicroseconds(1600);
				esc3.writeMicroseconds(1600);
				esc2.writeMicroseconds(1600);
				esc1.writeMicroseconds(1600);
				Serial.println("3");
			break;

			case 52 :
				esc4.writeMicroseconds(1800);
				esc3.writeMicroseconds(1800);
				esc2.writeMicroseconds(1800);
				esc1.writeMicroseconds(1800);
				Serial.println("4");
			break;

			case 53 :
				esc4.writeMicroseconds(MAX_SIGNAL);
				esc3.writeMicroseconds(MAX_SIGNAL);
				esc2.writeMicroseconds(MAX_SIGNAL);
				esc1.writeMicroseconds(MAX_SIGNAL);
				Serial.println("5");
			break;
		}
	}*/

	//-------------------joystick-------------------------------------
	// pulseIn(pin,HIGH) -> returns the length of the pulse in microseconds

	throttle = pulseIn(CH2,HIGH); // return joystick data for throttle
	if (digitalRead(2) == LOW)
	{
		esc1.writeMicroseconds(throttle); 
	  	esc2.writeMicroseconds(throttle);
	  	esc3.writeMicroseconds(throttle);
	  	esc4.writeMicroseconds(throttle);
	}

	//------------------------------------------------------------------


	else
	{

		//----------------accel data------------------------------
		ax = GY85.accelerometer_x( GY85.readFromAccelerometer() );
	    ay = GY85.accelerometer_y( GY85.readFromAccelerometer() );
	    az = GY85.accelerometer_z( GY85.readFromAccelerometer() );

	    ax_g = ax/128.0;
	    ay_g = ay/128.0;
	    az_g = az/128.0;
	    attitude(ax_g, ay_g, az_g);

	    //--------------PID--------------------------------------
	    Input = (double)filtered_roll;

	    if(x_axis.Compute())
	    {
	    	esc1_vel = (float)throttle + Output;
	    	esc2_vel = (float)throttle - Output;

	    	if (esc1_vel > MAX_SIGNAL)
		    {
		    	esc1_vel = MAX_SIGNAL;
		    }
		    if (esc1_vel < MIN_SIGNAL)
		    {
		    	esc1_vel = MIN_SIGNAL;
		    }

		    if (esc2_vel > MAX_SIGNAL)
		    {
		    	esc2_vel = MAX_SIGNAL;
		    }
		    if (esc2_vel < MIN_SIGNAL)
		    {
		    	esc2_vel = MIN_SIGNAL;
		    }

	    }

	    Serial.print(esc1_vel);
	    Serial.print("     ");
	    Serial.print(esc2_vel);
	    Serial.print("     ");
	    Serial.print(Output);
	    Serial.print("    ");
	    Serial.println(filtered_roll);


	    //-----------------ESCs---------------------------------

		// sends velocity to esc
		esc1.writeMicroseconds(esc1_vel); 
	  	esc2.writeMicroseconds(esc2_vel);
	  	esc3.writeMicroseconds(esc3_vel);
	  	esc4.writeMicroseconds(esc4_vel);
	  	//-------------------------------------------------------

	}


}