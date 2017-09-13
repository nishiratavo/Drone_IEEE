#include <Servo.h>
#include <TimerOne.h>
#include <Wire.h>
#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SparkFunLSM9DS1.h>

//--------esc calibrator define------------
#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000

//-------- radio controller defines ------------
#define CH1 5
#define CH2 6
#define CH3 7
#define CH4 8

//----------- IMU defines --------------------------
#define LSM9DS1_M 0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 21.58 // Declination (degrees) in Boulder, CO.



Servo esc1, esc2, esc3, esc4;
LSM9DS1 IMU;

// ----------radio controller variables-------------
int climb; // elevation
int roll; // left right
int pitch; // forward backward
int yaw; // turn clockwise counterclockwise

// ---------PID variables---------------
int pid_flag = 1;
float pitch_error;
float roll_error;
float p = 0.5;
int pwm1 = 0;
int pwm2 = 0;
int pwm3 = 0;
int pwm4 = 0;

// --------- IMU variables --------------
float measured_yaw;
float measured_roll;
float measured_pitch;



void ISR_timer() // timer interrupt for sampling time
{
  pid_flag = 1;
}


// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  measured_roll = atan2(ay, az);
  measured_pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  
  if (my == 0)
    measured_yaw = (mx < 0) ? PI : 0;
  else
    measured_yaw = atan2(mx, my);
    
  measured_yaw -= DECLINATION * PI / 180;
  
  if (measured_yaw > PI) measured_yaw -= (2 * PI);
  else if (measured_yaw < -PI) measured_yaw += (2 * PI);
  else if (measured_yaw < 0) measured_yaw += 2 * PI;
  
  // Convert everything from radians to degrees:
  measured_yaw *= 180.0 / PI;
  measured_pitch *= 180.0 / PI;
  measured_roll  *= 180.0 / PI;
  
}


void setup()
{
  Serial.begin(9600);
  esc1.attach(5);
  esc2.attach(9);
  esc3.attach(10);
  esc4.attach(11);
  IMU.settings.device.commInterface = IMU_MODE_I2C;
  IMU.settings.device.mAddress = LSM9DS1_M;
  IMU.settings.device.agAddress = LSM9DS1_AG;
  if (!IMU.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    while (1)
      ;
  }

  //Timer1.initialize(20000); // interrupt every 0.02 second -> 50Hz
  //Timer1.attachInterrupt(ISR_timer);
  //esc1.attach(5);
  //esc2.attach(9);
  //esc3.attach(10);
  //esc4.attach(11);
  esc1.writeMicroseconds(MIN_SIGNAL);
  esc2.writeMicroseconds(MIN_SIGNAL);
  esc3.writeMicroseconds(MIN_SIGNAL);
  esc4.writeMicroseconds(MIN_SIGNAL);
  delay(3000);
  esc1.writeMicroseconds(MAX_SIGNAL);
  esc2.writeMicroseconds(MAX_SIGNAL);
  esc3.writeMicroseconds(MAX_SIGNAL);
  esc4.writeMicroseconds(MAX_SIGNAL);

  //Timer1.initialize(20000); // interrupt every 0.02 second -> 50Hz
  //Timer1.attachInterrupt(ISR_timer);
}
 
void loop()
{
  // --------------- radio controller data aquisition ---------------------
  //yaw = pulseIn(CH1,HIGH); //min-max -> 1041-1846
  climb = pulseIn(CH2, HIGH); //min-max -> 1138-1939
  //roll = pulseIn(CH4, HIGH); //min-max -> 1030-1843
  //pitch = pulseIn(CH3, HIGH); //min-max -> 1050-1854

  climb = map(climb,1138,1939,1000, 1500);

  // ------------- PID algorithm ----------------------
  if(pid_flag == 1)
  {
    IMU.readAccel();
    IMU.readMag();
    printAttitude(IMU.ax, IMU.ay, IMU.az, -IMU.my, -IMU.mx, IMU.mz);
    roll_error = measured_roll - 0;
    pitch_error = measured_pitch - 0;
    pwm1 = pwm1 + p*roll_error + p*pitch_error;
    pwm2 = pwm2 - p*roll_error - p*pitch_error;
    pwm3 = pwm3 + p*roll_error - p*pitch_error;
    pwm4 = pwm4 - p*roll_error + p*pitch_error;
    if (pwm1 > 500)
    {
      pwm1 = 500;
    }
    if (pwm1 < 0)
    {
      pwm1 = 0;
    }
    if (pwm2 > 500)
    {
      pwm2 = 500;
    }
    if (pwm2 < 0)
    {
      pwm2 = 0;
    }
    if (pwm3 > 500)
    {
      pwm3 = 500;
    }
    if (pwm3 < 0)
    {
      pwm3 = 0;
    }
    if (pwm4 > 500)
    {
      pwm4 = 500;
    }
    if (pwm4 < 0)
    {
      pwm4 = 0;
    }
    esc1.writeMicroseconds(climb + pwm1);
    esc2.writeMicroseconds(climb + pwm2);
    esc3.writeMicroseconds(climb + pwm3);
    esc4.writeMicroseconds(climb + pwm4);
    // needs to see which motor does what and IMU position
    //pwm = pwm -p*error;
    pid_flag = 0;
  }

  // ----------------------------------------------------
  /*climb = map(elevation, 968, 1950, 0, 160);
  roll = map(leftRight, 900, 1900, -22, 22);
  pitch = map(fowBack, 900, 1900, -22, 22);*/

  Serial.print("esc1 = ");
  Serial.print(climb + pwm1);
  Serial.print("  ");
  
  Serial.print("esc2 = ");
  Serial.print(climb + pwm2);
  Serial.print("  ");

  Serial.print("esc3 = ");
  Serial.print(climb + pwm3);
  Serial.print("  ");

  Serial.print("esc4 = ");
  Serial.print(climb + pwm4);
  Serial.print("  ");

  Serial.print("roll_error = ");
  Serial.print(roll_error);
  Serial.print("  ");

  Serial.print("pitch_error = ");
  Serial.println(pitch_error);


  
  /*esc1.write(elevation + leftRight - fowBack);
  esc2.write(elevation + leftRight + fowBack);
  esc3.write(elevation - leftRight + fowBack);
  esc4.write(elevation - leftRight - fowBack);*/
}