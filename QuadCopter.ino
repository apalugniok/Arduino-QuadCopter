#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BMP085_U.h>
#include <U8glib.h>
#include <Math.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <MPU6050.h>
#include <Servo.h>
#include <EnableInterrupt.h>

#define SERIAL_PORT_SPEED 9600
#define RC_NUM_CHANNELS  4

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

#define thrust_pin 7 //Define which pins get which signals from the radio receiver
#define yaw_pin 5
#define pitch_pin 4
#define roll_pin 8
#define start_pin 3

#define motor_gaussian_constant 0.20 //Max thrust muliplier that can be given to the motors from the gaussian mapping function 0.5 -> 50% increase
#define roll_differential_constant 0.5
#define pitch_differential_constant 0.5


int thrust;
float yaw;
float pitch;
float roll;
int start;
int extra;

float previous_pitch_error;
float previous_roll_error;

int front_right_thrust=1000; //Declare thrust values for motors specifying initial minimum
int front_left_thrust=1000;
int back_right_thrust=1000;
int back_left_thrust=1000;

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345); //Declare board components
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
//U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NO_ACK);
MPU6050 mpu;

Servo front_right_motor; //Declare servo names
Servo front_left_motor;
Servo back_right_motor;
Servo back_left_motor;

float heading; //Magsensor variable  
float height; //BMP variable

//DMP variables copy, pasted and modified from example sketch

bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, roll, pitch]   yaw/pitch/roll container 




void setup(){
//Serial.begin(SERIAL_PORT_SPEED); //Start serial communication
//  u8g.firstPage();
//  u8g.setFont(u8g_font_unifont);                                 //Possible LCD display code currently not enough program storage 
//  if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {
//    u8g.setColorIndex(255);     // white
//  }
//  else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
//    u8g.setColorIndex(3);         // max intensity
//  }
//  else if ( u8g.getMode() == U8G_MODE_BW ) {
//    u8g.setColorIndex(1);         // pixel on
//  }
//  else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
//    u8g.setHiColorByRGB(255,255,255);
//  }
//  
//  if(!mag.begin())
//  {
//   u8g.drawStr( 0, 22, "HMC Failure!");
//    while(1);
//  }
//  else{
//  u8g.drawStr( 0, 22, "HMC Started!");
//  }
//  
//  if(!bmp.begin())
//  {
//  u8g.drawStr( 24, 46, "BMP Failure!");
//    while(1);
// }
//  else{
// u8g.drawStr( 24, 46, "BMP Started!");
//  }

pinMode(thrust_pin, INPUT);
pinMode(pitch_pin, INPUT);
pinMode(roll_pin, INPUT);
pinMode(yaw_pin, INPUT);

enableInterrupt(thrust_pin, calc_ch1, CHANGE);
enableInterrupt(pitch_pin, calc_ch2, CHANGE);
enableInterrupt(roll_pin, calc_ch3, CHANGE);
enableInterrupt(yaw_pin, calc_ch4, CHANGE);
  
front_right_motor.attach(9); //Attach ESCs to correcponding pins
front_left_motor.attach(10);
back_right_motor.attach(6);
back_left_motor.attach(11);
  
  //DMP startup copy, pasted and modified from example sketch
  
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  mpu.initialize();

    // verify connection
    mpu.testConnection();

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(1123);
    mpu.setYAccelOffset(-371);
    mpu.setZAccelOffset(1658);
    mpu.setXGyroOffset(68);
    mpu.setYGyroOffset(31);
    mpu.setZGyroOffset(55);
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        dmpReady = true;
    

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    
    //Startup of ESCs
    
    front_right_motor.writeMicroseconds(2000); //Give max pulse value
    front_left_motor.writeMicroseconds(2000);
    back_right_motor.writeMicroseconds(2000);
    back_left_motor.writeMicroseconds(2000);
    
    while(pulseIn(start_pin, HIGH)<1400) { //Wait for switch on remote to be flipped
       //Serial.println(pulseIn(start_pin, HIGH));
    }
    
    front_right_motor.writeMicroseconds(1000); //Give minimum
    front_left_motor.writeMicroseconds(1000);
    back_right_motor.writeMicroseconds(1000);
    back_left_motor.writeMicroseconds(1000);
    
    while(pulseIn(thrust_pin, HIGH)>1100) { //Make sure thrust is near minimum before starting main loop
      //Serial.println(pulseIn(thrust_pin, HIGH));
    }
}

void loop(){
//  getMagCompassData();
//  getBMPData();
  getControls();
  getDMPdata();
  setThrust(thrust);
  correctPitch(pitch);
  correctRoll(roll);

  //currently no yaw functionality

// Serial.print(front_left_thrust);
// Serial.print("\t");
// Serial.print(front_right_thrust);
// Serial.print("\t");
// Serial.print(back_left_thrust);
// Serial.print("\t");
// Serial.println(back_right_thrust);
  updateMotors();
}
void getControls(){
  //Gets data from radio receiver and maps onto desired ranges
  rc_read_values();
  thrust = mapfloat(rc_values[RC_CH1],965,1985,1000,1700);
  yaw = rc_values[RC_CH4];
  pitch = mapfloat(rc_values[RC_CH2],978,1974,-30,30); //Max pitch and roll allowed to be set by the remote is 30 degrees 
  roll = mapfloat(rc_values[RC_CH3],987,1981,-30,30);
//    Serial.print("Thrust: ");
//  Serial.print(thrust);
//    Serial.print("\t Yaw: ");
//  Serial.print(yaw);
//    Serial.print("\t Pitch: ");
//  Serial.print(pitch);
//    Serial.print("\t Roll: ");
//  Serial.println(roll);
}

void getDMPdata() //Get data from DMP copy, pasted and modified from example sketch
{    
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    while (fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    } 
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.dmpGetQuaternion(&q, fifoBuffer); 
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetEuler(euler, &q);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//    Serial.print("ypr\t");
//    Serial.print(ypr[0] * 180/M_PI);
//    Serial.print("\t");
//    Serial.print(ypr[1] * 180/M_PI);
//    Serial.print("\t");
//    Serial.println(ypr[2] * 180/M_PI);
//    Serial.print("areal\t");
//    Serial.print(aaReal.x);
//    Serial.print("\t");
//    Serial.print(aaReal.y);
//    Serial.print("\t");
//    Serial.println(aaReal.z);
//    Serial.println(euler[2] * 180/M_PI);
}

void getMagCompassData() //Gets heading from magcompass copy and pasted from example sketch
{
  sensors_event_t event; 
  mag.getEvent(&event);
  heading = atan2(event.magnetic.y, event.magnetic.x);
  const float declinationAngle=0.035;
  heading += declinationAngle;
  if(heading < 0) heading += 2*PI;
  if(heading > 2*PI) heading -= 2*PI;
    
}

void getBMPData() //Gets height from BMP copy and pasted from example sketch
{
  sensors_event_t event;
  bmp.getEvent(&event);
  height=bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA,event.pressure);
}

void updateMotors() //Updates all motors with values they should have after any calculations
{
  front_right_motor.writeMicroseconds(front_right_thrust);
  front_left_motor.writeMicroseconds(front_left_thrust);
  back_right_motor.writeMicroseconds(back_right_thrust);
  back_left_motor.writeMicroseconds(back_left_thrust);
}

void correctPitch(int required_pitch) //Sets motor values in order to correct pitch misalignement using a gaussian mapping. Positive pitch is forward down
{
  float pitch_error = required_pitch - (ypr[2] * 180/M_PI);
  float pitch_error_difference = pitch_error - previous_pitch_error;
  //Serial.println(pitch_error_difference);
  previous_pitch_error = pitch_error;
  float motor_multiplier = motor_gaussian_constant - motor_gaussian_constant * pow(M_E , -1 * pitch_error * pitch_error / 30);
  if(pitch_error < 0){
    motor_multiplier = motor_multiplier * -1;
  }
  motor_multiplier = motor_multiplier + pitch_differential_constant * pitch_error_difference;
  front_right_thrust = front_right_thrust * (1 - motor_multiplier); //front motors spin slower and back faster to correct pitch or vice versa
  front_left_thrust = front_left_thrust * (1 - motor_multiplier);
  back_right_thrust = back_right_thrust * (1 + motor_multiplier);
  back_left_thrust = back_left_thrust * (1 + motor_multiplier);
} 

void correctRoll(int required_roll) //Sets motor values in order to correct pitch misalignement using a gaussian mapping. Positive pitch is forward down
{ 
  float roll_error = required_roll - (ypr[1] * 180/M_PI);
  float roll_error_difference = roll_error - previous_roll_error;
//  Serial.print(roll_error);
//  Serial.print("\t");
  previous_roll_error = roll_error;
  float motor_multiplier = motor_gaussian_constant - motor_gaussian_constant * pow(M_E , -1 * roll_error * roll_error / 30);
  //Serial.println(motor_multiplier);
  if(roll_error < 0){
    motor_multiplier = motor_multiplier * -1;
  }
  motor_multiplier = motor_multiplier + roll_differential_constant * roll_error_difference;
  //Serial.println(motor_multiplier);
  front_right_thrust = front_right_thrust * (1 - motor_multiplier);
  front_left_thrust = front_left_thrust * (1 + motor_multiplier);
  back_right_thrust = back_right_thrust * (1 - motor_multiplier);
  back_left_thrust = back_left_thrust * (1 + motor_multiplier);
} 

void setThrust(int thrust) //Sets thrust of motors before any pitch and roll misalignement correction according to thrust stick
{
  float tilt_boost = 1 / cos(euler[2]); //increases motor speed according to angle to ground normal to stop falling when quadcopter is tilted 
  thrust = thrust * tilt_boost;
  front_right_thrust = thrust;
  front_left_thrust = thrust;
  back_right_thrust = thrust;
  back_left_thrust = thrust;
}

//Functions which calculate input from the remote

void rc_read_values() 
{
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) 
{
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void calc_ch1() {calc_input(RC_CH1, thrust_pin);}
void calc_ch2() {calc_input(RC_CH2, pitch_pin);}
void calc_ch3() {calc_input(RC_CH3, roll_pin);}
void calc_ch4() {calc_input(RC_CH4, yaw_pin);}

double mapfloat(double x, double in_min, double in_max, double out_min, double out_max) //modified map() function to work with floating point numbers
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
