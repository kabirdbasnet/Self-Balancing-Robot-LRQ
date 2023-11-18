#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "PinChangeInt.h"
#include "MsTimer2.h"
#include <QTRSensors.h>
#include <SPI.h>


#define LED 13
#define PWM_R 5
#define PWM_L 10
#define BIN_1 3 //LEFT
#define BIN_2 2
#define AIN_2 7 //RIGHT
#define AIN_1 8
#define ENCB_CHB A3// LEFT
#define ENCB_CHA A2
#define ENCA_CHB A0// RIGHT
#define ENCA_CHA A1
#define MPU_INT 2//0
#define STBY_PIN 4
#define RPM_radian_converter 2.327

//Navigation Initialization

#define Kp 1.4 
#define Kd 1.8 //( Note: Kp < Kd) 

#define rightMaxSpeed 200 // max speed of the robot
#define leftMaxSpeed 200 // max speed of the robot
int lastError = 0;

//IR-Sensor Initialization
QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];
int16_t position;
//-------------------------------------
// MUX control pins
const int S0 = 12;
const int S1 = 11;
const int S2 = 9;
const int S3 = 6;
// MUX SIG and EN pins
const int muxSIG = A7; // Connected to D3
const int muxEN = 13;  // Connected to D2
//-------------------------------------


MPU6050 mpu;            // AD0 low = 0x68


bool isBalanced;
bool isTrackingLine;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
int16_t gyro[3];        // [x, y, z]            gyro vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

double K_angle,K_angle_dot,K_position,K_position_dot;
double K_angle_AD,K_angle_dot_AD,K_position_AD,K_position_dot_AD;
double position_add,position_dot;
double position_dot_filter;
int speed_real_l,speed_real_r;
int pwm,pwm_l,pwm_r;
int Turn_Need,Speed_Need;

int right_wheel_offset;
int left_wheel_offset;
float angle, angular_rate;
bool blinkState = false;
int rx_count=0;
byte buf_tmp=0;
uint8_t i2cData[14]; // Buffer for I2C data

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

void left_encoder_interrupt()
{
  int state = digitalRead(ENCA_CHA);
  if(digitalRead(ENCA_CHB)) 
  state ? speed_real_l-- : speed_real_l++;
  else 
  state ? speed_real_l++ : speed_real_l--;
}

void right_encoder_interrupt()
{
  int state = digitalRead(ENCB_CHA);
  if(digitalRead(ENCB_CHB)) 
  state ? speed_real_r++ : speed_real_r--;
  else 
  state ? speed_real_r-- : speed_real_r++;
}

void encoder_config()
{
  pinMode(ENCA_CHA, INPUT_PULLUP); // Encoder 1 - Channel A 
  pinMode(ENCA_CHB, INPUT_PULLUP); // Encoder 1 - Channel B

  pinMode(ENCB_CHA, INPUT_PULLUP); // Encoder 2 - Channel A 
  pinMode(ENCB_CHB, INPUT_PULLUP); // Encoder 2 - Channel B

  attachPinChangeInterrupt(ENCB_CHA, right_encoder_interrupt, CHANGE);
  attachPinChangeInterrupt(ENCA_CHA, left_encoder_interrupt, CHANGE);
}


void speed_int_l()
{
  if (digitalRead(ENCA_CHB))
    speed_real_l-=1;
  else
    speed_real_l+=1;
   Serial.print("left: ");
  Serial.print(speed_real_l);


}
void speed_int_r()
{
  if (digitalRead(ENCB_CHB))
    speed_real_r-=1;
  else
    speed_real_r+=1;
   Serial.print(" right: ");
Serial.println(speed_real_r);
}

void initializeNavigation()
{
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(muxEN, OUTPUT);
  // Enable the MUX
  digitalWrite(muxEN, LOW); 
   // Initialize the QTR sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){muxSIG}, SensorCount);
  qtr.setEmitterPin(3);
  delay(500);
}

void right360(int speed)
{
  right_wheel_offset = -speed;
  left_wheel_offset = speed;
}

void left360(int speed)
{
  right_wheel_offset = speed;
  left_wheel_offset = -speed;
}

void rightTurn(int speed)
{
    right_wheel_offset = -speed;
    
}

void leftTurn(int speed)
{
    left_wheel_offset = -speed;
}

void forward(int speed)
{
  Speed_Need = -speed;
}

void backward(int speed)
{
  Speed_Need = speed;
}

void setMuxChannel(uint8_t channel) {
    digitalWrite(S0, bitRead(channel, 0));
    digitalWrite(S1, bitRead(channel, 1));
    digitalWrite(S2, bitRead(channel, 2));
    digitalWrite(S3, bitRead(channel, 3));
}

void getPositionData()
{
  for (uint8_t sensorIndex = 0; sensorIndex < SensorCount; sensorIndex++) {
        // Set MUX channel
        setMuxChannel(sensorIndex);

        // Read the sensor value
        sensorValues[sensorIndex] = analogRead(muxSIG);
    }
    position = qtr.readLineBlack(sensorValues);

    //Print the position
    Serial.print("Line Position: ");
    Serial.print(position);
}

void calculateMotorOffsets()
{
  Speed_Need = -1;
  int error = position - 2500;
  Serial.print(" error: ");
  Serial.print(error);
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  right_wheel_offset = -motorSpeed;
  left_wheel_offset = motorSpeed;

    Serial.print(" left pwm: ");
    Serial.print(left_wheel_offset);
    Serial.print(" right pwm: ");
    Serial.print(right_wheel_offset);

  if (right_wheel_offset > rightMaxSpeed ) right_wheel_offset = rightMaxSpeed;
  if (left_wheel_offset > leftMaxSpeed ) left_wheel_offset = leftMaxSpeed;
  if (right_wheel_offset < 0) right_wheel_offset = 0;
  if (left_wheel_offset < 0) left_wheel_offset = 0; 


}

void navigationControl()
{
  getPositionData();
  calculateMotorOffsets();
}



void setup()
{  
  init_cal();
  init_IO();

  Wire.begin();
  TWBR = 24; 
  Serial.begin(115200);
  initializeNavigation();
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  delay(2);
  
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  
  mpu.setXAccelOffset(-275);
  mpu.setYAccelOffset(390);
  mpu.setZAccelOffset(1454);
  mpu.setXGyroOffset(52);
  mpu.setYGyroOffset(22);
  mpu.setZGyroOffset(-15);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    
    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    //attachInterrupt(MPU_INT, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  
  }
  else
  {

    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  encoder_config();
  delay(5000);
}

void loop()
{

  //if programming failed, don't try to do anything
  if (!dmpReady) return;
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    mpu.resetFIFO();
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    //Get sensor data
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGyro(gyro, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    // angle and angular rate unit: radian
//    angle_X = ypr[2] + 0;                  // 0.017 is center of gravity offset
//    angular_rate_X = -((double)gyro[0]/131.0); // converted to radian
    angle = ypr[1] + 0.07;                   // 0.02 is center of gravity offset
    angular_rate = -((double)gyro[1]/131.0); // converted to radian

    isBalanced = ((angle < 0.15) &&(angle > -0.10)) ? true : false;

    Serial.print(" angle: ");
    Serial.print(angle);
    Serial.print("\t");


    if(isBalanced)
    {
      navigationControl();
    }
    Serial.println();

    control();
    PWM_calculate();
 } 
}

void init_IO()
{
  // configure I/O
  pinMode(ENCA_CHA, INPUT);
  pinMode(ENCB_CHA, INPUT);
  pinMode(ENCA_CHB, INPUT);
  pinMode(ENCB_CHB, INPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(AIN_1, OUTPUT);
  pinMode(AIN_2, OUTPUT);
  pinMode(BIN_1, OUTPUT);
  pinMode(BIN_2, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);
  // configure external interruption
  attachInterrupt(ENCA_CHA, speed_int_l, RISING);
  attachInterrupt(ENCB_CHA, speed_int_r, RISING);
}

void init_cal()
{
  K_angle = 34 * 25.6;
  K_angle_dot = 2 * 25.6;	
  K_position = 0.8 * 0.209;	
  K_position_dot = 1.09 * 20.9;	
}

void control()
{
  if(++rx_count > 200)
  {
    rx_count = 0;
    Speed_Need = 0;
    Turn_Need = 0;
  }
}
void PWM_calculate(void)	
{

  K_angle_AD = -10.69;
  K_angle_dot_AD = -10.41;
  K_position_AD = -5.9;
  K_position_dot_AD = -4.9;

  position_dot = (speed_real_l + speed_real_r)*0.5;  

  position_dot_filter*=0.95;		//
  position_dot_filter+=position_dot*0.05;
  
  position_add+=position_dot_filter;  //
  position_add+=Speed_Need;  //
  	
  if(position_add<-10000)
    position_add=-10000;
  else if(position_add>10000)
    position_add=10000;

  //
  pwm = K_angle * angle * K_angle_AD
      + K_angle_dot * angular_rate * K_angle_dot_AD
      + K_position * position_add * K_position_AD
      + K_position_dot * position_dot_filter * K_position_dot_AD;
  // Serial.print(" K_position: ");
  //   Serial.print(K_position);
  pwm_r = pwm - right_wheel_offset; // negative makes right wheels forwards and vice versa
  pwm_l = pwm - left_wheel_offset;      //negative makes left wheels forward and vice versa
  pwm_out(pwm_l*0.17,pwm_r*0.195); //higher values makes position hold better  each wheel.
  speed_real_l = 0;
  speed_real_r = 0;
}

void pwm_out(int l_val,int r_val)
{
  if (l_val<0)
  {
    digitalWrite(AIN_1, HIGH);
    digitalWrite(AIN_2, LOW);
    l_val=-l_val;
  }
  else
  {
    digitalWrite(AIN_1, LOW);
    digitalWrite(AIN_2, HIGH);
  }
  
  if (r_val<0)
  {
    digitalWrite(BIN_1, HIGH);
    digitalWrite(BIN_2, LOW);
    r_val=-r_val;
  }
  else
  {
    digitalWrite(BIN_1, LOW);
    digitalWrite(BIN_2, HIGH);
  }
  l_val=l_val+6;
  r_val=r_val+6;
  // Serial.print(" l = ");
  // Serial.print(l_val);
  // Serial.print(" r = ");
  // Serial.print(r_val);
  // Serial.print("\n");
  analogWrite(PWM_L, l_val>255? 255:l_val);
  analogWrite(PWM_R, r_val>255? 255:r_val);
}


