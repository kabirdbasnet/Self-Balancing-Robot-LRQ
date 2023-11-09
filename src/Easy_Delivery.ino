#include <Wire.h>
#include "I2C.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "PinChangeInt.h"

#define LED 13
#define PWM_L 5
#define PWM_R 10
#define DIR_L1 3
#define DIR_L2 2
#define DIR_R1 7
#define DIR_R2 8
#define SPD_INT_L A3//1
#define SPD_PUL_L A2
#define SPD_INT_R A0//7
#define SPD_PUL_R A1
#define MPU_INT 2//0
#define STBY_PIN 4
#define RPM_radian_converter 2.327


MPU6050 mpu;            // AD0 low = 0x68


bool isBalanced;
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
  int state = digitalRead(SPD_INT_L);
  if(digitalRead(SPD_PUL_L)) 
  state ? speed_real_l-- : speed_real_l++;
  else 
  state ? speed_real_l++ : speed_real_l--;
}

void right_encoder_interrupt()
{
  int state = digitalRead(SPD_INT_R);
  if(digitalRead(SPD_PUL_R)) 
  state ? speed_real_r++ : speed_real_r--;
  else 
  state ? speed_real_r-- : speed_real_r++;
}

void encoder_config()
{
  pinMode(SPD_INT_L, INPUT_PULLUP); // Encoder 1 - Channel A 
  pinMode(SPD_PUL_L, INPUT_PULLUP); // Encoder 1 - Channel B

  pinMode(SPD_INT_R, INPUT_PULLUP); // Encoder 2 - Channel A 
  pinMode(SPD_PUL_R, INPUT_PULLUP); // Encoder 2 - Channel B

  attachInterrupt(SPD_INT_R, right_encoder_interrupt, CHANGE);
  attachInterrupt(SPD_INT_L, left_encoder_interrupt, CHANGE);
}


void speed_int_l()
{
  if (digitalRead(SPD_PUL_L))
    speed_real_l-=1;
  else
    speed_real_l+=1;
   Serial.print("left: ");
  Serial.print(speed_real_l);


}
void speed_int_r()
{
  if (digitalRead(SPD_PUL_R))
    speed_real_r-=1;
  else
    speed_real_r+=1;
   Serial.print(" right: ");
Serial.println(speed_real_r);
}


void setup()
{  
  init_cal();
  init_IO();

  Wire.begin();
  Wire.setClock(400000);
  //TWBR = 24; 
  //Serial.begin(9600);
  
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
  
  mpu.setXAccelOffset(-2769);
  mpu.setYAccelOffset(-5516);
  mpu.setZAccelOffset(1841);
  mpu.setXGyroOffset(-7);
  mpu.setYGyroOffset(31);
  mpu.setZGyroOffset(39);
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
}

void loop()
{
  // if programming failed, don't try to do anything
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
    angle = ypr[1] + 0.06;                   // 0.02 is center of gravity offset
    angular_rate = -((double)gyro[1]/131.0); // converted to radian
    Serial.print(" angl: ");
    Serial.println(angle);
    control();
    PWM_calculate();
  } 
}

void init_IO()
{
  // configure I/O
  pinMode(SPD_INT_L, INPUT);
  pinMode(SPD_INT_R, INPUT);
  pinMode(SPD_PUL_L, INPUT);
  pinMode(SPD_PUL_R, INPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_L1, OUTPUT);
  pinMode(DIR_L2, OUTPUT);
  pinMode(DIR_R1, OUTPUT);
  pinMode(DIR_R2, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);
  // configure external interruption
  attachInterrupt(SPD_INT_L, speed_int_l, RISING);
  attachInterrupt(SPD_INT_R, speed_int_r, RISING);
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

  K_angle_AD = -6.01;
  K_angle_dot_AD = -7.51;
  K_position_AD = -2.9;
  K_position_dot_AD = -2.5;

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
      
  pwm_r = pwm;
  pwm_l = pwm;
  pwm_out(pwm_l*0.143,pwm_r*0.143);
  speed_real_l = 0;
  speed_real_r = 0;
}

void pwm_out(int l_val,int r_val)
{
  if (l_val<0)
  {
    digitalWrite(DIR_L1, HIGH);
    digitalWrite(DIR_L2, LOW);
    l_val=-l_val;
  }
  else
  {
    digitalWrite(DIR_L1, LOW);
    digitalWrite(DIR_L2, HIGH);
  }
  
  if (r_val<0)
  {
    digitalWrite(DIR_R1, HIGH);
    digitalWrite(DIR_R2, LOW);
    r_val=-r_val;
  }
  else
  {
    digitalWrite(DIR_R1, LOW);
    digitalWrite(DIR_R2, HIGH);
  }
  l_val=l_val+6;
  r_val=r_val+6;
  analogWrite(PWM_L, l_val>255? 255:l_val);
  analogWrite(PWM_R, r_val>255? 255:r_val);
}



