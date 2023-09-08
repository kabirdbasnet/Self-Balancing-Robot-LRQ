/*
 * @Description: In User Settings Edit
 * @Author: Kabir Basnet
 * @Date: 2019-10-08 09:35:07
 * @LastEditTime: 2019-10-11 16:25:04
 * @LastEditors: Please set LastEditors
 */
#include "MsTimer2.h"
#include "KalmanFilter.h"
#include "I2Cdev.h"
//#include <iostream>
// #include <ArduinoEigenDense.h>
// #include <ArduinoEigen.h>
//#include "MPU6050_6Axis_MotionApps20.h"

#include "MPU6050.h"
#include "Wire.h"
MPU6050 mpu;
KalmanFilter kalmanfilter;

//Setting PID parameters

double kp_balance = 55, kd_balance = 0.75;
double kp_speed = 10, ki_speed = 0.26;
double kp_turn = 2.5, kd_turn = 0.5;

//Setting the state space parameters
extern float v[2] = {0};
extern float u[2] = {0};
float k[4] ={18.8 ,  -7746.3  , 40.1 ,  -70};//22 ,-7458.4,  40  ,-60.8};

// float chassis_mass = 1, wheels_mass = 1, friction_coeff = 0.1, inertia = 0.0005, g = 9.8, l = 0.125, p = 0 ;

//Setting MPU6050 calibration parameters
double angle_zero = 0;            //x axle angle calibration
double angular_velocity_zero = 0; //x axle angular velocity calibration

volatile unsigned long encoder_count_right_a = 0;
volatile unsigned long encoder_count_left_a = 0;
int16_t ax, ay, az, gx, gy, gz;
float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;
int encoder_left_pulse_num_speed = 0;
int encoder_right_pulse_num_speed = 0;
double speed_control_output = 0;
double rotation_control_output = 0;
double speed_filter = 0;
int speed_control_period_count = 0;
double car_speed_integeral = 0;
double speed_filter_old = 0;
int setting_car_speed = 0;
int setting_turn_speed = 0;
double pwm_left = 0;
double pwm_right = 0;
float kalmanfilter_angle;
// char balance_angle_min = -27;
// char balance_angle_max = 27;
char balance_angle_min = -22;
char balance_angle_max = 22;


void carStop()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(STBY_PIN, HIGH);
  analogWrite(PWMA_LEFT, 0);
  analogWrite(PWMB_RIGHT, 0);
}

void carForward(unsigned char speed)
{
  digitalWrite(AIN1, 0);
  digitalWrite(BIN1, 0);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, speed);
}

void carBack(unsigned char speed)
{
  digitalWrite(AIN1, 1);
  digitalWrite(BIN1, 1);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, speed);
}

void leftWheelBack(unsigned char speed) {
    digitalWrite(AIN1, 1);
    analogWrite(PWMA_LEFT, speed);
}

void leftWheelFront(unsigned char speed) {
  digitalWrite(AIN1, 0);
  analogWrite(PWMA_LEFT, speed);
}

void rightWheelBack(unsigned char speed) {
    digitalWrite(BIN1, 1);
    analogWrite(PWMB_RIGHT, speed);
}

void rightWheelFront(unsigned char speed) {
  digitalWrite(BIN1, 0);
  analogWrite(PWMB_RIGHT, speed);
}

void update_motors()
{
  u[0] = ( k[0]
          +k[1] 
          +k[2]
          +k[3]);
          
  v[0] = (0.5 *(u[0] )); // for right motor
  v[1] = (0.5 *(u[0])); // for left motor
}

void balanceRobot()
{
  sei();
  //initialize the encoder values
  // encoder_left_pulse_num_speed += pwm_left < 0 ? -encoder_count_left_a : encoder_count_left_a;
  // encoder_right_pulse_num_speed += pwm_right < 0 ? -encoder_count_right_a : encoder_count_right_a;
  // encoder_count_left_a = 0;
  // encoder_count_right_a = 0;
  //obtain the values from the MPU6050 and then apply a Kalman Filter to it to obtain filtered value
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
  kalmanfilter_angle = kalmanfilter.angle;
  //implement LQR model
  update_motors();
  //utilize the LQR model to obtain the left and right PWM values for the motor.
  v[0] = (0.5 *(u[0])); // for right motor
  v[1] = (0.5 *(u[0])); // for left motor
  pwm_left = v[1];
  pwm_right = v[0];

  //use the left and right pwm values to balance the robot
  pwm_left = constrain(pwm_left, -255, 255);
  pwm_right = constrain(pwm_right, -255, 255);
  if (motion_mode != START && motion_mode != STOP && (kalmanfilter_angle < balance_angle_min || balance_angle_max < kalmanfilter_angle))
  {
    motion_mode = STOP;
    carStop();
  }

  if (motion_mode == STOP && key_flag != '4')
  {
    car_speed_integeral = 0;
    setting_car_speed = 0;
    pwm_left = 0;
    pwm_right = 0;
    carStop();
  }
  else if (motion_mode == STOP)
  {
    car_speed_integeral = 0;
    setting_car_speed = 0;
    pwm_left = 0;
    pwm_right = 0;
  }
  else
  {
    if (pwm_left < 0)
    {
      digitalWrite(AIN1, 1);
      analogWrite(PWMA_LEFT, -pwm_left);
    }
    else
    {
      digitalWrite(AIN1, 0);
      analogWrite(PWMA_LEFT, pwm_left);
    }
    if (pwm_right < 0)
    {
      digitalWrite(BIN1, 1);
      analogWrite(PWMB_RIGHT, -pwm_right);
    }
    else
    {
      digitalWrite(BIN1, 0);
      analogWrite(PWMB_RIGHT, pwm_right);
    }
  }

}

void balanceCar()
{
  sei();
  encoder_left_pulse_num_speed += pwm_left < 0 ? -encoder_count_left_a : encoder_count_left_a;
  encoder_right_pulse_num_speed += pwm_right < 0 ? -encoder_count_right_a : encoder_count_right_a;
  encoder_count_left_a = 0;
  encoder_count_right_a = 0;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
  kalmanfilter_angle = kalmanfilter.angle;
  
  double balance_control_output = kp_balance * (kalmanfilter_angle - angle_zero) + kd_balance * (kalmanfilter.Gyro_x - angular_velocity_zero);

  speed_control_period_count++;
  if (speed_control_period_count >= 8)
  {
    speed_control_period_count = 0;
    double car_speed = (encoder_left_pulse_num_speed + encoder_right_pulse_num_speed) * 0.5;
    encoder_left_pulse_num_speed = 0;
    encoder_right_pulse_num_speed = 0;
    speed_filter = speed_filter_old * 0.7 + car_speed * 0.3;
    speed_filter_old = speed_filter;
    car_speed_integeral += speed_filter;
    car_speed_integeral += -setting_car_speed;
    car_speed_integeral = constrain(car_speed_integeral, -3000, 3000);
    speed_control_output = -kp_speed * speed_filter - ki_speed * car_speed_integeral;
    rotation_control_output = setting_turn_speed + kd_turn * kalmanfilter.Gyro_z;
  }

  pwm_left = balance_control_output - speed_control_output - rotation_control_output;
  pwm_right = balance_control_output - speed_control_output + rotation_control_output;

  pwm_left = constrain(pwm_left, -255, 255);
  pwm_right = constrain(pwm_right, -255, 255);
  if (motion_mode != START && motion_mode != STOP && (kalmanfilter_angle < balance_angle_min || balance_angle_max < kalmanfilter_angle))
  {
    motion_mode = STOP;
    carStop();
  }

  if (motion_mode == STOP && key_flag != '4')
  {
    car_speed_integeral = 0;
    setting_car_speed = 0;
    pwm_left = 0;
    pwm_right = 0;
    carStop();
  }
  else if (motion_mode == STOP)
  {
    car_speed_integeral = 0;
    setting_car_speed = 0;
    pwm_left = 0;
    pwm_right = 0;
  }
  else
  {
    if (pwm_left < 0)
    {
      digitalWrite(AIN1, 1);
      analogWrite(PWMA_LEFT, -pwm_left);
    }
    else
    {
      digitalWrite(AIN1, 0);
      analogWrite(PWMA_LEFT, pwm_left);
    }
    if (pwm_right < 0)
    {
      digitalWrite(BIN1, 1);
      analogWrite(PWMB_RIGHT, -pwm_right);
    }
    else
    {
      digitalWrite(BIN1, 0);
      analogWrite(PWMB_RIGHT, pwm_right);
    }
  }
}

void encoderCountRightA()
{
  encoder_count_right_a++;
}

void encoderCountLeftA()
{
  encoder_count_left_a++;
}

//encoder code for statespace
void left_encoder_interrupt()
{
  int state = digitalRead(ENCODER_LEFT_A_PIN);
  state ? encoder_count_left_a++ : encoder_count_left_a--;
}
void right_encoder_interrupt()
{
  int state = digitalRead(ENCODER_RIGHT_A_PIN);
  state ? encoder_count_right_a-- : encoder_count_right_a++;
}

void carInitialize()
{
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(PWMA_LEFT, OUTPUT);
  pinMode(PWMB_RIGHT, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  carStop();
  Wire.begin();
  mpu.initialize();
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN), encoderCountLeftA, CHANGE);
  attachPinChangeInterrupt(ENCODER_RIGHT_A_PIN, encoderCountRightA, CHANGE);
  //MsTimer2::set(5, balanceCar);
  MsTimer2::set(5, balanceCar);
  MsTimer2::start();
}
