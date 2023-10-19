#include "MsTimer2.h"
#include "KalmanFilter.h"
#include "I2Cdev.h"
#include "PinChangeInt.h"

#include "MPU6050.h"
#include "Wire.h"
#define LEFT_PWM_MIN 45
#define RIGHT_PWM_MIN 45
MPU6050 mpu;
KalmanFilter kalmanfilter;

//Setting PID parameters

float test_ax = 0, test_ay =0, test_az =0;
float test_gx =0, test_gy =0, test_gz = 0;
float ax_lp = 0, ay_lp = 0, az_lp =0;
float gx_hp =0, gy_hp =0, gz_hp =0;
#define dT 0.005
#define one_by_dT 200
#define alpha1 0.03
#define f_cut 5
float gx_minusOne = 0, gy_minusOne = 0, gz_minusOne = 0;
float Tau  = 1/(2*PI*f_cut);
float alpha = Tau*(Tau+dT);
extern float current_pitch = 0;

//Setting the state space parameters

extern float pitch = 0, previous_pitch = 0, pitch_dot = 0, pitch_set_point = 0, pitch_dot_set_point = 0;
extern float v[2] = {0};
extern float u[2] = {0};
//angular position | pitch | angular velocity | pitch _velocity
float k[4] ={ 26.1 , -8340.5 , 45.6 , -75.8};// 26.1 , -7940.5 , 55.6 , -75.8}}; -124.3163, -117.1737,  519.8914,  112.6627 // 27.6 , -2621.6 , 79.1 , -98.4 
float k1[2] = {-2,   -0.3};

extern volatile float left_RPM=0, right_RPM=0, left_prev_count=0, right_prev_count=0;

extern volatile float left_encoder_count =0;
extern volatile float right_encoder_count=0;

extern float encoder_set_point =0;
extern float velocity_set_point = 0;

extern float average_theta = 0;
extern float average_RPM = 0;

#define RPM_radian_converter 2.327
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
char balance_angle_min = -22;
char balance_angle_max = 22;

void update_encoder_states()
{
 
   // Make a local copy of the global encoder count
  volatile float left_current_count = left_encoder_count;
  volatile float right_current_count = right_encoder_count;
  
  //                (Change in encoder count) 
  // RPS =   __________________________________________
  //          (Change in time --> 5ms) * (PPR --> 540)
  
  left_RPM  = (float)((left_current_count - left_prev_count)  *  RPM_radian_converter);      
  right_RPM = (float)((right_current_count - right_prev_count)*  RPM_radian_converter);    

  // Store current encoder count for next iteration
  
  left_prev_count = left_current_count;
  right_prev_count = right_current_count;

  average_theta = ((left_current_count + right_current_count)*0.0116); // 2pi/540

  average_RPM = (left_RPM + right_RPM)*0.5; 
}

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

void leftWheelFront(unsigned char speed) {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    analogWrite(PWMA_LEFT, speed);
}

void leftWheelBack(unsigned char speed) {
  digitalWrite(AIN1, 0);
  digitalWrite(AIN2, 1);
  analogWrite(PWMA_LEFT, speed);
}

void rightWheelBack(unsigned char speed) {
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB_RIGHT, speed);
}

void rightWheelFront(unsigned char speed) {
  digitalWrite(BIN1, 1);
  digitalWrite(BIN2, 0);
  analogWrite(PWMB_RIGHT, speed);
}

void update_motors()
{
  u[0] = ( k[0]*(encoder_set_point - average_theta)
          +k[1]*(-pitch) 
          +k[2]*(velocity_set_point-average_RPM)
          +k[3]*(-pitch_dot));
  u[1] = (k1[0] +k1[1]) ;
  v[0] = (0.5* (u[0])); // for right motor
  v[1] = (0.5*(u[0] )); // for left motor
  // Serial.print("avg theta: ");
  // Serial.print(average_theta);

}
void driveMotors()
{
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
      digitalWrite(AIN2, 0);
      analogWrite(PWMA_LEFT, -pwm_left);
    }
    else
    {
      digitalWrite(AIN1, 0);
      digitalWrite(AIN2, 1);
      analogWrite(PWMA_LEFT, pwm_left);
    }
    if (pwm_right < 0)
    {
      digitalWrite(BIN1, 1);
      digitalWrite(BIN2, 0);
      analogWrite(PWMB_RIGHT, -pwm_right);
    }
    else
    {
      digitalWrite(BIN1, 0);
      digitalWrite(BIN2, 1);
      analogWrite(PWMB_RIGHT, pwm_right);
    }
  }
}

void scale_values()
{
  test_ax=(float)(ax)/16384;
  test_ay=(float)(ay)/16384;
  test_az=(float)(az)/16384;
  test_gx=(float)(gx)/131;
  test_gy=(float)(gy)/131;
  test_gz=(float)(gz)/131;
}  

void lowpassfilter(float* ax_addr, float* ay_addr, float* az_addr)
{
  ax_lp=(1-alpha)*(*ax_addr) + (alpha*(ax_lp));
  ay_lp=(1-alpha)*(*ay_addr) + (alpha*(ay_lp));
  az_lp=(1-alpha)*(*az_addr) + (alpha*(az_lp));

  (*ax_addr)=ax_lp;
  (*ay_addr)=ay_lp;
  (*az_addr)=az_lp;

}

void highpassfilter(float* gx, float* gy)
{

  gx_hp = (1-alpha)*gx_hp + (1-alpha)*((*gx) - gx_minusOne);
  gy_hp = (1-alpha)*gy_hp + (1-alpha)*((*gy) - gy_minusOne);
  
  gx_minusOne = *gx;
  gy_minusOne = *gy;


  (*gx)=gx_hp;
  (*gy)=gy_hp;
  
}
float complementary_filter()
{
  
  scale_values();
  lowpassfilter(&test_ax, &test_ay, &test_az);
  highpassfilter(&test_gx, &test_gy);
  
  current_pitch   = current_pitch - (test_gy*dT);
  float acc_angle = atan2(test_ax,sqrt(test_az*test_az))*(180/PI); 

  current_pitch   = (1-alpha1)*current_pitch + (alpha1)*(acc_angle); //complementary filter
  return current_pitch*(PI/180); // converting to radians

}

void balanceRobot()
{
  sei();
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  //kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
  kalmanfilter_angle = complementary_filter();
  Serial.print(kalmanfilter_angle);
  // Serial.print("ax: ");
  // Serial.print(ax);
    Serial.print("\n");
  
  previous_pitch = pitch;
  pitch = kalmanfilter_angle;
  pitch_dot = (pitch - previous_pitch)*100;//100;
  update_encoder_states();
  //implement LQR model
  update_motors();
  
  //utilize the LQR model to obtain the left and right PWM values for the motor.
  pwm_left = v[1];
  pwm_right = v[0];

  //use the left and right pwm values to balance the robot
  pwm_right = map(constrain(v[0] ,-255, 255), 0, -255, RIGHT_PWM_MIN, 255);
  pwm_left = map(constrain(v[1],-255, 255), 0, -255, LEFT_PWM_MIN, 255);
  // pwm_left = constrain(pwm_left, -255, 255);
  // pwm_right = constrain(pwm_right, -255, 255);
  driveMotors();
}

void encoderCountRightA()
{
  encoder_count_right_a++;
}

void encoderCountLeftA()
{
  encoder_count_left_a++;
}

void left_encoder_interrupt()
{
  int state = digitalRead(ENCODER_LEFT_A_PIN);
  if(digitalRead(ENCODER_LEFT_B_PIN)) 
  state ? left_encoder_count-- : left_encoder_count++;
  else 
  state ? left_encoder_count++ : left_encoder_count--;
}

void right_encoder_interrupt()
{
  int state = digitalRead(ENCODER_RIGHT_A_PIN);
  if(digitalRead(ENCODER_RIGHT_B_PIN)) 
  state ? right_encoder_count++ : right_encoder_count--;
  else 
  state ? right_encoder_count-- : right_encoder_count++;
}

//encoder code for statespace
// void left_encoder_interrupt()
// {
//   int state = digitalRead(ENCODER_LEFT_A_PIN);
//   state ? encoder_count_left_a++ : encoder_count_left_a--;
// }
// void right_encoder_interrupt()
// {
//   int state = digitalRead(ENCODER_RIGHT_A_PIN);
//   state ? encoder_count_right_a-- : encoder_count_right_a++;
// }

void encoder_config()
{
  pinMode(ENCODER_LEFT_A_PIN, INPUT_PULLUP); // Encoder 1 - Channel A 
  pinMode(ENCODER_LEFT_B_PIN, INPUT_PULLUP); // Encoder 1 - Channel B

  pinMode(ENCODER_RIGHT_A_PIN, INPUT_PULLUP); // Encoder 2 - Channel A 
  pinMode(ENCODER_RIGHT_B_PIN, INPUT_PULLUP); // Encoder 2 - Channel B

    attachPinChangeInterrupt(ENCODER_RIGHT_A_PIN, right_encoder_interrupt, CHANGE);
  attachPinChangeInterrupt(ENCODER_LEFT_A_PIN, left_encoder_interrupt, CHANGE);

}


void testMPU()
{
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
  Serial.print("x: ");
  Serial.print(ax);
  Serial.print(" y: ");
  Serial.print(ay);
  Serial.print(" z: ");
  Serial.print(az);
  Serial.print("\n");
  delay(10);
}
void set_mpu_offset()
{
    // mpu.setXAccelOffset(-1912);
    // mpu.setYAccelOffset(-5313);  // Only considering the Y-axis for accelerometer due to the x and z issues
    // mpu.setZAccelOffset(2002);
    // mpu.setXGyroOffset(-11);
    // mpu.setYGyroOffset(40);
    // mpu.setZGyroOffset(38);
    // mpu.setXAccelOffset(-2292);
    // mpu.setYAccelOffset(-5519);  // Only considering the Y-axis for accelerometer due to the x and z issues
    // mpu.setZAccelOffset(1860);
    // mpu.setXGyroOffset(-12);
    // mpu.setYGyroOffset(40);
    // mpu.setZGyroOffset(38);

    mpu.setXAccelOffset(-2796);
    mpu.setYAccelOffset(-5397);  // Only considering the Y-axis for accelerometer due to the x and z issues
    mpu.setZAccelOffset(1832);
    mpu.setXGyroOffset(-13);
    mpu.setYGyroOffset(40);
    mpu.setZGyroOffset(39);
}
void carInitialize()
{
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA_LEFT, OUTPUT);
  pinMode(PWMB_RIGHT, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  carStop();

  Wire.begin();
  mpu.initialize();
  set_mpu_offset();

  MsTimer2::set(5, balanceRobot);
  MsTimer2::start();
}
