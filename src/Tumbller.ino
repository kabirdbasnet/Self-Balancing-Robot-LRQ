


/*
 * @Description: In User Settings Edit
 * @Author: your name
 * @Date: 2019-09-12 14:51:36
 * @LastEditTime: 2019-10-11 16:39:57
 * @LastEditors: Please set LastEditors
 */
#include <Arduino.h>
#include "PinChangeInt.h"
#include "Pins.h"
#include "mode.h"
#include "Command.h"
#include "Balance.h"
#include "timers.h"
unsigned long start_prev_time = 0;
boolean carInitialize_en = true;

void read_tilt_angle()
{
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
  kalmanfilter_angle = kalmanfilter.angle;
  previous_pitch = pitch;
  pitch = kalmanfilter_angle;
  pitch_dot = (pitch - previous_pitch)*100;//100;
  
}
void functionMode()
{
  switch (function_mode)
  {
  case IDLE:
    break;
  case IRREMOTE:
    break;
  case OBSTACLE:
    break;
  case FOLLOW:
    break;
  case BLUETOOTH:
    break;
  case FOLLOW2:
    break;
  default:
    break;
  }
}

void setMotionState()
{
  switch (motion_mode)
  {
  case FORWARD:
    switch (function_mode)
    {
    case FOLLOW:
      setting_car_speed = 20;
      setting_turn_speed = 0;
      break;
    case FOLLOW2:
      setting_car_speed = 20;
      setting_turn_speed = 0;
      break;
    case BLUETOOTH:
      setting_car_speed = 80;
      break;
    case IRREMOTE:
      setting_car_speed = 80;
      setting_turn_speed = 0;
      break;
    default:
      setting_car_speed = 40;
      setting_turn_speed = 0;
      break;
    }
    break;
  case BACKWARD:
    switch (function_mode)
    {
    case FOLLOW:
      setting_car_speed = -20;
      setting_turn_speed = 0;
      break;
    case FOLLOW2:
      setting_car_speed = -20;
      setting_turn_speed = 0;
      break;
    case BLUETOOTH:
      setting_car_speed = -80;
      break;
    case IRREMOTE:
      setting_car_speed = -80;
      setting_turn_speed = 0;
      break;
    default:
      setting_car_speed = -40;
      setting_turn_speed = 0;
      break;
    }
    break;
  case TURNLEFT:
    switch (function_mode)
    {
    case FOLLOW:
      setting_car_speed = 0;
      setting_turn_speed = 50;
      break;
    case FOLLOW2:
      setting_car_speed = 0;
      setting_turn_speed = 50;
      break;
    case BLUETOOTH:
      setting_turn_speed = 80;
      break;
    case IRREMOTE:
      setting_car_speed = 0;
      setting_turn_speed = 80;
      break;
    default:
      setting_car_speed = 0;
      setting_turn_speed = 50;
      break;
    }
    break;
  case TURNRIGHT:
    switch (function_mode)
    {
    case FOLLOW:
      setting_car_speed = 0;
      setting_turn_speed = -50;
      break;
    case FOLLOW2:
      setting_car_speed = 0;
      setting_turn_speed = -50;
      break;
    case BLUETOOTH:
      setting_turn_speed = -80;
      break;
    case IRREMOTE:
      setting_car_speed = 0;
      setting_turn_speed = -80;
      break;
    default:
      setting_car_speed = 0;
      setting_turn_speed = -50;
      break;
    }
    break;
  case STANDBY:
    setting_car_speed = 0;
    setting_turn_speed = 0;
    break;
  case STOP:
    if (millis() - start_prev_time > 1000)
    {
      function_mode = IDLE;
      if (balance_angle_min <= kalmanfilter_angle && kalmanfilter_angle <= balance_angle_max)
      {
        motion_mode = STANDBY;
      }
    }
    break;
  case START:
    if (millis() - start_prev_time > 2000)
    {
      if (balance_angle_min <= kalmanfilter_angle && kalmanfilter_angle <= balance_angle_max)
      {
        car_speed_integeral = 0;
        setting_car_speed = 0;
        motion_mode = STANDBY;
      }
      else
      {
        motion_mode = STOP;
        carStop();
      }
    }
    break;
  default:
    break;
  }
}

void keyEventHandle()
{
  if (key_value != '\0')
  {
    key_flag = key_value;

    switch (key_value)
    {
    case 's':
      motion_mode = STANDBY;
      break;
    case 'f':
      motion_mode = FORWARD;
      break;
    case 'b':
      motion_mode = BACKWARD;
      break;
    case 'l':
      motion_mode = TURNLEFT;
      break;
    case 'i':
      motion_mode = TURNRIGHT;
      break;
    case '1':
      function_mode = FOLLOW;
      break;
    case '2':
      function_mode = OBSTACLE;
      break;
    case '3':
    rgb_loop:
      key_value = '\0';
      break;
    case '4':
      function_mode = IDLE;
      motion_mode = STOP;
      carBack(110);
      delay((kalmanfilter_angle - 30) * (kalmanfilter_angle - 30) / 8);
      carStop();
      start_prev_time = millis();
      break;
    case '5':
      if (millis() - start_prev_time > 500 && kalmanfilter_angle >= balance_angle_min)
      {
        start_prev_time = millis();
        motion_mode = START;
      }
      motion_mode = START;
      break;
    case '6':
      break;
    case '7':
      break;
    case '8':
      break;
    case '9':
      break;
    case '0':
      function_mode = FOLLOW2;
      break;
    case '*':
      break;
    case '#':
      break;
    default:
      break;
    }
    if (key_flag == key_value)
    {
      key_value = '\0';
    }
  }
}

void setup()
{

   Serial.begin(9600);
  // ultrasonicInit();
   keyInit();

  // rgb.initialize();
  encoder_config();
  start_prev_time = millis();
  carInitialize();
  set_mpu_offset();
  carStop();
}
void loop()
{
  getKeyValue();
  getBluetoothData();
  keyEventHandle();
  setMotionState();
  functionMode();
  static unsigned long print_time;
  if (millis() - print_time > 100)
  {
    print_time = millis();
  }
  static unsigned long start_time;
  if (millis() - start_time < 10)
  {
    function_mode = IDLE;
    motion_mode = STOP;
    carStop();
  }
  if (millis() - start_time == 2000) // Enter the pendulum, the car balances...
  {
    key_value = '5';
  }
}
