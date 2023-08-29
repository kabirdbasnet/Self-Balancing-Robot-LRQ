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
#include "BalanceCar.h"
#include "Rgb.h"
#include "Ultrasonic.h"
#include "voltage.h"
#include "timers.h"

unsigned long start_prev_time = 0;
boolean carInitialize_en = true;



void functionMode()
{
  switch (function_mode)
  {
  case IDLE:
    break;
  case IRREMOTE:
    break;
  case OBSTACLE:
    obstacleAvoidanceMode();
    break;
  case FOLLOW:
    followMode();
    break;
  case BLUETOOTH:
    break;
  case FOLLOW2:
    followMode2();
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
        rgb.lightOff();
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
        rgb.lightOff();
      }
      else
      {
        motion_mode = STOP;
        carStop();
        rgb.brightRedColor();
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
      rgb.lightOff();
      motion_mode = STANDBY;
      break;
    case 'f':
      rgb.flashBlueColorFront();
      motion_mode = FORWARD;
      break;
    case 'b':
      rgb.flashBlueColorback();
      motion_mode = BACKWARD;
      break;
    case 'l':
      rgb.flashBlueColorLeft();
      motion_mode = TURNLEFT;
      break;
    case 'i':
      rgb.flashBlueColorRight();
      motion_mode = TURNRIGHT;
      break;
    case '1':
      function_mode = FOLLOW;
      follow_flag = 0;
      follow_prev_time = millis();
      break;
    case '2':
      function_mode = OBSTACLE;
      obstacle_avoidance_flag = 0;
      obstacle_avoidance_prev_time = millis();
      break;
    case '3':
    rgb_loop:
      key_value = '\0';
      rgb.flag++;
      if (rgb.flag > 6)
      {
        rgb.flag = 1;
      }
      switch (rgb.flag)
      {
      case 0:
        break;
      case 1:
        if (rgb.theaterChaseRainbow(50) && key_value == '3')
          goto rgb_loop;
        break;
      case 2:
        if (rgb.rainbowCycle(20) && key_value == '3')
          goto rgb_loop;
        break;
      case 3:
        if (rgb.theaterChase(127, 127, 127, 50) && key_value == '3')
          goto rgb_loop;
        break;
      case 4:
        if (rgb.rainbow(20) && key_value == '3')
          goto rgb_loop;
        break;
      case 5:
        if (rgb.whiteOverRainbow(20, 30, 4) && key_value == '3')
          goto rgb_loop;
        break;
      case 6:
        if (rgb.rainbowFade2White(3, 50, 50) && key_value == '3')
          goto rgb_loop;
        break;
        break;
      default:
        break;
      }
      break;
    case '4':
      function_mode = IDLE;
      motion_mode = STOP;
      carBack(110);
      delay((kalmanfilter_angle - 30) * (kalmanfilter_angle - 30) / 8);
      carStop();
      start_prev_time = millis();
      rgb.brightRedColor();
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
      rgb.brightness = 50;
      rgb.setBrightness(rgb.brightness);
      rgb.show();
      break;
    case '7':
      rgb.brightRedColor();
      rgb.brightness -= 25;
      if (rgb.brightness <= 0)
      {
        rgb.brightness = 0;
      }
      rgb.setBrightness(rgb.brightness);
      rgb.show();
      break;
    case '8':
      rgb.brightRedColor();
      rgb.brightness += 25;
      if (rgb.brightness >= 255)
      {
        rgb.brightness = 255;
      }
      rgb.setBrightness(rgb.brightness);
      rgb.show();
      break;
    case '9':
      rgb.brightness = 0;
      rgb.setBrightness(rgb.brightness);
      rgb.show();
      break;
    case '0':
      function_mode = FOLLOW2;
      follow_flag = 0;
      follow_prev_time = millis();
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
  ultrasonicInit();
  keyInit();
  rgb.initialize();
  voltageInit();
  start_prev_time = millis();
  //encoder initialization for LQR
  // encoder_config();
  //time initialization for LQR
  // timer1_init();
	// timer5_init();
	// start_timer1();
	// start_timer5();

  carInitialize();
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(PWMA_LEFT, OUTPUT);
  pinMode(PWMB_RIGHT, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);

  digitalWrite(STBY_PIN, HIGH);
}
void loop()
{
   getKeyValue();
   getBluetoothData();
   keyEventHandle();
   getDistance();
   voltageMeasure();
   setMotionState();
   functionMode();
   checkObstacle();
   rgb.blink(100);
  static unsigned long print_time;
  if (millis() - print_time > 100)
  {
    print_time = millis();
    Serial.println(kalmanfilter.angle);
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