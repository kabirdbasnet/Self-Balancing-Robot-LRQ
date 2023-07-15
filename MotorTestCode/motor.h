#ifndef Motor_H
#define Motor_H
#include "Arduino.h"
enum MotorCommand {Forward, Reverse, Brake, Coast};  // available motor commands
class Motor {
   char _name[21];         // motor name (20 characters max)
   MotorCommand _command;  // motor command
   uint8_t _speed;         // motor speed (percentage of maximum speed)
   uint8_t _posPin;        // motor driver positive input control pin
   uint8_t _negPin;        // motor driver negative input control pin
   uint8_t _pwmPin;        // motor driver PWM control pin
   void _driveMotor();  // drive motor with current command and speed attributes
 public:
   Motor(uint8_t posPin, uint8_t negPin, uint8_t pwmPin, MotorCommand command = Coast, uint8_t speed = 0, const char* name = "Unknown");
   const char* name();  // get motor name
   void setName(const char* name);  // set motor name
   MotorCommand command();  // get motor command
   void setCommand(MotorCommand command);  // set motor command
   uint8_t speed();  // get motor speed
   void setSpeed(uint8_t speed);  // set motor speed
   void drive(MotorCommand command = Forward, uint8_t speed = 100);  // drive motor with specified attributes
   void stop(MotorCommand command = Coast);  // stop motor with specified command
};
#endif