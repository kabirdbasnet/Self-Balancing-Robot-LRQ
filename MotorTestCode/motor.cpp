#include "Motor.h"

Motor::Motor(uint8_t posPin, uint8_t negPin, uint8_t pwmPin, MotorCommand command, uint8_t speed, const char* name) {
   _posPin = posPin;
   _negPin = negPin;
   _pwmPin = pwmPin;
   _command = command;
   _speed = speed;
   strcpy(_name, name);
   pinMode(posPin, OUTPUT);
   pinMode(negPin, OUTPUT);
   pinMode(pwmPin, OUTPUT);
   _driveMotor();
}

const char* Motor::name() { return _name; }

void Motor::setName(const char* name) { strcpy(_name, name); }

MotorCommand Motor::command() { return _command; }

void Motor::setCommand(MotorCommand command) {
   _command = command;
   _driveMotor();
}

uint8_t Motor::speed() { return _speed; }

void Motor::setSpeed(uint8_t speed) {
   _speed = speed;
   _driveMotor();
}

void Motor::drive(MotorCommand command, uint8_t speed) {
   _command = command;
   _speed = speed;
   _driveMotor();
}

void Motor::stop(MotorCommand command) {
   _command = (command == Brake) ? Brake : Coast;
   _driveMotor();
}

void Motor::_driveMotor() {
   if (_command == Brake || _command == Coast) _speed = 0;  // set speed to 0 if motor is stopping
   _speed = constrain(_speed, 0, 100);  // constrain speed to valid percentage range
   analogWrite(_pwmPin, map(_speed, 0, 100, 0, 255));  // use PWM to adjust speed
   switch (_command) {
      case Forward:
         digitalWrite(_posPin, HIGH);
         digitalWrite(_negPin, LOW);
         break;
      case Reverse:
         digitalWrite(_posPin, LOW);
         digitalWrite(_negPin, HIGH);
         break;
      case Brake:
         digitalWrite(_posPin, HIGH);
         digitalWrite(_negPin, HIGH);
         break;
      default:  // Coast
         digitalWrite(_posPin, LOW);
         digitalWrite(_negPin, LOW);
         break;
   }
}
