#include "Motor.h"

#define DEBUG 1           // mode of operation; 0 = normal, 1 = debug
#define OP_DURATION 5000  // operation duration in milliseconds

const uint8_t TB6612FNG_STBY = 15;    // STBY (standby) pin of TB6612FNG board
const uint8_t TB6612FNG_AIN1 = 26;    // AIN1 (motor A control 1) pin of TB6612FNG board
const uint8_t TB6612FNG_AIN2 = 25;    // AIN2 (motor A control 2) pin of TB6612FNG board
const uint8_t TB6612FNG_PWMA = 24;     // PWMA (motor A speed control) pin of TB6612FNG board

Motor motorA = Motor(TB6612FNG_AIN1, TB6612FNG_AIN2, TB6612FNG_PWMA); // Motor A instance

void setup() {
  // Serial Monitor
  if (DEBUG) {
    Serial.begin(9600);  // initialize serial bus
    while (!Serial);     // wait for serial connection
    Serial.println(F("Running in DEBUG mode.  Turn off for normal operation."));
  }

  // Pin configurations
  pinMode(TB6612FNG_STBY, OUTPUT);

  // Enable (turn on) motor driver
  digitalWrite(TB6612FNG_STBY, HIGH);
}

void loop() {
  basicOperations();  // perform basic motor control operations on motor A
}

void basicOperations() {
   // Basic operations with default attributes
   motorA.drive();                       // drive forward at full throttle
   if (DEBUG) printMotorStatus(motorA);
   delay(OP_DURATION);
   motorA.stop();                        // coast (soft) to a stop
   if (DEBUG) printMotorStatus(motorA);
   delay(OP_DURATION);
   // Basic operations with specific attributes
   motorA.drive(Forward, 75);            // drive forward at 75% throttle
   if (DEBUG) printMotorStatus(motorA);
   delay(OP_DURATION);
   motorA.stop(Coast);                   // coast (soft) to a stop
   if (DEBUG) printMotorStatus(motorA);
   delay(OP_DURATION);
   motorA.drive(Reverse, 50);            // drive in reverse at 50% throttle
   if (DEBUG) printMotorStatus(motorA);
   delay(OP_DURATION);
   motorA.stop(Brake);                   // brake (hard) to a stop
   if (DEBUG) printMotorStatus(motorA);
   delay(OP_DURATION);
   // Setting attributes
   motorA.setName("A");                  // change name of motor to A
   if (DEBUG) printMotorStatus(motorA);
   motorA.setCommand(Forward);           // drive forward at previously set speed
   if (DEBUG) printMotorStatus(motorA);
   delay(OP_DURATION);
   motorA.setSpeed(75);                  // set speed to 75% throttle with previously set command
   if (DEBUG) printMotorStatus(motorA);
   delay(OP_DURATION);
   motorA.setSpeed(0);                   // set speed to 0% throttle with previously set command
   if (DEBUG) printMotorStatus(motorA);
   delay(OP_DURATION);
   motorA.setCommand(Reverse);           // drive in reverse with previously set speed
   if (DEBUG) printMotorStatus(motorA);
   delay(OP_DURATION);
   motorA.setSpeed(75);                  // set speed to 75% throttle with previously set command
   if (DEBUG) printMotorStatus(motorA);
   delay(OP_DURATION);
   motorA.setCommand(Coast);             // coast (soft) to a stop
   if (DEBUG) printMotorStatus(motorA);
   delay(OP_DURATION);
}

void printMotorStatus(Motor motor) {
  const char* const command_states[] = {"Forward", "Reverse", "Brake", "Coast"};  // constant array of constant strings
  Serial.print(F("Motor "));
  Serial.print(motor.name());
  Serial.print(F(": Command = "));
  Serial.print(command_states[motor.command()]);
  Serial.print(F(", Speed = "));
  Serial.println(motor.speed());
}
