#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h> // Inclui as definições do Arduino aqui

class DCMotor
{
private:
  int pin1, pin2;
  int pwm_pin;

public:
  void attach(int in1, int in2, int pwm_pin);
  void run(double speed);
  void stop();
};

#endif