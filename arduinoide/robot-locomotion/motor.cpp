#include "motor.h"

void DCMotor::attach(int in1, int in2, int in3)
{
  pin1 = in1;
  pin2 = in2;
  pwm_pin = in3;
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
}

void DCMotor::run(double speed)
{
  if (speed > 0) {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    ledcWrite(pwm_pin, abs(speed));
  } else if (speed < 0) {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    ledcWrite(pwm_pin, abs(speed));
  } else {
    stop();
  }
}

void DCMotor::stop()
{
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, LOW);
  ledcWrite(pwm_pin, 0);
}