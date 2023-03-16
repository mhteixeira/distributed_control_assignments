#include "pid.h"
const int LED_PIN = 15;

pid my_pid {0.01, 1, 0, 0.05, 0, 10};
float r {0.0};

void setup() {
  analogReadResolution(12);
  Serial.begin(115200);
}

void loop() {
  if (Serial.available())
    r = Serial.parseInt(SKIP_ALL, '\n');
  float y = analogRead(A0);
  float u = my_pid.compute_control(r, y);
  int pwm = (int)u;
  analogWrite(LED_PIN, pwm);
  my_pid.housekeep(r, y);
  delay(10);
  Serial.print("R:");Serial.print(r);Serial.print(",");
  Serial.print("Y:");Serial.print(y);Serial.print(",");
  Serial.print("U:");Serial.print(u);Serial.print(",");
  Serial.print("I:");Serial.print(my_pid.I);Serial.println("");
}
