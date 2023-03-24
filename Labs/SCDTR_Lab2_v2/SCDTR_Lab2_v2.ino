#include "pid.h"
const int LED_PIN = 15;
const int DAC_RANGE = 4096;
float incomingByte = 0.0;

float V_adc = 0.0;
float R_ldr = 0.0;
float lux = 0.0;

float Pmax = 100;
float delta_t = 0.01;

const int NUM_MEASUREMENTS = 6000;
float lux_buffer[NUM_MEASUREMENTS];
float ref_buffer[NUM_MEASUREMENTS];
float dtc_buffer[NUM_MEASUREMENTS];
int buffer_pos = 0;

//     pid (h,    K,   b,   Ti,   Td, N,  Tt)
pid my_pid{0.01, 0.4, 0.1, 0.01, 0, 10, 5};
float r{5.0};

void setup()
{
  Serial.begin(115200);
  analogReadResolution(12);

  for (int i = NUM_MEASUREMENTS - 1; i >= 0; i--)
  {
    dtc_buffer[i] = 0;
    lux_buffer[i] = 0;
    ref_buffer[i] = 0;
  }
}

void loop()
{
  int sensorValue = analogRead(A0);
  lux = sensorValueToLux(sensorValue);

  float y = lux;
  float u = my_pid.compute_control(r, y);
  int pwm = (int)u;
  analogWrite(LED_PIN, pwm);
  my_pid.housekeep(r, y);
  delay(10);
  //  Serial.print("r:");Serial.print(r);Serial.print(",");
  //  Serial.print("y:");Serial.print(y);Serial.print(",");
  //  Serial.print("PWM:");Serial.print(u/10);Serial.print(",");
  //  Serial.print("Integral:");Serial.print(my_pid.I/10);Serial.println("");

  dtc_buffer[buffer_pos] = pwm / 255.0;
  lux_buffer[buffer_pos] = lux;
  ref_buffer[buffer_pos] = r;
  float pwm_sum = dtc_buffer[NUM_MEASUREMENTS - 1];
  float lux_error_sum = max(lux - r, 0);
  float flicker_sum =
      (dtc_buffer[NUM_MEASUREMENTS - 1] - dtc_buffer[NUM_MEASUREMENTS - 2]) *
                  (dtc_buffer[NUM_MEASUREMENTS - 2] - dtc_buffer[NUM_MEASUREMENTS - 3]) >=
              0
          ? 0
          : abs(dtc_buffer[NUM_MEASUREMENTS - 1] - dtc_buffer[NUM_MEASUREMENTS - 2]) +
                abs(dtc_buffer[NUM_MEASUREMENTS - 2] - dtc_buffer[NUM_MEASUREMENTS - 3]);

  for (int i = NUM_MEASUREMENTS - 2; i >= 0; i--)
  {
    pwm_sum += dtc_buffer[i];
    lux_error_sum += abs(lux_buffer[i] - ref_buffer[i]);
    if (i > 3)
    {
      flicker_sum +=
          (dtc_buffer[NUM_MEASUREMENTS - 1] - dtc_buffer[NUM_MEASUREMENTS - 2]) *
                      (dtc_buffer[NUM_MEASUREMENTS - 2] - dtc_buffer[NUM_MEASUREMENTS - 3]) >=
                  0
              ? 0
              : abs(dtc_buffer[NUM_MEASUREMENTS - 1] - dtc_buffer[NUM_MEASUREMENTS - 2]) +
                    abs(dtc_buffer[NUM_MEASUREMENTS - 2] - dtc_buffer[NUM_MEASUREMENTS - 3]);
    }
  }
  buffer_pos++;
  if (buffer_pos >= NUM_MEASUREMENTS)
    buffer_pos = 0;
  float energy = Pmax * pwm_sum * delta_t;
  float visibility_error = lux_error_sum / NUM_MEASUREMENTS;
  float flicker_error = flicker_sum / NUM_MEASUREMENTS;
  //  Serial.print("Energy:");Serial.print(energy);Serial.print(",");
  //  Serial.print("Visibility_error:");Serial.print(1000*visibility_error);Serial.print(",");
  //  Serial.print("Flicker_error:");Serial.print(1000*flicker_error);Serial.println("");

  if (Serial.available())
  {
    String input = Serial.readStringUntil('\n');
    if (input.charAt(0) == 'r')
    {                                   // if the first character is 'r'
      r = input.substring(2).toFloat(); // update r with the new value
      Serial.print("Reference set:");
      Serial.println(r);
    }
    if (input.charAt(0) == 'g')
    { // if the first character is 'r'
      Serial.print("Reference:");
      Serial.println(r);
    }
    if (input.charAt(0) == 'e')
    { // if the first character is 'r'
      Serial.print("Energy:");
      Serial.println(energy);
    }
    if (input.charAt(0) == 'v')
    { // if the first character is 'r'
      Serial.print("Visibility:");
      Serial.println(1000 * visibility_error);
    }
    if (input.charAt(0) == 'f')
    { // if the first character is 'r'
      Serial.print("Flicker error:");
      Serial.println(1000 * flicker_error);
    }
    if (input.charAt(0) == 'g' && input.charAt(2) == 'b' && input.charAt(4) == 'l')
    {
      for (int i = 0; i <= NUM_MEASUREMENTS - 1; i++)
      {
        Serial.print(lux_buffer[i]);
        Serial.print(", ");
      }
      Serial.println("");
    }
    if (input.charAt(0) == 'g' && input.charAt(2) == 'b' && input.charAt(4) == 'd')
    {
      for (int i = 0; i <= NUM_MEASUREMENTS - 1; i++)
      {
        Serial.print(dtc_buffer[i]);
        Serial.print(", ");
      }
      Serial.println("");
    }
  }
}

float sensorValueToLux(int sensorValue)
{
  V_adc = 3.3 * sensorValue / DAC_RANGE;
  R_ldr = 10000 * (3.3 - V_adc) / V_adc;
  lux = pow(pow(10, 6.15) / R_ldr, 1.25);
  return lux;
}
