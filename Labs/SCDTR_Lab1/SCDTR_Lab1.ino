//BASIC IO
const int LED_PIN = 15;
const int DAC_RANGE = 4096;
float incomingByte = 0.0;

float V_adc = 0.0;
float R_ldr = 0.0;
float lux = 0.0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); //default is 10
}

void loop() {
  int sensorValue = analogRead(A0);
  lux = sensorValueToLux(sensorValue);
  
  if (Serial.available() > 0) {
    incomingByte = Serial.parseFloat(SKIP_ALL, '\n');
    analogWrite(LED_PIN, int(incomingByte*256)); // set led PWM
  }
  
  Serial.print(0); Serial.print(" "); 
  Serial.print(sensorValue);Serial.print(" ");
  Serial.print(V_adc);Serial.print(" ");
  Serial.print(R_ldr);Serial.print(" ");
  Serial.print(lux);Serial.print(" ");
  Serial.print(incomingByte);Serial.print(" ");
  Serial.println();

  delay(200);
}

float sensorValueToLux(int sensorValue){
  V_adc = 3.3*sensorValue/DAC_RANGE;
  R_ldr = 10000*(3.3 - V_adc)/V_adc;
  lux = pow(pow(10, 6.15)/R_ldr, 1.25);
  return lux;
}
