#ifdef READ
#include <Arduino.h>

int pwmReadPin = 2;  // Pin that reads the PWM signal

void setup() {
  Serial.begin(9600);
  pinMode(pwmReadPin, INPUT);
}

void loop() {
  unsigned long pulseWidth = pulseIn(pwmReadPin, HIGH);  // Measure HIGH duration in microseconds
  Serial.print("Pulse Width: ");
  Serial.print(pulseWidth);
  Serial.println(" µs");
  delay(100);  // Small delay for readability
}

#endif
