// HC-SR04 ultrasonic Sensor

#include <Arduino.h>
#include <NewPing.h>

#define ultrasonicTrigger 11
#define ultrasonicEcho 12
NewPing sonar(ultrasonicTrigger, ultrasonicEcho);

void setup() {
  pinMode(ultrasonicEcho, INPUT);
  pinMode(ultrasonicTrigger, OUTPUT);
}

void loop() {
  double distance = sonar.ping_cm();
  double distanceIn = sonar.ping_in();

  Serial.print(distance);
  Serial.print(" cm, ");
  Serial.print(distanceIn);
  Serial.print(" in");
  Serial.println();

  double duration = sonar.ping_median(3)*1.0/2.0;

  // This is how to calculate distance manually
  //  double speedOfSoundEnglish = 1133.0; // ft/s
  //  double speedOfSoundMetric = 343.0; // m/s
  //
  //  double distanceIn = speedOfSoundEnglish*duration
  //  Serial.print(" in");
  //  Serial.println();
}
