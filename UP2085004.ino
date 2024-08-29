#include "arduino_secrets.h"
// Include only necessary libraries
#include <ESP32Servo.h>  // Make sure this library is installed and compatible

/* 
  Sketch for Smart Gardening System using ESP32 and Arduino IoT Cloud
*/
#include "thingProperties.h"
#include <DHT.h>  // For DHT sensor

////// Pins
#define DHTPIN 4
#define AOUT_PIN 34
#define TRIG_PIN 22 // ESP32 pin GPIO22 connected to Ultrasonic Sensor's TRIG pin
#define ECHO_PIN 19 // ESP32 pin GPIO19 connected to Ultrasonic Sensor's ECHO pin
#define humidifier 12
#define waterPump 5
#define fan 18
#define SERVO_PIN 23 // Pin connected to the servo motor

//// DHT VARIABLES
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

/// Ultrasonic variables
float duration_us, distance_cm;

/// Servo
Servo myservo;
int pos = 0;



int setPointTemp = 25;
int humsetPoint = 50;


// Calibration constants
const int maxDistance = 40;  // Measured distance in cm when the tank is full
const int minDistance = 0.1;   // Measured distance in cm when the tank is empty - cchange to 10 later


void setup() {
  Serial.begin(9600);
  delay(1500);

  myservo.attach(26);

  initProperties();

  ArduinoCloud.begin(ArduinoIoTPreferredConnection);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(humidifier, OUTPUT);
  pinMode(waterPump, OUTPUT);
  pinMode(fan, OUTPUT);
  myservo.attach(SERVO_PIN);

  dht.begin();

  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
}

void loop() {
  ArduinoCloud.update();
  delay(2000);

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  humidity = h;
  temperature = t;
  
  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.println(F("°C "));

  // Soil moisture calculation adjusted
  int rawValue = analogRead(AOUT_PIN);
  // Calibration points
  int dryValue = 3200;
  int wetValue = 700; 
  
  // Map the raw sensor value to a percentage of moisture
  soilMoisture = map(rawValue, wetValue, dryValue, 100, 0);
  soilMoisture = constrain(soilMoisture, 0, 100);  // Ensure the values are within 0% and 100%

  Serial.print("Soil Moisture (%): ");
  Serial.println(soilMoisture);

  ArduinoCloud.update();

  
  
  // Trigger the ultrasonic sensor
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration_us = pulseIn(ECHO_PIN, HIGH, 30000); // 30 ms timeout for robustness
  if (duration_us == 0) {
    Serial.println("Timeout or out of range");
    waterLevel = -1; // Indicate an error or out of range
  } else {
    distance_cm = duration_us * 0.0343 / 2.0; // Calculate the distance

    // Map the distance to a water level percentage
    waterLevel = map(distance_cm, minDistance, maxDistance, 100, 0);
    waterLevel = constrain(waterLevel, 0, 100); // Ensure the value is between 0 and 100

    Serial.print("Water Level (%): ");
    Serial.println(waterLevel);
  }
  
  
  
  

  controlActuators(t, h);
  
  if (schedule.isActive()) {
    Serial.println("Scheduler active - Servo moving to 90 degrees.");
    myservo.write(90);
  } else {
    Serial.println("Scheduler inactive - Servo returning to 0 degrees.");
    myservo.write(0);
  }

  
  ArduinoCloud.update();
  
}

void controlActuators(float temp, float humidity) {
  digitalWrite(fan, temp > setPointTemp ? HIGH : LOW);
  digitalWrite(humidifier, humidity > humsetPoint ? HIGH : LOW);
}


void onTemperatureChange() {
  // Add logic here if needed, otherwise leave empty
}

void onHumidityChange() {
  // Add logic here if needed, otherwise leave empty
}

void onSoilMoistureChange() {
  // Add logic here if needed, otherwise leave empty
}

void onWaterLevelChange() {
  // Add logic here if needed, otherwise leave empty
}

void onScheduleChange() {
  Serial.println("Schedule change detected.");
  // Add logic here if needed, otherwise leave empty
}
