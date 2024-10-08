// Code generated by Arduino IoT Cloud, DO NOT EDIT.

#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>

const char DEVICE_LOGIN_NAME[]  = "89768c48-fc63-4968-a72e-17a8b6074d7c";

const char SSID[]               = SECRET_SSID;    // Network SSID (name)
const char PASS[]               = SECRET_OPTIONAL_PASS;    // Network password (use for WPA, or use as key for WEP)
const char DEVICE_KEY[]  = SECRET_DEVICE_KEY;    // Secret device password

void onHumidityChange();
void onSoilMoistureChange();
void onTemperatureChange();
void onWaterLevelChange();
void onScheduleChange();

float humidity;
float soilMoisture;
float temperature;
float waterLevel;
CloudSchedule schedule;

void initProperties(){

  ArduinoCloud.setBoardId(DEVICE_LOGIN_NAME);
  ArduinoCloud.setSecretDeviceKey(DEVICE_KEY);
  ArduinoCloud.addProperty(humidity, READWRITE, ON_CHANGE, onHumidityChange);
  ArduinoCloud.addProperty(soilMoisture, READWRITE, ON_CHANGE, onSoilMoistureChange);
  ArduinoCloud.addProperty(temperature, READWRITE, ON_CHANGE, onTemperatureChange);
  ArduinoCloud.addProperty(waterLevel, READWRITE, ON_CHANGE, onWaterLevelChange);
  ArduinoCloud.addProperty(schedule, READWRITE, ON_CHANGE, onScheduleChange);

}

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);
