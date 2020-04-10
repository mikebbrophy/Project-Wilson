#include <OneWire.h>
#include <DallasTemperature.h>

//Globals
//***** PHOTOCELL *****
int sensorPin = A1;    // select the input pin for the potentiometer
float sensorValue = 0; // variable to store the value coming from the sensor

//***** AIR TEMP SENSOR *****
#define AIRTEMP_BUS 4
OneWire oneWireAir(AIRTEMP_BUS);
DeviceAddress airThermometer;

//***** WATER TEMP SENSOR *****
#define WATERTEMP_BUS 5
OneWire oneWireWater(WATERTEMP_BUS);
DeviceAddress waterThermometer;

DallasTemperature sensorsAir(&oneWireAir);
DallasTemperature sensorsWater(&oneWireWater);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  sensorsAir.begin();
  sensorsWater.begin();

  if (!sensorsAir.getAddress(airThermometer, 0)) Serial.println("Unable to find address for Device 0 - AirTemp"); 
  if (!sensorsWater.getAddress(waterThermometer, 0)) Serial.println("Unable to find address for Device 0 - WaterTemp"); 

  //***** INITIALIZE AIR TEMP SENSOR *****
  sensorsAir.setResolution(airThermometer, 9);
  sensorsWater.setResolution(waterThermometer, 9);
  

}

void loop() {
  // put your main code here, to run repeatedly:

  //***** READ PHOTOCELL
  sensorValue = analogRead(sensorPin);
  Serial.print("photocell: ");
  Serial.println(sensorValue);
  delay(500);

  //***** READ AIR AND WATER TEMP SENSORS
  sensorsAir.requestTemperatures(); // Send the command to get temperatures
  float airtempC = sensorsAir.getTempC(airThermometer);
  sensorsWater.requestTemperatures(); // Send the command to get temperatures
  float watertempC = sensorsWater.getTempC(waterThermometer);

  Serial.print("airtemp: ");
  printAirTemperature(airThermometer);
  Serial.print("watertemp: ");
  printWaterTemperature(waterThermometer);

  delay(2000);
}

void printAirTemperature(DeviceAddress deviceAddress)
{
  // method 1 - slower
  //Serial.print("Temp C: ");
  //Serial.print(sensorsAir.getTempC(deviceAddress));
  //Serial.print(" Temp F: ");
  //Serial.print(sensorsAir.getTempF(deviceAddress)); // Makes a second call to getTempC and then converts to Fahrenheit

  // method 2 - faster
  float tempC = sensorsAir.getTempC(deviceAddress);
  if(tempC == DEVICE_DISCONNECTED_C) 
  {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  Serial.print("Temp C: ");
  Serial.print(tempC);
  Serial.print(" Temp F: ");
  Serial.println(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
}

void printWaterTemperature(DeviceAddress deviceAddress)
{
  // method 1 - slower
  //Serial.print("Temp C: ");
  //Serial.print(sensorsAir.getTempC(deviceAddress));
  //Serial.print(" Temp F: ");
  //Serial.print(sensorsAir.getTempF(deviceAddress)); // Makes a second call to getTempC and then converts to Fahrenheit

  // method 2 - faster
  float tempC = sensorsWater.getTempC(deviceAddress);
  if(tempC == DEVICE_DISCONNECTED_C) 
  {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  Serial.print("Temp C: ");
  Serial.print(tempC);
  Serial.print(" Temp F: ");
  Serial.println(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
}
