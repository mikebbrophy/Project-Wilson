#include <OneWire.h>
#include <DallasTemperature.h>
#include <SHT1x.h>

//Globals
//***** PHOTOCELL *****
int sensorPin = A1;    // select the input pin for the potentiometer
double sensorValue = -1; // variable to store the value coming from the sensor

//***** AIR TEMP SENSOR *****
#define AIRTEMP_BUS 4
OneWire oneWireAir(AIRTEMP_BUS);
DeviceAddress airThermometer;

//***** WATER TEMP SENSOR *****
#define WATERTEMP_BUS 5
OneWire oneWireWater(WATERTEMP_BUS);
DeviceAddress waterThermometer;

//***** HUMID SENSOR *****
#define dataPin A2
#define sckPin A3 //serial clock
SHT1x th_sensor(dataPin, sckPin);

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
  //***** READ PHOTOCELL
  Serial.print("photocell: ");
  printLight();

  //***** READ AIR AND WATER TEMP SENSORS
  Serial.print("airtemp: ");
  printTemperature(airThermometer, sensorsAir);
  Serial.print("watertemp: ");
  printTemperature(waterThermometer, sensorsWater);

  //***** READ HUMIDITY SENSOR *****
  Serial.print("humidity: ");
  printHumidity();
  Serial.println();

  delay(3000);
}

void printLight()
{
  sensorValue = analogRead(sensorPin);
  Serial.print(sensorValue*100/1024);
  Serial.println("%");
}

void printTemperature(DeviceAddress deviceAddress, DallasTemperature sensorsAddress)
{
  sensorsAddress.requestTemperatures(); // Send the command to get temperatures
  float tempC = sensorsAddress.getTempC(deviceAddress);
  if(tempC == DEVICE_DISCONNECTED_C) 
  {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  Serial.print(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
  Serial.println("F");
}

void printHumidity() 
{
  float humid;
  humid = th_sensor.readHumidity();
  Serial.print(humid);
  Serial.println("%");
}
