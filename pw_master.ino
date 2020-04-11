#include <OneWire.h>
#include <DallasTemperature.h>
#include <SHT1x.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

//Globals
  //***** PHOTOCELL *****
  int sensorPin = A1;    // select the input pin for the potentiometer
  double sensorValue = -1; // variable to store the value coming from the sensor
  
  //***** AIR TEMP SENSOR *****
  #define AIRTEMP_BUS 4
  OneWire oneWireAir(AIRTEMP_BUS);
  DeviceAddress airThermometer;
  DallasTemperature sensorsAir(&oneWireAir);
  
  //***** WATER TEMP SENSOR *****
  #define WATERTEMP_BUS 5
  OneWire oneWireWater(WATERTEMP_BUS);
  DeviceAddress waterThermometer;
  DallasTemperature sensorsWater(&oneWireWater);
  
  
  //***** HUMID SENSOR *****
  #define dataPin A2
  #define sckPin A3 //serial clock
  SHT1x th_sensor(dataPin, sckPin);
  
  //***** GPS *****
  SoftwareSerial mySerial(3, 2);
  Adafruit_GPS GPS(&mySerial);
  #define GPSECHO  true

  char uplinkPayload[80] = { };

  float photocell_f;
  float airTemp_f;
  float waterTemp_f;
  float humidity_f;

void setup() {
  //***** INITIALIZE MOSFET *****
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(8, LOW); //Iridium MOSFET
  digitalWrite(9, HIGH);  //GPS + Sensor Array MOSFET
  
  // put your setup code here, to run once:
  Serial.begin(115200);


  //**** GPS INTIALIZATION *****
  GPS.begin(9600);
  delay(1000);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  mySerial.println(PMTK_Q_RELEASE);
  
  Serial.println("STARTING UP...");
  sensorsAir.begin();
  sensorsWater.begin();

  if (!sensorsAir.getAddress(airThermometer, 0)) Serial.println("Unable to find address for Device 0 - AirTemp"); 
  if (!sensorsWater.getAddress(waterThermometer, 0)) Serial.println("Unable to find address for Device 0 - WaterTemp"); 

  //***** INITIALIZE AIR TEMP SENSOR *****
  sensorsAir.setResolution(airThermometer, 9);
  sensorsWater.setResolution(waterThermometer, 9);
}

void loop() 
{ 
   
  //***** READ PHOTOCELL
  Serial.print("photocell: ");
  photocell_f = printLight();
  
  //***** READ AIR AND WATER TEMP SENSORS
  Serial.print("airtemp: ");
  airTemp_f = printTemperature(airThermometer, sensorsAir);
  Serial.print("watertemp: ");
  waterTemp_f = printTemperature(waterThermometer, sensorsWater);

  //***** READ HUMIDITY SENSOR *****
  Serial.print("humidity: ");
  humidity_f = printHumidity();
  Serial.println();

  //***** READ GPS *****
  if(printGPS()) Serial.println ("\n\n-------------------GPS SUCCESSFUL----------------------\n\n");
  
  delay(60000);
}

boolean printGPS() {
  delay(60000);
  boolean fixAcq = false;
  uint32_t timer = millis();
  while (!fixAcq) {
    if (millis() - timer > 180000) {
      Serial.println("\n\n-------------------GPS TIMEOUT--------------------\n\n");
      return fixAcq;
    }
    GPS.parse(GPS.lastNMEA());
    if(GPS.fix) {
      Serial.print("Time: ");
      Serial.print(GPS.hour, DEC); Serial.print(":");  Serial.print(GPS.minute, DEC); Serial.print(":");  Serial.println(GPS.seconds, DEC);
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      Serial.println();
      fixAcq = true;
      return fixAcq;
    }  
  }
  
}

float printLight()
{
  sensorValue = analogRead(sensorPin);
  photocell_f = sensorValue;
  Serial.print(sensorValue*100/1024);
  Serial.println("%");

  return (sensorValue*100/1024);
}

float printTemperature(DeviceAddress deviceAddress, DallasTemperature sensorsAddress)
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
  return DallasTemperature::toFahrenheit(tempC);
}

float printHumidity() 
{
  float humid;
  humid = th_sensor.readHumidity();
  Serial.print(humid);
  Serial.println("%");
  return humid;
}
