#include <OneWire.h>
#include <DallasTemperature.h>
#include <SHT1x.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

//Globals
//***** PHOTOCELL *****
int sensorPin = A1;    // select the input pin for the potentiometer
double sensorValue = -1; // variable to store the value coming from the sensor
uint32_t timer = millis();

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

void setup() {
  //***** INITIALIZE MOSFET *****
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(8, HIGH);
  digitalWrite(9, HIGH);  
  
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

  //***** READ GPS *****
  printGPS();
  delay(10000);
}

void printGPS() {
  boolean fixAcq = false;

  while(!fixAcq) {
    char c = GPS.read();
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
    }  
  }
  
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
  tempC = 0;
}

void printHumidity() 
{
  float humid;
  humid = th_sensor.readHumidity();
  Serial.print(humid);
  Serial.println("%");
}
