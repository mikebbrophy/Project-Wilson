#include <IridiumSBD.h>
#include <TinyGPS.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SHT1x.h>
#include <SoftwareSerial.h>
#include <RTClib.h>
#include <Wire.h>
#include <Adafruit_MPL115A2.h>

//Globals
  //***** REAL TIME CLOCK ******
  RTC_PCF8523 rtc;
   
  //***** GPS *****
  TinyGPS gps;
  SoftwareSerial ss(3, 2);  

  //***** BAROMETER *****
  Adafruit_MPL115A2 mpl115a2;
  
  //***** PHOTOCELL ******
  int sensorPin = A1;    // select the input pin for the potentiometer
  
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

  //****** Define SBD message format *******
  typedef struct {
      String  latitude = "null";
      String  longitude = "null";
      String  month = "null";
      String  day = "null";
      String  year = "null";
      String  hour = "null";
      String  minute = "null";
      String  second = "null";
      String  airTemp = "null";
      String  waterTemp = "null";
      String  insideTemp = "null";
      String  humidity = "null";
      String  pressure = "null";
      String  photocell = "null";
  } SBDStruct;

  SBDStruct data;

  String payload;
  
void updatePayload() {
   payload = "PAYLOAD:\n" + data.latitude + ", " + data.longitude + ", " + data.month + "/" + data.day + "/" + data.year + ", " + 
   data.hour + ":" + data.minute + ":" + data.second + ", aT: " + data.airTemp + ", wT: " + data.waterTemp + ", iT: " + data.insideTemp + 
   ", H: " + data.humidity + ", bP: " + data.pressure + ", P: " + data.photocell; 
}
  
void setup() {  
  //***** INITIALIZE MOSFET *****
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(8, LOW); //Iridium MOSFET
  digitalWrite(9, LOW);  //GPS + Sensor Array MOSFET
  delay(500);
  digitalWrite(9, HIGH);
  
  Serial.begin(115200);
  Serial.println("Starting Up...");

  //***** SENSOR INTITIALIZATION *****
  rtc.begin();
  mpl115a2.begin();
  sensorsAir.begin();
  sensorsWater.begin();
  if (!sensorsAir.getAddress(airThermometer, 0)) Serial.println("Unable to find address for Device 0 - AirTemp"); 
  if (!sensorsWater.getAddress(waterThermometer, 0)) Serial.println("Unable to find address for Device 0 - WaterTemp"); 
  sensorsAir.setResolution(airThermometer, 9);
  sensorsWater.setResolution(waterThermometer, 9);
}

void loop() { 
  //***** READ BAROMETER *****
  data.pressure = getPressure();

  //***** READ INSIDE TEMP *****
  data.insideTemp = getInsideTemp();
  
  //***** READ PHOTOCELL
  data.photocell = getLight();
  
  //***** READ AIR AND WATER TEMP SENSORS
  data.airTemp = getTemperature(airThermometer, sensorsAir);
  data.waterTemp = getTemperature(waterThermometer, sensorsWater);

  //***** READ HUMIDITY SENSOR *****
  data.humidity = getHumidity();
  
  //***** READ GPS *****
  getGPS();//FIX 1
  updatePayload();
  Serial.println(payload);
  delay(10000);
}

void getGPS() {
  ss.begin(9600); 
  parseDelay(60000); //Give 1 minutes for GPS to aquire fix
    float flat, flon;
    gps.f_get_position(&flat, &flon);
    if(abs(flat) <= 90) {
      data.latitude = String(flat, 6);
      data.longitude = String(flon, 6);
      int year;
      byte month, day, hour, minute, second;
      gps.crack_datetime(&year, &month, &day, &hour, &minute, &second);
      //Putting Time into payload
      data.year = year;
      data.month = month;
      data.day = day;
      data.hour = hour;
      data.minute = minute;
      data.second = second;
      //Calibrating RTC
      rtc.adjust(DateTime(year, month, day, hour, minute, second));
    } else {
      DateTime now = rtc.now();
      delay(100);
      data.year = now.year();
      data.month = now.month();
      data.day = now.day();
      data.hour = now.hour();
      data.minute = now.minute();
      data.second = now.second();
    }
}

static void parseDelay(unsigned long ms) {
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

float getLight() {
  return (analogRead(sensorPin)*100/1024);
}

float getTemperature(DeviceAddress deviceAddress, DallasTemperature sensorsAddress) {
  sensorsAddress.requestTemperatures(); // Send the command to get temperatures
  float tempC = sensorsAddress.getTempC(deviceAddress);
  return DallasTemperature::toFahrenheit(tempC);
}

float getPressure() {
  float pressureinHg;  
  pressureinHg = mpl115a2.getPressure() / 3.386;  
  return pressureinHg;
}

float getInsideTemp() {
  float insideTempC;
  insideTempC = mpl115a2.getTemperature();
  return insideTempC / 5 * 9 + 32;  
}

float getHumidity() {
  float humid;
  humid = th_sensor.readHumidity();
  return humid;
}
