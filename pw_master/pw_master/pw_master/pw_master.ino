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

  // Define SBD message format ------------------------------------------

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
  
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting Up...");

  //***** SENSOR INTITIALIZATION *****
  rtc.begin();
  mpl115a2.begin();
  sensorsAir.begin();
  sensorsWater.begin();
  //if (!sensorsAir.getAddress(airThermometer, 0)) Serial.println("Unable to find address for Device 0 - AirTemp"); 
  //if (!sensorsWater.getAddress(waterThermometer, 0)) Serial.println("Unable to find address for Device 0 - WaterTemp"); 
  sensorsAir.setResolution(airThermometer, 9);
  sensorsWater.setResolution(waterThermometer, 9);
}

void loop() 
{ 
  //***** READ BAROMETER *****
  data.pressure = getPressure();

  //***** READ INSIDE TEMP *****
  data.insideTemp = getInsideTemp();
  
  //***** READ PHOTOCELL
  //Serial.print("photocell: ");
  data.photocell = getLight();
  
  //***** READ AIR AND WATER TEMP SENSORS
  //Serial.print("airtemp: ");
  data.airTemp = getTemperature(airThermometer, sensorsAir);
  //Serial.print("watertemp: ");
  data.waterTemp = getTemperature(waterThermometer, sensorsWater);

  //***** READ HUMIDITY SENSOR *****
  //Serial.print("humidity: ");
  data.humidity = getHumidity();
  //Serial.println();
  
  //***** READ GPS *****
  getGPS();//FIX 1
  updatePayload();
  //timeCheck();
  Serial.println(payload);
  //getGPS();//FIX 2
  //getGPS();//FIX 3
  
  delay(10000);
}

void getGPS() {
  ss.begin(9600); 
  parseDelay(60000); //Give 1 minutes for GPS to aquire fix
    float flat, flon;
    gps.f_get_position(&flat, &flon);
    data.latitude = String(flat, 6);
    data.longitude = String(flon, 6);
    /*
    Serial.print("LAT=");
    Serial.print(flat, 6);
    Serial.print(" LON=");
    Serial.print(flon, 6);
    Serial.print(" SAT=");
    Serial.println(gps.satellites());
    */
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
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d ",
        month, day, year, hour, minute, second);
    //Serial.println(sz);
}

static void parseDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

/*void timeCheck() {
    DateTime now = rtc.now();
    Serial.print("ONBOARD TIME = ");
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
}*/

float getLight()
{
  sensorValue = analogRead(sensorPin);
  //Serial.print(sensorValue*100/1024);
  //Serial.println("%");
  return (sensorValue*100/1024);
}

float getTemperature(DeviceAddress deviceAddress, DallasTemperature sensorsAddress)
{
  sensorsAddress.requestTemperatures(); // Send the command to get temperatures
  float tempC = sensorsAddress.getTempC(deviceAddress);
  if(tempC == DEVICE_DISCONNECTED_C) 
  {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  //Serial.print(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
  //Serial.println("F");
  return DallasTemperature::toFahrenheit(tempC);
}

float getPressure() {
  float pressureinHg;  
  pressureinHg = mpl115a2.getPressure() / 3.386;  
  //Serial.print("Pressure (inHg): "); 
  //Serial.print(pressureinHg, 4); 
  //Serial.println(" inHg");
  return pressureinHg;
}

float getInsideTemp() {
  float insideTempC;
  insideTempC = mpl115a2.getTemperature();
  //Serial.print("insideTemp: ");
  //Serial.print(insideTempC, 4);
  //Serial.print("C");
  return insideTempC / 5 * 9 + 32;  
}

float getHumidity() 
{
  float humid;
  humid = th_sensor.readHumidity();
  //Serial.print(humid);
  //Serial.println("%");
  return humid;
}
