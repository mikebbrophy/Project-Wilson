#include <LowPower.h>
#include <IridiumSBD.h>
#include <TinyGPS.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SHT1x.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_MPL115A2.h>


//Globals

  //***** SLEEP *****
  //1 hour = 393 iterations
  //Increments of 9.14766 seconds (+1.14766s deviation from 8s)
  //Sleep consumes ~3ma while normal operation are ~50ma(.)(.)

  int startCollection;
  float collectionTime = 130.0; //In seconds
  int sleepTime = 21600; //6 Hour itr
  int sleepItr;
          
  //***** IRIDIUM *****
  #define IridiumSerial Serial

  // Declare the IridiumSBD object
  IridiumSBD modem(IridiumSerial);
     
  //***** GPS *****
  TinyGPS gps;
  SoftwareSerial ss(3, 2);  
  
  //***** BAROMETER *****
  Adafruit_MPL115A2 mpl115a2;
  
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
      String  state = "RESET";
  } SBDStruct;

  SBDStruct data;

  char payload[100];

void updatePayload() {
  sprintf(payload, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s", 
      data.latitude.c_str(),
      data.longitude.c_str(),
      data.month.c_str(),
      data.day.c_str(),
      data.year.c_str(),
      data.hour.c_str(),
      data.minute.c_str(),
      data.second.c_str(),
      data.airTemp.c_str(),
      data.waterTemp.c_str(),
      data.insideTemp.c_str(),
      data.humidity.c_str(),
      data.pressure.c_str(),
      data.state.c_str());
} 

void setup() {  
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
}
  
void loop() {
        sleepItr = (sleepTime - collectionTime) / 9.147666;
        for(int s = 1; s <= sleepItr; s++) {
             LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);   
        }
        startCollection = millis();
        
   //-------------  COLLECTION LOOP --------------

        //***** INITIALIZE MOSFET *****
        digitalWrite(9, HIGH); //Sensor + GPS Mosfet
        delay(1000);

        mpl115a2.begin();
        sensorsAir.begin();
        sensorsWater.begin();
        sensorsAir.getAddress(airThermometer, 0);
        sensorsWater.getAddress(waterThermometer, 0);
        sensorsAir.setResolution(airThermometer, 9);
        sensorsWater.setResolution(waterThermometer, 9);
        
        data.pressure = getPressure(); //***** READ BAROMETER *****
        data.insideTemp = getInsideTemp(); //***** READ INSIDE TEMP *****
        data.airTemp = getTemperature(airThermometer, sensorsAir);  //***** READ AIR *****
        data.waterTemp = getTemperature(waterThermometer, sensorsWater); //***** READ WATER *****
        data.humidity = getHumidity();  //***** READ HUMIDITY SENSOR *****
        getGPS(); /***** READ GPS *****/ delay(1000); 
        getGPS(); //***** GPS CONFIRM ***** 

        //***** TURNING OFF SENSORS + GPS ******
        digitalWrite(9, LOW);
        
   //-------------  END COLLECTION LOOP --------------   

   //-------- SENDING DATA TO IRIDIUM NETWORK --------
        digitalWrite(8, HIGH); // Turn on Modem
        delay(5000); //Give time for cap to charge
        updatePayload(); //Puts structure data into char buffer
        sendData(); //Does as stated
        digitalWrite(8, LOW);
    //------------ END OF DATA TRANSMISSION --------
        data.state = "NOMINAL";
        collectionTime = (millis() - startCollection) / 1000; 
        //Record how long collection took in order to calibrate sleep
}

void sendData() {
        int err;
        
        IridiumSerial.begin(19200);
        err = modem.begin();
        
        int IR_counter = 0;
        while(err != 0 && IR_counter <= 4)
        {
           delay(100);
           err = modem.begin();
           IR_counter++;
        }
        err = modem.sendSBDText(payload);
        if (err != ISBD_SUCCESS)
        {
          delay(60000);
          modem.sendSBDText(payload);
        }
}

void getGPS() {
    ss.begin(9600); 
    parseDelay(60000); //Give 1 minutes for GPS to aquire fix
    float flat, flon;
    gps.f_get_position(&flat, &flon);
    if(flat <= 90) {
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

float getTemperature(DeviceAddress deviceAddress, DallasTemperature sensorsAddress) {
  sensorsAddress.requestTemperatures(); // Send the command to get temperatures
  float tempC = sensorsAddress.getTempC(deviceAddress);
  return DallasTemperature::toFahrenheit(tempC);
}

float getPressure() {
  float pressureinHg;
  pressureinHg = (mpl115a2.getPressure() / 3.386) + .25;  
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
