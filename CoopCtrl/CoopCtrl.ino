// Board: "ArduinoGenuino Mega or Mega 2560"
// Processor: "ATmega2560 (Mega 2560)
// Programmer: "Arduino as ISP

const float CoopCtrl_Version = 2.13;
const String versionDate = "03/18/2022";

const boolean debugOn = false;              // debug to monitor
const boolean superDebugOn = false;        // advanced debugging with variable info and timers

// include
  #include <secureSettings.h>
  #include <SoftwareSerial.h>
  #include <stdlib.h>
  #include <DHT.h>                          // humiture
  #include <stdio.h>
  #include <string.h>
  #include <DS1302.h>                       // Real Time Clock
  #include <Wire.h> 
  #include <OneWire.h>                      // OneWire DS18B20 Temperature Probe
  #include <DallasTemperature.h>            // for processing data from DS18B20
  #include <SPI.h>
  #include <Adafruit_BMP280.h>              // barometer
  #include <NewPing.h>                      // waterer barrel ultrasonic sensor
//  #include <arduino.h>                      // to get PI // 9/13/20, code using this is commented out, will work on this when I uncomment

// pin definitions
  // digital
  uint8_t RTC_SDA = 2;                      // Real Time Clock SDA
  uint8_t RTC_SCL = 3;                      // Real Time Clock SCL
  uint8_t RTC_RST = 22;                     // Real Time Clock RST
  const int insideCoopLights = 4;           // inside coop lights Mosfet
  const int insideDHT22 = 7;                // Inside humiture
  const int outsideDHT22 = 8;               // Outside humiture
  const int rainSenseDigital = 10;          // rain sensor digital connection
  const int insideLightsAutoLed = 34;       // Blue LED - inside lights in auto mode
  const int insideLightsAutoPin = 35;       // inside lights auto mode switch
  const int insideLightsOnLed = 26;         // Green LED - inside lights on
  const int insideLightsOverridePin = 37;   // inside lights on switch
  const int insideLightsOverrideLed = 38;   // Red LED - inside lights override
  const int watererLevel = 44;              // waterer level contact closure - open=full, closed=needs addressing
  OneWire watererTempUpper(46);             // waterer highgher temperature reading (a 4.7k resistor to power required)
  DallasTemperature watererSensors(&watererTempUpper);
  DeviceAddress watererTempUpperSensor = {0x28, 0xBB, 0xE8, 0x77, 0x91, 0x09, 0x02, 0xA6};
  #define watererTriggerPin 28              // trigger pin ultrasonic sensor
  #define watererEchoPin 29                 // echo pin ultrasonic sensor
  #define maxDistance 92                    // maximum distance to ping (in cemtimeters)
  const int sendToThingspeakLed = 53;       // white LED - on during sendToThingSpeak 
  // Analog
  const int ctrlTemp = A0;                  // control box temperature sensor
  const int ambientLightSensor = A1;        // photoresistor sensor for outside light level reading
  const int rainSenseAnalog = A2;           // rain sensor analog connection
  
// timing
  unsigned long currentMillis = 0;          // reference
  unsigned long oneSecMillis = 0;           // one second reference
  unsigned long oneMinMillis = 0;           // one minute reference
  unsigned long fiveMinMillis = 0;          // five minute reference
  unsigned long fifteenMinMillis = 0;       // fifteeen minute reference
  unsigned long thirtyMinMillis = 0;        // thirty minute reference
  unsigned long thingSpeakMillis = 0;       // for pauses in sending commands to thingSpeak
  const long oneMinInterval = 60000;        // one minute interval
  unsigned long lastSonicDebounceTime = 0;  // ultrasonic waterer reading debounce timing
  unsigned long lastFloatDebounceTime = 0;  // ultrasonic waterer reading debounce timing
  long debounceDelay = 100;                 // 10 milliseconds

// thingspeak
  String sensorStr ="empty";                // string to send to ThingSpeak channel 1
  String lightingStr ="empty";              // string to send to ThingSpeak channel 2
  int thingSpeakSetup = 0;                  // for only running thingspeak in minute interval after setup routine

// waterer monitoring
  float watererUpperTempC;                  // variable to hold temperature retrieved
  float watererUpperTempF;                  // degrees F
  int currentWatererFloat;                  // monitor watererLevel pin
  int currentWatererFloat2;                 // reference monitor watererLevel pin
  int previousWatererFloat;                 // for tracking difference
  int sendFloatToThingSpeak = 0;            // for tracking when to send Float sensor info to ThingSpeak
  NewPing sonar(watererTriggerPin, watererEchoPin, maxDistance);
  float watererCm = 0;                      // ping variable for ultrasonic in centimeters
  float watererCm2 = 0;                     // reference monitor for ultrasonic reading
  float previousWatererCm = 0;              // for tracking differences
  float watererIn = 0;                      // ping variable for ultrasonic in inches
  float currentWatererLevel = 0;            // monitoring water level in inches
  const float barrelMaxDepth = 30.25;       // 30-1/4" max depth
  const float barrelRadius = 10.75;         // 10-3/4" barrel radius
  const int gallonFactor = 231;    // divide the volume by this number to get gallons
  float barrelCurrentGallons = 0;           // how much water is in the barrel

// humitures
  DHT insideDHT(insideDHT22, DHT22);                            //create a variable type for inside dht
  float insideTempC;
  float insideTempF;
  float insideHumidity;
  DHT outsideDHT(outsideDHT22, DHT22);                           //create a variable type for outside dht
  float outsideTempC;
  float outsideTempF;
  float outsideHumidity;

// TMP36 (inside controller box)
  float ctrlTempReading = 0;
  float ctrlTempC;
  float ctrlTempF;

// Light Sensor
  int ambientLightSensorReading;
  String ambientLightSensorLevel = "";
  volatile boolean darknessHasSet = true;
  String lastLightLevel = "";
  
// communication with Coop_Door_Control_v2.03
  String coopDoorSays;
  boolean newData = false;
  String doorStatus;
  String doorOpenTime = "0000";               // for tracking time stamp for door opening
  String yesterdayDoorOpenTime;             // allowing data to be sent once to thingSpeak

// barometer
  Adafruit_BMP280 bmp280;                   //Define a variable bmp280 of Adafruit_BMP280 type. Subsequently bmp280 represents Adafruit_BMP280
  float barometricPressurePascal = 0;
  float barometricPressureInHG = 0;
  float barometricTempC = 0;

// clock
/* Create buffers */
  char buf[50];
  char RTCday[10];
  int currentHour;                          // for looking at the hour to turn the light on
  int currentYear;
  int currentMonth;
  int currentDay;
  int currentMin;
  String comdata = "";
  int numdata[7] ={ 0}, j = 0, mark = 0;
  DS1302 rtc(RTC_RST, RTC_SDA, RTC_SCL);    //create a variable type of DS1302
  String hourMin;

// coop lights
  boolean insideLightOn = false;            // for tracking when the coop light is on
  int insideLightsOnTime = 4;               // turn lights on at 4 AM
  int insideLightsOffTime = 8;              // turn lights off at 8 AM
  int insideLightsOverrideState = 0;      
  boolean insideLightsOverride = false;      
  int insideLightsAutoState = 0; 
  int insideFadeTime = 15;                  // take 15 minutes to turn coop lights on full
  int insideLightLevel = 0;                 // for tracking light level

// raindrop sensor
  int rainAState = 0;                       // store the value of rainSenseAnalog
  boolean rainDState = 0;                   // store the value of rainSenseDigital
  int rainingState = 0;


void setup() {
// NOTE: to set clock: change rtc.write_protect to false, set time  in Time t(year, mo, da, h, m, sec, day of week (sunday = 1))
  rtc.write_protect(true);
  rtc.halt(false);
  //Time t(2022, 03, 18, 15, 00, 00, 6);       //initialize the time
  //rtc.time(t);                              // Set the time and date on the chip

// communications and quick notification
  Serial.begin(115200);                       // monitor
  Serial2.begin(115200);                    // ESP8266 wifi
  Serial3.begin(9600);                      // Coop_Door_Control_v2.03
  Serial.println();

// pin definitions
  pinMode(insideCoopLights, OUTPUT);        // inside coop lights Mosfet
  pinMode(rainSenseDigital, INPUT);         // rain sensor digital connection
  pinMode(insideLightsAutoLed, OUTPUT);     // Blue LED - inside lights in auto mode
  pinMode(insideLightsAutoPin, INPUT);      // inside lights auto mode switch
  pinMode(insideLightsOnLed, OUTPUT);       // Green LED - inside lights on
  pinMode(insideLightsOverridePin, INPUT);  // inside lights on switch
  pinMode(insideLightsOverrideLed, OUTPUT); // Red LED - inside lights override
  pinMode(sendToThingspeakLed, OUTPUT);     // clear LED - send to Thingspeak
  pinMode(watererLevel, INPUT_PULLUP);      // waterer level contact closure - open=full, closed=needs addressing

  delay (1);
  Serial.println("|- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -|");
  Serial.print("|                                    Coop Control ");
  Serial.print(CoopCtrl_Version);
  Serial.println("                                      |");
  Serial.print("|                                     --");
  Serial.print(versionDate);
  Serial.println("--                                        |");
  Serial.println("|                                       @kayaking_t                                         |");
  Serial.println("|                                                                                           |");
  Serial.print("| Debug Status: ");
  if (debugOn == true) {
    Serial.print("on, Super Debug Status: ");
    if (superDebugOn == true) {
      Serial.println("on                                                  |");
    }
    else {
      Serial.println("off                                                 |");
    }
  }
  else {
    Serial.print("off, Super Debug Status: ");
    if (superDebugOn == true) {
      Serial.println("on                                                 |");
    }
    else {
      Serial.println("off                                                |");
    }
  }
  Serial.println("| Serial 2 = ESP8266 wifi                                                                   |");
  Serial.println("| Serial 3 = Coop Door Control connection                                                   |");
  Serial.println("|                                                                                           |");

// barometer

  if (!bmp280.begin())                      //if bmp280.begin()==0, it means bmp280 initialization fails.
  {
    Serial.println(F("| Could not find a valid BMP280 sensor, check wiring!                                       |"));
    Serial.println("|                                                                                           |");
  }
  Serial.println("|- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -|");

// waterer temp
  watererSensors.setResolution(watererTempUpperSensor, 9); // set the resolution to 9 bit - Valid values are 9, 10, or 11 bit.

// calls
  readBarometer();
  watererPing();
  readHumiture();
  controlBoxTemp();
  lightSensorProcessing();
  readWatererTemp();
  insideLightsControl();
  rainSensor();
  debugPrint();
  readHumiture();
  thingSpeakSetup = 1;
}

void loop() {
  currentMillis = millis();
  readBarometer();
  readHumiture();
  controlBoxTemp();
  lightSensorProcessing();
  recWithEndMarker();
  readWatererTemp();
  insideLightsControl();
  insideLightsLEDControl();
  rainSensor();
  timing();
}

void sendToESP32() {
  if ((darknessHasSet == true) && (ambientLightSensorLevel == "Light")) { // 15 minutes at average /min should be 185 (averages taken from actual data July & October 2019)
    Serial3.print("raise coop door>");
    Serial.println("command to ESP32: raise coop door");
    darknessHasSet = false;
    insideLightOn = false;                                // v2.04 - set flag to turn lights off inside coop
    insideLightLevel = 0;                                 // v2.04 - set flag to turn lights off inside coop
    timeStamp();                                          // mark the time the door was opened
    doorOpenTime = hourMin;                               // v2.04 - helping to track time door opens at ThingSpeak
  }
}

void timing() {
  if(currentMillis - oneSecMillis >= oneMinInterval/60) {
    oneSecMillis = currentMillis;
  }
  if(currentMillis - oneMinMillis >= oneMinInterval) {
    oneMinMillis = currentMillis;
    debugPrint();
    if (thingSpeakSetup == 1)
    {
      sendToThingSpeak();
      thingSpeakSetup = 0;
    }
  }
  if(currentMillis - fiveMinMillis >= oneMinInterval*5) {
    watererPing();
    fiveMinMillis = currentMillis;
  }
  if(currentMillis - fifteenMinMillis >= oneMinInterval*15) {
    fifteenMinMillis = currentMillis;
    sendToThingSpeak();
  }
  if(currentMillis - thirtyMinMillis >= oneMinInterval*30) {
    thirtyMinMillis = currentMillis;
  }
}

void sendToThingSpeak() {
  timeStamp();
  digitalWrite(sendToThingspeakLed,HIGH);
  Serial2.println("AT+CIPSTART=\"TCP\",\"184.106.153.149\",80");
  Serial.println("connection start - 2.5 sec delay");
  delay(2500);
  if(Serial2.available()) {
    while(Serial2.available()) {
      char c = Serial2.read();
      Serial.write(c);
    }
  }
  sensorStr = "GET /update?api_key=";
  sensorStr += sensorKey;
  sensorStr +="&field1=";
  sensorStr +=String(insideTempF);
  sensorStr +="&field2=";
  sensorStr +=String(outsideTempF);
  sensorStr +="&field3=";
  sensorStr +=String(insideHumidity);
  sensorStr +="&field4=";
  sensorStr +=String(outsideHumidity);
  sensorStr +="&field5=";
  sensorStr +=String(barometricPressureInHG);
  sensorStr +="&field6=";
  sensorStr +=String(rainingState);
  sensorStr +="&field7=";
  sensorStr +=String(watererUpperTempF);
  sensorStr +="&field8=";
  sensorStr +=String(ctrlTempF);
  sensorStr +="\r\n\r\n";
  String sensorCmd = "AT+CIPSEND=";
  sensorCmd += String(sensorStr.length());
  Serial2.println(sensorCmd);
  Serial.println("sensor cmd - 2.5 sec delay");
  delay(2500);
  if(Serial2.available()) {
    while(Serial2.available()) {
      char c = Serial2.read();
      Serial.write(c);
    }
  }
  Serial2.print(sensorStr);
  Serial.print("sensor sent to thingSpeak: ");
  Serial.print(sensorCmd);
  Serial.println(sensorStr);
  Serial.println("end sensor ThingSpeak send - 2.5 sec delay");
  delay(2500);
  if(Serial2.available()) {
    while(Serial2.available()) {
      char c = Serial2.read();
      Serial.write(c);
    }
  }
  Serial2.println("AT+CIPSTART=\"TCP\",\"184.106.153.149\",80");
  Serial.println("lighting connection start - 2.5 sec delay");
  delay(2500);
  lightingStr = "GET /update?api_key=";
  lightingStr += lightingKey;
  lightingStr +="&field1=";
  lightingStr +=String(ambientLightSensorReading);
  if (sendFloatToThingSpeak == 1){
    lightingStr +="&field2=";
    lightingStr +=String(currentWatererFloat);
    sendFloatToThingSpeak = 0;
  }
  //lightingStr +="&field3=";
  //lightingStr +=String(currentWatererLevel);
  //lightingStr +="&field4=";
  //lightingStr +=String(barrelCurrentGallons);
  if (doorOpenTime != yesterdayDoorOpenTime) {
    lightingStr +="&field5=";
    lightingStr +=String(doorOpenTime);
    yesterdayDoorOpenTime = doorOpenTime;
  }
/*  lightingStr +="&field6=";
  lightingStr +=String(rainingState);
  lightingStr +="&field7=";
  lightingStr +=String(watererTempF);
  lightingStr +="&field8=";
  lightingStr +=String(ctrolTempF); */
  lightingStr +="\r\n\r\n";
  String lightingCmd = "AT+CIPSEND=";
  lightingCmd += String(lightingStr.length());
  Serial2.println(lightingCmd);
  Serial.println("lighting cmd - 2.5 sec delay");
  delay(2500);
  if(Serial2.available()) {
    while(Serial2.available()) {
      char c = Serial2.read();
      Serial.write(c);
    }
  }
  Serial2.print(lightingStr);
  Serial.print("lighting sent to thingSpeak: ");
  Serial.print(lightingCmd);
  Serial.println(lightingStr);
//  Serial.println("end lighting #3 - 2.5 sec delay");
//  delay(2500);
  if(Serial2.available()) {
    while(Serial2.available()) {
      char c = Serial2.read();
      Serial.write(c);
    }
  }
  Serial.println();
  Serial.println("end thingSpeak");
  digitalWrite(sendToThingspeakLed,LOW);
}

void rainSensor() {
  rainAState=analogRead(rainSenseAnalog);         //read the value of A2
  rainDState=digitalRead(rainSenseDigital);       //read the value of D10
  if(rainDState==HIGH) 
  {
    rainingState = 1;
  }
  else                                            //if the value of D10 is LOW
  {
    rainingState = 0;
  }  
}

void readBarometer() {
  barometricPressurePascal = (bmp280.readPressure());
  barometricTempC = (bmp280.readTemperature());
  barometricPressureInHG = (barometricPressurePascal / 3389.39);
}

void watererPing() {
  currentWatererFloat = digitalRead(watererLevel);
  if ((millis() - lastFloatDebounceTime) > debounceDelay) {   // delay to allow for consistent readings
    currentWatererFloat2 = digitalRead(watererLevel);         // comparison reading
    if (currentWatererFloat == currentWatererFloat2) {        // looking for consistent readings
      if(currentWatererFloat != previousWatererFloat) {       // the float changed state
        previousWatererFloat = currentWatererFloat;           // reset the float state
        lastFloatDebounceTime = currentMillis;                // reset reference
        sendFloatToThingSpeak = 1;                            // trigger to send update to ThingSpeak
      }
      if (currentWatererFloat == 0){                          // waterer is empty ALWAYS send a note
        sendFloatToThingSpeak = 1;                            // trigger to send update to ThingSpeak
      }
    }
  }
  else {
    lastFloatDebounceTime = currentMillis;                     // reset reference
  }
//  NOTE: I don't think this is right because it won't keep going back and taking the reading until it's positive... need to look at interupts...
/*  watererCm = sonar.ping_median(5); // Send ping, get ping time in microseconds (uS).  Serial.println ("15");
  if (watererCm < 0) {
  Serial.println ("16");
    watererCm = sonar.ping_median(5); // Send ping, get ping time in microseconds (uS).
  } else if ((millis() - lastSonicDebounceTime) > debounceDelay) {    // delay to allow for consistent readings
  Serial.println ("18");
    watererCm2 = sonar.ping_median(5);      // comparison reading
  Serial.println ("19");
    if (watererCm == watererCm2) {            // looking for consistent readings
  Serial.println ("20");
      if(watererCm != previousWatererCm) {          // the door changed state
  Serial.println ("21");
        previousWatererCm = watererCm;              // reset the door state
  Serial.println ("22");
        watererIn = ((watererCm / US_ROUNDTRIP_CM)/2.54);
  Serial.println ("23");
        currentWatererLevel = (barrelMaxDepth - watererIn);
  Serial.println ("24");
        barrelCurrentGallons = ((M_PI*(barrelRadius*barrelRadius)*(currentWatererLevel))/gallonFactor);
  Serial.println ("25");
        lastSonicDebounceTime = currentMillis;                 // reset reference
  Serial.println ("26");
      }
  Serial.println ("27");
    }
  Serial.println ("28");
  }
  else {
  Serial.println ("30");
    lastSonicDebounceTime = currentMillis;                     // reset reference
  Serial.println ("31");
  }
  Serial.println ("32"); */
}

void readHumiture() {
// Inside Humiture
  insideTempC = insideDHT.readTemperature();
  insideTempF = insideDHT.readTemperature(true);
  insideHumidity = insideDHT.readHumidity();
// Outside Humiture
  outsideTempC = outsideDHT.readTemperature();
  outsideTempF = outsideDHT.readTemperature(true);
  outsideHumidity = outsideDHT.readHumidity();
}

void controlBoxTemp() {
  ctrlTempReading = analogRead(ctrlTemp);
  float tempVoltage = (ctrlTempReading/1024.0) *5.0;
  ctrlTempC = (tempVoltage - .5) *100;
  ctrlTempF = (ctrlTempC * 1.8) + 32;
}

void lightSensorProcessing() {
  ambientLightSensorReading = analogRead(ambientLightSensor);
  if (ambientLightSensorReading >=0 && ambientLightSensorReading <= 40) {
    ambientLightSensorLevel = "Dark";
    darknessHasSet = true;
  }
  else if (ambientLightSensorReading >= 45 && ambientLightSensorReading <= 190) {
    ambientLightSensorLevel = "Twilight";
  }
  else if (ambientLightSensorReading >= 200) {
    ambientLightSensorLevel = "Light";
  }
  if (lastLightLevel != ambientLightSensorLevel) {        // light level string changed, send update to ESP32
    lastLightLevel = ambientLightSensorLevel;             // change the level only send once per change
    sendToESP32();
  }
}

void recWithEndMarker() {
  char endMarker = '>';
  char rc;
  while (Serial3.available() >0 && newData == false) {
    rc = Serial3.read();
    if (rc == endMarker) {
      newData = true;
      coopDoorSaysNewData();
    }
    else {
      coopDoorSays += rc;
    }
  }
}

void coopDoorSaysNewData() {
  if (newData == true) {
    doorStatus ="";
    Serial.print("Coop Door says ");
    Serial.println(coopDoorSays);
    if (coopDoorSays == "door down") {
      doorStatus = "door is down";
    } else if (coopDoorSays == "door up") {
        doorStatus = "door is up";
        doorOpenTime = hourMin;
    } else if (coopDoorSays == "door closing") {
        doorStatus = "door is closing";
    } else if (coopDoorSays == "door opening") {
        doorStatus = "door is opening";
    }
    coopDoorSays = "";
    newData = false;
  }
}

void readWatererTemp(){                               
  watererSensors.requestTemperaturesByAddress(watererTempUpperSensor);   // Send the command to get temperatures
 
  watererUpperTempC = watererSensors.getTempC(watererTempUpperSensor);        // Get the temperature that you told the sensor to measure
  watererUpperTempF = DallasTemperature::toFahrenheit(watererUpperTempC);
}

void timeStamp() {
  Time t = rtc.time();                      // Get the current time and date from the chip
  memset(RTCday, 0, sizeof(RTCday));        // Name the day of the week
  switch (t.day)
  {
  case 1: 
  strcpy(RTCday, "Sun"); 
  break;
  case 2: 
  strcpy(RTCday, "Mon"); 
  break;
  case 3: 
  strcpy(RTCday, "Tue"); 
  break;
  case 4: 
  strcpy(RTCday, "Wed"); 
  break;
  case 5: 
  strcpy(RTCday, "Thu"); 
  break;
  case 6: 
  strcpy(RTCday, "Fri"); 
  break;
  case 7: 
  strcpy(RTCday, "Sat"); 
  break;
  }
  /* Format the time and date and insert into the temporary buffer */
  snprintf(buf, sizeof(buf), "%s %04d-%02d-%02d %02d:%02d:%02d", RTCday, t.yr, t.mon, t.date, t.hr, t.min, t.sec);
  /* Print the formatted string to serial so we can see the time */
  Serial.print("Current time info: ");
  Serial.println(buf);
  currentHour = t.hr;
  currentMin = t.min;
  hourMin = (String(currentHour) + String(currentMin));
}

void insideLightsControl() {
  insideLightsOverrideState = digitalRead(insideLightsOverridePin);
  insideLightsAutoState = digitalRead(insideLightsAutoPin);

  if ((insideLightsOverrideState == HIGH) && (insideLightsOverride == false)) {
    insideLightsOverride = true;
    insideLightLevel = 255;
  }
  if ((insideLightsOverrideState == LOW) && (insideLightsOverride == true)) {
    insideLightsOverride = false;
    insideLightLevel = 0;
  }
  if(insideLightsAutoState == HIGH) {
    if((currentHour == insideLightsOnTime) && (insideLightOn == false)) {
      insideLightOn = true;
      Serial.println("insideLightOn = true;");
    }
    if((currentHour == insideLightsOffTime) && (insideLightOn == true)) {
      insideLightOn = false;
      insideLightLevel = 0;
    }
    if((insideLightOn == true) && (currentHour == insideLightsOnTime)) {
      if (currentMin < insideFadeTime) {
        insideLightLevel = (currentMin * (255/insideFadeTime));
      } else if (currentMin >= insideFadeTime) {
        insideLightLevel = 255;
      }
    }
  }
  analogWrite(insideCoopLights, insideLightLevel);
}

void insideLightsLEDControl() {
  // Green LED for inside coop lights feedback
  if(insideLightLevel > 0) {
    digitalWrite(insideLightsOnLed, HIGH);
  } else {
    digitalWrite(insideLightsOnLed, LOW);
  }
  // Red LED for override feedback
  if(insideLightsOverride == false) {
    digitalWrite(insideLightsOverrideLed, LOW);
  } else {
    digitalWrite(insideLightsOverrideLed, HIGH);
  }
  if(insideLightsAutoState == HIGH) {
    digitalWrite(insideLightsAutoLed, HIGH);
  } else {
    digitalWrite(insideLightsAutoLed, LOW);
  }
}

void debugPrint () {
  if (debugOn == true) {
    Serial.println();
    timeStamp();
    // inside lighting status
    Serial.print("Inside Lights Switch current setting: ");
    if(insideLightsOverrideState == HIGH) {
      Serial.println("Override on - lights should be on and Red LED lit");
    } else if(insideLightsAutoState == HIGH) {
      Serial.println("Automatic mode on - blue LED should be lit");
    } else {
      Serial.println("switch in off position - no Lights or LEDs");
    }
    // humitures & Control temp
    Serial.print("Humitures: insideTempF=");
    Serial.print(insideTempF);
    Serial.print("F, outsideTempF=");
    Serial.print(outsideTempF);
    Serial.print("F, Control Box Temp: ctrlTempF=");
    Serial.print(ctrlTempF);
    Serial.println("F");
    // rain sensor
    Serial.print("Rain sensor: ");
    if(rainDState == HIGH) {
      Serial.println("it's raining");
    } else {
      Serial.println("not raining");
    }
    // waterer
    Serial.print("Waterer: barrelCurrentGallons=");
    Serial.print(barrelCurrentGallons);
    Serial.print(", watererUpperTempF=");
    Serial.print(watererUpperTempF);
    Serial.println("F");
    // ambient light sensor
    Serial.print("Light sensor: Ambient Light Level=");
    Serial.println(ambientLightSensorLevel);
    Serial.print("door open time: ");
    Serial.println(doorOpenTime);
    if (superDebugOn == true) {
      superDebugPrint();
    }
    Serial.println("----------end debugPrint----------");
    Serial.println();
  }
}

void superDebugPrint () {
  // inside lighting status
  Serial.println("-----superDebugPrint-----");
  Serial.print("Time: currentHour=");
  Serial.println(currentHour);
  Serial.print("inside lights: insideLightsOverride=");
  Serial.print(insideLightsOverride);
  Serial.print(", insideLightOn=");
  Serial.print(insideLightOn);
  Serial.print(", insideLightLevel=");
  Serial.println(insideLightLevel);
  // humitures & control box temo
  Serial.print("Humitures: insideTempC=");
  Serial.print(insideTempC);
  Serial.print("C, insideHumidity=");
  Serial.print(insideHumidity);
  Serial.print("%, outsideTempC=");
  Serial.print(outsideTempC);
  Serial.print("C, outsideHumidity=");
  Serial.print(outsideHumidity);
  Serial.print("%, Control Box: ctrlTempReading=");
  Serial.print(ctrlTempReading);
  Serial.print(", ctrlTempC=");
  Serial.print(ctrlTempC);
  Serial.println("C");
  // Rain sensor
  Serial.print("Rain sensor: rainAState=");
  Serial.print(rainAState);
  Serial.print(", rainDState=");
  Serial.print(rainDState);
  Serial.print(", rainingState=");
  Serial.println(rainingState);
  // Waterer
  Serial.print("Waterer: watererUpperTempC=");
  Serial.print(watererUpperTempC);
  Serial.print("C, currentWatererFloat=");
  Serial.print(currentWatererFloat);
  Serial.print(", currentWatererFloat2=");
  Serial.print(currentWatererFloat2);
  Serial.print(", previousWatererFloat=");
  Serial.println(previousWatererFloat);
  Serial.print("Waterer ultrasonic: watererCm=");
  Serial.print(watererCm);
  Serial.print(", watererCm2=");
  Serial.print(watererCm2);
  Serial.print(", previousWatererCm=");
  Serial.print(previousWatererCm);
  Serial.print("cm, watererIn=");
  Serial.print(watererIn);
  Serial.print("in, currentWatererLevel=");
  Serial.println(currentWatererLevel);
  // ambient light sensor
  Serial.print("Light Sensor: ambientLightSensorReading=");
  Serial.print(ambientLightSensorReading);
  Serial.print(", darknessHasSet=");
  Serial.println(darknessHasSet);
  // ESP32 - Coop Door controller    
  Serial.print("ESP32 commands: coopDoorSays:");
  Serial.print(coopDoorSays);
  Serial.print(" - newData: ");
  Serial.print(newData);
  Serial.print(" - doorStatus: ");
  Serial.print(doorStatus);
  Serial.print(", hourMin=");
  Serial.print(hourMin);
  Serial.print(", doorOpenTime=");
  Serial.print(doorOpenTime);
  Serial.print(", yesterdayDoorOpenTime=");
  Serial.println(yesterdayDoorOpenTime);
}
