// V1.01 deployed 11/21/2018
// V1.02 tested but not deployed (AlarmTime.h didn't work)
// V1.03 deployed 11/25/2018 - fixed auto lights and engaged barometer which doesn't seem to be working
// V1.04 deployed 12/1/2018 - updates comments and starts troubleshooting rain sensor & barometer
// v1.05 deployed 3/19/2019 - moves rain sensor to loop, fixes the auto lights control so that it does care about the switch
// v1.06 deployed 6/16/2019 - adds waterer level sensor - still an issue with lighting in the coop, hasn't been addressed yet, adds lightsensor also
// v1.07 deployed 7/2/2019 - adds lightsensor tracking to ThingSpeak more frequently, also adds ultrasonic sensor for waterer exact level
// v1.08 deployed 10/9/2019 - adds communication with Coop Door ESP32 - current version Coop_Door_Control_v2.03.ino
    /* strings for communication to Coop Door ESP32:
     *  "lower coop door"
     *  "raise coop door"
     *  "stop coop door"
     *  strings for communciation from Coop Door ESP32:
     *  door down>
     *  door up>
     *  door moving>
     *  door stopped>
     */

// include
  #include <SoftwareSerial.h>
  #include <stdlib.h>
  #include <dht.h>                      // humiture
  #include <stdio.h>
  #include <string.h>
  #include <DS1302.h>                   // RTC
  #include <Wire.h> 
  #include <OneWire.h>                  // OneWire DS18B20 Temperature Probe
  #include <DallasTemperature.h>        // for processing data from DS18B20
  #include <SPI.h>
  #include <Adafruit_BMP280.h>          // barometer
  #include <NewPing.h>
  #include <arduino.h>                  // to get PI
  //#include <LiquidCrystal_I2C.h>        // LCD - not in use as of V1.04
  //LiquidCrystal_I2C lcd(0x27,16,2);     // set the LCD address to 0x27 for a 16 chars and 2 line display

// timing
  unsigned long currentMillis = 0;      // reference
  unsigned long fifteenMinMillis = 0;   // 15 minute reference
  unsigned long thirtyMinMillis = 0;    // 30 minute reference
  unsigned long fiveMinMillis = 0;      // 5 minute reference
  unsigned long oneMinMillis = 0;       // 1 minute reference
  unsigned long oneSecMillis = 0;       // 1 second reference
  const long interval = 60000;          // 1 minute interval
  int fifteenMinuteRule = 0;            // 15 minutes - send a lot to ThingSpeak
  int oneMinuteRule = 0;                // 1 minute - send a little to ThingSpeak

// thingspeak below
  String apiKey = "Z8VL15KKDUBNAUBI";   // 11/28/2018 key for arduino@liskfamily.com account - major items we are tracking
  String lightingKey = "8VPX0I3SRBXTXRD9";  // 7/1/19 key for lighting monitoring at ThingSpeak
  String getStr ="empty";                        // string to send to ThingSpeak
  
// pin definitions
// Digital
  uint8_t SDA_PIN = 2;                  // RTC SDA
  uint8_t SCL_PIN = 3;                  // RTC SCL
  const int coopLights = 4;             // Coop lights MOSFET
  const int InsideDHT22 = 7;            // Inside humiture
  const int OutsideDHT22 = 8;           // Outside humiture
  const int rainSenseDigital = 10;      // rain sensor digital connection
  uint8_t RST_PIN = 22;                 // RTC RST
  const int lightsAutoLED = 34;         // Blue LED - lights in auto mode reference
  const int lightsAutoPin = 35;         // lights auto mode switch
  const int lightsOnLED = 36;           // Green LED - lights on reference
  const int lightsOverridePin = 37;     // lights on switch
  const int lightsOverrideLED = 38;     // Red LED - lights override reference
  const int watererLevel(44);           // waterer level contact closure - open = full, closed = needs addressing
  OneWire  watererTemp(46);             // waterer temperature (a 4.7K resistor to power required)
  // declare as sensor referenec by passing oneWire reference to Dallas Temperature. 
  DallasTemperature watererSensors(&watererTemp);
  // declare your device address
  DeviceAddress watererTempSensor = {0x28, 0xBB, 0xE8, 0x77, 0x91, 0x09, 0x02, 0xA6};

// Analog
  const int ctrlTemp = A0;              // control box temperature sensor
  const int lightSensor = A1;           // photoresistor sensor
  const int rainSenseAnalog = A2;       // rain sensor analog connection

// waterer monitoring
  float watererTempC;               // A Variable to hold the temperature you retrieve
  float watererTempF;               // another variable for temperature, but in F
// waterer float sensor
  int currentWatererFloat;          // for monitoring current state
// ultrasonic sensor
  #define TRIGGER_PIN 28                // Arduino pin tied to trigger pin on the ultrasonic sensor.
  #define ECHO_PIN 29                   // Arduino pin tied to echo pin on the ultrasonic sensor.
  #define MAX_DISTANCE 92               // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
  NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
  float uS = 0;                         // ping variable for ultrasonic will be in CM
  float iN = 0;                         // ping variable for ultrasonic will be in in
  float currentWatererLevel = 0;        // monitoring water level in inches
  const float barrelMaxDepth = 30.25;   // 30-1/4" max depth
  const float barrelRadius = 10.75;     // radius of barrel
  const int gallonFactor = 231;         // divide the volume by this number to get gallons
  float barrelCurrentGallons = 0;       // how much water is in the barrel
  

// possibly remove below
// clock setting variable - possibly remove
//  int i=0;

// humitures below
  dht InsideDHT;          //create a variable type for inside dht
  float InsideTempC;
  float InsideTempF;
  float InsideHumidity;
  dht OutsideDHT;         //create a variable type for outside dht
  float OutsideTempC;
  float OutsideTempF;
  float OutsideHumidity;

// TMP36 below (inside controller box)
  float ctrlTempReading = 0;
  float ctrlTempC;
  float ctrlTempF;

// Light Sensor below
  int lightSensorReading;
  String lightSensorLevel;
  bool darknessHasSet = false;

// communication with Coop_Door_Control_v2.03
  String slaveSays;
  boolean newData = false;
  String doorStatus;
  String lightSensorSays;

// barometer below
  Adafruit_BMP280 bmp280;       //Define a variable bmp280 of Adafruit_BMP280 type. Subsequently bmp280 represents Adafruit_BMP280
  float barometricPressurePascal = 0;
  float barometricPressureInHG = 0;
  float barometricTempC = 0;

// clock
/* Create buffers */
  char buf[50];
  char RTCday[10];
  int currentHour;                            // for looking at the hour to turn the light on
  int currentYear;
  int currentMonth;
  int currentDay;
  int currentMin;
  String comdata = "";
  int numdata[7] ={ 0}, j = 0, mark = 0;
  DS1302 rtc(RST_PIN, SDA_PIN, SCL_PIN);      //create a variable type of DS1302

// coop lights
  bool lightOn = false;             // for tracking when the coop light is on
  int lightsOnTime = 4;             // turn lights on at 4 AM
  int lightsOffTime = 8;            // turn lights off at 8 AM
  int lightsOverrideState = 0;      
  bool lightsOverride = false;      
  int lightsAutoState = 0; 
  int fadeTime = 15;                // take 15 minutes to turn coop lights on full
  int lightLevel = 0;               // for tracking light level

// raindrop sensor
  int rainAState = 0;           // store the value of rainSenseAnalog
  bool rainDState = 0;          // store the value of rainSenseDigital
  int rainingState = 0;


void setup() {                
  Serial.begin(9600);       // monitor
  Serial.println("CoopCtrl_1.08 - October, 9, 2019");
  Serial.println("Serial.begin 9600 - monitor");
  Serial2.begin(115200);      // ESP8266 wifi
  Serial.println("Serial2.begin 115200 - ESP8266 ThingSpeak");
  Serial3.begin(9600);        // Coop_Door_Control_v2.03
  Serial.println("Serial3.begin 9600 - ESP32 Coop Door");
  Serial.println("- - - - - - - - - -");

  // pin definitions
//  pinMode(12,OUTPUT);                       // possibly remove - not sure it's needed
  pinMode(coopLights, OUTPUT);              // connected to MOSFET to control LED strip lights
  pinMode(lightsOverridePin, INPUT);        // override switch to force coop lights on
  pinMode(lightsAutoPin, INPUT);            // auto light switch
  pinMode(lightsOnLED, OUTPUT);             // green LED for lights on
  pinMode(lightsAutoLED, OUTPUT);           // blue LED for auto lighting
  pinMode(lightsOverrideLED, OUTPUT);       // red LED for override
  pinMode(rainSenseDigital,INPUT);          // rain sensor
  pinMode(watererLevel,INPUT_PULLUP);              // waterer level float sensor
  
  // time
// to set clock: change rtc.write_protect to false, set time  in Time t(year, mon, day, hour, min, sec, day of week (sunday = 1))
  rtc.write_protect(true);
  rtc.halt(false);
  //Time t(2019, 10, 9, 17, 16, 00, 3);      //initialize the time
  //rtc.time(t);                              // Set the time and date on the chip

  // barometer
  Serial.println(F("BMP280_I2C"));      //print BMP280_I2C on serial monitor
  if (!bmp280.begin())                  //if bmp280.begin()==0, it means bmp280 initialization fails.
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);                          //Infinite loop, no stop until the initialization succeeds.
  }

  // waterer temp
  watererSensors.setResolution(watererTempSensor, 9); // set the resolution to 9 bit - Valid values are 9, 10, or 11 bit.
  // confirm that we set that resolution by asking the DS18B20 to repeat it back
  Serial.print("Waterer Temp Sensor Resolution: ");
  Serial.println(watererSensors.getResolution(watererTempSensor), DEC); 
  Serial.println();

  // read all sensors on reboot
  currentWatererFloat = digitalRead(watererLevel);
  watererPing();
  ReadHumiture ();
  ControlBoxTemp ();
  lightSensorProcessing ();
  readWatererTemp();
  readBarometer();
  print_time();

  // update ThingSpeak
  sendToThingSpeak();  

  sensorPrint();
}

// the loop 
void loop() {
  recWithEndMarker();
  slaveSaysNewData();
  sendToESP32();
  
  // coop lights control
  lightsOverrideState = digitalRead(lightsOverridePin);     // read override switch state
  lightsAutoState = digitalRead(lightsAutoPin);             // read auto lighting switch state

  // Green LED for coop lights on feedback
  if(lightLevel > 0) { // lights are on, light up LED
    digitalWrite(lightsOnLED, HIGH);
  }
  else {
    digitalWrite(lightsOnLED, LOW);
  }

  // lights override switch
  if(lightsOverrideState == HIGH && lightsOverride == false) {    // override switch went high and only run it once
    Serial.print("Override Lights On time: ");                    // troubleshooting
    print_time();                                                 // troubleshooting
    lightsOverride = true;                                        // only allow override state change to run once
    lightLevel = 255;                                             // full on
    analogWrite(coopLights, lightLevel);                          // write the level
  }
  if (lightsOverrideState == LOW && lightsOverride == true) {     // override switch went low and only run once
    Serial.print("Override Lights Off time: ");                   // troubleshooting
    print_time();                                                 // troubleshooting
    lightsOverride = false;                                       // only allow override state change to run once
    lightLevel = 0;                                               // full off
    analogWrite(coopLights, lightLevel);                          //write the level
  }

  // Red LED for override feedback
  if (lightsOverride == false) { 
    digitalWrite(lightsOverrideLED, LOW);
  }
  else {
    digitalWrite(lightsOverrideLED, HIGH);
  }

  // Blue LED for automatic light control feedback              
  if(lightsAutoState == HIGH) {
    digitalWrite(lightsAutoLED, HIGH);
    //Serial.print("Auto Lights On time: ");                    // troubleshooting
    //print_time();                                             // troubleshooting
    // automatic lights control                                 // v1.05 moves the automatic lights control out of mainline into the lightsAutoState switch sense
    if(currentHour == lightsOnTime && lightOn == false) {
      lightOn = true;
      print_time();
      Serial.print(" lightOn variable = "); 
      Serial.println(lightOn);
    }
    if(currentHour == lightsOffTime && lightOn == true) {
      lightOn = false;
      print_time();
      Serial.print(" lightOn variable = "); 
      Serial.println(lightOn);
      lightLevel = 0;
      analogWrite(coopLights, lightLevel);
    }
    if(lightOn == true && currentHour == lightsOnTime) {      // lights should be fading up
        if (currentMin < fadeTime) {
          lightLevel = (currentMin * (255/fadeTime));
          analogWrite(coopLights, lightLevel);
        }
        else if (currentMin >= fadeTime) {
          lightLevel = 255;
          analogWrite(coopLights, lightLevel);
        }
      }
  }
  else {
    digitalWrite(lightsAutoLED, LOW);
    //Serial.print("Auto Lights Off time: ");                    // troubleshooting
    //print_time();                                              // troubleshooting
  }

  // watererlevel
  currentWatererFloat = digitalRead(watererLevel);

  // rain sensor
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
  
    
  // when to run system calls
  currentMillis = millis();                                       // for comparison
  if(currentMillis - oneSecMillis >= interval/60){                // 1 second
    oneSecMillis = currentMillis;                                 // reset reference
  }
  if(currentMillis - oneMinMillis >= interval){                   // 1 minute
    oneMinMillis = currentMillis;                                 // reset reference
    print_time();
    watererPing();                
    ReadHumiture ();
    ControlBoxTemp ();
    readWatererTemp();
    readBarometer ();
    if(lightLevel > 0) {                                          // troubleshooting
      Serial.print("lightLevel: ");
      Serial.println(lightLevel);
    }
    lightSensorProcessing();
    sensorPrint();                                                // troubleshooting
    sendToThingSpeak();                                           // addeded in 1.07 to allow for more up to date information on a few sensors (lighting and waterer)
    oneMinuteRule = 1;                                            // for tracking what is sent to ThingSpeak
    }
  if(currentMillis - fiveMinMillis >= (interval * 5)){            // 5 minutes
    fiveMinMillis = currentMillis;                                // reset reference
    Serial.println("Five Min Awake");
    //wirelessConnection();
  }
  if(currentMillis - fifteenMinMillis >= (interval * 15)){        // 15 minutes
    fifteenMinMillis = currentMillis;                             // reset reference
    Serial.println("Fifteen Min Awake");
    sendToThingSpeak();
    fifteenMinuteRule = 1;                                        // for tracking what is sent to ThingSpeak
  }
  if(currentMillis - thirtyMinMillis >= (interval * 30)){         // 30 minutes
    thirtyMinMillis = currentMillis;                              //reset reference
    Serial.println("Thirty Min Awake");
  }
}

void sendToThingSpeak() {                                             // thinkspeak communications
  Serial2.println("AT+CIPSTART=\"TCP\",\"184.106.153.149\",80");      // establish connection
  String state1 = String(InsideTempF);                                // inside humiture
  String state2 = String(OutsideTempF);                               // outside humiture
  String state3 = String(InsideHumidity);                             // inside humiture
  String state4 = String(OutsideHumidity);                            // outside humiture
  String state5 = String(barometricPressureInHG);                     // barometer
  String state6 = String(rainingState);                               // rain sensor // I wasn't really watching raining, used this for time being for waterer level
  String state7 = String(watererTempF);                               // waterer temperature
  String state8 = String(ctrlTempF);                                  // control box temperature // I wasn't really watching ctrl box tem, used this for light sensor tracking
  String stateB1 = String(lightSensorReading);                        // lightsensor reading
  String stateB2 = String(currentWatererFloat);                       // waterer float sensor
  String stateB3 = String(currentWatererLevel);                       // waterer level from ultrasonic sensor
  String stateB4 = String(barrelCurrentGallons);                      // calculated gallons in barrel
  
  delay(5000);
   if(Serial2.available()) { 
    while(Serial2.available())
    {
      char c = Serial2.read();
      Serial.write(c);
    }  
  }
  delay(5000);
  
  // prepare GET string
  if(fifteenMinuteRule == 1){
    getStr = "GET /update?api_key=";
    getStr += apiKey;
    getStr +="&field1=";                        //Inside Temp
    getStr += String(state1);
    getStr +="&field2=";                        // Outside Temp
    getStr += String(state2);
    getStr +="&field3=";                        // Inside Humidity
    getStr += String(state3);
    getStr +="&field4=";                        // Outside Humidity
    getStr += String(state4);
    getStr +="&field5=";                        // Barometer
    getStr += String(state5);
    getStr +="&field6=";                        // current Waterer Level - formerly the Rain Sensor
    getStr += String(state6);
    getStr +="&field7=";                        // Water Tank Temp
    getStr += String(state7);
    getStr +="&field8=";                        // Control Temp
    getStr += String(state8);
    getStr += "\r\n\r\n";
    // Sending data via TCP to ThingSpeak
    String cmd = "AT+CIPSEND=";             // identify string length to send to thingspeak
    cmd += String(getStr.length());
    Serial2.println(cmd);
    fifteenMinuteRule = 0;                      // reset what to send timer    
  }
  else {
    getStr = "GET /update?api_key=";
    getStr += lightingKey;
    getStr +="&field1=";                        //Light Level
    getStr += String(stateB1);
    getStr +="&field2=";                        // Waterer float sensor
    getStr += String(stateB2);
    getStr +="&field3=";                        // Waterer Level - ultrasonic reading
    getStr += String(stateB3);
    getStr +="&field4=";                        // calculated gallons
    getStr += String(stateB4);
    // getStr +="&field5=";                        // not used
    // getStr += String(stateB5);
    // getStr +="&field6=";                        // not used
    // getStr += String(stateB6);
    // getStr +="&field7=";                        // not used
    // getStr += String(stateB7);
    // getStr +="&field8=";                        // not used
    // getStr += String(stateB8);
    getStr += "\r\n\r\n";
    // Sending data via TCP to ThingSpeak
    String cmd = "AT+CIPSEND=";             // identify string length to send to thingspeak
    cmd += String(getStr.length());
    Serial2.println(cmd);
    oneMinuteRule = 0;                      // reset what to send timer    
  }
      
  delay(2500);                                // wait for response
  if(Serial2.available()) { 
    while(Serial2.available())
    {
      char c = Serial2.read();
      Serial.write(c);
    }  
  }
  delay(2500);
  // Sending data via TCP to ThingSpeak
  Serial2.print(getStr);                      // send strings to thingspeak
  Serial.print("getStr=");
  Serial.println(getStr);
  Serial.println("okkk sent to ThingSpeak");
  delay(2500);
  
  if(Serial2.available()) { 
    while(Serial2.available())
    {
      char c = Serial2.read();
      Serial.write(c);
    }  
  }
}

void readBarometer() {
  barometricPressurePascal = (bmp280.readPressure());
  barometricTempC = (bmp280.readTemperature());
  barometricPressureInHG = (barometricPressurePascal / 3389.39);
  
}

void watererPing() {
  uS = sonar.ping_median(5); // Send ping, get ping time in microseconds (uS).
  iN = ((uS / US_ROUNDTRIP_CM)/2.54);
  currentWatererLevel = (barrelMaxDepth - iN);
  barrelCurrentGallons = ((M_PI*(barrelRadius*barrelRadius)*(currentWatererLevel))/gallonFactor);
  Serial.print("Ping: ");
  Serial.print(uS / US_ROUNDTRIP_CM); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.print("cm, ");
  Serial.print(iN);
  Serial.print("in, ");
  Serial.print(currentWatererLevel);
  Serial.print(" deep, ");
  Serial.print(barrelCurrentGallons);
  Serial.println(" gallons.");
}

void ReadHumiture() {
  // READ Inside Humiture Data
    int inHumiture = InsideDHT.read22(InsideDHT22);         //read the value returned from the sensor
    InsideTempC = (InsideDHT.temperature);
    InsideTempF = (InsideTempC * 1.8) +32;
    InsideHumidity = (InsideDHT.humidity);
//    Serial.print("inside: ");                               // troubleshooting
//    Serial.println(InsideTempF);                            // troubleshooting
  // READ Outside Humiture Data
    int outHumiture = OutsideDHT.read22(OutsideDHT22);      //read the value returned from the sensor
    OutsideTempC = (OutsideDHT.temperature);
    OutsideTempF = (OutsideTempC * 1.8) +32;
    OutsideHumidity = (OutsideDHT.humidity);
//    Serial.print("outside: ");                              // troubleshooting
//    Serial.println(OutsideTempF);                           // troubleshooting
}

void ControlBoxTemp() {
  // Control box temperature
    ctrlTempReading = analogRead(ctrlTemp);
    float tempVoltage = (ctrlTempReading/1024.0) *5.0;
    ctrlTempC = (tempVoltage - .5) *100;
    ctrlTempF = (ctrlTempC * 1.8) + 32;
}

void lightSensorProcessing() {
  lightSensorReading = analogRead(lightSensor);
  if (lightSensorReading >=0 && lightSensorReading <= 40) {
    lightSensorLevel = "Dark";
    darknessHasSet = true;
  }
  else if (lightSensorReading >= 45 && lightSensorReading <= 120) {
    lightSensorLevel = "Twilight";
  }
  else if (lightSensorReading >= 125) {
    lightSensorLevel = "Light";
    darknessHasSet = false;
  }
}

void sendToESP32() {
  if ((darknessHasSet == true) && (lightSensorLevel = "Twilight")) {
    Serial3.print("raise coop door>");
    Serial.println("command to ESP32: raise coop door");
    darknessHasSet = false;
  }
}

void recWithEndMarker() {
  char endMarker = '>';
  char rc;
  while (Serial3.available() >0 && newData == false) {
    rc = Serial3.read();
    if (rc == endMarker) {
      newData = true;
      slaveSaysNewData();
    }
    else {
      slaveSays += rc;
    }
  }
}
void slaveSaysNewData() {
  if (newData == true) {
    doorStatus ="";
    Serial.print("Slave says ");
    Serial.println(slaveSays);
    if (slaveSays == "door down") {
      doorStatus = "door is down";
    } else if (slaveSays == "door up") {
        doorStatus = "door is up";
    } else if (slaveSays == "door closing") {
        doorStatus = "door is closing";
    } else if (slaveSays == "door opening") {
        doorStatus = "door is opening";
    }
    slaveSays = "";
    newData = false;
  }
}

void readWatererTemp(){                               
  watererSensors.requestTemperaturesByAddress(watererTempSensor);   // Send the command to get temperatures
 
  watererTempC = watererSensors.getTempC(watererTempSensor);        // Get the temperature that you told the sensor to measure
  watererTempF = DallasTemperature::toFahrenheit(watererTempC);
  Serial.print("Temp C: ");
  Serial.print(watererTempC,4);                                     // The four just increases the resolution that is printed
  Serial.print(" Temp F: ");
  // The Dallas Temperature Control Libray has a conversion function... we'll use it
  Serial.println(watererTempF,4);
//  Serial.println(DallasTemperature::toFahrenheit(tempC),4);
}

void print_time()
{
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
  Serial.println(buf);
/*  lcd.setCursor(2,0);
  lcd.print(t.yr);
  lcd.print("-");
  lcd.print(t.mon/10);
  lcd.print(t.mon%10);
  lcd.print("-");
  lcd.print(t.date/10);
  lcd.print(t.date%10);
  lcd.print(" ");
  lcd.print(RTCday);
  lcd.setCursor(4,1);
  lcd.print(t.hr);
  lcd.print(":");
  lcd.print(t.min/10);
  lcd.print(t.min%10);
  lcd.print(":");
  lcd.print(t.sec/10);
  lcd.print(t.sec%10);
  currentHour = t.hr;
  currentMin = t.min;
*/
}

void sensorPrint() {
  Serial.print("temperature: inside: ");
  Serial.print(InsideTempC);
  Serial.print("C, ");
  Serial.print(InsideTempF);
  Serial.print("F. outside: ");
  Serial.print(OutsideTempC);
  Serial.print("C, ");
  Serial.print(OutsideTempF);
  Serial.print("F.");
  Serial.print(" Ctrl Temp: ");
  Serial.print(ctrlTempC);
  Serial.print("C, ");
  Serial.print(ctrlTempF);
  Serial.println("F"); 
  Serial.print("humidity: inside: ");
  Serial.print(InsideHumidity);
  Serial.print(" %, outside: ");
  Serial.print(OutsideHumidity);
  Serial.println(" %");
  Serial.print("barometric pressure: ");
  Serial.print(barometricPressurePascal);
  Serial.print(" Pascal, ");
  Serial.print(barometricPressureInHG);
  Serial.print(" inHG, temp: ");
  Serial.print(barometricTempC);
  Serial.println("C");
  Serial.print("rainstate: ");
  Serial.println(rainingState);
  Serial.print("waterer Info: currentWatererFloat: ");
  Serial.print(currentWatererFloat);
  Serial.print(" - watererTempF: ");
  Serial.print(watererTempF);
  Serial.println("F");  
  Serial.print("Light sensor: lightSensorReading: ");
  Serial.print(lightSensorReading);
  Serial.print(" - Level: ");
  Serial.print(lightSensorLevel);
  Serial.print(" - darknessHasSet: ");
  Serial.println(darknessHasSet);
  Serial.print("ESP32 commands: slaveSays:");
  Serial.print(slaveSays);
  Serial.print(" - newData: ");
  Serial.print(newData);
  Serial.print(" - doorStatus: ");
  Serial.print(doorStatus);
  Serial.print(" - lightSensorSays: ");
  Serial.println(lightSensorSays);
  Serial.println("");
}
