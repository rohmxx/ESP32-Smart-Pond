#include <Arduino.h>
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif

#define WIFI_SSID "kolam"
#define WIFI_PASSWORD "initrojan01"

#define aerator_pin 4
#define feeder_pin 2
#define limit_pin 5

uint8_t counter_second = 0;

int temperature_value = 0, ph_value = 0, tds_value = 0, oxygen_value = 0;;
int aerator_state = 0, feeder_state = 0, feedlast = 0, aeratorlast = 0;
int hour_now = 0;
int feed_time[3] = {0,0,0};

unsigned long timefeed = 0;

String oxygen_display = "";

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

// WiFi init -->
void wifi_init(){
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
}
// <-- WiFi init

// temperature -->

#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 32

OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);

void temperature_setup(){
  sensors.begin();
}

int get_temperature(){
  sensors.requestTemperatures(); 
  
  Serial.print(sensors.getTempCByIndex(0)); 
  return sensors.getTempCByIndex(0);
}

// <-- temperature

// ph -->

#define SensorPin 34            //pH meter Analog output to Arduino Analog Input 0
#define Offset 0.00            //deviation compensate
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0;

double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}

int get_ph(){
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue,voltage;
  if(millis()-samplingTime > samplingInterval)
  {
      pHArray[pHArrayIndex++]=analogRead(SensorPin);
      if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
      voltage = avergearray(pHArray, ArrayLenth)*3.3/4096;
      pHValue = 3.5*voltage+Offset;
      samplingTime=millis();
  }
  if(millis() - printTime > printInterval)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
    Serial.print("pH value: ");
    Serial.println(pHValue,2);
    printTime=millis();
  }
  return pHValue;
}

// <-- ph

// tds -->

#include "GravityTDS.h"

#define TdsSensorPin 35
GravityTDS gravityTds;

int tdsValue = 0;

void tds_setup(){
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(3.3);  //reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(4096);  //1024 for 10bit ADC;4096 for 12bit ADC
  gravityTds.begin();  //initialization
}

int get_tds(){
  // gravityTds.setTemperature(get_temperature());  // set the temperature and execute temperature compensation
  gravityTds.update();  //sample and calculate
  tdsValue = gravityTds.getTdsValue();  // then get the value
  Serial.print(tdsValue,0);
  Serial.println("ppm");
  return tdsValue;
}

// <-- tds

// time -->

#include <time.h>
const char* ntpServer = "pool.ntp.org";
unsigned long epochTime; 

unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}

int get_hour(){
  struct tm timehour;
  if (!getLocalTime(&timehour)) {
    //Serial.println("Failed to obtain time");
    return(0);
  }
  char timeHour[3];
  strftime(timeHour,3, "%H", &timehour);
  int hour_result = atoi(timeHour) + 7;
  return hour_result;
}
// <-- time

// firebase -->
#include <Firebase_ESP_Client.h>

//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert Firebase project API Key
#define API_KEY "AIzaSyAHQg9OlvvRymXbVb9G87D9p5YMN71FYI0"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://smartpond-joss-default-rtdb.asia-southeast1.firebasedatabase.app/" 

//Define Firebase Data object
FirebaseData fbdo;
FirebaseData stream;
FirebaseAuth auth;
FirebaseConfig config;

// Variables to save database paths
String controlPath = "control_state/";
String feedPath = "feeding_time/";

unsigned long sendDataPrevMillis = 0;
int count = 0;
bool signupOK = false;

// Callback function that runs on database changes
void streamCallback(FirebaseStream data){
  Serial.printf("stream path, %s\nevent path, %s\ndata type, %s\nevent type, %s\n\n",
                data.streamPath().c_str(),
                data.dataPath().c_str(),
                data.dataType().c_str(),
                data.eventType().c_str());
  printResult(data); //see addons/RTDBHelper.h
  Serial.println();

  // Get the path that triggered the function
  String streamPath = String(data.dataPath());

  // if the data returned is an integer, there was a change on the GPIO state on the following path /{gpio_number}
  if (data.dataTypeEnum() == fb_esp_rtdb_data_type_string){
    String data_received = data.stringData();
    int dataintrcv = data_received.toInt();

    if(streamPath == "/aerator_state"){
      aerator_state = dataintrcv;
    }
    if(streamPath == "/feeder_state"){
      feeder_state = dataintrcv;
    }
    if(streamPath == "/time1"){
      feed_time[0] = dataintrcv;
    }
    if(streamPath == "/time2"){
      feed_time[1] = dataintrcv;
    }
    if(streamPath == "/time3"){
      feed_time[2] = dataintrcv;
    }
  }

  /* When it first runs, it is triggered on the root (/) path and returns a JSON with all keys
  and values of that path. So, we can get all values from the database and updated the GPIO states*/
  if (data.dataTypeEnum() == fb_esp_rtdb_data_type_json){
    FirebaseJson json = data.to<FirebaseJson>();

    // To iterate all values in Json object
    size_t count = json.iteratorBegin();
    for (size_t i = 0; i < count; i++){
        FirebaseJson::IteratorValue value = json.valueAt(i);
        if(value.key.c_str() == "aerator_state"){
          aerator_state = value.value.toInt();
        }
        if(value.key.c_str() == "feeder_state"){
          feeder_state = value.value.toInt();
        }
        if(value.key.c_str() == "time1"){
          feed_time[0] = value.value.toInt();
        }
        if(value.key.c_str() == "time2"){
          feed_time[1] = value.value.toInt();
        }
        if(value.key.c_str() == "time3"){
          feed_time[2] = value.value.toInt();
        }
    }
    json.iteratorEnd(); // required for free the used memory in iteration (node data collection)
  }
  
  //This is the size of stream payload received (current and max value)
  //Max payload size is the payload size under the stream path since the stream connected
  //and read once and will not update until stream reconnection takes place.
  //This max value will be zero as no payload received in case of ESP8266 which
  //BearSSL reserved Rx buffer size is less than the actual stream payload.
  Serial.printf("Received stream payload size: %d (Max. %d)\n\n", data.payloadLength(), data.maxPayloadLength());
}

void streamTimeoutCallback(bool timeout){
  if (timeout)
    Serial.println("stream timeout, resuming...\n");
  if (!stream.httpConnected())
    Serial.printf("error code: %d, reason: %s\n\n", stream.httpCode(), stream.errorReason().c_str());
}
void firebase_setup(){
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  // Assign the maximum retry of token generation
  config.max_token_generation_retry = 5;

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  // Streaming (whenever data changes on a path)
  // Begin stream on a database path --> control_state
  if (!Firebase.RTDB.beginStream(&stream, controlPath.c_str()))
    Serial.printf("stream begin error, %s\n\n", stream.errorReason().c_str());

  // Assign a calback function to run when it detects changes on the database
  Firebase.RTDB.setStreamCallback(&stream, streamCallback, streamTimeoutCallback);

  delay(2000);
}

void control_update(){
  epochTime = getTime();
  String path = "";

  path = "control_update/" + String(epochTime) + "/aerator_update";
  if (Firebase.RTDB.setInt(&fbdo, path, aerator_state)){
    Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());
    Serial.println("TYPE: " + fbdo.dataType());
  }
  else {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
  }

  path = "control_update/" + String(epochTime) + "/feeder_update";
  if (Firebase.RTDB.setInt(&fbdo, path, feeder_state)){
    Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());
    Serial.println("TYPE: " + fbdo.dataType());
  }
  else {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
  }

  path = "control_update/" + String(epochTime) + "/oxygen";
  if (Firebase.RTDB.setInt(&fbdo, path, oxygen_value)){
    Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());
    Serial.println("TYPE: " + fbdo.dataType());
  }
  else {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
  }

  path = "control_update/" + String(epochTime) + "/temperature";
  if (Firebase.RTDB.setInt(&fbdo, path, temperature_value)){
    Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());
    Serial.println("TYPE: " + fbdo.dataType());
  }
  else {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
  }
  
  path = "control_update/" + String(epochTime) + "/ph";
  if (Firebase.RTDB.setInt(&fbdo, path, ph_value)){
    Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());
    Serial.println("TYPE: " + fbdo.dataType());
  }
  else {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
  }
  
  path = "control_update/" + String(epochTime) + "/tds";
  if (Firebase.RTDB.setInt(&fbdo, path, tds_value)){
    Serial.println("PASSED");
    Serial.println("PATH: " + fbdo.dataPath());
    Serial.println("TYPE: " + fbdo.dataType());
  }
  else {
    Serial.println("FAILED");
    Serial.println("REASON: " + fbdo.errorReason());
  }
}

// <-- firebase

void setup(void)
{
  pinMode(aerator_pin, OUTPUT);
  pinMode(feeder_pin, OUTPUT);
  pinMode(limit_pin, INPUT_PULLUP);

  digitalWrite(aerator_pin, 1);
  digitalWrite(feeder_pin, 1);

  Serial.begin(115200);
  Serial.println("Starting ...");

  lcd.init();                  
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SmartPond - JOSS");
  delay(2000);

  lcd.clear();
  lcd.print("Connecting WiFi...");
  wifi_init();
  lcd.clear();
  lcd.print("WiFi Connected!");

  configTime(0, 0, ntpServer);

  firebase_setup();

  tds_setup();

  temperature_setup();
}

void loop(void)
{
  if (Firebase.isTokenExpired()){
    Firebase.refreshToken(&config);
    Serial.println("Refresh token");
  }

  if(aerator_state != aeratorlast){
    digitalWrite(aerator_pin, !aerator_state);
    control_update();
    aeratorlast = aerator_state;
  }
  
  if(feeder_state != feedlast){
    digitalWrite(feeder_pin, !feeder_state);
    control_update();
    feedlast = feeder_state;
    timefeed = millis();
  }
  
  if(digitalRead(feeder_pin) == 0 && millis() >= timefeed + 5000){
    digitalWrite(feeder_pin, 1);
    control_update();
  }
  
  ph_value = get_ph();
  if(millis() - sendDataPrevMillis > 1000 || sendDataPrevMillis == 0){
    sendDataPrevMillis = millis();

    tds_value = get_tds();
    temperature_value = get_temperature();
    if(tds_value >= 80){
      temperature_value = 21;
    }
    else if(tds_value == 0){
      temperature_value = 30;
    }
    
    if(tds_value > 500 || ph_value < 3 || ph_value > 9){
      oxygen_value = 0;
      oxygen_display = "LOW ";
      aerator_state = 1;
    }
    else if(tds_value <= 500 || ph_value >= 3 || ph_value <= 9){
      oxygen_value = 1;
      oxygen_display = "HIGH";
      aerator_state = 0;
    }

    for (int i = 0; i < 3; i++){
      if(feed_time[i] == get_hour()){
        feeder_state = 1;
      }
    }

    if(counter_second == 30 && WiFi.status() != WL_CONNECTED){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("WiFi disconnected.");
      lcd.setCursor(0,1);
      lcd.print("Connecting...");
      wifi_init();
    }

    else if(counter_second == 60 && WiFi.status() == WL_CONNECTED && Firebase.ready() && signupOK){
      epochTime = getTime();
      String path = "";

      path = "sensor_data/" + String(epochTime) + "/oxygen";
      if (Firebase.RTDB.setInt(&fbdo, path, oxygen_value)){
        Serial.println("PASSED");
        Serial.println("PATH: " + fbdo.dataPath());
        Serial.println("TYPE: " + fbdo.dataType());
      }
      else {
        Serial.println("FAILED");
        Serial.println("REASON: " + fbdo.errorReason());
      }

      path = "sensor_data/" + String(epochTime) + "/temperature";
      if (Firebase.RTDB.setInt(&fbdo, path, temperature_value)){
        Serial.println("PASSED");
        Serial.println("PATH: " + fbdo.dataPath());
        Serial.println("TYPE: " + fbdo.dataType());
      }
      else {
        Serial.println("FAILED");
        Serial.println("REASON: " + fbdo.errorReason());
      }
      
      path = "sensor_data/" + String(epochTime) + "/ph";
      if (Firebase.RTDB.setInt(&fbdo, path, ph_value)){
        Serial.println("PASSED");
        Serial.println("PATH: " + fbdo.dataPath());
        Serial.println("TYPE: " + fbdo.dataType());
      }
      else {
        Serial.println("FAILED");
        Serial.println("REASON: " + fbdo.errorReason());
      }
      
      path = "sensor_data/" + String(epochTime) + "/tds";
      if (Firebase.RTDB.setInt(&fbdo, path, tds_value)){
        Serial.println("PASSED");
        Serial.println("PATH: " + fbdo.dataPath());
        Serial.println("TYPE: " + fbdo.dataType());
      }
      else {
        Serial.println("FAILED");
        Serial.println("REASON: " + fbdo.errorReason());
      }

      lcd.setCursor(0,0);
      lcd.printf("Temp:%3d O2:%3d", temperature_value, oxygen_value);
      lcd.setCursor(0,1);
      lcd.printf("pH:%3d TDS:%5d", ph_value, tds_value);
    }

    lcd.setCursor(0,0);
    lcd.printf("O2:%s Temp:%3d", oxygen_display, temperature_value);
    lcd.setCursor(0,1);
    lcd.printf("pH:%2d   TDS:%4d", ph_value, tds_value);

    counter_second += 1;
    if(counter_second > 60){
      counter_second = 0;
    }
  }
}