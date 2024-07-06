#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <ArduinoJson.h>
#include <string.h>
#include <NTPClient.h>
#include <time.h>
#include <SoftwareSerial.h>

// Global variables
char timeBuffer[9];
bool pumpDiedLineNoti = false, refilling = false, firstLoop = true;
int currentMode = 1, onLight = 0, onLightTime = 0, offLight = 0, offLightTime = 0, onFan = 0, onFanTime = 0, offFan = 0, offFanTime = 0, pumpDiedCount = 0, nitrogenSet, nitrogenDiff, phosphorusSet, phosphorusDiff, potassiumSet, potassiumDiff, fertilizer = 0, pipHole = 3, maxTankCm = 14 - pipHole, fertilizerA[3] = { 118, 35, 18 }, fertilizerB[3] = { 116, 59, 53 };
float humidity = 0.00, temperature = 0.00, avgLight, minTem, maxTem, area = 0.1221, lux = 0.00, minLux, maxLux, waterInTank, waterInTankPrevious = 0, rangeWater = 0, maxTankL = (16 * 24 * maxTankCm) / 1000, inaccurate = 0.11, temValue, humidValue, lightValueS1;
unsigned long previousMillis = 0, previousMillis5Seconds = 0, dataMillis = 0, lightTime, openLightTime = 8 * 60 * 60 * 1000, pauseLightTime = 1 * 60 * 60 * 1000, fanTime, openFanTime = 3 * 60 * 60 * 1000;
String documentPath, mask, name, nameCheck, fertilizerChoose = "...", fertilizerProblem = "...";
const byte nitro[] = { 0x01, 0x03, 0x00, 0x1e, 0x00, 0x01, 0xe4, 0x0c };
const byte phos[] = { 0x01, 0x03, 0x00, 0x1f, 0x00, 0x01, 0xb5, 0xcc };
const byte pota[] = { 0x01, 0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xc0 };
byte nitrogen, phosphorous, potassium;
const char *ssid[] = { "Zack", "Khaopod_Iphone", "ENGR_IOT" }, *password[] = { "1234567890", "12345678", "tse@iot2018" }, *lineNotifyToken = "gGZt93lrXl7Bl5qfjMtnYHmcF05cDzBJMNEWnO31xHi", *lineNotifyAPI = "https://notify-api.line.me/api/notify";
const long interval = 60 * 1000, interval5Seconds = 5000;
int ultrasonicsBrokeCount = 0;

byte values[11];

// Firebase configuration
#define API_KEY "AIzaSyB1Z4AJFUATXr_EwjpzELr2f0rsW6pCAIs"
#define FIREBASE_PROJECT_ID "hydroponicsproject-29703"
#define DATABASE_URL "https://hydroponicsproject-29703-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define USER_EMAIL "admin@hydroponics.com"
#define USER_PASSWORD "admin0123"

//LDR Sensor
#define LIGHTSENSORPIN 32  //ADC1

//DHT22 Sensor
#define DHTTYPE DHT22
#define DHTPIN 33  //Special Functions //ADC1 //From 26
DHT dht(DHTPIN, DHTTYPE);

//RELAY
#define LIGHTPIN 12  //39 //Special Functions //ADC1 //From 15
#define FAN1PIN 13   //35  //Special Functions //ADC1 //From 13
#define FAN2PIN 14   //36  //Special Functions //ADC1 //From 14

//SOIL Sensor
#define RS485_SERIAL Serial2
#define RX 16
#define TX 17
#define RS485_RE 18
#define RS485_DE 19

//ULTRASONICS Sensor
#define ECHO_PIN 5
#define TRIG_PIN 23
#define MAX_DISTANCE maxTankCm * 10

// For LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

//Time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "asia.pool.ntp.org", 3600, 60000);

void connectToKnownWiFi(const char *ssidList[], const char *passwordList[], int listSize, int maxTries = 20) {
  int networkCount = WiFi.scanNetworks();

  if (networkCount == 0) {
    return;
  }
  // Loop through the found networks to find a match
  for (int i = 0; i < networkCount; i++) {
    String foundSSID = WiFi.SSID(i);
    for (int j = 0; j < listSize; j++) {
      if (foundSSID == ssidList[j]) {
        byte smile[8] = {
          0b00000,
          0b00000,
          0b01010,
          0b00000,
          0b10001,
          0b01110,
          0b00000,
          0b00000
        };
        byte backslashChar[8] = {
          B00000,
          B00000,
          B00000,
          B00000,
          B00000,
          B00000,
          B00000,
          B10000
        };
        printLCD("> Connecting", 0);
        printLCD("> To", 1);
        printLCD("> Wifi", 2);
        printLCD("> " + String(ssidList[j]) + "", 3);
        int tries = 0, loading = 1;
        WiFi.begin(ssidList[j], passwordList[j]);
        while (WiFi.status() != WL_CONNECTED && tries < maxTries) {
          lcd.clear();
          printLCD("> Connecting", 0);
          printLCD("> To", 1);
          printLCD("> Wifi", 2);
          printLCD("> " + String(ssidList[j]) + "", 3);

          loading = (loading % 4) + 1;

          // Display different information based on the mode
          switch (loading) {
            case 1:
              printLCD("..", 3);
              break;
            case 2:
              printLCD("....", 3);
              break;
            case 3:
              printLCD("......", 3);
              break;
            case 4:
              printLCD("........", 3);
              break;
          }
          delay(1250);
          tries++;
        }

        if (WiFi.status() == WL_CONNECTED) {
          lcd.clear();
          lcd.createChar(0, smile);
          printLCD("> Connected", 0);
          printLCD("> To", 1);
          printLCD("> Wifi", 2);
          for (int i = 0; i < 20; i++) {
            lcd.setCursor(i, 3);
            lcd.write((uint8_t)0);
          }
          break;  // Exit loop if connected successfully
        } else {
          return;
        }
      }
    }
  }
}

void setup() {
  // Begin serial communication
  Serial.begin(115200);

  // Initialize LCD
  lcd.begin();
  lcd.backlight();

  // Initialize DHT sensor
  dht.begin();

  // Initialize RS485 communication
  // Initialize RS485 communication
  RS485_SERIAL.begin(4800, SERIAL_8N1, RX, TX);  // RX Pin: 16, TX Pin: 17
  pinMode(RS485_RE, OUTPUT);
  pinMode(RS485_DE, OUTPUT);

  // Set Ultrasonics
  pinMode(TRIG_PIN, OUTPUT);  // Sets the trigPin as an Output
  pinMode(ECHO_PIN, INPUT);   // Sets the echoPin as an Input

  // Set relay pins as output
  pinMode(FAN1PIN, OUTPUT);
  pinMode(FAN2PIN, OUTPUT);
  pinMode(LIGHTPIN, OUTPUT);

  // Turn off fans and light initially
  digitalWrite(FAN1PIN, HIGH);
  digitalWrite(FAN2PIN, HIGH);
  digitalWrite(LIGHTPIN, LOW);

  // Set WiFi mode
  WiFi.mode(WIFI_STA);

  // Connect to known WiFi network
  connectToKnownWiFi(ssid, password, sizeof(ssid) / sizeof(ssid[0]));

  // If WiFi connection fails, set default values
  if (WiFi.status() != WL_CONNECTED) {
    name = "Green Cos Letture";
    nitrogenSet = 150;
    phosphorusSet = 20;
    potassiumSet = 50;
    minLux = 50000;
    maxLux = 70000;
    minTem = 10;
    maxTem = 24;
  } else {
    // Initialize Firebase and time client if WiFi connection successful
    initializeFirebase();
    timeClient.begin();
  }

  delay(1000);

  lcd.clear();
  printLCD("> Hydroponic", 0);
  printLCD("> System Setup", 1);
  printLCD("> Success", 3);
  yield();
}

void loop() {
  unsigned long currentMillis = millis();  // Subtracting 5000 milliseconds for time adjustment
  timeClient.update();

  // Call the function to convert and format the time, updating the global timeBuffer
  convertMillisToTime(currentMillis);

  Serial.println(timeBuffer);  // Print the formatted time

  // Read humidity
  humidity = readHumidity();

  // Read Temperature
  temperature = readTemperature();

  // Read Light intensity
  lux = readLightIntensity();

  // Read Nutrient Values
  nitrogen = readNitrogen();
  delay(250);
  phosphorous = readPhosphorous();
  delay(250);
  potassium = readPotassium();
  delay(250);

  // Read Ultrasonics and Calculate Fertilizer
  readWaterAndCalculateFertilizer(nitrogen, phosphorous, potassium);

  // Calculate Temperature
  controlFan(temperature);

  // Check if a minute has passed
  if (currentMillis - previousMillis >= interval) {
    hourlyProcess(humidity, temperature, lux, nitrogen, phosphorous, potassium);
    // Save the current time for hourly check
    previousMillis = currentMillis;
  }

  // Check WiFi connection status
  if (WiFi.status() != WL_CONNECTED) {
    connectToKnownWiFi(ssid, password, sizeof(ssid) / sizeof(ssid[0]));
    if (WiFi.status() == WL_CONNECTED) {
      initializeFirebase();
      timeClient.begin();
    }
  }

  if (firstLoop) {
    float speedOfSound = 331.3 * sqrt(1 + temperature / 273.15);
    int duration = getStablePing();
    float distance = (duration / 1000000.0) * speedOfSound / 2;
    if (distance == 0) {
      waterInTank = 0;
      waterInTankPrevious = waterInTank;
    } else {
      waterInTank = (maxTankL - ((16 * 24 * distance) / 1000)) + inaccurate;
      waterInTankPrevious = waterInTank;
    }
    firstLoop = false;
  }

  switchScreenMode(nitrogen, phosphorous, potassium);
  sendToRTDB(fertilizerChoose, fertilizer, fertilizerProblem, humidity, lux, name, nitrogen, phosphorous, potassium, temperature, waterInTank);
  fetchFirestoreData();

  delay(1000);  // Delay for 1 second
}

//Process
byte readRS485(const byte *cmd, byte *result) {
  byte maxRetries = 10;            // จำนวนครั้งสูงสุดที่จะพยายามอ่านค่า
  byte delayBetweenRetries = 100;  // เวลาที่รอคอยระหว่างแต่ละรอบการตรวจสอบ

  byte retries = 0;

  while (retries < maxRetries) {
    // ตั้งค่าโหมดส่งข้อมูล RS485
    digitalWrite(RS485_DE, HIGH);
    digitalWrite(RS485_RE, HIGH);
    delay(10);

    // ส่งคำสั่ง RS485
    RS485_SERIAL.write(cmd, sizeof(nitro));
    RS485_SERIAL.flush();

    // ตั้งค่าโหมดรับข้อมูล RS485
    digitalWrite(RS485_DE, LOW);
    digitalWrite(RS485_RE, LOW);

    // รอคอยการตอบสนอง
    delay(delayBetweenRetries);

    // ตรวจสอบว่าได้รับข้อมูลหรือไม่
    byte bytesRead = RS485_SERIAL.available();
    if (bytesRead == 7) {  // สมมติว่าต้องการ 7 ไบต์
      RS485_SERIAL.readBytes(result, bytesRead);
      return result[4];  // คืนค่าที่ต้องการ
    }

    retries++;  // เพิ่มจำนวนครั้งที่พยายาม
  }

  return 0;  // หากครบจำนวนครั้งที่กำหนดแล้ว ยังไม่ได้ผลลัพธ์
}

byte readNitrogen() {
  byte result[7];
  return readRS485(nitro, result);
}

byte readPhosphorous() {
  byte result[7];
  return readRS485(phos, result);
}

byte readPotassium() {
  byte result[7];
  return readRS485(pota, result);
}

float readHumidity() {
  humidValue = dht.readHumidity();
  return humidValue;
}

float readTemperature() {
  temValue = dht.readTemperature();
  return temValue;
}

float readLightIntensity() {
  int lightSensorValue = analogRead(LIGHTSENSORPIN);
  lux = map(lightSensorValue, 0, 4095, 0, 40950);
  return lux;
}

void readWaterAndCalculateFertilizer(byte nitrogen, byte phosphorous, byte potassium) {
  DynamicJsonDocument jsonDocument(1024);
  if (getFirestoreDocument("Setup/Plant", "pumpStatus", jsonDocument)) {
    refilling = jsonDocument["fields"]["pumpStatus"]["booleanValue"];
  } else {
    Serial.println("Failed to get pumpStatus data");
  }

  float speedOfSound = 331.3 * sqrt(1 + temperature / 273.15);
  int duration = getStablePing();
  float distance = (duration / 1000000.0) * speedOfSound / 2;
  if (distance == 0) {
    waterInTank = 0;
  } else {
    waterInTank = (maxTankL - ((16 * 24 * distance) / 1000)) + inaccurate;
  }

  if (refilling) {
    waterInTankPrevious = waterInTank;
  }

  Serial.println("waterInTank: " + String(waterInTank) + "|| waterInTankPrevious: " + String(waterInTankPrevious) + "|| rangeWater: " + String(rangeWater));
  Serial.println("maxTankCm: " + String(maxTankCm) + "|| distance: " + distance + "|| GetSatblePing: " + getStablePing());

  // Calculate fertilizer only if the tank is not empty
  if (waterInTank > 0) {
    // Determine fertilizer type and amount
    if (nitrogen < nitrogenSet && phosphorous > phosphorusSet && potassium > potassiumSet) {
      fertilizerProblem = "Nitrogen";
      fertilizerChoose = "A";
      // Calculate fertilizer amount
      fertilizer = round(waterInTank) * 9;
    } else if (nitrogen > nitrogenSet && phosphorous < phosphorusSet && potassium > potassiumSet) {
      fertilizerProblem = "Phosphorus";
      fertilizerChoose = "B";
      // Calculate fertilizer amount
      fertilizer = round(waterInTank) * 9;
    } else if (nitrogen > nitrogenSet && phosphorous > phosphorusSet && potassium < potassiumSet) {
      fertilizerProblem = "Potassium";
      fertilizerChoose = "B";
      // Calculate fertilizer amount
      fertilizer = round(waterInTank) * 9;
    } else if (nitrogen < nitrogenSet && phosphorous < phosphorusSet && potassium < potassiumSet) {
      fertilizerProblem = "All";
      fertilizerChoose = "A&B";
      // Calculate fertilizer amount
      fertilizer = round(waterInTank) * 9;
    } else {
      // No need for fertilizer
      fertilizerProblem = "...";
      fertilizerChoose = "...";
      fertilizer = 0;
    }
  } else {
    // Tank is empty, no need for fertilizer
    fertilizerProblem = "...";
    fertilizerChoose = "...";
    fertilizer = 0;
  }
}

unsigned long getStablePing() {
  unsigned long durationInFunc;
  unsigned long totalDuration = 0;
  const int numReadings = 5;  // number of readings to average

  for (int i = 0; i < numReadings; i++) {
    // Clear the trigPin
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);  // ensure it is low for a short period

    // Sets the trigPin on HIGH state for 10 microseconds
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);  // 10 microseconds pulse
    digitalWrite(TRIG_PIN, LOW);

    // Reads the echoPin, returns the sound wave travel time in microseconds
    durationInFunc = pulseIn(ECHO_PIN, HIGH);

    // Accumulate the duration
    totalDuration += durationInFunc;

    // Short delay between readings to ensure stable measurements
    delay(50);
  }

  // Return the average duration
  return totalDuration / numReadings;
}

void controlFan(float temperature) {
  if (temperature >= maxTem) {
    //Hot Temperature
    if (digitalRead(FAN1PIN) == HIGH || digitalRead(FAN2PIN) == HIGH) {
      if (offFan >= offFanTime || firstLoop == true) {
        // Turn on Fan
        digitalWrite(FAN1PIN, LOW);
        digitalWrite(FAN2PIN, LOW);
        offFan = 0;
      }
    }
  } else {
    digitalWrite(FAN1PIN, HIGH);
    digitalWrite(FAN2PIN, HIGH);
    onFan = 0;  // Reset onFan counter
  }
}

void hourlyProcess(float humidity, float temperature, float lux, float nitrogen, float phosphorous, float potassium) {
  //Check Fan Time
  if (digitalRead(FAN1PIN) == LOW || digitalRead(FAN2PIN) == LOW) {
    onFan += 1;
    if (onFan >= onFanTime) {
      digitalWrite(FAN1PIN, HIGH);
      digitalWrite(FAN2PIN, HIGH);
      onFan = 0;  // Reset onFan counter
    }
  } else {
    offFan += 1;
  }


  //Check Light Time
  if (digitalRead(LIGHTPIN) == LOW) {
    onLight += 1;
    if (onLight >= 8) {
      // Turn off Light
      digitalWrite(LIGHTPIN, HIGH);
      onLight = 0;
    }
  } else {
    offLight += 1;
    if (offLight >= 1) {
      // Turn on Light
      digitalWrite(LIGHTPIN, LOW);
      offLight = 0;
    }
  }

  // Check Nutrient Values
  if (nitrogen <= nitrogenSet || phosphorous <= phosphorusSet || potassium <= potassiumSet) {
    sendLineMessage("Nutrient Levels", "Nutrient levels below threshold");
  }

  // Check NaN value from Light sensor
  if (isnan(lightValueS1)) {
    sendLineMessage("Light Sensor", "There must be a problem with the light sensor");
  }

  // Check NaN value from Temperature sensor
  if (isnan(temValue)) {
    sendLineMessage("Temperature Sensor", "There must be a problem with the Temperature sensor");
  }

  if (getStablePing() == 0) {
    ultrasonicsBrokeCount += 1;
    if (ultrasonicsBrokeCount == 2) {
      sendLineMessage("Ultrasonics Sensor", "There must be a problem with the ultrasonics sensor");
      ultrasonicsBrokeCount = 0;
    }
  } else {
    ultrasonicsBrokeCount = 0;
    if (waterInTank < 2.5) {
      sendLineMessage("Water", "There is too little water in the system");
    }
  }

  if (waterInTankPrevious - waterInTank < rangeWater) {
    pumpDiedCount += 1;
    Serial.println("pumpDiedCount: " + String(pumpDiedCount));
    //Set pumpDied LineNotify to true
    if (pumpDiedCount >= 2) {
      sendLineMessage("Water Pump", "There is a problem with the water pump");
      pumpDiedCount = 0;
    }
    waterInTankPrevious = waterInTank;
  } else {
    pumpDiedCount -= 1;
    if (pumpDiedCount < 0) {
      pumpDiedCount = 0;
    }
    Serial.println("pumpDiedCount: " + String(pumpDiedCount));
    waterInTankPrevious = waterInTank;
  }

  rangeWater = waterInTankPrevious - waterInTank;

  sendLogToFirebase(humidity, temperature, lux, nitrogen, phosphorous, potassium);
}
//END Process

//Firebase
void initializeFirebase() {
  // Clear LCD and display initialization status
  lcd.clear();
  printLCD("> Firebase", 0);
  printLCD("> Connecting", 1);

  // Display Firebase client version
  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

  // Set API key and user credentials
  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.database_url = DATABASE_URL;

  // Set the token status callback
  config.token_status_callback = tokenStatusCallback;  // Default callback

  // Configure Firebase connection settings
  Firebase.reconnectNetwork(true);     // Automatically reconnect on network loss
  fbdo.setBSSLBufferSize(4096, 1024);  // Adjust buffer sizes if needed
  fbdo.setResponseSize(2048);          // Set response size

  int firebaseCount = 0;
  // Initialize Firebase with the configuration and authentication
  Firebase.begin(&config, &auth);

  while (!Firebase.ready()) {
    // Initialize Firebase with the configuration and authentication
    Firebase.begin(&config, nullptr);
    if (firebaseCount == 20) {
      Serial.println("Failed to initialize Firebase");
      printLCD("> Error", 0);
      return;
    }  // Early exit if initialization fails
    firebaseCount += 1;
  }

  // Display success message on LCD
  printLCD("> Success", 3);
  delay(2000);
  lcd.clear();

  // Fetch data from Firestore
  fetchFirestoreData();
}

void fetchFirestoreData() {
  DynamicJsonDocument jsonDocument(1024);

  // Fetch data for Soil
  if (getFirestoreDocument("Setup/Plant", "Soil", jsonDocument)) {
    nitrogenSet = jsonDocument["fields"]["Soil"]["mapValue"]["fields"]["Nitrogen"]["integerValue"];
    phosphorusSet = jsonDocument["fields"]["Soil"]["mapValue"]["fields"]["Phosphorus"]["integerValue"];
    potassiumSet = jsonDocument["fields"]["Soil"]["mapValue"]["fields"]["Potassium"]["integerValue"];
  } else {
    Serial.println("Failed to get Soil data");
  }

  // Fetch data for Time
  if (getFirestoreDocument("Setup/Plant", "Fan", jsonDocument)) {
    offFanTime = jsonDocument["fields"]["Fan"]["mapValue"]["fields"]["offTime"]["integerValue"];
    onFanTime = jsonDocument["fields"]["Fan"]["mapValue"]["fields"]["onTime"]["integerValue"];
  } else {
    Serial.println("Failed to get Fan data");
  }

  // Fetch data for Light
  if (getFirestoreDocument("Setup/Plant", "Light", jsonDocument)) {
    minLux = jsonDocument["fields"]["Light"]["mapValue"]["fields"]["minLux"]["integerValue"];
    maxLux = jsonDocument["fields"]["Light"]["mapValue"]["fields"]["maxLux"]["integerValue"];
    offLightTime = jsonDocument["fields"]["Light"]["mapValue"]["fields"]["offTime"]["integerValue"];
    onLightTime = jsonDocument["fields"]["Light"]["mapValue"]["fields"]["onTime"]["integerValue"];
  } else {
    Serial.println("Failed to get Light data");
  }

  // Fetch data for Temperature
  if (getFirestoreDocument("Setup/Plant", "Temperature", jsonDocument)) {
    minTem = jsonDocument["fields"]["Temperature"]["mapValue"]["fields"]["minTem"]["integerValue"];
    maxTem = jsonDocument["fields"]["Temperature"]["mapValue"]["fields"]["maxTem"]["integerValue"];
  } else {
    Serial.println("Failed to get Temperature data");
  }

  // Fetch data for Name
  if (getFirestoreDocument("Setup/Plant", "Name", jsonDocument)) {
    name = jsonDocument["fields"]["Name"]["stringValue"].as<String>();
  } else {
    Serial.println("Failed to get Name data");
  }
}

bool getFirestoreDocument(const String &documentPath, const String &mask, DynamicJsonDocument &jsonDocument) {
  // Get a Firestore document and handle errors
  if (Firebase.Firestore.getDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath.c_str(), mask.c_str())) {
    deserializeJson(jsonDocument, fbdo.payload());  // Parse the response
    return true;                                    // Document retrieved successfully
  } else {
    Serial.println(fbdo.errorReason());  // Print the error reason
    return false;                        // Failed to retrieve document
  }
}

bool setRTDBValue(FirebaseData &fbdo, const char *path, FirebaseJson &data, int retries = 3, int delayBetweenRetries = 200) {
  for (int i = 0; i < retries; i++) {
    if (Firebase.RTDB.setJSON(&fbdo, path, &data)) {  // Use pointer to FirebaseJson
      return true;                                    // Success
    }
    delay(delayBetweenRetries);
  }
  return false;  // Failure after retries
}

void sendToRTDB(String fertilizerChoose, int fertilizer, String fertilizerProblem, float humidity, float lux, String name, float nitrogen, float phosphorous, float potassium, float temperature, float waterInTank) {
  Serial.println("----------");

  FirebaseJson data;
  data.set("Fertilizer/Choose", fertilizerChoose);
  data.set("Fertilizer/Amount", fertilizer);
  data.set("Fertilizer/Problem", fertilizerProblem);
  data.set("Humidity", humidity);
  data.set("Lux", lux);
  data.set("Name", name);
  data.set("Soil/Nitrogen", nitrogen);
  data.set("Soil/Phosphorus", phosphorous);
  data.set("Soil/Potassium", potassium);
  data.set("Water", waterInTank);
  data.set("Temperature", temperature);

  if (digitalRead(FAN1PIN) == LOW || digitalRead(FAN2PIN) == LOW) {
    data.set("Fan/Status", true);
    data.set("Fan/Time", onFan);
  } else {
    data.set("Fan/Status", false);
    data.set("Fan/Time", offFan);
  }

  if (digitalRead(LIGHTPIN) == LOW) {
    data.set("Light/Status", true);
    data.set("Light/Time", onLight);
  } else {
    data.set("Light/Status", false);
    data.set("Light/Time", offLight);
  }

  const char *path = "/realtimeData";

  if (setRTDBValue(fbdo, path, data)) {
    Serial.println("Data sent successfully");
  } else {
    Serial.println("Failed to send data");
    Serial.println(fbdo.errorReason());
  }

  Serial.println("----------");
}

void sendLogToFirebase(double humidity, double temperature, double lux, double nitrogen, double phosphorus, double potassium) {
  if (Firebase.ready() && (millis() - dataMillis > 60000 || dataMillis == 0)) {
    dataMillis = millis();
    FirebaseJson log;

    log.set("fields/Humidity/doubleValue", String(humidity));
    log.set("fields/Temperature/doubleValue", String(temperature));
    log.set("fields/Lux/doubleValue", String(lux));

    log.set("fields/Soil/mapValue/fields/Nitrogen/doubleValue", String(nitrogen));
    log.set("fields/Soil/mapValue/fields/Phosphorus/doubleValue", String(phosphorus));
    log.set("fields/Soil/mapValue/fields/Potassium/doubleValue", String(potassium));

    // Update the time client to get the latest time
    timeClient.update();
    unsigned long epochTime = timeClient.getEpochTime();

    // Convert epoch time to ISO 8601 format
    char timestampString[25];
    sprintf(timestampString, "%04d-%02d-%02dT%02d:%02d:%02dZ",
            year(epochTime), month(epochTime), day(epochTime),
            hour(epochTime), minute(epochTime), second(epochTime));

    log.set("fields/Time/timestampValue", timestampString);
    String documentPath = "History_Demo/" + String(epochTime);

    Serial.print("Create document... ");

    if (Firebase.Firestore.createDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath.c_str(), log.raw()))
      Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());
    else
      Serial.println(fbdo.errorReason());
  }
  dataMillis = 0;
}

int year(unsigned long int t) {
  time_t timeVal = static_cast<time_t>(t);
  return (1900 + gmtime(&timeVal)->tm_year);
}

int month(unsigned long int t) {
  time_t timeVal = static_cast<time_t>(t);
  return (1 + gmtime(&timeVal)->tm_mon);
}

int day(unsigned long int t) {
  time_t timeVal = static_cast<time_t>(t);
  return gmtime(&timeVal)->tm_mday;
}

int hour(unsigned long int t) {
  time_t timeVal = static_cast<time_t>(t);
  return gmtime(&timeVal)->tm_hour - 1;
}

int minute(unsigned long int t) {
  time_t timeVal = static_cast<time_t>(t);
  return gmtime(&timeVal)->tm_min;
}

int second(unsigned long int t) {
  time_t timeVal = static_cast<time_t>(t);
  return gmtime(&timeVal)->tm_sec;
}

void sendLineMessage(const char *message, const char *description) {
  HTTPClient http;
  http.begin(lineNotifyAPI);
  http.addHeader("Authorization", "Bearer " + String(lineNotifyToken));
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  String payload = "message=\nProblem : " + String(message) + "\nDescription : " + String(description);
  int httpResponseCode = http.POST(payload);

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    Serial.println("Message sent successfully");
  } else {
    Serial.print("Error sending message. HTTP Response code: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}
//END Firebase

//LCD
void switchScreenMode(byte nitrogen, byte phosphorous, byte potassium) {
  // Increment the mode or reset to 1 if it reaches the maximum mode
  currentMode = (currentMode % 5) + 1;

  // Clear the LCD screen
  lcd.clear();

  // Display different information based on the mode
  switch (currentMode) {
    case 1:
      printLCD("> Information", 0);
      printLCD("> Tem: " + String(temperature) + " C", 1);
      printLCD("> Lux: " + String(lux) + "", 2);
      printLCD("> Humid: " + String(humidity) + " %", 3);
      break;
    case 2:
      printLCD("> Nitro: " + String(nitrogen), 0);
      printLCD("> Phospho: " + String(phosphorous), 1);
      printLCD("> Potass: " + String(potassium), 2);
      printLCD("> Unit : mg/kg.", 3);
      break;
    case 3:
      if (digitalRead(LIGHTPIN) == LOW) {
        printLCD("> Light: ON", 0);
        printLCD("> Time: " + String(onLight) + "/" + String(onLightTime) + " hr", 1);
      } else {
        printLCD("> Light: OFF", 0);
        printLCD("> Time: " + String(offLight) + "/" + String(offLightTime) + " hr", 1);
      }

      if (digitalRead(FAN1PIN) == LOW && digitalRead(FAN2PIN) == LOW) {
        printLCD("> Fan: ON", 2);
        printLCD("> Time: " + String(onFan) + "/" + String(onFanTime) + " hr", 3);
      } else {
        printLCD("> Fan: OFF", 2);
        printLCD("> Time: " + String(offFan) + " hr", 3);
      }

      break;
    case 4:
      printLCD("> Water: " + String(waterInTank) + " l", 0);
      printLCD("> Problem: " + String(fertilizerProblem), 1);
      printLCD("> Fertilize: " + String(fertilizerChoose), 2);
      printLCD("> Amount: " + String(fertilizer) + " cc", 3);
      break;
    case 5:
      if (WiFi.status() != WL_CONNECTED) {
        printLCD("> Wifi : X", 0);
        printLCD("> Name : X", 1);
        printLCD("> Time :", 2);
        printLCD("> " + String(timeBuffer), 3);
      } else {
        printLCD("> Wifi : /", 0);
        printLCD("> Name : " + WiFi.SSID(), 1);
        printLCD("> Time :", 2);
        printLCD("> " + String(timeBuffer), 3);
      }
      break;
  }
}

void printProgressBar(float progress, int maxBarLength) {
  // Ensure progress is within the range [0, 100]
  progress = constrain(progress, 0.0, 100.0);

  // Calculate the length of the progress bar
  int barLength = int(progress / (100.0 / maxBarLength));

  printLCD("> Progress Bar", 2);
  printLCD("[", 3);
  lcd.setCursor(1, 3);

  for (int i = 0; i <= maxBarLength; i++) {
    if (i < barLength) {
      lcd.print("=");
    } else {
      lcd.print(" ");
    }
  }
  lcd.print("]");
}

void printLCD(String text, int row) {
  lcd.setCursor(0, row);
  lcd.print(text);
}

void convertMillisToTime(unsigned long currentMillis) {
  unsigned long totalSeconds = currentMillis / 1000;  // Convert milliseconds to seconds
  unsigned long seconds = totalSeconds % 60;          // Get the remaining seconds
  unsigned long totalMinutes = totalSeconds / 60;     // Total minutes
  unsigned long minutes = totalMinutes % 60;          // Get the remaining minutes
  unsigned long hours = totalMinutes / 60;            // Total hours

  // Format the time in hh:mm:ss format
  snprintf(timeBuffer, sizeof(timeBuffer), "%02lu:%02lu:%02lu", hours, minutes, seconds);
}
//END LCD