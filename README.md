# Hydroponics Automation System 🌿
Hydroponics Automation System is an automated hydroponics control system that uses various sensors to monitor and control environmental conditions such as humidity, temperature, light, and water levels. This system also integrates with Firebase for data storage and sends notifications through Line Notify.

## Key Features 💻
- **Environment Monitoring and Control**: Monitors humidity, temperature, light intensity, and water levels using various sensors.
- **Firebase Integration**: Stores sensor data in Firebase Realtime Database and Firestore.
- **Line Notify Alerts**: Sends system status notifications through Line Notify.
- **Fan and Light Control**: Automatically controls the fan and light based on set parameters.

## Hardware Used ⚙️
- ESP32
- DHT22 Sensor (humidity and temperature)
- Light Sensor (LDR)
- Ultrasonic Sensor (water level)
- RS485 (measures nitrogen, phosphorus, and potassium levels)
- LCD Display
- Relay Module (controls fan and light)

## System Diagram
<p align="center">
  <img src="https://github.com/zacntk/Hydroponics-Automation-System/blob/main/Images/System%20Diagram.png" alt="System Diagram" width="600"/>
</p>

## Flowchart
<p align="center">
  <img src="https://github.com/zacntk/Hydroponics-Automation-System/blob/main/Images/Main.png" alt="Main" width="400"/>
</p>
<p align="center">
  <img src="https://github.com/zacntk/Hydroponics-Automation-System/blob/main/Images/Setup.png" alt="Setup" width="400"/>
</p>
<p align="center">
  <img src="https://github.com/zacntk/Hydroponics-Automation-System/blob/main/Images/Loop.png" alt="Loop" width="400"/>
</p>

## Installation and Usage 📥
### Hardware Setup
1. Connect sensors according to the wiring diagram.
2. Configure input and output pins as specified in the code.

### Software Setup 🔧
1. Install [Arduino IDE](https://www.arduino.cc/en/software).
2. Install the ESP32 board in Arduino IDE following the instructions [here](https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/).
3. Download the necessary libraries:
   - Adafruit GFX
   - Adafruit SSD1306
   - LiquidCrystal_I2C
   - DHT sensor library
   - WiFi
   - HTTPClient
   - Firebase ESP Client
   - ArduinoJson
   - NTPClient
   - SoftwareSerial
4. Download or clone this project from GitHub.
5. Open the .ino file in Arduino IDE.
6. Update WiFi credentials, Firebase details, and Line Notify token in the code.
7. Upload the code to the ESP32 board.

## Usage
1. Upon startup, the system will attempt to connect to the configured WiFi network.
2. Once connected, the system will start collecting data and controlling the environment based on the set parameters.
3. You can monitor and control the system via Firebase and receive notifications through Line Notify.
