// Include the necessary libraries
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "RTClib.h"
#include <DHT.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "I2Cdev.h"
#include "MPU6050.h"

// Define LCD and rotary encoder
LiquidCrystal_I2C lcd(0x27, 16, 2);
int CLK = 2;
int DT = 3;
int currentStateCLK;
int previousStateCLK;

// Define real-time clock
RTC_DS3231 rtc;
DateTime now;

// Define DHT11 temperature and humidity sensor
#define DHTPIN 4     
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Define HC-SR04 distance sensor
#define TRIG_PIN 5
#define ECHO_PIN 6

// Variable to track which screen to display
// 0 = date/time, 1 = temp/humidity, 2 = distance, 3 = GPS, 4 = Acceleration
int displayScreen = 0;

// Variable to hold last update time
unsigned long lastUpdateTime = 0;

// Define GPS module
SoftwareSerial ss(8, 9); // RX, TX
TinyGPSPlus gps;

// Define MPU6050 Accelerometer/Gyroscope
MPU6050 accelgyro(0x69); // I2C address set to 0x69
int16_t ax, ay, az;
float mpsAx, mpsAy, mpsAz;
float sumAx = 0, sumAy = 0, sumAz = 0;
int samples = 0;

void setup() {
  // Start the serial communication for debugging
  Serial.begin(9600);
  Serial.println("Starting serial communication...");

  // Start the software serial for GPS
  ss.begin(9600);
  Serial.println("Starting software serial for GPS...");

  // Set up the LCD Display
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  Serial.println("LCD Display setup done...");

  // Set up the Rotary Encoder
  pinMode(CLK, INPUT_PULLUP);
  pinMode(DT, INPUT_PULLUP);
  Serial.println("Rotary Encoder setup done...");

  // Set up HC-SR04 distance sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.println("HC-SR04 sensor setup done...");

  // Read initial state of the rotary encoder
  previousStateCLK = digitalRead(CLK);

  // Initialize the RTC
  if (! rtc.begin()) {
    lcd.print("Couldn't find RTC");
    while (1);
  } else {
    Serial.println("RTC Initialization done...");
  }

  // Set the RTC time to the compile time if RTC lost power
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Initialize the DHT sensor
  dht.begin();
  Serial.println("DHT sensor initialization done...");

  // Initialize the MPU6050 sensor
  accelgyro.initialize();
  Serial.println("MPU6050 sensor initialization done...");
}

void loop() {
  // Process GPS data
  while (ss.available() > 0) {
    gps.encode(ss.read());
  }

  // Check if rotary encoder has been moved
  currentStateCLK = digitalRead(CLK); // Reads the current state of CLK

  // If CLK changed from high to low (falling edge)
  if (currentStateCLK != previousStateCLK && currentStateCLK == 0) {
    if (digitalRead(DT) != currentStateCLK) {
      // Switch to the next display screen
      displayScreen = (displayScreen + 1) % 5;
    } else {
      // Switch to the previous display screen
      displayScreen = (displayScreen == 0) ? 4 : displayScreen - 1;
    }
    updateDisplay();
  }
  previousStateCLK = currentStateCLK;  // Updates the previous state of CLK with the current state

  // Update the display every second even if the encoder has not been turned
  if (millis() - lastUpdateTime >= 1000) {
    updateDisplay();
    lastUpdateTime = millis();
  }

  // Calculate and store acceleration values, account for biases
  accelgyro.getAcceleration(&ax, &ay, &az);
  sumAx += ax / 16384.0 * 9.81;
  sumAy += ay / 16384.0 * 9.81;
  sumAz += az / 16384.0 * 9.81;
  samples++;

  if (samples >= 100) {  
    mpsAx = (sumAx / samples) + 0.125;
    mpsAy = (sumAy / samples) + 0.045;
    mpsAz = (sumAz / samples) - 9.11;

    // reset sums and sample count
    sumAx = 0;
    sumAy = 0;
    sumAz = 0;
    samples = 0;
  }
}

void printLine(int line, const String& message) {
  // Clear line on LCD and print new message
  lcd.setCursor(0, line);
  lcd.print(message);
  for (int i = message.length(); i < 16; ++i) {
    lcd.print(" ");
  }
}

void updateDisplay() {
  // Update the displayed data based on the current screen selection
  String line1, line2;

  if (displayScreen == 0) {
    // Display date and time
    now = rtc.now();
    line1 = "Date: " + String(now.day(), DEC) + "/" + String(now.month(), DEC) + "/" + String(now.year(), DEC);
    line2 = "Time: " + String(now.hour(), DEC) + ":" + String(now.minute(), DEC) + ":" + String(now.second(), DEC);
    Serial.println("Date and Time updated...");

  } else if (displayScreen == 1) {
    // Display humidity and temperature
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    line1 = "Temp: " + String(t) + "C";
    line2 = "Humid: " + String(h) + "%";
    Serial.println("Humidity and Temperature updated...");

  } else if (displayScreen == 2) {
    // Display distance from HC-SR04 sensor
    long duration, distance;
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    duration = pulseIn(ECHO_PIN, HIGH);
    distance = (duration / 2) / 29.1; // Convert duration to distance
    line1 = "Distance: " + String(distance) + "cm";
    Serial.println("Distance updated...");

  } else if (displayScreen == 3) {
    // Display GPS data
    if (gps.location.isValid()) {
      line1 = "Lat: " + String(gps.location.lat(), 6);
      line2 = "Lon: " + String(gps.location.lng(), 6);
      Serial.println("GPS location updated...");
    } else {
      line1 = "Awaiting GPS...";
      Serial.println("Waiting for GPS location...");
    } 

  } else if (displayScreen == 4) { 
    // Display MPU6050 sensor data
    line1 = "Accel  X: " + String(mpsAx);
    line2 = "Y: " + String(mpsAy) + " Z: " + String(mpsAz);
    Serial.println("Acceleration data updated...");
  }

  printLine(0, line1);
  printLine(1, line2);
}
