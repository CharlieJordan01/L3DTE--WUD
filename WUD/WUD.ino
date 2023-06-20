#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "RTClib.h"
#include <DHT.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "I2Cdev.h"
#include "MPU6050.h"

// Define the LCD Display (assuming it's connected via I2C)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Define the Rotary Encoder pins (CLK, DT)
int CLK = 2;
int DT = 3;

int currentStateCLK;
int previousStateCLK;

RTC_DS3231 rtc;
DateTime now;

#define DHTPIN 4     // what digital pin the DHT11 is connected to
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Pins for HC-SR04
#define TRIG_PIN 5
#define ECHO_PIN 6

// Variable to track which screen to display
int displayScreen = 0;  // 0 = date/time, 1 = temp/humidity, 2 = distance, 3 = GPS, 4 = Acceleration


// SoftwareSerial for GPS
SoftwareSerial ss(8, 9); // RX, TX

// The TinyGPS++ object
TinyGPSPlus gps;

// MPU6050 Accelerometer/Gyroscope
MPU6050 accelgyro(0x69); // I2C address set to 0x69

int16_t ax, ay, az;
float mpsAx, mpsAy, mpsAz;
float sumAx = 0, sumAy = 0, sumAz = 0;
int samples = 0;

void setup() {
  // Start the serial communication
  Serial.begin(9600);

  // Start the software serial for GPS
  ss.begin(9600);

  // Set up the LCD Display
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);

  // Set up the Rotary Encoder
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);

  // Set up HC-SR04 sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Read initial state
  previousStateCLK = digitalRead(CLK);

  // Initialize the RTC
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Initialize the DHT sensor
  dht.begin();

  // Initialize the MPU6050 sensor
  accelgyro.initialize();
}


void loop() {
  while (ss.available() > 0) {
    gps.encode(ss.read());
  }

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
  if (millis() % 1000 == 0) {
    updateDisplay();
  }
}


void updateDisplay() {
  // Data update for MPU6050 sensor
  accelgyro.getAcceleration(&ax, &ay, &az);
  sumAx += ax / 16384.0 * 9.81;
  sumAy += ay / 16384.0 * 9.81;
  sumAz += az / 16384.0 * 9.81;
  samples++;

  if (samples >= 10) {  // assuming loop() runs about 10 times per second
    mpsAx = sumAx / samples;
    mpsAy = sumAy / samples;
    mpsAz = sumAz / samples;

    // reset sums and sample count
    sumAx = 0;
    sumAy = 0;
    sumAz = 0;
    samples = 0;
  }

  // Display updates based on the current screen
  if (displayScreen == 0) {
    // Display date and time
    now = rtc.now();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Date: ");
    lcd.print(now.day(), DEC);
    lcd.print('/');
    lcd.print(now.month(), DEC);
    lcd.print('/');
    lcd.print(now.year(), DEC);
    lcd.setCursor(0, 1);
    lcd.print("Time: ");
    lcd.print(now.hour(), DEC);
    lcd.print(':');
    lcd.print(now.minute(), DEC);
    lcd.print(':');
    lcd.print(now.second(), DEC);

  } else if (displayScreen == 1) {
    // Display humidity and temperature
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(t);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("Humid: ");
    lcd.print(h);
    lcd.print("%");

  } else if (displayScreen == 2) {
    // Display distance
    long duration, distance;
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    duration = pulseIn(ECHO_PIN, HIGH);
    distance = (duration / 2) / 29.1; // Speed of sound wave travelling will be 340m/s = 29 us/cm. So, convert to cm.

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Distance: ");
    lcd.print(distance);
    lcd.print("cm");

  } else if (displayScreen == 3) {
    // Display GPS location 
    if (gps.location.isValid()) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Lat: ");
      lcd.print(gps.location.lat(), 6);
     lcd.setCursor(0, 1);
      lcd.print("Lon: ");
      lcd.print(gps.location.lng(), 6);
    } else {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Awaiting GPS...");
   } 

  } else if (displayScreen == 4) { 
    // Display acceleration
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Accel  X: ");
    lcd.print(mpsAx);
    lcd.setCursor(0, 1);
    lcd.print("Y: ");
    lcd.print(mpsAy);
    lcd.print(" Z: ");
    lcd.print(mpsAz);
  }
} 