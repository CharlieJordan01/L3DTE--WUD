#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "RTClib.h"
#include <DHT.h>

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

// Variable to track which screen to display
int displayScreen = 0;  // 0 = date/time, 1 = temp/humidity

void setup() {
  // Set up the LCD Display
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);

  // Set up the Rotary Encoder
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  
  // Read initial state
  previousStateCLK = digitalRead(CLK);

  // Initialize the RTC
  if (! rtc.begin()) {
    lcd.print("Couldn't find RTC");
    while (1);
  }

  if (rtc.lostPower()) {
    // Uncomment the following line to set the time to the compile time
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

    // Uncomment the following line and adjust the date and time to set an explicit date & time,
    // for example to set January 21, 2024 at 3am you would call:
    // rtc.adjust(DateTime(2024, 1, 21, 3, 0, 0));
  }

  // Initialize the DHT sensor
  dht.begin();
}

void loop() {
  currentStateCLK = digitalRead(CLK); // Reads the current state of CLK

  // If CLK changed from high to low (falling edge)
  if (currentStateCLK != previousStateCLK && currentStateCLK == 0) {
    if (digitalRead(DT) != currentStateCLK) {
      // If the DT state is different than the CLK state, that means the encoder is rotating clockwise (up)
      // Switch to the next display screen
      displayScreen = (displayScreen + 1) % 2;
    } else {
      // If the DT state is the same as the CLK state, the encoder is rotating counter-clockwise
      // Switch to the previous display screen
      displayScreen = (displayScreen == 0) ? 1 : 0;
    }
    // Update display immediately when encoder is turned
    updateDisplay();
  }
  previousStateCLK = currentStateCLK;  // Updates the previous state of CLK with the current state

  // Update the display every second even if the encoder has not been turned
  if (millis() % 1000 == 0) {
    updateDisplay();
  }
}

void updateDisplay() {
  // Based on the current screen, display the relevant information
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
  } else {
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
  }
}
