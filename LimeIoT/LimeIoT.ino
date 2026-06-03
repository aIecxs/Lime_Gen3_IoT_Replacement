#include <CRC16.h>
#include <CRC.h>
#include <NimBLEDevice.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include "display.h"

#define SCOOTER_NAME "lme-UJEYGJA"
const uint32_t BLE_PASSWORD = 123456; // 6-digit TK BLE legacy pairing

// hardware config: uncomment here if module installed
//#define CONFIG_I2S // MAX98357A Audio I2S Output
//#define CONFIG_IMU // SW-420 tilt sensor
//#define CONFIG_PSM // MP4560 DC-DC Converter with EN (Enable) Pin (power-saving mode)
//#define CONFIG_PNP // Display has pnp transistor (inverted Pin)
//#define CONFIG_TAG // ST17H66 BLE Beacon unlocking allowed

// Set pins
#define LED_BUILTIN 2
#define TX0 1  // UART0 - Controller
#define RX0 3
#define RX1 9  // UART1 - SPI Flash (do not use)
#define TX1 10
#define RX2 16 // UART2 - Display
#define TX2 17
// RX2 17 = ESP32-DevKitC V2, RX2 4 = WeMos D32 / Pro, RX2 5 = WeMos Lolin32, RX2 2 = WeMos Lolin32 Pro (32 pin)
// TX2 5 = ESP32-DevKitC V2, ** WeMos D32 / Pro, TX2 18 = WeMos Lolin32, TX2 0 = WeMos Lolin32 Pro (32 pin)
#define RX3 22 // UART3 - Debug
#define TX3 23
#define LOCK_PIN 12 // 13 = WeMos D32 / Pro
#define BUZZER_PIN 25 // 26 = WeMos D32 / Pro
#define DISPLAY_PIN GPIO_NUM_21
#define SHOCK_PIN GPIO_NUM_14 // GPIO_NUM_12 = WeMos D32 / Pro
#define BOOT_PIN GPIO_NUM_33 // GPIO_NUM_25 = WeMos D32 / Pro, GPIO_NUM_35 = WeMos Lolin32

// Adafruit I2S Amplifier MAX98357A
#define WCLK_PIN 25 // 26 = WeMos D32 / Pro
#define BCLK_PIN 26 // 27 = WeMos D32 / Pro
#define DOUT_PIN 27 // 14 = WeMos D32 / Pro

BLEScan *pBLEScan = nullptr;
BLEServer *pServer = nullptr;
BLECharacteristic *pMainCharacteristic = nullptr;
BLECharacteristic *pSettingsCharacteristic = nullptr;
BLECharacteristic *pDebugCharacteristic = nullptr;


// Display Status Codes
const String DISPLAY_STATUS_SCAN = "21";
const String DISPLAY_STATUS_ERROR = "22";
const String DISPLAY_STATUS_PAUSED = "23";
const String DISPLAY_STATUS_LOCKED = "24";
const String DISPLAY_STATUS_DONE = "25";
const String DISPLAY_STATUS_CHARGING = "26";
const String DISPLAY_STATUS_DRIVING = "31";
const String DISPLAY_STATUS_DRIVING_LOW_BATTERY = "41";
const String DISPLAY_STATUS_DRIVING_ALERT = "42";
const String DISPLAY_STATUS_DRIVING_NO_PARKING = "43";
const String DISPLAY_STATUS_DRIVING_NO_RIDING = "44";
const String DISPLAY_STATUS_DRIVING_MAX_SPEED = "45";
const String DISPLAY_STATUS_UPGRADING = "51";


// Controller Codes
byte hearthBeatEscByte[16] = { 0x46, 0x43, 0x11, 0x01, 0x00, 0x08, 0x4C, 0x49, 0x4D, 0x45, 0x42, 0x49, 0x4B, 0x45, 0xBE, 0x8A };
byte onEscByte[9] = { 0x46, 0x43, 0x16, 0x61, 0x00, 0x01, 0xF1, 0xF2, 0x8F };
byte offEscByte[9] = { 0x46, 0x43, 0x16, 0x61, 0x00, 0x01, 0xF0, 0xE2, 0xAE };
byte lightOnEscByte[9] = { 0x46, 0x43, 0x16, 0x12, 0x00, 0x01, 0xF1, 0x2B, 0x26 };
byte lightOffEscByte[9] = { 0x46, 0x43, 0x16, 0x12, 0x00, 0x01, 0xF0, 0x3B, 0x07 };
byte lightBlinkEscByte[9] = { 0x46, 0x43, 0x16, 0x13, 0x00, 0x01, 0x06, 0xC2, 0x6A };


// Status
volatile unsigned long lastDisconnected = 0;
volatile bool deviceConnected = false;
bool oldDeviceConnected = false;
bool isDisconnected = false;
bool commandIsSending = false;
volatile bool isMP3Playing = false;
bool isBooted = false;
bool isIdle = false;
uint8_t isUnlocked = 0;
uint8_t controllerIsOn = 0;
uint8_t lightIsOn = 0;
uint8_t unlockForEver = 0;
int speed = 0;
volatile uint8_t alarmIsOn = 0;
bool shockState = false;
bool prevShockState = false;
uint8_t throttle = 1;
byte LEDmode = 0x10;
byte battery = 0x00;
byte isCharging = 0x00;
String customDisplayStatus = "";
volatile bool isUpgrading = false;
volatile float upgradingProgress = 0.0;

#ifdef CONFIG_TAG
typedef struct beacon_t {
  bool connected;
  bool proximity;
  bool button;
  int rssi;
  unsigned long time;
} beacon_t;
volatile beacon_t beacon;
#endif

// Set Settings
int alarm_delay = 200;
int alarm_freq = 3000;
int alarm_reps = 15;
int max_speed = 28;
RTC_DATA_ATTR volatile byte alarm_cnt = 0;
RTC_DATA_ATTR byte lastBattery = 0x00;

#ifndef CONFIG_IMU
#define BUTTON_PIN_BITMASK (1ULL << BOOT_PIN)
#else
#define BUTTON_PIN_BITMASK ((1ULL << SHOCK_PIN) | (1ULL << BOOT_PIN))
#endif

// BLE
#define SERVICE_UUID "653bb0e0-1d85-46b0-9742-3b408f4cb83f"
#define CHARACTERISTIC_UUID_MAIN "00c1acd4-f35b-4b5f-868d-36e5668d0929"
#define CHARACTERISTIC_UUID_SETTINGS "7299b19e-7655-4c98-8cf1-69af4a65e982"
#define CHARACTERISTIC_UUID_DEBUG "83ea7700-6ad7-4918-b1df-61031f95cf62"

// BLE Beacon for unlocking - paste your ServiceUUID here
#define BEACON_SERVICE_UUID "0000ffe0-0000-1000-8000-00805f9b34fb"
#define BEACON_MAC "ff:ff:12:a3:e7:16" // paste your MAC here
uint8_t beaconMac[6];

#ifdef CONFIG_TAG
void BLEScanTaskCode(void *pvParameters) {
  if (!deviceConnected && millis() - lastDisconnected > 2000) {
    lastDisconnected = millis(); 
    if (!pBLEScan->isScanning()) {
      pBLEScan->start(0, false, true);
    }
  }
  if (beacon.connected) {
    beacon.connected = false;
    if (beacon.rssi > -70) {
      if (!beacon.proximity) {
        if (!deviceConnected) {
          deviceConnected = true;
          Serial.printf("BLE Beacon connected. RSSI: %d\n", beacon.rssi);
          playMP3("/connected.mp3");
          delay(100);
        }
        beacon.proximity = true;
      }
    } else {
      beacon.proximity = false;
    }
    if (millis() - beacon.time < 1000) {
      if (!beacon.button) {
        beacon.button = true;
        deviceConnected = true;
        if (!isUnlocked) {
          Serial.println("BLE Beacon: Unlock Scooter");
          playMP3("/unlock.mp3");
          delay(100);
          unlockScooter();
        }
      }
    } else {
      beacon.button = false;
    }
    beacon.time = millis();
  }
  if (millis() - beacon.time > 5000) {
    if (beacon.proximity && beacon.button) {
      beacon.proximity = false;
      beacon.button = false;
      beacon.time += 2000UL;
      if (isUnlocked) {
        Serial.println("BLE Beacon: Lock Scooter");
        playMP3("/lock.mp3");
        delay(100);
        unlockForEver = 0;
        lockScooter();
      }
    } else if (beacon.rssi > -100) {
        beacon.rssi = -100;
        deviceConnected = false;
      }
  }
}
#endif

// Display Task
TaskHandle_t UARTTask;

// UARTTaskCode: read controller and send command to display every 300ms
void UARTTaskCode(void *pvParameters) {
  while (true) {
    // reset ESP32 once a week
    if (millis() > 600000000) {
      ESP.restart();
    }
    if (isUpgrading) {
      if (LEDmode != 0x30 && !alarmIsOn) {
        LEDmode = (LEDmode == 0xF0) ? 0x30 : 0xF0;
        sendDisplayLED(red, blink);
        delay(300);
      }
      speed = ((int)upgradingProgress * 256 + 5) / 10;
      sendDisplayCommand(speed, battery, customDisplayStatus != "" ? customDisplayStatus : DISPLAY_STATUS_UPGRADING);
    } else if (isUnlocked) {
      if (LEDmode != 0x03 && !alarmIsOn) {
        LEDmode = (LEDmode == 0xC3) ? 0x03 : 0xC3;
        sendDisplayLED(green, blink);
        delay(300);
      }
      sendDisplayCommand(speed, battery, customDisplayStatus != "" ? customDisplayStatus : DISPLAY_STATUS_DRIVING);
    } else {
      if (isCharging) {
        if (battery < 100) {
          if (LEDmode != 0x0C && !alarmIsOn) {
            LEDmode = (LEDmode == 0xCC) ? 0x0C : 0xCC;
            sendDisplayLED(yellow, blink);
            delay(300);
          }
        } else if (LEDmode != 0x03 && !alarmIsOn) {
            LEDmode = (LEDmode == 0xC3) ? 0x03 : 0xC3;
            sendDisplayLED(green, blink);
            delay(300);
        }
        sendDisplayCommand(speed, battery, customDisplayStatus != "" ? customDisplayStatus : DISPLAY_STATUS_CHARGING);
      } else if (deviceConnected) {
          if (LEDmode != 0x01 && !alarmIsOn) {
            LEDmode = (LEDmode == 0xC1) ? 0x01 : 0xC1;
            sendDisplayLED(green, on);
            delay(300);
        }
        sendDisplayCommand(speed, battery != 0x00 ? battery : lastBattery, customDisplayStatus != "" ? customDisplayStatus : DISPLAY_STATUS_LOCKED);
      } else {
          if (LEDmode != 0x00 && !alarmIsOn) {
            LEDmode = (LEDmode == 0xC0) ? 0x00 : 0xC0;
            sendDisplayLED(green, off);
            delay(300);
        }
        sendDisplayCommand(speed, battery != 0x00 ? battery : lastBattery, customDisplayStatus != "" ? customDisplayStatus : DISPLAY_STATUS_SCAN);
      }
    }
    if (!battery && !alarmIsOn) {
      LEDmode = 0xC1;
    }
    delay(300);
  }
}
