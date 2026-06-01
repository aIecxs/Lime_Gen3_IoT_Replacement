unsigned long prevMillis = 0;
const long linterval = 250;

unsigned long lastOnTime = 0;
const unsigned short sleepTimer = 3 * 60 * 60; // 3 hours

// inhibit driving without hearthbeat timeout
unsigned long lastConnected = 0;
const unsigned short lockTimer = 3 * 60 ; // 3 minutes

// debounce GPIO input
getPin bootPin(BOOT_PIN, 700); // mV = HIGH
getPin shockPin(SHOCK_PIN, 700); // mV = HIGH

void loop() {
  unsigned long currentTime = millis() / 1000;

#ifdef CONFIG_IMU
  // If more than 3 hours have passed, go to deep sleep
  if ((currentTime - lastOnTime) > sleepTimer) {
    lockScooter();
    turnOffController();
  #ifdef CONFIG_PNP
    digitalWrite(DISPLAY_PIN, HIGH);
    rtc_gpio_set_level(DISPLAY_PIN, HIGH);
  #else
    digitalWrite(DISPLAY_PIN, LOW);
  #endif
    // wait for BOOT_PIN state was updated
    if (!isBooted || (currentTime - lastOnTime) > sleepTimer + 30) {
      lastOnTime = currentTime;
      esp_deep_sleep_start();
    }
  }
#endif

  if (isUnlocked || isCharging) {
    lastOnTime = currentTime;
  }
  // arm the alarm on shock sensor
  if (isDisconnected && (currentTime - lastConnected) > lockTimer && (currentTime - lastOnTime) > lockTimer) {
    isDisconnected = false;
  }

  // wake on shock sensor
#ifdef CONFIG_IMU
  shockPin.get(&shockState, 10); // 10 ms debounce
  if (shockState && !prevShockState && !alarmIsOn && battery && !deviceConnected && !isDisconnected && !isUnlocked && !unlockForEver) {
  #ifdef CONFIG_PNP
    digitalWrite(DISPLAY_PIN, LOW);
  #else
    digitalWrite(DISPLAY_PIN, HIGH);
  #endif
    alarmBeeb();
    alarm_cnt++; // avoid disorderly conduct in night mode
    currentTime = millis() / 1000;
    lastOnTime = currentTime;
    isIdle = false;
  }
  // keep prevShockState true while shockState remains HIGH so a stuck input
  // doesn't produce repeated rising-edge events; only clear when pin goes LOW
  if (!shockState) prevShockState = shockState;
#endif

  // wake on charger (decrease idle time with pull-down resistor)
  bootPin.get(&isBooted, 5000); // ms
  if (isBooted || (currentTime % 80000 == 0)) {
    if (!controllerIsOn && !isIdle) { // update battery once a day
#ifdef CONFIG_PNP
      digitalWrite(DISPLAY_PIN, LOW);
#else
      digitalWrite(DISPLAY_PIN, HIGH);
#endif
      lastOnTime = currentTime;
      turnOnController();
    }
  }
  // lock on charging
  if (isUnlocked && isCharging && !commandIsSending && !alarmIsOn && !isMP3Playing) {
    playMP3("/lock.mp3");
    lockScooter();
  }
  // turn off on charging idle
  if (!isIdle && controllerIsOn && !alarmIsOn && !deviceConnected && !unlockForEver && (currentTime - lastOnTime) > 30) {
    lockScooter();
    turnOffController();
  }
  if (isIdle && !isBooted) {
    isIdle = false;
  }

  unsigned long currMillis = millis();
  byte txByte[] = { isUnlocked, unlockForEver, (byte)speed, battery, throttle, lightIsOn, controllerIsOn, isCharging, alarmIsOn };
  byte settingsByte[] = { (byte)max_speed, (byte)alarm_delay, (byte)alarm_freq, (byte)alarm_reps };

  if (currMillis - prevMillis >= linterval) {
    prevMillis = currMillis;  // update prevMillis with current time

    if (deviceConnected) {
      pSettingsCharacteristic->setValue(settingsByte, sizeof(txByte));
      pMainCharacteristic->setValue(txByte, sizeof(txByte));
      pMainCharacteristic->notify();
    } else if (isUnlocked && !unlockForEver && (currentTime - lastConnected > lockTimer)) {
        lockScooter();
      }
    if (controllerIsOn && !commandIsSending) {
      sendControllerCommand(hearthBeatEscByte, sizeof(hearthBeatEscByte));
      if (deviceConnected || unlockForEver) {
        lastConnected = currentTime;
      }
    }
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    if (!unlockForEver && !isCharging) {
      turnOffController();
    }
    playMP3("/disconnected.mp3");
    delay(500);                   // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
    isDisconnected = true;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    unlockForEver = 0;
    Serial.println("connecting");
    oldDeviceConnected = deviceConnected;
  }
  if (controllerIsOn || isUnlocked) {
    readController();
  }
  BLEOTA.process();
  delay(10);
#ifdef CONFIG_TAG
  // Scan for BLE beacon
  BLEScanTaskCode(NULL);
#endif
}
