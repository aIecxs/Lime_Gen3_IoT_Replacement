void setup() {

  // create a task that will be executed along the loop() function, with priority 2 and executed on core 0
  xTaskCreatePinnedToCore(
    UARTTaskCode,  // Task function.
    "DisplayTask", // name of task.
    4096,          // Stack size of task
    NULL,          // parameter of the task
    2,             // priority of the task
    &UARTTask,     // Task handle to keep track of created task
    0);            // pin task to core 0
/*
  // BLEScanTask for BLE Beacon
  xTaskCreatePinnedToCore(
    BLEScanTaskCode, // Task function.
    "BLEScanTask", // name of task.
    4096,          // Stack size of task
    NULL,          // parameter of the task
    2,             // priority of the task
    &BLEScanTask,  // Task handle to keep track of created task
    1);            // pin task to core 1
*/
  // ESP32 onboard LED
  pinMode(LED_BUILTIN,OUTPUT);

  // Controller
  pinMode(LOCK_PIN, OUTPUT);
  digitalWrite(LOCK_PIN, LOW);
  controllerIsOn = 0;

  // Display LOW = off, HIGH = on for npn transistor
  // Display LOW = on, HIGH = off for pnp transistor
  pinMode(DISPLAY_PIN, OUTPUT);
#ifdef CONFIG_PNP
  digitalWrite(DISPLAY_PIN, LOW);
#else
  digitalWrite(DISPLAY_PIN, HIGH);
#endif
//  gpio_hold_en(DISPLAY_PIN);
/*
  //Setup sleep wakeup on Touch Pad 3 ( GPIO15 )
  touchSleepWakeUpEnable(T3,40);
  //Setup sleep wakeup to shock sensor
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_14,1);
*/
  // wake on shock sensor
//  adcAttachPin(SHOCK_PIN); // Removed API: docs.espressif.com/projects/arduino-esp32/en/latest/migration_guides/2.x_to_3.0.html
  pinMode(SHOCK_PIN, INPUT_PULLDOWN);
//  rtc_gpio_deinit(SHOCK_PIN);
//  rtc_gpio_pulldown_en(SHOCK_PIN);

  // wake on charger
//  adcAttachPin(BOOT_PIN); // Removed API
  pinMode(BOOT_PIN, INPUT);

  // SHOCK_PIN | BOOT_PIN
//  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK,ESP_EXT1_WAKEUP_ANY_HIGH);

  /* SPI flash file system
  |--------------|-------|---------------|--|--|--|--|--|
  ^              ^       ^               ^     ^
  Sketch    OTA update   File system   EEPROM  WiFi config (SDK) */
  LittleFS.begin(true);

  // do not print any debug messages on controller to reduce noise
  Serial.begin(115200, SERIAL_8N1, RX3, TX3);  // swapped -> UART3
  Serial1.begin(9600, SERIAL_8N1, RX0, TX0);   // swapped -> UART0
  Serial2.begin(115200, SERIAL_8N1, RX2, TX2);
/*
  // WARNING: nvs_flash_erase() wipes the entire NVS, including the Bluetooth identity keys (IR/IRK).
  // This causes the BLE stack to generate a new IRK on every boot, so the IRK will change each time.
  // This is intentional here to force fresh authentication for testing purposes.
  //
  // For production use where a stable IRK is needed (e.g. Home Assistant presence detection),
  // remove these lines. The server's IRK will remain the same across reboots as long as NVS is intact.
  // To clear only bond data without affecting identity keys, use esp_ble_remove_bond_device() (Bluedroid)
  // or ble_store_util_delete_all() (NimBLE) instead.
  Serial.println("Clearing NVS pairing data...");
  nvs_flash_erase();
  nvs_flash_init();
*/
  Serial.println("Starting BLE work!");
  BLEDevice::init(SCOOTER_NAME);

  uint32_t insecure_properties = NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY;
  uint32_t secure_properties = insecure_properties;

  // NimBLE uses properties to secure characteristics.
  // These special permission properties are not supported by Bluedroid and will be ignored.
  // This can be removed if only using Bluedroid (ESP32).
  // Check the BLECharacteristic.h file for more information.
  secure_properties |= NIMBLE_PROPERTY::READ_AUTHEN | NIMBLE_PROPERTY::WRITE_AUTHEN;

  pServer = BLEDevice::createServer();

  /*
   * Required in authentication process to provide displaying and/or input passkey or yes/no butttons confirmation
   */
  pServer->setCallbacks(new MySecurityCallbacks());
//  pServer->setCallbacks(new MyServerCallbacks()); // merged into -> ble_security.ino -> MySecurityCallbacks
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pMainCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_MAIN, secure_properties);
  pMainCharacteristic->setCallbacks(new MainBLECallback());

/* // (Debug Characteristic UUID currently not used by the app)
  pDebugCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_DEBUG,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
*/
  pSettingsCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_SETTINGS, secure_properties);
  pSettingsCharacteristic->setCallbacks(new SettingsBLECallback());

  pServer->advertiseOnDisconnect(true);
  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setName(SCOOTER_NAME);
  pAdvertising->enableScanResponse(true);
  pAdvertising->start();
  BLEDevice::setMTU(BLE_ATT_MTU_MAX);
  BLEDevice::setSecurityAuth(true, true, true);               // bonding with peer device after authentication
  BLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);        // set the IO capability to No output No input
  BLEDevice::setSecurityPasskey(BLE_PASSWORD);                // set static passkey
  uint8_t init_key = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
  uint8_t rsp_key = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
  /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribut to you,
    and the response key means which key you can distribut to the Master;
    If your BLE device act as a master, the response key means you hope which types of key of the slave should distribut to you,
    and the init key means which key you can distribut to the slave. */
  BLEDevice::setSecurityInitKey(init_key);
  BLEDevice::setSecurityRespKey(rsp_key);
  Serial.println("Ready!");
  delay(2500);

  // Play ready sound
#ifndef CONFIG_I2S
  tone(BUZZER_PIN, 300, 100);
  delay(100);
  tone(BUZZER_PIN, 400, 100);
  delay(100);
  tone(BUZZER_PIN, 500, 100);
  delay(100);
  noTone(BUZZER_PIN);
#else
  beep(300, 100);
  beep(400, 100);
  beep(500, 100);
#endif

  // disable AudioLogger
  Print* audioLogger = &silencedLogger;

  LEDmode = 0x10;
  turnOnController();
}
