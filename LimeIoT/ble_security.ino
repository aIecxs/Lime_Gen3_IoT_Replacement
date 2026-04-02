class MySecurityCallbacks : public BLEServerCallbacks
{

  uint32_t onPassKeyDisplay()
  {
    ESP_LOGI(LOG_TAG, "PassKeyRequest");
    return (uint32_t)BLE_PASSWORD;
  }
  void onPassKeyEntry(NimBLEConnInfo& connInfo)
  {
    ESP_LOGI(LOG_TAG, "The passkey Notify number:%d", BLE_PASSWORD);
    BLEDevice::injectPassKey(connInfo, BLE_PASSWORD);
  }
  void onConfirmPasskey(NimBLEConnInfo& connInfo, uint32_t pass_key)
  {
    ESP_LOGI(LOG_TAG, "The passkey YES/NO number:%d", pass_key);
    bool accepted = (pass_key == BLE_PASSWORD);
    BLEDevice::injectConfirmPasskey(connInfo, accepted);
    vTaskDelay(5000);
  }

  void onAuthenticationComplete(NimBLEConnInfo& connInfo)
  {
    if (connInfo.isAuthenticated()) {
      ESP_LOGI(LOG_TAG, "Starting BLE work!");
      playMP3("/connected.mp3");
      delay(100);
      alarm_cnt = 0;   // disable night mode
    }
  }

  // class MyServerCallbacks : public BLEServerCallbacks
  void onConnect(BLEServer *pServer, NimBLEConnInfo& connInfo) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer, NimBLEConnInfo& connInfo, int reason) {
    deviceConnected = false;
  }
};