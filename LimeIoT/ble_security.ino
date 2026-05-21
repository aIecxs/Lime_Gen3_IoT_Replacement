class MySecurityCallbacks : public BLEServerCallbacks {
  void onAuthenticationComplete(NimBLEConnInfo& connInfo)
  {
    if (connInfo.isAuthenticated()) {
      ESP_LOGI(LOG_TAG, "Starting BLE work!");
      playMP3("/connected.mp3");
    } else {
      pServer->disconnect(connInfo.getConnHandle());
    }
    delay(100);
    alarm_cnt = 0;   // disable night mode
  }

  void onConnect(BLEServer *pServer, NimBLEConnInfo& connInfo) {
    deviceConnected = true;
    pBLEScan->stop();
    BLEDevice::startSecurity(connInfo.getConnHandle());
  };

  void onDisconnect(BLEServer *pServer, NimBLEConnInfo& connInfo, int reason) {
    deviceConnected = false;
    lastDisconnected = millis();
  }
};

#ifdef CONFIG_TAG
class MyScanCallbacks : public NimBLEScanCallbacks {
  void onDiscovered(const BLEAdvertisedDevice* device) {
    if (device->haveServiceUUID() && device->getServiceUUID().equals(BLEUUID(BEACON_SERVICE_UUID))) {
      beacon.rssi = device->getRSSI();
      beacon.connected = true;
    }
  }
};
#endif