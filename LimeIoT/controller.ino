// This is the code for the original controller

unsigned long previousMillis = 0;
const long interval = 500;

void readController() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    while (Serial1.available() > 0 && Serial1.peek() != 0x46) {
      if (millis() - previousMillis >= interval) {
        return;
      }
      Serial1.read();
    }

    byte command[44];
    if (Serial1.available() >= 6) {
      for (int i = 0; i < 6; i++) {
        command[i] = Serial1.read();
      }
      if (command[4] == 0x00) {

        int msgLen = command[5];
        uint32_t id = (uint32_t(command[0]) << 24) |
                      (uint32_t(command[1]) << 16) |
                      (uint32_t(command[2]) <<  8) |
                       uint32_t(command[3]);

        if (msgLen <= 0x24 && Serial1.available() >= msgLen + 2) {
          for (int i = 0; i < msgLen + 2; i++) {
            command[i+6] = Serial1.read();
          }
          for (int i = msgLen + 8; i < 44; i++) {
            command[i] = 0;
          }
        }

        pDebugCharacteristic->setValue(command, msgLen + 2);
        pDebugCharacteristic->notify();

        uint16_t new_checksum = calcCRC16(command, msgLen + 6, 0x1021, 0x0000, 0x0000, false, false);
        uint16_t old_checksum = (uint16_t(command[msgLen + 6]) << 8) | uint16_t(command[msgLen + 7]);

        // Check if the command has correct checksum
        if (old_checksum == new_checksum) {
          switch (id) {
            case 0x46580CFF:
              speed = (max_speed == 20) ? command[8] * max_speed / 119 : command[8] * max_speed / 172;
              battery = command[19];
              throttle = command[28];
              isCharging = command[21];
              isUnlocked = command[23] & 0x01;
              lightIsOn = command[29] & 0x01;
              if (battery) {
                lastBattery = battery;
              } else {
                  lastBattery = 1;
                }
              break;
          }
        }
      }
    }
    while (Serial1.available() > 0) {
      Serial1.read();
    }
  }
}

void sendControllerCommand(byte* cmd, size_t len) {
  static bool isSending = 0;
  while (isSending) {
    delay(100);
  }
  isSending = 1;
  Serial1.write(cmd, len);
  delay(500);
  isSending = 0;
}
