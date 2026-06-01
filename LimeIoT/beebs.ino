void unlockBeeb() {
#ifndef CONFIG_I2S
  tone(BUZZER_PIN, 400, 100);
  delay(100);
  tone(BUZZER_PIN, 500, 100);
  delay(100);
  noTone(BUZZER_PIN);
#else
  beep(400, 100);
  beep(500, 100);
#endif
}

void lockBeeb() {
#ifndef CONFIG_I2S
  tone(BUZZER_PIN, 500, 100);
  delay(100);
  tone(BUZZER_PIN, 400, 100);
  delay(100);
  noTone(BUZZER_PIN);
#else
  beep(500, 100);
  beep(400, 100);
#endif
}

void connectedBeeb() {
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
}

void disconnectedBeeb() {
#ifndef CONFIG_I2S
  tone(BUZZER_PIN, 300, 100);
  delay(100);
#else
  beep(300, 100);
#endif
}


TaskHandle_t alarmTask;

// Run alarmBeeb in a separate low-priority FreeRTOS task to avoid blocking loop
void alarmBeebTask(void* pvParameters) {
  alarmIsOn = 1;
  if (!controllerIsOn) {
    turnOnController();
    delay(1500);
  }
  // avoid disorderly conduct in night mode
  if (alarm_cnt < 20 && alarm_cnt % 2) {
    playMP3("/alarm.mp3");
    sendControllerCommand(lightBlinkEscByte, sizeof(lightBlinkEscByte));
    for (int i = 0; i < alarm_reps; i++) {
      LEDmode = 0x10;
      sendDisplayLED(red, on);
//      beep(alarm_freq, alarm_delay);
      delay(alarm_delay);
      LEDmode = 0x00;
      sendDisplayLED(red, off);
      delay(alarm_delay);
    }
    sendControllerCommand(lightBlinkEscByte, sizeof(lightBlinkEscByte));
    for (int i = 0; i < alarm_reps; i++) {
      LEDmode = 0x10;
      sendDisplayLED(red, on);
      delay(alarm_delay);
      LEDmode = 0x00;
      sendDisplayLED(red, off);
      delay(alarm_delay);
    }
  } else {
    delay((4UL * alarm_delay * alarm_reps) + 1000); // debounce GPIO input
  }
  alarmIsOn = 0;
  alarmTask = NULL;
  vTaskDelete(NULL);
}

// create a low-priority task to run the alarm
void alarmBeeb() {
  if (!alarmIsOn) {
    xTaskCreatePinnedToCore(
    alarmBeebTask, // Task function.
    "alarmBeeb",   // name of task.
    4096,          // Stack size of task
    NULL,          // parameter of the task
    1,             // priority of the task
    &alarmTask,    // Task handle to keep track of created task
    1);            // pin task to core 1
  }
}
