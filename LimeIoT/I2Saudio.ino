#include <AudioFileSourceLittleFS.h>
#include <AudioGeneratorMP3.h>
#include <AudioOutputI2S.h>
#include <AudioLogger.h>

/* Requirements:
 *
 * http://github.com/lorol/arduino-esp32fs-plugin/releases download esp32fs.zip
 *  ~/Arduino/tools/ESP32FS/tool/esp32fs.jar               <-   place file here
 * goto "Sketch" -> "Show Sketch Folder" -> create directory "data" -> file "example.mp3"
 * Use the "Tools" -> "ESP32 Sketch Data Upload" menu to upload the MP3 files to LittleFS
 *
 * http://github.com/earlephilhower/ESP8266Audio
 * "Tools" -> "Manage Libraries..." -> "ESP8266Audio"
 */

AudioFileSourceLittleFS *file;
AudioGeneratorMP3 *mp3;
AudioOutputI2S *out;

TaskHandle_t mp3Task;

void playMP3Task(void *pvParameters) {
  mp3 = new AudioGeneratorMP3();
  mp3->begin(file, out);
  isMP3Playing = true;
  while (mp3->isRunning()) {
    if (!mp3->loop()) {
      mp3->stop();
      break;
    }
  }
  isMP3Playing = false;
  mp3Task = NULL;
  delete mp3; mp3 = nullptr;
  delete file; file = nullptr;
  delete out; out = nullptr;
  vTaskDelete(NULL);
}

void playMP3(const char *mp3File) {
  if (!isMP3Playing) {
#ifndef CONFIG_I2S
    out = new AudioOutputI2S(0, 1);   // built-in DAC
#else
    out = new AudioOutputI2S();       // MAX98357A I2S
    out->SetPinout(BCLK_PIN, WCLK_PIN, DOUT_PIN);
#endif
    out->SetGain(1.0);
    file = new AudioFileSourceLittleFS(mp3File);
    xTaskCreatePinnedToCore(
    playMP3Task,   // Task function.
    "MP3Task",     // name of task.
    6144,          // Stack size of task
    NULL,          // parameter of the task
    1,             // priority of the task
    &mp3Task,      // Task handle to keep track of created task
    1);            // pin task to core 1
  }
}

void beep(int freq, unsigned int duration) {
#ifndef CONFIG_I2S
  out = new AudioOutputI2S(0, 1);   // built-in DAC
#else
  out = new AudioOutputI2S();       // MAX98357A I2S
  out->SetPinout(BCLK_PIN, WCLK_PIN, DOUT_PIN);
#endif
  out->SetGain(1.0);

  // Set sample rate and start the I2S engine
  const uint32_t sampleRate = 22050;
  out->SetRate(sampleRate);
  out->begin();

  unsigned long totalSamples = (unsigned long)(duration + 100U) * sampleRate / 1000UL;
  uint32_t samplesPerWave = (sampleRate << 10) / freq;

  // Stream raw square-wave samples directly into the I2S buffer
  for (unsigned long sentSamples = 0; sentSamples < totalSamples; sentSamples++) {
    int samplesSent = sentSamples << 10;
    int rem = samplesSent % samplesPerWave;

    // Generate square wave amplitude
    int16_t val = (rem > (samplesPerWave / 2)) ? 8192 : -8192;
    int16_t frame[2] = { val, val };

    // Push sample to I2S. If buffer is full, wait and retry
    while (!out->ConsumeSample(frame)) {
      delay(0);
    }
  }

  // Send a final silent frame to prevent a popping noise at the end
  int16_t silence[2] = {0, 0};
  out->ConsumeSample(silence);

  // Stop the I2S engine to free the channel until the next beep
  out->stop();
  delete out; out = nullptr;
}