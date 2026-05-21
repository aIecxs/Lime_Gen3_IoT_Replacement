/* NOTE: This file was renamed with a leading '0' so the Arduino build
 * system processes it first. Arduino sorts .ino files alphabetically,
 * so placing all class definitions here ensures they are compiled before
 * any code that uses them. Put all your classes in this file.
 */

void lockScooter() {
  sendControllerCommand(offEscByte, sizeof(offEscByte));
  delay(100);
  isUnlocked = 0;
  sendControllerCommand(lightOffEscByte, sizeof(lightOffEscByte));
  delay(100);
  lightIsOn = 0;
}

void turnOnController() {
  digitalWrite(LOCK_PIN, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  controllerIsOn = 1;
}

void unlockScooter() {
  if (!controllerIsOn) {
    turnOnController();
    delay(1500);
  }
  sendControllerCommand(onEscByte, sizeof(onEscByte));
  delay(100);
  isUnlocked = 1;
  sendControllerCommand(lightOnEscByte, sizeof(lightOnEscByte));
  delay(100);
  lightIsOn = 1;
}

void turnOffController() {
  digitalWrite(LOCK_PIN, LOW);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  controllerIsOn = 0;
  isIdle = true;
}


/**
 * @brief Debounces an analog GPIO input.
 *
 * This class filters noisy or unstable GPIO signals by requiring the input
 * to remain stable for a specified duration before the state is considered valid.
 */
class getPin {

private:
  gpio_num_t pin;
  int threshold;
  unsigned long lastRead = 0;
  unsigned long lastHigh = 0;
  unsigned long lastLow = 0;
  bool high = false;
  bool low = false;

public:
  /**
   * @brief Constructor for the GPIO debounce handler.
   *
   * @param pin        gpio_num_t - GPIO input pin to poll
   * @param threshold  mV - HIGH threshold in millivolts
   *
   * Example:
   *   getPin gpio33(GPIO_NUM_33, 2500);
   */
  getPin(gpio_num_t pin, int threshold) {
    this->pin = pin;
    this->threshold = threshold;
  }

  /**
   * @brief Reads the debounced state of the GPIO pin.
   *
   * The input must remain stable for the specified duration before the
   * output state changes. This prevents false triggering due to noise.
   *
   * @param state  bool - will be updated with the debounced state
   * @param dur    ms - required stable duration in milliseconds
   *
   * Example:
   *   gpio33.get(&isHigh, 100);
   */
  void get(bool* state, unsigned long dur) {
    unsigned long cur = millis();
    if (cur == lastRead) {
      return;
    }
    else {
      lastRead = cur;
    }
#ifndef CONFIG_PSM
    if (digitalRead(pin) == HIGH) {
#else
    if (analogReadMilliVolts(pin) > threshold) {
#endif
      if (!high) {
        lastHigh = cur;
        high = true;
      }
      if ((cur - lastHigh) > dur) {
        *state = true;
        low = false;
      }
    }
    else {
      if (!low) {
        lastLow = cur;
        low = true;
      }
      if ((cur - lastLow) > dur) {
        *state = false;
        high = false;
      }
    }
  }
};
