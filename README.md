# Lime_Gen3_IoT_Replacement
<b>Note:</b> This project is not endorsed or supported by Lime or any affiliated companies. Only do this on legally obtained scooter that you own! You can often buy them on auctions.

![cover](iot_original.png)

The goal of this project is to replace the IoT of the Lime Gen 3 with a custom one, so we can control it with our own app.
If you find out more about the communication, please submit it here.

## How it works
The IoT module gets replaced with an ESP32 microcontroller to enable us to control the scooter with our app. The app communicates with the ESP32 using Bluetooth Low Energy (BLE). The ESP32 replaces the function of the original IoT while also providing real-time feedback on speed, battery level, and other information.

## Installation
Install the ESP32 add-on for Arduino IDE if you doesnt have already. [Here is a tutorial](https://randomnerdtutorials.com/installing-esp32-arduino-ide-2-0).

Install ESP32 LittleFS Uploader add-on for Arduino IDE. [Here is a tutorial](https://randomnerdtutorials.com/arduino-ide-2-install-esp32-littlefs).

Install the [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino) library by h2zero from the library manager.

Install the [BLEOTA](https://github.com/gb88/BLEOTA) library by gb88 from the library manager (or install the modified fork, see [Over-The-Air updates](README.md#over-the-air-updates))

Install the [crc](https://github.com/RobTillaart/CRC) library by robtillaart from the library manager.

Install the [ESP8266Audio](https://github.com/earlephilhower/ESP8266Audio) library by earlephilhower.

Flash the controller with [unlocked firmware](https://cloud.scooterhacking.org/release/lime_dropbox) (`Lime Gen3 25kmh locked.hex`).

Flash the arduino code from [LimeIoT](../../tree/2.3.x/LimeIoT) folder to the esp32. The Sketch is created with Arduino 1.8.19 for board platform esp32 by Espressif Systems version 2.0.13. For Arduino 1.8.19 downgrade version to avoid exceeding of program storage space. Maximum is 1310720 bytes. For Arduino IDE 2.3.8 clone the [2.3.x](../../tree/2.3.x) branch. For board platform esp32 by Espressif Systems version 3.x (based on ESP-IDF 5.x) install the NimBLE-Arduino library by h2zero from the library manager.

Flash the MP3 files from "Tools" -> "ESP32 Sketch Data Upload" menu to LittleFS (Arduino 1.8.19) or `[Ctrl]` + `[Shift]` + `[P]` -> `">Upload LittleFS to Pico/ESP8266/ESP32"` (Arduino IDE 2.3.8). You can change theme by copying files from [themes](../../tree/2.3.x/LimeIoT/themes) into sketch `'data'` folder. only the sketch data folder is uploaded. If you forgot to upload mp3 files, the [themed](../../tree/2.3.x) branch will crash the ESP32 when playing audio / file not found. Troubleshooting: If you get the following error message [ERROR: No port specified, check IDE menus](https://github.com/earlephilhower/arduino-littlefs-upload/issues/12), restart the Arduino IDE and try again. 

<b>Note:</b> The controller gives you 42v. So you have to convert it to stable 5v for the display and the esp32. I have done it using a buck converter.

Connect the wires:

![LimeIoTConnector](https://user-images.githubusercontent.com/76005215/227743332-2c972cca-d37c-4bcd-b67e-097f84796bc5.jpg)

| Connector | ESP32 |
| -------- | ------- |
| Controller lock  | GPIO 12   |
| Controller RX    | GPIO 1    |
| Controller TX    | GPIO 3    |
| Controller 42v   | Buck converter -> 5V |
| Controller Gnd   | Gnd       |
| Controller Charge (2) | GPIO 33 |
| Display 5v       | 5V        |
| Display Gnd      | Gnd       |
| Display TX       | GPIO 16   |
| Display RX       | GPIO 17   |

(Optional) If you want, you can connect the speaker to `GPIO 13` with a base resistor and transistor. For the [pcb board](https://web.archive.org/web/2024/scootertalk.org/forum/viewtopic.php?t=5474&start=180) the speaker is `GPIO 25`. For more gain connect Adafruit I2S Amplifier [MAX98357A](https://www.adafruit.com/product/3006) DIN `GPIO 27` BCLK `GPIO 26` LRCLK `GPIO 25`.

(Optional) You can connect any alarm sensor to `GPIO 14` max input voltage 4.6v (!)

## Usage
You can download the app here: [App.apk](../../raw/2.3.x/App.apk)

The default bluetooth password is `123456`. You can change it in the [LimeIoT.ino](../../tree/2.3.x/LimeIoT/LimeIoT.ino#L11) file.

Currently only compatible on android and is only looking good with Material You compatible phones.

If you dont want/can use the app, you can just download a bluetooth terminal app like nRF Connect ([Play Store](https://play.google.com/store/apps/details?id=no.nordicsemi.android.mcp) | [App Store](https://apps.apple.com/us/app/nrf-connect-for-mobile/id1054362403)) and send the commands yourself:

| Commands | Action |
| -------- | ------- |
| `unlock`  | Unlocks the scooter |
| `lock`    | Turns off the controller |
| `unlockforever`    | Keeps the scooter unlocked when disconnecting |
| `alarm`   | let the scooter beeb (if speaker is connected ) |

## Over-The-Air updates

You can update the ESP32 Over-The-Air with Chrome or Edge Web App.

-> visit [https://gb88.github.io/BLEOTA](https://gb88.github.io/BLEOTA)  
- connect BLE Device  
- select file `'LimeIoT-ota_esp32-signed.bin'`  

#### Creating Over-The-Air updates

For NimBLE-Arduino on board platform esp32 by Espressif Systems version v3.3.8  
you may use that modified fork instead of the original BLEOTA Library  
**Credits:** [@gb88](https://buymeacoffee.com/gb88)
 
[github.com/aIecxs/BLEOTA/tree/nimble](https://github.com/aIecxs/BLEOTA/tree/nimble) -> download zip file ->  
[github.com/aIecxs/BLEOTA/archive/refs/heads/nimble.zip](https://github.com/aIecxs/BLEOTA/archive/refs/heads/nimble.zip)

Install the Library in Arduino IDE 2 -> Menu ->  
Sketch -> Include Library -> Add .ZIP Library... -> [BLEOTA-nimble.zip](https://github.com/aIecxs/BLEOTA/archive/refs/heads/nimble.zip)

Install [Git for Windows](https://git-scm.com)

- compile the arduino code from [LimeIoT](../../tree/2.3.x/LimeIoT) folder at least once  
- place your own `priv_key.pem` / `rsa_key.pub` key pair into sketch `'data'` folder  
- copy the [make_ota.sh](https://github.com/aIecxs/BLEOTA/tree/nimble/tools) file into sketch folder  
- run the `$ ./make_ota.sh` bash script from Git Bash -> hit `ESC` to sign with your own keys

Flash the `'LimeIoT-ota_esp32-signed.bin'` file via BLEOTA Web App.  
Flash the `'LimeIoT-littlefs_esp32-signed.bin'` file via BLEOTA Web App.

**Note:** With no RSA key pair files given in sketch `'data'` folder, you can sign with random auto generated RSA keys from ESP32 memory dump instead. But it is recommended to keep the (known) RSA keys published with this GitHub Repo. The Signature is used for file validation only. BLE Bluetooth connection still remains secured against unauthorized access with 6-digit `BLE_PASSWORD`.

## Controller Communication
To unlock the controller, supply 3.3V to the blue wire connected to the IoT and send the command `464316610001F1F28F` to power it on. Once powered on, send the heartbeat `4643110100084C494D4542494B45BE8A` every 500ms. To power off the controller, cut the 3.3V supply and send the command `464316610001F0E2AE`. The baudrate for all commands is `9600`.

The command sent by the controller to the IoT consists of 42 bytes. The 9th byte represents the speed, and the 20th byte represents the battery level. The last two bytes of the command are a CRC-16/XMODEM checksum.

| Byte number | Meaning |
|--|--|
| 9 | Speed |
| 20 | Battery |
| last two bytes | CRC-16/XMODEM checksum |

#### Example: `46 58 0C FF 00 22 11 00 00 40 00 00 41 3F 60 42 00 FF 44 64 52 00 61 F1 80 00 00 72 01 5C 01 59 82 00 00 00 00 E0 00 00 0A B3`


## Display Communication
- Baudrate: `115200`
- Checksum: `width=8  poly=0x31  init=0x0a  refin=true  refout=true  xorout=0x00  check=0xc1  residue=0x00`

The following table shows the known meaning of the bytes in the commands send to the display:
| Byte | Meaning |
|--|--|
| 12 | Status (see below) |
| 14 | Battery |
| 16-17 | Speed |
| last byte | checksum |


#### Status Bytes:
| Byte | Status |
|--|--|
| 21 | Scan To Ride |
| 22 | Unavailable |
| 23 | Paused |
| 24 | Locked |
| 25 | Done |
| 26 | Charging |
| 31 | Driving |
| 41 | Driving Low Battery |
| 42 | Driving Alert |
| 43 | Driving No Parking |
| 44 | Driving No Riding |
| 45 | Driving Max Speed |
| 51 | Upgrading |

#### Example: `4C 42 44 43 50 01 10 11 00 09 01 31 01 1E 02 00 CD 01 9A`


## LED Communication

You can turn off the red LED with the following command: `4C 42 44 43 50 01 10 1B 00 08 03 00 00 00 03 00 00 00`

| Byte | Meaning |
|--|--|
| 12 | red |
| 13 | yellow |
| 14 | green |


#### LED Bits:
| Bits | State |
|--|--|
| 00 | off |
| 01 | on |
| 11 | blink |

LED byte has two bits = bit for blink + bit for power


## Hardware configuration flags

#### The following `#define` flags are used to select the hardware features:

| Flag | Module | meaning |
| --- | --- | --- |
| `CONFIG_I2S` | MAX98357A | Adafruit I2S Amplifier installed |
| `CONFIG_IMU` | SW-420 | Tilt sensor wakeup/input installed |
| `CONFIG_PSM` | MP4560 | DC-DC Converter with EN (Enable) Pin (power-saving mode) |
| `CONFIG_PNP` | | Display has pnp transistor (inverted Pin) |
| `CONFIG_TAG` | ST17H66 | BLE Beacon unlocking allowed |

= = = > > > [ESP32_Manual.pdf](https://web.archive.org/web/2024/scootertalk.org/forum/viewtopic.php?t=5474&start=190) < < < = = =

### PCB Board Power Configuration
| | Case 0<br>(No PCB) | Case A<br>(Sleep, DC-DC ON) | Case B<br>(PSM, DC-DC OFF) |
| --- | --- | --- | --- |
| `CONFIG_IMU`<br>`CONFIG_PSM`<br>`CONFIG_PNP` | disabled<br>disabled<br>disabled | enabled<br>disabled<br>enabled | enabled<br>enabled<br>disabled |
| Wake Sources<br>Display/EN<br>Power (idle) | None<br>Always ON<br>~300mA | Digital I/O<br>RTC Hold ON<br>~20mA | ADC<br>OFF (cuts power)<br><1mA |

**BLE Beacon note:**

If you want the BLE Beacon unlock feature, you must use some keyfinder with a static, non-randomized MAC address such as the ST17H66. You need to discover the MAC address yourself (for example with NimBLE-Arduino -> [BLE_Beacon_Scanner](https://github.com/h2zero/NimBLE-Arduino/blob/master/examples/BLE_Beacon_Scanner/BLE_Beacon_Scanner.ino) sketch) and configure it in the code. Enable `CONFIG_TAG`, edit the beacon MAC and ServiceUUID in [LimeIoT.ino](../../tree/2.3.x/LimeIoT/LimeIoT.ino#L129). Short press unlocks the scooter. Holding BLE Beacon near the green box + long press will lock the Scooter immediately. If BLE Beacon is out of distance the timeout will lock Scooter after 3 minutes.

<img width="350" height="200" alt="ST17H66" src="https://github.com/user-attachments/assets/ed1fc262-d844-4df6-aea6-11bcc4f64b03" />
