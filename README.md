Arduino Library for network connections management
==================================================

[![Check Arduino status](https://github.com/arduino-libraries/Arduino_ConnectionHandler/actions/workflows/check-arduino.yml/badge.svg)](https://github.com/arduino-libraries/Arduino_ConnectionHandler/actions/workflows/check-arduino.yml)
[![Compile Examples status](https://github.com/arduino-libraries/Arduino_ConnectionHandler/actions/workflows/compile-examples.yml/badge.svg)](https://github.com/arduino-libraries/Arduino_ConnectionHandler/actions/workflows/compile-examples.yml)
[![Spell Check status](https://github.com/arduino-libraries/Arduino_ConnectionHandler/actions/workflows/spell-check.yml/badge.svg)](https://github.com/arduino-libraries/Arduino_ConnectionHandler/actions/workflows/spell-check.yml)

Library for handling and managing network connections by providing keep-alive functionality and automatic reconnection in case of connection-loss. It supports the following boards:

* **Wi-Fi**: [`MKR 1000`](https://store.arduino.cc/arduino-mkr1000-wifi), [`MKR WiFi 1010`](https://store.arduino.cc/arduino-mkr-wifi-1010), [`Nano 33 IoT`](https://store.arduino.cc/arduino-nano-33-iot), [`Portenta H7`](https://store.arduino.cc/products/portenta-h7), [`Nano RP2040 Connect`](https://store.arduino.cc/products/arduino-nano-rp2040-connect), [`Nicla Vision`](https://store.arduino.cc/products/nicla-vision), [`OPTA WiFi`](https://store.arduino.cc/products/opta-wifi), [`GIGA R1 WiFi`](https://store.arduino.cc/products/giga-r1-wifi), [`Portenta C33`](https://store.arduino.cc/products/portenta-c33), [`UNO R4 WiFi`](https://store.arduino.cc/products/uno-r4-wifi), [`Nano ESP32`](https://store.arduino.cc/products/nano-esp32), [`ESP8266`](https://github.com/esp8266/Arduino/releases/tag/2.5.0), [`ESP32`](https://github.com/espressif/arduino-esp32),
[`Notecard`](https://shop.blues.com/collections/notecard)
* **GSM**: [`MKR GSM 1400`](https://store.arduino.cc/arduino-mkr-gsm-1400-1415), [`Notecard`](https://shop.blues.com/collections/notecard)
* **LTE**: [`Notecard`](https://shop.blues.com/collections/notecard)
* **5G**: [`MKR NB 1500`](https://store.arduino.cc/arduino-mkr-nb-1500-1413)
* **LoRa**: [`MKR WAN 1300/1310`](https://store.arduino.cc/mkr-wan-1310), [`Notecard`](https://shop.blues.com/collections/notecard)
* **Ethernet**: [`Portenta H7`](https://store.arduino.cc/products/portenta-h7) + [`Vision Shield Ethernet`](https://store.arduino.cc/products/arduino-portenta-vision-shield-ethernet), [`Max Carrier`](https://store.arduino.cc/products/portenta-max-carrier), [`Breakout`](https://store.arduino.cc/products/arduino-portenta-breakout), [`Portenta Machine Control`](https://store.arduino.cc/products/arduino-portenta-machine-control), [`OPTA WiFi`](https://store.arduino.cc/products/opta-wifi), [`OPTA RS485`](https://store.arduino.cc/products/opta-rs485), [`OPTA Lite`](https://store.arduino.cc/products/opta-lite), [`Portenta C33`](https://store.arduino.cc/products/portenta-c33) + [`Vision Shield Ethernet`](https://store.arduino.cc/products/arduino-portenta-vision-shield-ethernet)
* **Satellite**: [`Notecard`](https://shop.blues.com/collections/notecard)

#### More About Notecard Connectivity

The Notecard is a wireless, secure abstraction for device connectivity, that can
be used to enable _ANY*_ device with I2C, or UART, to connect to the Arduino IoT
Cloud via cellular, LoRa, satellite or Wi-Fi (including the devices listed
above)!

As a result, the STM32 architecture has now been added to this library. If you
have an STM32 device, you are now able to connect it to the Arduino IoT Cloud by
using a Notecard to provide a secure communication channel.

> \*_Unfortunately, the AVR architecture is not supported by the Arduino IoT
> Cloud library. Therefore, those devices are ineligible for use with the
> Notecard._

### How-to-use

```C++
#include <Arduino_ConnectionHandler.h>
/* ... */
#if defined(BOARD_HAS_ETHERNET)
EthernetConnectionHandler conMan;
#elif defined(BOARD_HAS_WIFI)
WiFiConnectionHandler conMan("SECRET_WIFI_SSID", "SECRET_WIFI_PASS");
#elif defined(BOARD_HAS_GSM)
GSMConnectionHandler conMan("SECRET_PIN", "SECRET_APN", "SECRET_GSM_LOGIN", "SECRET_GSM_PASS");
#elif defined(BOARD_HAS_NB)
NBConnectionHandler conMan("SECRET_PIN", "SECRET_APN", "SECRET_GSM_LOGIN", "SECRET_GSM_PASS");
#elif defined(BOARD_HAS_LORA)
LoRaConnectionHandler conMan("SECRET_APP_EUI", "SECRET_APP_KEY");
#endif
/* ... */
void setup() {
  Serial.begin(9600);
  while(!Serial) { }

  setDebugMessageLevel(DBG_INFO);

  conMan.addCallback(NetworkConnectionEvent::CONNECTED, onNetworkConnect);
  conMan.addCallback(NetworkConnectionEvent::DISCONNECTED, onNetworkDisconnect);
  conMan.addCallback(NetworkConnectionEvent::ERROR, onNetworkError);
}

void loop() {
  /* The following code keeps on running connection workflows on our
   * ConnectionHandler object, hence allowing reconnection in case of failure
   * and notification of connect/disconnect event if enabled (see
   * addConnectCallback/addDisconnectCallback) NOTE: any use of delay() within
   * the loop or methods called from it will delay the execution of .check(),
   * which might not guarantee the correct functioning of the ConnectionHandler
   * object.
   */
  conMan.check();
}
/* ... */
void onNetworkConnect() {
  Serial.println(">>>> CONNECTED to network");
}

void onNetworkDisconnect() {
  Serial.println(">>>> DISCONNECTED from network");
}

void onNetworkError() {
  Serial.println(">>>> ERROR");
}
```
