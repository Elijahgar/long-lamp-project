# Long Distance Lamp (Arduino + ESP32)

Synchronizes two lamps over the internet using Adafruit IO (MQTT).

## Hardware
- Board: ESP32 Dev Module
- OLED: SSD1306 128x64 I²C (addr 0x3C)
- Button on GPIO 26 (INPUT_PULLUP), MOSFET gate on GPIO 25

## Libraries (Arduino Library Manager)
- PubSubClient (by Nick O'Leary)
- Adafruit GFX Library (by Adafruit)
- Adafruit SSD1306 (by Adafruit)
*(WiFi.h, Wire.h, time.h come with the ESP32 core)*

## Setup
1. Open `LongLamp.ino` in Arduino IDE.
2. Replace the placeholder values at the top:
   - `WIFI_SSID`, `WIFI_PASS`, `AIO_USERNAME`, `AIO_KEY`, `DEVICE_ID`
3. Tools → Board: choose your ESP32 board.  
   Tools → Port: pick the correct port.
4. Install libraries: **Sketch → Include Library → Manage Libraries…** and search the names above.
5. **Upload**.
