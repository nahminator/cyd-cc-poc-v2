# CYD Flight Tracker

A simple real-time flight tracker for the **Cheap Yellow Display** (ESP32-2432S028R) — an ESP32 development board with a 320×240 ILI9341 touchscreen. It polls a local [dump1090](https://github.com/flightaware/dump1090) ADS-B receiver over WiFi and displays nearby aircraft on two swipeable screens.  As implemented here, it's intended to run on the same network as your ADS-B receiver with dump1090.

## Views

### Flight Detail View (default)
Shows the closest aircraft in range:
- Large callsign at the top
- Altitude, speed, distance, and airline name
- Animated compass rose showing the aircraft's heading

### Radar View (swipe to switch)
A top-down radar display centered on your home position:
- 4 concentric range rings at 10 / 20 / 30 / 40 mile intervals
- Color-coded dot for each aircraft within 40 miles
- **Flight Board** panel listing the 10 closest flights by callsign, each in a matching color
- Closest aircraft highlighted in the status bar

Swipe anywhere on the touchscreen to toggle between views.

## Hardware

| Part | Notes |
|---|---|
| ESP32-2432S028R ("Cheap Yellow Display") | ESP32 + ILI9341 320×240 LCD + XPT2046 touchscreen |
| ADS-B receiver + antenna | RTL-SDR dongle or similar |
| Computer running dump1090 | On the same local network as the CYD |

The CYD is widely available on AliExpress and Amazon for ~$10–15.

## Requirements

### Software
- [Arduino IDE](https://www.arduino.cc/en/software) or [arduino-cli](https://arduino.github.io/arduino-cli/)
- ESP32 board package: `esp32:esp32` (install via Board Manager)

### Arduino Libraries
Install all of these via the Arduino Library Manager:

| Library | Version tested |
|---|---|
| TFT_eSPI | 2.5.43 |
| XPT2046_Touchscreen | 1.4 |
| ArduinoJson | 7.x |

> **TFT_eSPI user setup:** The library must be configured for the CYD's ILI9341. Copy or symlink the appropriate `User_Setup.h` for the ESP32-2432S028R into the TFT_eSPI library folder before compiling.

## Setup

### 1. Clone the repo
```bash
git clone https://github.com/your-username/cyd-flight-tracker.git
cd cyd-flight-tracker
```

### 2. Create your config file
```bash
cp flight_tracker/config.h.example flight_tracker/config.h
```

Edit `flight_tracker/config.h` with your values:

```cpp
#define WIFI_SSID     "your_wifi_network_name"
#define WIFI_PASSWORD "your_wifi_password"
#define API_URL       "http://192.168.x.x/dump1090/data/aircraft.json"

const double HOME_LAT =  12345;   // your latitude
const double HOME_LON = -12345;  // your longitude
```

**Finding your coordinates:** Open Google Maps, right-click your location, and copy the lat/lon shown at the top of the context menu.

### 3. Compile and flash
```bash example - paths may vary based on your setup
arduino-cli compile --fqbn esp32:esp32:esp32 --libraries ~/Documents/Arduino/libraries flight_tracker
arduino-cli upload --fqbn esp32:esp32:esp32 --port /dev/cu.usbserial-XXXXX --upload-property upload.speed=115200 flight_tracker
```

Replace `/dev/cu.usbserial-XXXXX` with your CYD's serial port (`/dev/ttyUSB0` on Linux).  Update --libraries path for your installation.

## Project Structure

```
flight_tracker/
  flight_tracker.ino   — main sketch
  airlines.h           — ICAO prefix → airline name lookup (281 entries)
  config.h             — your personal config (gitignored)
  config.h.example     — template for config.h
.gitignore
```

## How It Works

1. On boot, connects to WiFi and shows a status splash screen
2. Every 10 seconds, fetches `aircraft.json` from dump1090 over HTTP
3. Filters out aircraft older than 60 seconds and calculates haversine distance from home
4. **Flight view** displays the single closest aircraft with its heading on a compass rose
5. **Radar view** plots all aircraft within 40 miles using bearing + distance, sorted and color-coded by proximity
6. The status bar counts down to the next refresh and shows the closest callsign on the radar view

## Compatibility

Tested with both `dump1090-mutability` and `dump1090-fa` — the parser handles both sets of JSON field names (`altitude`/`alt_baro`, `speed`/`gs`).
