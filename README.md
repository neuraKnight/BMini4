# BMini4 – WiFi Remote Control Mini Car

A full-featured remote-controlled car project using ESP8266 and Android.

## Features

- Phone tilt steering (accelerometer)
- Proportional 5-level throttle
- Realistic braking with reverse pulse
- Long brake → reverse gear
- Headlights, turn signals, horn, auxiliary lighting effects
- Breathing LED fade mode
- Connection status monitoring with timeout detection
- Auto-connect on app launch
- Clean, responsive UI in landscape mode

## Hardware

- ESP8266 (NodeMCU / Wemos D1 Mini recommended)
- DC motor + driver (L298N, TB6612, etc.)
- Standard RC servo for steering
- LEDs for headlights, brake lights, indicators
- Active buzzer
- Proper power separation for motors and logic

See pinout table in documentation.

## Setup

### ESP8266
1. Open `BMini4.ino` in Arduino IDE
2. Install ESP8266 board package
3. Select your board and port
4. Upload the sketch

### Android App
1. Open project in Android Studio
2. Build and install on your phone
3. Connect to **BMini4** WiFi network (password: 26032009)
4. App auto-connects on launch

## Usage

- Hold phone in landscape
- Tilt left/right to steer
- Touch and drag up on **Gas** pedal for speed
- Touch and hold **Brake** pedal to stop (long hold = reverse)
- Use switches and buttons for lights, indicators, horn

## Debugging

Define `DEBUG 1` at the top of the .ino file to enable verbose Serial output (9600 baud).

Android logs available via Logcat (filter tag: "BMini4").

## Protocol

Simple text commands ending in newline:

- `servo 90`
- `gas 3`
- `brake on`
- `ind left`
- `led fade`
- etc.

See full command list in documentation.

## License & Credits

Personal project by neuraknight – feel free to modify and improve!

Enjoy driving your BMini4! 🚗💨
