# BMini4 — ESP8266 RC Car

A WiFi-controlled RC car built on an ESP8266. Plain-text TCP protocol drives
steering, throttle, lights, horn, and telemetry.

> 🛠️ **Vibe-coded.** Built with AI assistance; no formal review. Tested on the
> bench, not on a track.

## Hardware

SG90 servo · Mini L298 · JQ24 DC motor · buzzer · white/red/yellow/RGB LEDs ·
Li-Ion BMS.

| Component | Pin |
|-----------|-----|
| SG90 Servo | D1 (GPIO5) |
| Mini L298 | D2 / D6 |
| Buzzer | D5 (GPIO15) |
| Headlights | D4 (GPIO2) |
| Turn indicators | D0 / D3 |
| Brake/reverse | D7 (GPIO13) |
| RGB LED | D8 / RX / TX |

## Protocol (TCP, newline-terminated)

```
servo <angle>        Steering (45-135)
gas <level>          Throttle (0-5, auto gear shift)
brake on/off         Brake (hold 1.2s for reverse)
beep on/off          Horn
head on/off          Headlights
ind left/right/off   Turn indicators
led <mode>           LED animations
gear <-1/0-5>        Manual gear
settings <key> <val> Configure behavior
```

Telemetry: `TELEM:battery%,gear,gas,dir,brake,conn`

## Network

- AP: SSID **`BMini4`**, password `26032009`
- IP: `192.168.4.1:80`, single-client TCP server

## Related project

A bigger sibling, **[Rio](https://github.com/Polymath2603/Rio)**, uses an
ESP32-S3 with a camera and a WebSocket protocol — see it for the next step up.

## Known issues / Limitations

- Single-client only; no reconnect logic.
- Servo threshold 8°; brake pulse 200ms, long-brake 1200ms (tunable in code).

## Apology

No photos or drive videos are included.

## License

MIT (see LICENSE)
