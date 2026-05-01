# BMini4 - ESP8266 RC Car

WiFi-controlled RC car using ESP8266. Features a 5-speed transmission, steering servo, realistic braking, sound system, and dynamic lighting.

## Hardware

| Component | Role | Pin |
|-----------|------|-----|
| SG90 Servo | Steering | D1 (GPIO5) |
| Mini L298 | Motor driver | D2/D6 |
| JQ24-25H440 | DC Motor | L298 output |
| Buzzer | Horn | D5 (GPIO15) |
| 2x White LEDs | Headlights | D4 (GPIO2) |
| 4x Yellow LEDs | Turn indicators | D0/D3 |
| 2x Red LEDs | Brake/reverse | D7 (GPIO13) |
| RGB LED | Aux lighting | D8/RX/TX |

## Protocol

Commands are plain text, newline-terminated:

```
servo <angle>       Steering (45-135)
gas <level>         Throttle (0-5, auto gear shift)
brake on/off        Brake (hold 1.2s for reverse)
beep on/off         Horn
head on/off         Headlights
ind left/right/off  Turn indicators
led <mode>          LED animations
sound <mode>        Engine/siren/alarm
gear <-1/0-5>       Manual gear selection
settings <key> <val> Configure car behavior
```

Telemetry: `TELEM:battery%,gear,gas,dir,brake,conn`

## Network

- AP: SSID "BMini4", password "26032009"
- IP: 192.168.4.1:80
- Single client TCP server

## Timings

- Servo threshold: 8 deg minimum change
- Brake pulse: 200ms reverse burst
- Brake long: 1200ms to enter reverse
- Telemetry interval: 8000ms

## Support

| | |
|---|---|
| PayPal | `paypal.com/ncp/payment/W78F6W4TXZ4CS` |
| Binance | `1011264323` |
| Bybit | `467077834` |
| TRC20 | `TMW5uSDN6sMUBNirMoqY1icpsfa7GhPZfK` |
| BEP20/ERC20 | `0x7a8887c2ac3e596f6170c9e28b44e6b6d025c854` |
| LTC | `LVswXiD6Vd2dejXvGbZLa1R8jkvg748F4q` |
| TON | `UQAllRezWgHi3LPrSwyvAb4zazIph6j6goU7lMaqcFWFBxVH` |
| BTC | `1rSX6BDN1nqDMyBHqceySkZSs6PHUP23m` |
| SOL | `d8RonhC8oEHssrQjN1Y4UWHnd6MMP33XGCKtfNL4j59` |
