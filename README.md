# BMini4 – ESP8266 RC Car


## Note about the : 
- This readme is outdated, I changed a lot in the code but didn't update the readme.
I'm currently busy studying, so there will be no updates for now.
i will update the readme and add some documentation comments to the code. 

## OVERVIEW
- WiFi-controlled RC car using ESP8266 with enhanced features:
- Steering servo (45-135° range) with dynamic sensitivity
- 5-speed manual transmission with automatic shifting
- Realistic braking: hold 1.2s to engage reverse gear
- Enhanced sound system: engine sounds, police siren, alarm
- Dynamic lighting: proportional brake lights, pulsing reverse lights
- Advanced telemetry: real-time gear, speed, and status data
- Smooth acceleration/deceleration with configurable rates
- Settings system for customizing car behavior

## HARDWARE & CONNECTIONS

- Component (used model) : role -> pin

- SEVO (SG90) : Steering -> D1 (GPIO5)
- Driver (Mini L298) : Drives the motor
  - Motor forward -> D2 (GPIO4)
  - Motor reverse -> D6 (GPIO12)
- DC Motor (JQ24-25H440) : motor -> to the driver's output
- Active Buzze (regular) : Beep (Horn) -> D5 (GPIO15)
- x2 White LEDs (5mm) : Headlights - D4 (GPIO2)
- x4 Yellow LEDs (5nm) : Turn indicator -> D0 (GPIO16), D3 (GPIO0)
- x2 Red LEDs (5nm) : Brake/reverse lights - D7 (GPIO13)
- RGB LED (5nm) : Aux LEDs -> D8 (GPIO1), RX, TX

## PROTOCOL & DETAILS

### Commands received (plain text, newline-terminated)

- servo <angle> - Set steering (45-135, dynamic sensitivity based on speed)
- gas <level> - Set throttle (0-5, with automatic gear shifting)
- brake on/off - Brake control (hold 1.2s → reverse gear)
- beep on/off - Horn
- head on/off - Headlights
- ind left/right/off - Turn indicators
- led off/blink/on - Simple LED modes
- led linear/breathe/heartbeat/strobe/glitch/candle - Fade animations

### Enhanced Commands

- sound on/off/horn/police/alarm - Sound system control
- gear <-1/0-5> - Manual gear selection (-1=reverse, 0=neutral, 1-5=forward)
- settings maxSpeed <100-1023> - Maximum motor speed
- settings steeringSens <50-100> - Steering sensitivity percentage
- settings accRate <10-100> - Acceleration rate (PWM per update)
- settings decRate <10-100> - Deceleration rate (PWM per update)
- settings engineSound on/off - Engine sound effects
- settings policeSiren on/off - Police siren mode
- settings dynamicSteer on/off - Dynamic steering sensitivity
- settings telemInterval <1000-10000> - Telemetry update interval (ms)

### Telemetry sent

- TELEM,battery%,currentGear,gasLevel,direction,brakeStatus,connection\n
  Format: TELEM:85,2,3,F,B,1 (Battery%, Gear, Throttle, Direction[F/R], Brake[B/N], Connection[1/0])
  Enhanced telemetry includes gear status, direction, and brake information

### NETWORK

- Access Point: SSID "BMini4", password "26032009"
- IP: 192.168.4.1:80
- Single client TCP server
- Proper client cleanup on disconnect for reliable reconnection

### TIMINGS

- Servo threshold: 8° minimum change to update
- Brake pulse: 200ms reverse burst when braking while moving
- Brake long: 1200ms hold to enter reverse gear
- Blink interval: 500ms (indicators, blink mode, reverse lights)
- Fade interval: 50ms (smooth animations)
- Telemetry: 8000ms keep-alive

## DASHBOARD & TODO (not really in order though)

- [x] AP init & Connection handler & CMD handler
- [x] motor & gas states
- [x] steering servo
- [x] Headlights / indicators / backlights
- [x] Brake & reverse logic
- [x] LED animations
- [x] DEBUG macros (i needed them)
- [x] Docmentation
- [x] Some Logic enhancements

- [ ] Add battery level
- [x] Aux LED and add animatons

## ENHANCED FEATURES

### 🚗 Transmission System
- **5-speed manual transmission** with realistic gear shifting
- **Automatic gear selection** based on throttle input
- **Smooth gear transitions** with sound effects
- **Reverse gear** with dedicated indicator lights

### 🔊 Sound System
- **Engine sounds** with RPM-based frequency variation
- **Police siren** with authentic siren pattern
- **Horn and alarm** modes
- **Configurable sound effects**

### 🎛️ Performance Settings
- **Adjustable acceleration/deceleration rates**
- **Dynamic steering sensitivity** (reduces sensitivity at high speed)
- **Configurable maximum speed**
- **Customizable telemetry intervals**

### 📊 Enhanced Telemetry
- **Real-time gear status**
- **Direction indicator** (Forward/Reverse)
- **Brake status monitoring**
- **Connection quality indicators**

### 💡 Dynamic Lighting
- **Proportional brake lights** (intensity based on brake force)
- **Pulsing reverse lights** when in reverse
- **Enhanced turn indicators** with realistic patterns
- **RGB LED animations** synchronized with car state

### 🎮 Control Optimizations
- **Smooth motor control** with configurable ramping
- **Anti-jerk acceleration** prevents wheel spin
- **Gear change protection** reduces throttle during shifts
- **Improved response time** for all controls

---

## Support This Project

If you find this useful, consider supporting:

### 💰 Cryptocurrency

<img src="https://img.shields.io/badge/Bitcoin-000000?style=for-the-badge&logo=bitcoin&logoColor=white" alt="Bitcoin"/>

```
15kPSKNLEgVH6Jy3RtNaT2mPsxTMS6MAEp
```

<img src="https://img.shields.io/badge/Ethereum-3C3C3D?style=for-the-badge&logo=ethereum&logoColor=white" alt="Ethereum"/>

```
0xc4f7076dd25a38f2256b5c23b8ca859cc42924cf
```

<img src="https://img.shields.io/badge/BNB-F3BA2F?style=for-the-badge&logo=binance&logoColor=white" alt="BNB"/>

```
0xc4f7076dd25a38f2256b5c23b8ca859cc42924cf
```

<img src="https://img.shields.io/badge/Solana-9945FF?style=for-the-badge&logo=solana&logoColor=white" alt="Solana"/>

```
EWcxGVtbohy8CdFLb2HNUqSHdecRiWKLywgMLwsXByhn
```

### 🏦 Exchange Platforms

<img src="https://img.shields.io/badge/Binance-FCD535?style=for-the-badge&logo=binance&logoColor=white" alt="Binance"/>

- **URL:** https://app.binance.com/uni-qr/Uzof5Lrq
- **ID:** `1011264323`

<img src="https://img.shields.io/badge/Bybit-F7A600?style=for-the-badge&logo=bybit&logoColor=white" alt="Bybit"/>

- **URL:** https://i.bybit.com/W2abUWF
- **ID:** `467077834`

### 💳 Traditional

<img src="https://img.shields.io/badge/PayPal-00457C?style=for-the-badge&logo=paypal&logoColor=white" alt="PayPal"/>

https://www.paypal.com/ncp/payment/W78F6W4TXZ4CS
