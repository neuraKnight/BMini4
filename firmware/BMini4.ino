/**
 * BMini4 – ESP8266 Remote Control Car Controller
 * 
 * Full-featured WiFi-controlled mini car with realistic driving physics.
 * Acts as Access Point and TCP server on port 80.
 * 
 * Author: Full-Stack Scientist
 * Date: 2025
 */

#include <Servo.h>
#include <ESP8266WiFi.h>

// Uncomment to enable verbose serial debugging
//#define DEBUG 1

// =============================================================================
// PIN CONFIGURATION
// =============================================================================

const int PIN_MOTOR_ENA     = 4;   // D2  - Motor PWM (speed)
const int PIN_MOTOR_DIR     = 12;  // D6  - Motor direction
const int PIN_SERVO         = 5;   // D1  - Steering servo signal
const int PIN_BUZZER        = 15;  // D8  - Horn / beep
const int PIN_HEADLIGHTS    = 2;   // D4  - Front lights
const int PIN_FADE_LED      = 14;  // D5  - Auxiliary LEDs (PWM capable)
const int PIN_BACKLIGHTS    = 13;  // D7  - Brake / reverse lights
const int PIN_IND_LEFT      = 16;  // D0  - Left turn indicator
const int PIN_IND_RIGHT     = 0;   // D3  - Right turn indicator

// =============================================================================
// CONSTANTS & TIMINGS
// =============================================================================

const int SERVO_MIN         = 45;    // Safety limits for servo
const int SERVO_MAX         = 135;
const int SERVO_CENTER      = 90;

const int PWM_MAX           = 1023;
const int GAS_STEP          = 204;   // PWM per gas level (1023 / 5)

const int BEEP_FREQ         = 2000;

const int BRAKE_PULSE_MS    = 200;   // Short reverse burst when braking while moving
const int BRAKE_LONG_MS     = 1000;  // Time before long brake triggers reverse gear

const int BLINK_INTERVAL_MS = 500;   // 1 Hz blink rate
const int FADE_INTERVAL_MS  = 50;    // Smooth fade update
const int TELEM_INTERVAL_MS = 8000;  // Keep-alive telemetry interval

// Breathing effect sine table (values 0–1023, 23 steps)
const int SINE_TABLE_SIZE = 23;
const int SINE_TABLE[SINE_TABLE_SIZE] = {
  0, 0, 1, 5, 21, 64, 147, 280, 457, 657, 842, 975,
  1023, 975, 842, 657, 457, 280, 147, 64, 21, 5, 1, 0
};

// =============================================================================
// ENUMS & STATE
// =============================================================================

enum IndMode { IND_OFF = 0, IND_LEFT = 1, IND_RIGHT = 2 };
enum LedMode { LED_OFF = 0, LED_BLINK = 1, LED_FADE = 2, LED_ON = 3 };

struct ControlState {
  bool   forward            = true;     // Direction: true = forward
  int    servo              = SERVO_CENTER;
  bool   beep               = false;
  bool   headlights         = false;
  IndMode indMode           = IND_OFF;
  LedMode ledMode           = LED_OFF;
  int    gasLevel           = 0;
  bool   brake              = false;
  unsigned long brakeStart  = 0;
  bool   brakeLongTriggered = false;
  bool   brakePulseActive   = false;
  unsigned long brakePulseStart = 0;
} state;

Servo steeringServo;
WiFiServer server(80);
WiFiClient client;

// =============================================================================
// SETUP
// =============================================================================

void setup() {
#ifdef DEBUG
  Serial.begin(9600);
#endif

  // Start Access Point
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP("BMini4", "26032009", 1, 0, 1)) {
#ifdef DEBUG
    Serial.println("Failed to start AP");
#endif
  }
  server.begin();

#ifdef DEBUG
  Serial.println("=== BMini4 AP Started ===");
  Serial.print("IP: "); Serial.println(WiFi.softAPIP());
#endif

  // Initialize outputs
  pinMode(PIN_MOTOR_ENA, OUTPUT);
  pinMode(PIN_MOTOR_DIR, OUTPUT);
  pinMode(PIN_HEADLIGHTS, OUTPUT);
  pinMode(PIN_BACKLIGHTS, OUTPUT);
  pinMode(PIN_IND_LEFT, OUTPUT);
  pinMode(PIN_IND_RIGHT, OUTPUT);
  pinMode(PIN_FADE_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  // Initial state
  analogWrite(PIN_MOTOR_ENA, 0);
  digitalWrite(PIN_MOTOR_DIR, LOW);
  digitalWrite(PIN_HEADLIGHTS, LOW);
  digitalWrite(PIN_BACKLIGHTS, LOW);
  digitalWrite(PIN_IND_LEFT, LOW);
  digitalWrite(PIN_IND_RIGHT, LOW);
  analogWrite(PIN_FADE_LED, 0);
  noTone(PIN_BUZZER);

  // Attach and center servo
  steeringServo.attach(PIN_SERVO);
  steeringServo.write(SERVO_CENTER);
}

// =============================================================================
// MAIN LOOP
// =============================================================================

void loop() {
  handleClient();
  handleAnimations();
  yield();
}

// =============================================================================
// NETWORK & COMMAND HANDLING
// =============================================================================

/**
 * Handles TCP client connection and incoming commands using readStringUntil()
 */
void handleClient() {
  unsigned long now = millis();

  // Clean up disconnected client
  if (client && !client.connected()) {
    client.stop();
#ifdef DEBUG
    Serial.println("Client disconnected");
#endif
  }

  // Accept new client
  if (!client.connected()) {
    client = server.available();
    if (client) {
#ifdef DEBUG
      Serial.println("Client connected");
#endif
    }
    return;
  }

  // Read complete lines
  while (client.available()) {
    String line = client.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
#ifdef DEBUG
      Serial.print("Received: "); Serial.println(line);
#endif
      processCmd((char*)line.c_str());
    }
  }
}

/**
 * Parses incoming command and routes to appropriate handler
 */
void processCmd(char* cmd) {
  struct CmdHandler {
    const char* prefix;
    void (*handler)(const char* arg);
  };

  static const CmdHandler handlers[] = {
    {"servo ", handleServo},
    {"beep ",  handleBeep},
    {"head ",  handleHead},
    {"ind ",   handleInd},
    {"led ",   handleLed},
    {"gas ",   handleGas},
    {"brake ", handleBrake}
  };
  const int NUM_HANDLERS = sizeof(handlers) / sizeof(handlers[0]);

  bool handled = false;
  for (int i = 0; i < NUM_HANDLERS; i++) {
    size_t len = strlen(handlers[i].prefix);
    if (strncmp(cmd, handlers[i].prefix, len) == 0) {
      handlers[i].handler(cmd + len);
      handled = true;
#ifdef DEBUG
      Serial.print("Handled: "); Serial.println(handlers[i].prefix);
#endif
      break;
    }
  }

  if (!handled) {
#ifdef DEBUG
    Serial.print("Unknown command: "); Serial.println(cmd);
#endif
  }
}

// Command handlers (see individual comments in original code)

// =============================================================================
// MOTOR CONTROL
// =============================================================================

void updateMotor() {
  if (state.brakePulseActive) return;  // Pulse handled separately

  int pwm = 0;
  if (state.brakeLongTriggered) {
    pwm = 2 * GAS_STEP;  // Slow speed in reverse after long brake
  } else if (state.brake) {
    pwm = 0;
  } else {
    pwm = state.gasLevel * GAS_STEP;
    pwm = min(pwm, PWM_MAX);
  }

  digitalWrite(PIN_MOTOR_DIR, state.forward ? LOW : HIGH);
  analogWrite(PIN_MOTOR_ENA, pwm);
}

// =============================================================================
// ANIMATION & EFFECTS LOOP
// =============================================================================

void handleAnimations() {
  unsigned long now = millis();

  // End active brake pulse
  if (state.brakePulseActive && now - state.brakePulseStart >= BRAKE_PULSE_MS) {
    state.brakePulseActive = false;
    analogWrite(PIN_MOTOR_ENA, 0);
    digitalWrite(PIN_MOTOR_DIR, state.forward ? LOW : HIGH);
  }

  // Telemetry keep-alive
  static unsigned long lastTelem = 0;
  if (client.connected() && now - lastTelem > TELEM_INTERVAL_MS) {
    client.println("TELEM:,,100");
    lastTelem = now;
#ifdef DEBUG
    Serial.println("Sent telemetry");
#endif
  }

  // Update motor state
  updateMotor();

  // Blink logic (indicators, LED blink, reverse lights)
  static unsigned long lastBlink = 0;
  static bool blinkState = false;
  bool needBlink = (state.indMode != IND_OFF) || (state.ledMode == LED_BLINK) || !state.forward;

  if (needBlink && now - lastBlink >= BLINK_INTERVAL_MS) {
    blinkState = !blinkState;
    lastBlink = now;

    // Indicators
    if (state.indMode == IND_LEFT) {
      digitalWrite(PIN_IND_LEFT, blinkState);
      digitalWrite(PIN_IND_RIGHT, LOW);
    } else if (state.indMode == IND_RIGHT) {
      digitalWrite(PIN_IND_RIGHT, blinkState);
      digitalWrite(PIN_IND_LEFT, LOW);
    }

    // Blink mode for aux LEDs
    if (state.ledMode == LED_BLINK) {
      analogWrite(PIN_FADE_LED, blinkState ? PWM_MAX : 0);
    }

    // Reverse lights blink
    if (!state.forward) {
      digitalWrite(PIN_BACKLIGHTS, blinkState);
    }
  }

  // Solid states
  if (state.ledMode == LED_ON) {
    analogWrite(PIN_FADE_LED, PWM_MAX);
  } else if (state.ledMode == LED_OFF) {
    analogWrite(PIN_FADE_LED, 0);
  }

  // Long brake → reverse gear
  if (state.brake && now - state.brakeStart > BRAKE_LONG_MS && !state.brakeLongTriggered) {
    state.brakeLongTriggered = true;
    state.forward = false;
#ifdef DEBUG
    Serial.println("Entered reverse gear");
#endif
  }

  // Fade breathing effect
  static unsigned long lastFade = 0;
  static int fadeIdx = 0;
  if (state.ledMode == LED_FADE && now - lastFade >= FADE_INTERVAL_MS) {
    lastFade = now;
    analogWrite(PIN_FADE_LED, SINE_TABLE[fadeIdx]);
    fadeIdx = (fadeIdx + 1) % SINE_TABLE_SIZE;
  }
}
