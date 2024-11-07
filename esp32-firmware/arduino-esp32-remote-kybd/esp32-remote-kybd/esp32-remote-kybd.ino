/*

-----
SETUP
-----
  - Serial monitor: 115200 baud

------
TO RUN
------
  - Plug in both USB ports at the same time (no button held)

----------------
TO PROGRAM BOARD
----------------

  - Select board in top bar drop-down: "ESP32-S3-USB-OTG"
  - Plug in both USB ports to host PC
  - Press Upload arrow
  - Wait...
  - After "Hard resetting via RTS pin..." press the RESET button on board

If not having luck, this try instead:
  - Start with board unplugged
  - Hold down BOOT button while plugging in debug port USB
  - Release BOOT button
  - Press Upload arrow

*/

// this whole ifndef...elif...else can probably be eliminated as we are USB device-only not OTG
#define ARDUINO_USB_MODE 0
#ifndef ARDUINO_USB_MODE
#error This ESP32 SoC has no Native USB interface
#elif ARDUINO_USB_MODE == 1
#warning This sketch should be used when USB is in OTG mode
void setup() {}
void loop() {}
#else

#include "USB.h"
#include "USBHIDKeyboard.h"
USBHIDKeyboard Keyboard;
#include <WiFi.h>
#include "secrets.h"  // add WLAN Credentials in here
#include <Arduino.h>
#include <WebServer.h>

// GPIO pins
#define LED_AMBER           15
#define LED_GREEN           16
#define USER_BTN_OK         14
#define USER_BTN_POWER      11
#define USER_BTN_RESET      10
#define FPANEL_HDD_ACTIVITY 1
#define FPANEL_MOBO_RESET   2
#define FPANEL_MOBO_POWER   3

////////////////////////////////////////////////////////////
//
// hid keyboard functions
//
////////////////////////////////////////////////////////////

void kybd_do_login() {
    // Goal is to type "<ENTER>$PASSWORD<ENTER>"
    Keyboard.press(KEY_RETURN);       // press ENTER
    Keyboard.releaseAll();            // release ENTER
    delay(250);
    Keyboard.print(LOGIN_PASSWORD);
    Keyboard.press(KEY_RETURN);       // press ENTER
    Keyboard.releaseAll();            // release ENTER
}

void kybd_do_confirm_logout() {
    // Goal is to type "<right arrow><ENTER>"
    Keyboard.press(KEY_RIGHT_ARROW);  // arrow right
    Keyboard.releaseAll();            // release arrow
    Keyboard.press(KEY_RETURN);       // press ENTER
    Keyboard.releaseAll();            // release ENTER
}

////////////////////////////////////////////////////////////
//
// state machine
//
////////////////////////////////////////////////////////////

enum state_t {
  STATE_UNINITIALIZED,      // LEDs: off
  STATE_MOBO_OFF,           // LEDs: solid amber
  STATE_MOBO_POWERING_UP,   // LEDs: blinking green
  STATE_MOBO_POWERING_DOWN, // LEDs: blinking amber
  STATE_MOBO_ON             // LEDs: solid green
};
volatile state_t state = STATE_UNINITIALIZED;
volatile unsigned long last_saw_hdd_activity_high = 0;
#define MAX_EXPECTED_TIME_ON_WITHOUT_HDD_ACTIVITY_MILLIS 100
#define POWERING_ON_DELAY_TO_LOGIN_MS ((30+28)*1000)
#define POWERING_OFF_DELAY_TO_LOGOUT_MS (3*1000)
#define POWERING_OFF_DELAY_TO_OFF_STATE_MS ((POWERING_OFF_DELAY_TO_LOGOUT_MS)+(3*1000))
volatile unsigned long powering_state_since = 0;
volatile bool reset_requested = false;
volatile bool power_signal_requested = false;
volatile bool login_keystrokes_requested = false;
volatile bool logout_keystrokes_requested = false;
volatile bool logout_keystrokes_sent = false;

void send_power_signal_to_mobo() {
  digitalWrite(FPANEL_MOBO_POWER, LOW);
  delay(100);
  digitalWrite(FPANEL_MOBO_POWER, HIGH);
}

void send_reset_signal_to_mobo() {
  digitalWrite(FPANEL_MOBO_RESET, LOW);
  delay(100);
  digitalWrite(FPANEL_MOBO_RESET, HIGH);
}

void transition_to_powering_up() {
  // do not use Serial.print; may be called from an ISR

  // update state vars
  state = STATE_MOBO_POWERING_UP;
  powering_state_since = millis();
}

void transition_to_powering_down() {
  // do not use Serial.print; may be called from an ISR

  // update state vars
  state = STATE_MOBO_POWERING_DOWN;
  powering_state_since = millis();
  logout_keystrokes_sent = false;
  logout_keystrokes_requested = false;
}

void dbg_print_state(const char *caller = NULL) {
  String state_str;
  switch (state) {
    case STATE_MOBO_OFF:
      state_str = "STATE_MOBO_OFF";
      break;
    case STATE_MOBO_POWERING_UP:
      state_str = "STATE_MOBO_POWERING_UP";
      break;
    case STATE_MOBO_ON:
      state_str = "STATE_MOBO_ON";
      break;
    case STATE_MOBO_POWERING_DOWN:
      state_str = "STATE_MOBO_POWERING_DOWN";
      break;
    case STATE_UNINITIALIZED:
      state_str = "STATE_UNINITIALIZED";
      break;
    default:
      state_str = "UNKNOWN";
      break;
  }
  Serial.printf("state: %s", state_str.c_str());
  if (caller != NULL)
    Serial.printf(" (caller: %s)\n", caller);
  else
    Serial.printf("\n");
}

void fsm() {
  switch (state) {
    case STATE_MOBO_OFF:
      // LEDs: solid amber
      digitalWrite(LED_AMBER, HIGH);
      digitalWrite(LED_GREEN, LOW);

      // sense HDD activity: if it's on, we should be in ON state instead
      if (digitalRead(FPANEL_HDD_ACTIVITY) == HIGH) {
        state = STATE_MOBO_ON;
        dbg_print_state();
      }
      break;

    case STATE_MOBO_POWERING_UP:
      // LEDs: blinking green
      digitalWrite(LED_AMBER, LOW);
      if (millis() % 50 < 25)
        digitalWrite(LED_GREEN, HIGH);
      else
        digitalWrite(LED_GREEN, LOW);

      // after appropriate delay, send login sequence
      if (millis() - powering_state_since > POWERING_ON_DELAY_TO_LOGIN_MS) {
        login_keystrokes_requested = true;
        state = STATE_MOBO_ON;
        dbg_print_state();
      }
      break;
    
    case STATE_MOBO_POWERING_DOWN:
      // LEDs: blinking amber
      digitalWrite(LED_GREEN, LOW);
      if (millis() % 50 < 25)
        digitalWrite(LED_AMBER, HIGH);
      else
        digitalWrite(LED_AMBER, LOW);

      // after appropriate delay, send logout confirm sequence
      if ((millis() - powering_state_since > POWERING_OFF_DELAY_TO_LOGOUT_MS) && (!logout_keystrokes_sent)) {
        logout_keystrokes_requested = true;
        dbg_print_state();
      }
      if ((millis() - powering_state_since > POWERING_OFF_DELAY_TO_OFF_STATE_MS)) {
        state = STATE_MOBO_OFF;
        dbg_print_state();
      }
      break;

    case STATE_MOBO_ON:
      // LEDs: solid green
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(LED_AMBER, LOW);

      // sense HDD activity: if it's off for long enough, we should be in OFF state instead
      if (digitalRead(FPANEL_HDD_ACTIVITY) == LOW && (millis() - last_saw_hdd_activity_high > MAX_EXPECTED_TIME_ON_WITHOUT_HDD_ACTIVITY_MILLIS)) {
        state = STATE_MOBO_OFF;
        dbg_print_state();
      }
      break;

    case STATE_UNINITIALIZED:
    default:
      // LEDs: off
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_AMBER, LOW);

      // sense HDD activity and go to either mobo on or mobo off state
      if (digitalRead(FPANEL_HDD_ACTIVITY) == LOW) {
        state = STATE_MOBO_OFF;
        dbg_print_state();
      } else {
        state = STATE_MOBO_ON;
        dbg_print_state();
      }
      break;
  }
}

////////////////////////////////////////////////////////////
//
// ISRs for button presses
//
////////////////////////////////////////////////////////////

volatile unsigned long lastOkPressTime = 0;
volatile unsigned long lastPowerPressTime = 0;
volatile unsigned long lastResetPressTime = 0;
const unsigned long BTN_DEBOUNCE_MS = 50;
volatile unsigned long btnInterruptsEnabled = 0; // start with buttons disabled

void IRAM_ATTR okButtonISR() {
  if (btnInterruptsEnabled == 0)
    return;

  // debounce
  unsigned long now = millis();
  if (now - lastOkPressTime >= BTN_DEBOUNCE_MS) {
    lastOkPressTime = now;

    // code to handle OK button press
    // ...(button does nothing, just here for future use)...
  }
}

void IRAM_ATTR powerButtonISR() {
  if (btnInterruptsEnabled == 0) {
    return;
  }

  // debounce
  unsigned long now = millis();
  if (now - lastPowerPressTime >= BTN_DEBOUNCE_MS) {
    lastPowerPressTime = now;

    // code to handle POWER button press
    switch(state) {
      case STATE_MOBO_OFF:
        // send signal and transition to powering up state
        power_signal_requested = true;
        transition_to_powering_up();
        break;
      case STATE_MOBO_ON:
        // send signal and transition to off state
        power_signal_requested = true;
        transition_to_powering_down();
        break;
      case STATE_MOBO_POWERING_UP:
      case STATE_MOBO_POWERING_DOWN:
      case STATE_UNINITIALIZED:
        // no-op if already in a powering state or uninitialized
        break;
    }
  }
  //dbg_print_state("powerButtonISR");
}

void IRAM_ATTR resetButtonISR() {
  if (btnInterruptsEnabled == 0) {
    return;
  }

  // debounce
  unsigned long now = millis();
  if (now - lastResetPressTime >= BTN_DEBOUNCE_MS) {
    lastResetPressTime = now;

    // when resetting from off state, send power signal instead of reset signal
    if (state == STATE_MOBO_OFF) {
      power_signal_requested = true;
    } else {
      reset_requested = true;
    }

    transition_to_powering_up();
  }
}

////////////////////////////////////////////////////////////
//
// WiFi
//
////////////////////////////////////////////////////////////

void connectToWiFi() {
  Serial.println();
  Serial.println("******************************************************");
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

////////////////////////////////////////////////////////////
//
// webserver
//
////////////////////////////////////////////////////////////

// TRACE output simplified, can be deactivated here
#define TRACE(...) Serial.printf(__VA_ARGS__)

// name of the server. You reach it using http://webserver
#define HOSTNAME "webserver"

// local time zone definition (U.S. Mountain Time)
#define TIMEZONE "MST7MDT,M3.2.0,M11.1.0"

WebServer server(52000);

// return string with HDD activity status string
String getHddActivityString() {
  return "hdd: " + String(digitalRead(FPANEL_HDD_ACTIVITY)) + " (last HIGH: " + String(millis() - last_saw_hdd_activity_high) + "ms ago)";
}

// return string with current state
String getStateString() {
  String result;
  switch (state) {
    case STATE_MOBO_OFF:
      result = "state: STATE_MOBO_OFF\n" + getHddActivityString() + ")";
      break;
    case STATE_MOBO_POWERING_UP:
      result = "state: STATE_MOBO_POWERING_UP\n" + getHddActivityString();
      break;
    case STATE_MOBO_ON:
      result = "state: STATE_MOBO_ON\n" + getHddActivityString();
      break;
    case STATE_MOBO_POWERING_DOWN:
      result = "state: STATE_MOBO_POWERING_DOWN\n" + getHddActivityString();
      break;
    case STATE_UNINITIALIZED:
      result = "state: STATE_UNINITIALIZED\n" + getHddActivityString();
      break;
    default:
      result = "state: (unknown)\n" + getHddActivityString();
      break;
  }
  return result;
}

// endpoint: /
void handleRoot() {
  String result;
  Serial.println();
  Serial.println("******************************************************");
  Serial.println("GET /");
  result += getStateString();
  result += "\n\n";
  result += "endpoints:\n";
  result += "  /                 (this page)\n";
  result += "  /info             system info\n";
  result += "  /on               power on (no-op if already powering up or in progress)\n";
  result += "  /off              power off (no-op if already powering down or in progress)\n";
  result += "  /reset            reset\n";
  result += "\n";
  Serial.printf("responding with list of endpoints\n");
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "text/javascript; charset=utf-8", result);
}

// endpoint: /info
void handleInfo() {
  String result;
  Serial.println();
  Serial.println("******************************************************");
  Serial.println("GET /info");
  result += "{\n";
  result += "  \"Chip Model\": " + String(ESP.getChipModel()) + ",\n";
  result += "  \"Chip Cores\": " + String(ESP.getChipCores()) + ",\n";
  result += "  \"Chip Revision\": " + String(ESP.getChipRevision()) + ",\n";
  result += "  \"flashSize\": " + String(ESP.getFlashChipSize()) + ",\n";
  result += "  \"freeHeap\": " + String(ESP.getFreeHeap()) + ",\n";
  result += "}\n";
  result += getStateString();
  result += "\n\n";
  Serial.printf("responding with system info\n");
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "text/javascript; charset=utf-8", result);
}

// endpoint: /on
void handleOn() {
  String result = "OK";

  // update state
  switch (state) {
    case STATE_MOBO_OFF:
      power_signal_requested = true;
      transition_to_powering_up();
      result = "handleOn(): going to STATE_MOBO_POWERING_UP";
      break;
    case STATE_MOBO_ON:
      result = "handleOn(): no-op (already in STATE_MOBO_ON)";
      break;
    case STATE_MOBO_POWERING_UP:
      result = "handleOn(): no-op (already in STATE_MOBO_POWERING_UP for " + String(millis() - powering_state_since) + "ms)";
      break;
    case STATE_MOBO_POWERING_DOWN:
      result = "handleOff(): no-op (in STATE_MOBO_POWERING_DOWN for " + String(millis() - powering_state_since) + "ms)";
      break;
    case STATE_UNINITIALIZED:
      result = "handleOn(): no-op (in STATE_UNINITIALIZED)";
      break;
    default:
      result = "state: (unknown)";
      break;
  }

  Serial.println();
  Serial.println("******************************************************");
  Serial.println("GET /on");
  Serial.printf("responding: '%s'\n", result.c_str());
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "text/javascript; charset=utf-8", result);
}

// endpoint: /off
void handleOff() {
  String result = "OK";

  // update state
  switch (state) {
    case STATE_MOBO_OFF:
      result = "handleOff(): no-op (already in STATE_MOBO_OFF)";
      break;
    case STATE_MOBO_ON:
      power_signal_requested = true;
      transition_to_powering_down();
      result = "handleOff(): going to STATE_MOBO_POWERING_DOWN";
      break;
    case STATE_MOBO_POWERING_UP:
      result = "handleOff(): no-op (in STATE_MOBO_POWERING_UP for " + String(millis() - powering_state_since) + "ms)";
      break;
    case STATE_MOBO_POWERING_DOWN:
      result = "handleOff(): no-op (already in STATE_MOBO_POWERING_DOWN for " + String(millis() - powering_state_since) + "ms)";
      break;
    default:
    case STATE_UNINITIALIZED:
      result = "handleOff(): no-op (in STATE_UNINITIALIZED)";
      break;
  }

  Serial.println();
  Serial.println("******************************************************");
  Serial.println("GET /off");
  Serial.printf("responding: '%s'\n", result.c_str());
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "text/javascript; charset=utf-8", result);
}

// endpoint: /reset
void handleReset() {
  String result = "OK";

  // reset motherboard

  // send a power signal instead of reset if motherboard is off
  if (state == STATE_MOBO_OFF) {
    Serial.printf("handleReset(): setting power_signal_requested flag\n");
    power_signal_requested = true;
  } else {
    Serial.printf("handleReset(): setting reset_requested flag\n");
    reset_requested = true;
  }

  transition_to_powering_up();

  Serial.println();
  Serial.println("******************************************************");
  Serial.println("GET /reset");
  Serial.printf("responding: '%s'\n", result.c_str());
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "text/javascript; charset=utf-8", result);
}

// endpoint: /login
void handleLogin() {
  String result = "OK";

  // set flag to send login keystrokes
  login_keystrokes_requested = true;
  Serial.printf("handleLogin(): setting login_keystrokes_requested flag\n");

  Serial.println();
  Serial.println("******************************************************");
  Serial.println("GET /login");
  Serial.printf("responding: '%s'\n", result.c_str());
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "text/javascript; charset=utf-8", result);
}

void registerHttpHandlers() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/info", HTTP_GET, handleInfo);
  server.on("/on", HTTP_GET, handleOn);
  server.on("/off", HTTP_GET, handleOff);
  server.on("/reset", HTTP_GET, handleReset);
  server.on("/login", HTTP_GET, handleLogin);
}

////////////////////////////////////////////////////////////
//
// setup() and loop()
//
////////////////////////////////////////////////////////////

void setup() {
  // open the serial port
  Serial.begin(115200);

  // initialize control over the keyboard:
  Keyboard.begin();
  USB.begin();

  // initialize GPIO pins
  pinMode(LED_AMBER, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(USER_BTN_OK, INPUT_PULLUP);
  pinMode(USER_BTN_POWER, INPUT_PULLUP);
  pinMode(USER_BTN_RESET, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(USER_BTN_OK), okButtonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(USER_BTN_POWER), powerButtonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(USER_BTN_RESET), resetButtonISR, FALLING);
  pinMode(FPANEL_MOBO_POWER, OUTPUT); 
  pinMode(FPANEL_MOBO_RESET, OUTPUT);
  digitalWrite(FPANEL_MOBO_POWER, HIGH);
  digitalWrite(FPANEL_MOBO_RESET, HIGH);
  pinMode(FPANEL_HDD_ACTIVITY, INPUT);

  // connect to WiFi
  connectToWiFi();

  // start webserver
  server.begin();

  // get current time
  Serial.println();
  Serial.println("******************************************************");
  Serial.println("setting up ntp");
  configTzTime(TIMEZONE, "pool.ntp.org");

  Serial.println();
  Serial.println("******************************************************");
  Serial.println("registering http handlers");
  registerHttpHandlers();


  // clear all flags
  reset_requested = false;
  power_signal_requested = false;
  login_keystrokes_requested = false;
  logout_keystrokes_requested = false;
  logout_keystrokes_sent = false;

  // start state and enable interrupts
  state = STATE_UNINITIALIZED;
  btnInterruptsEnabled = 1;
}

void loop() {
  // keep track of last time HDD activity input was HIGH (used by FSM)
  if (digitalRead(FPANEL_HDD_ACTIVITY) == HIGH)
    last_saw_hdd_activity_high = millis();

  // state machine on every loop iteration
  fsm();

  // handle webserver
  server.handleClient();

  // handle power and reset requests
  if (reset_requested) {
    reset_requested = false;
    send_reset_signal_to_mobo();
    Serial.printf("loop(): handled reset request and cleared reset_requested flag\n");
  }
  if (power_signal_requested) {
    power_signal_requested = false;
    send_power_signal_to_mobo();
    Serial.printf("loop(): handled power signal request and cleared power_signal_requested flag\n");
  }

  // send login keystrokes if requested
  if (login_keystrokes_requested) {
    login_keystrokes_requested = false;
    kybd_do_login();
    Serial.printf("loop(): handled login keystrokes request and cleared login_keystrokes_requested flag\n");
  }
  if (logout_keystrokes_requested) {
    logout_keystrokes_requested = false;
    logout_keystrokes_sent = true;
    kybd_do_confirm_logout();
    Serial.printf("loop(): handled logout keystrokes request and cleared logout_keystrokes_requested flag\n");
  }
}
#endif /* ARDUINO_USB_MODE */
