// Camera Slider STM-2 Firmware
// ---------------------------------
// Wiring:
// - STEP -> GPIO12
// - DIR  -> GPIO13
// - LIMIT -> GPIO16 (pull-up, triggers LOW)
// - OLED SDA -> GPIO5
// - OLED SCL -> GPIO4
// Uses external STEP/DIR driver (A4988/DRV8825/TMC2208)
// Board: ESP32-WROOM-32

#include <Arduino.h>
#include <AccelStepper.h>
#include "SSD1306Wire.h"
#include "BLESerial.h"
#include "bluepad32.h"

// Configuration constants
constexpr int STEP_PIN = 12;
constexpr int DIR_PIN = 13;
constexpr int LIMIT_PIN = 16;
constexpr int OLED_SDA = 5;
constexpr int OLED_SCL = 4;
constexpr int TRACK_LENGTH = 2000;          // steps
constexpr int DEFAULT_SPEED = 400;          // steps/s
constexpr int ACCELERATION = 500;           // steps/s^2
constexpr int JOG_STEP = 50;                // steps
constexpr unsigned long DISPLAY_INTERVAL = 100; // ms

// Globals
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
SSD1306Wire display(0x3C, OLED_SDA, OLED_SCL);
BLESerial BLEUART;
GamepadPtr gamepad;

enum State {IDLE, HOMING, RUNNING, PAUSED, ERROR};
State currentState = IDLE;
String lastStatus;
unsigned long stateChangeTime = 0;
unsigned long lastDisplay = 0;
int targetSpeed = DEFAULT_SPEED;
bool dirForward = true;

// Function declarations
void setupHardware();
void initDisplay();
void initBLESerial();
void initBluepad32();
void initStepper();
void pollBLESerial();
void onBLECommand(const String& cmd);
void pollGamepad();
void onGamepadEvent(GamepadPtr gp, bp32::GamepadEvent event);
void stateMachine();
void homeSlider();
void startRun();
void stopRun();
void emergencyStop();
void bounceIfAtEnd();
void jogManual(int steps);
void updateDisplay(const String& status);
void sendStatusBLE(const char* txt);

// -- Setup ---------------------------------------------------------------
void setup() {
    setupHardware();
    initDisplay();
    initBLESerial();
    initBluepad32();
    initStepper();
    stateChangeTime = millis();
}

// -- Loop ----------------------------------------------------------------
void loop() {
    pollBLESerial();
    pollGamepad();
    stateMachine();
    bounceIfAtEnd();
    bp32::update();
}

// -- Hardware ------------------------------------------------------------
/** Setup GPIO pins and peripherals */
void setupHardware() {
    pinMode(LIMIT_PIN, INPUT_PULLUP);
}

/** Initialize OLED display with splash */
void initDisplay() {
    display.init();
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Camera Slider STM-2");
    display.drawString(0, 12, "Waiting for BT connection...");
    display.display();
    delay(20);
}

/** Initialize BLE UART */
void initBLESerial() {
    BLEUART.begin("SliderSTM2");
}

/** Initialize Bluepad32 stack */
void initBluepad32() {
    bp32::setup(&onGamepadEvent);
}

/** Initialize stepper parameters */
void initStepper() {
    stepper.setMaxSpeed(1000);
    stepper.setAcceleration(ACCELERATION);
    stepper.disableOutputs();
}

// -- BLE -----------------------------------------------------------------
/** Poll BLE UART for commands */
void pollBLESerial() {
    if (BLEUART.available()) {
        String line = BLEUART.readStringUntil('\n');
        onBLECommand(line);
    }
}

/** Handle BLE command text */
void onBLECommand(const String& cmd) {
    if (cmd.startsWith("speed:")) {
        int pct = cmd.substring(6).toInt();
        pct = constrain(pct, 0, 100);
        targetSpeed = map(pct, 0, 100, 100, 1000);
    } else if (cmd == "start") {
        startRun();
    } else if (cmd == "stop") {
        stopRun();
    } else if (cmd.startsWith("jog:")) {
        int delta = cmd.substring(4).toInt();
        jogManual(delta);
    } else if (cmd == "home") {
        currentState = HOMING;
        stateChangeTime = millis();
        homeSlider();
    }
}

// -- Gamepad -------------------------------------------------------------
/** Poll connected gamepad */
void pollGamepad() {
    if (gamepad && gamepad->isConnected()) {
        int16_t lx = gamepad->axisX();
        if (abs(lx) > 1000) {
            int speed = map(abs(lx), 0, 512, 100, 1000);
            stepper.setSpeed((lx > 0 ? 1 : -1) * speed);
            stepper.runSpeed();
        }
    }
}

/** Handle gamepad events */
void onGamepadEvent(GamepadPtr gp, bp32::GamepadEvent event) {
    gamepad = gp;
    if (event == bp32::BUTTON_DOWN) {
        if (gp->a()) startRun();
        if (gp->b()) stopRun();
        if (gp->y()) emergencyStop();
        if (gp->dpadLeft()) jogManual(-JOG_STEP);
        if (gp->dpadRight()) jogManual(JOG_STEP);
        if (gp->l1()) targetSpeed = max(100, targetSpeed - 50);
        if (gp->r1()) targetSpeed = min(1000, targetSpeed + 50);
    }
}

// -- State Machine -------------------------------------------------------
/** Main state machine */
void stateMachine() {
    switch (currentState) {
        case IDLE:
            stepper.disableOutputs();
            updateDisplay("Idle");
            break;
        case HOMING:
            homeSlider();
            updateDisplay("Homing");
            break;
        case RUNNING:
            stepper.enableOutputs();
            stepper.setMaxSpeed(targetSpeed);
            stepper.run();
            updateDisplay("Running");
            break;
        case PAUSED:
            stepper.stop();
            updateDisplay("Paused");
            break;
        case ERROR:
            stepper.stop();
            updateDisplay("ERROR");
            break;
    }
}

/** Perform homing sequence */
void homeSlider() {
    stepper.enableOutputs();
    stepper.setSpeed(-200);
    stepper.runSpeed();
    if (digitalRead(LIMIT_PIN) == LOW) {
        stepper.setCurrentPosition(0);
        currentState = IDLE;
        sendStatusBLE("Homed");
    }
}

/** Start continuous run */
void startRun() {
    if (currentState == IDLE) {
        stepper.enableOutputs();
        stepper.moveTo(TRACK_LENGTH);
        currentState = RUNNING;
        stateChangeTime = millis();
        sendStatusBLE("Running");
    }
}

/** Soft stop */
void stopRun() {
    if (currentState == RUNNING) {
        stepper.stop();
        currentState = PAUSED;
        stateChangeTime = millis();
        sendStatusBLE("Paused");
    }
}

/** Emergency stop */
void emergencyStop() {
    stepper.stop();
    currentState = ERROR;
    stateChangeTime = millis();
    sendStatusBLE("ERROR");
}

/** Reverse at track ends */
void bounceIfAtEnd() {
    if (currentState == RUNNING && stepper.distanceToGo() == 0) {
        stepper.moveTo(dirForward ? 0 : TRACK_LENGTH);
        dirForward = !dirForward;
        sendStatusBLE("Changing direction");
    }
}

/** Jog immediate move */
void jogManual(int steps) {
    stepper.enableOutputs();
    stepper.move(steps);
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }
    stepper.disableOutputs();
}

/** Update OLED */
void updateDisplay(const String& status) {
    if (millis() - lastDisplay < DISPLAY_INTERVAL) return;
    lastDisplay = millis();
    if (status != lastStatus) {
        lastStatus = status;
        display.clear();
        display.drawString(0, 0, status);
        int bar = map(abs(stepper.speed()), 0, 1000, 0, 128);
        display.fillRect(0, 54, bar, 10);
        display.display();
    }
}

/** Safe BLE print */
void sendStatusBLE(const char* txt) {
    if (BLEUART.connected()) {
        BLEUART.println(txt);
    }
}

