#include <ArduinoLog.h>         // Logging library
#include <RunningMedian.h>      // For smoothing throttle input
#include <TaskScheduler.h>      // For task scheduling
#include <Bounce2.h>            // For button debouncing
#include "FiniteStateMachine.h" // FSM library

// Enum for voltage modes
enum VoltageMode
{
    Voltage3V3,
    Voltage5V
};

// Configuration constants
#define SELECTED_VOLTAGE_MODE Voltage5V
#define USE_PROPORTIONAL 1
#define THROTTLE_SMOOTH_DURATION 2000
#define LOG_LEVEL LOG_LEVEL_VERBOSE

// Voltage and throttle mapping constants
const float SYSTEM_VOLTAGE_MAP[] = {3.3, 5.0};
const float THROTTLE_MIN_VOLTAGE_MAP[] = {0.6, 0.8};
const float THROTTLE_MAX_VOLTAGE_MAP[] = {2.8, 4.4};

const float SYSTEM_VOLTAGE = SYSTEM_VOLTAGE_MAP[SELECTED_VOLTAGE_MODE];
const float THROTTLE_MIN_VOLTAGE = THROTTLE_MIN_VOLTAGE_MAP[SELECTED_VOLTAGE_MODE];
const float THROTTLE_MAX_VOLTAGE = THROTTLE_MAX_VOLTAGE_MAP[SELECTED_VOLTAGE_MODE];
const int PWM_MIN = int((THROTTLE_MIN_VOLTAGE / SYSTEM_VOLTAGE) * 255);
const int PWM_MAX = int((THROTTLE_MAX_VOLTAGE / SYSTEM_VOLTAGE) * 255);

// Pin definitions
const int THROTTLE_PIN_IN = A0;
const int THROTTLE_PIN_OUT = 9;
const int PAS_PIN_IN = 2;
const int BRAKE_PIN_IN = 3;
const int BRAKE_PIN_OUT = 10;
const int LED_PIN_OUT = 13;
const int UP_BUTTON_PIN = 7;
const int DOWN_BUTTON_PIN = 8;

// PAS and timing constants
const int MAGNET_COUNT = 12;
const unsigned long PAS_TIMEOUT_MS = 1500;

// ISR variables
volatile unsigned long lastPasPulseTime = 0;
volatile unsigned long isrOldTime = 0;
volatile unsigned int periodHigh = 0, periodLow = 0, period = 0;
volatile unsigned int pulseCount = 0;

bool pedalingForward = false;
bool brakeActive = false;
bool errorHasOccurred = false;

float referencePWM = PWM_MIN;
int currentSpeedLevel = 1;
int lastSpeedLevel = 1;

unsigned long speedChangeTimestamp = 0;

// Speed level configurations
struct SpeedLevel
{
    float maxPWMFactor;
};

const SpeedLevel speedLevels[] = {
    {0.0}, // Level 0: Idle
    {0.2}, // Level 1
    {0.4}, // Level 2
    {0.6}, // Level 3
    {0.8}, // Level 4
    {1.0}  // Level 5: Full power
};

// Utility objects
RunningMedian throttleMedian(3);
Scheduler taskScheduler;
Bounce upButton, downButton;

// Task declarations
Task taskMonitorPAS(500, TASK_FOREVER, &monitorPASActivity);
Task taskUpdateLED(100, TASK_FOREVER, &updateLEDState);
Task taskMonitorButtons(50, TASK_FOREVER, &monitorButtonActivity);

// FSM States
State detectionState(onDetectEnter, onDetectUpdate, NULL);
State brakeActiveState(onBrakeActiveEnter, NULL, NULL);
State erroredState(onErrorEnter, NULL, NULL);

// FSM Instance
FiniteStateMachine stateMachine(detectionState);

void setup()
{
    Serial.begin(115200);
    Log.begin(LOG_LEVEL, &Serial);

    configurePins();
    attachInterrupts();
    initializeTasks();
    initializeButtons();
    initializeVariables();

    Log.info("System Initialized\n");
}

void loop()
{
    taskScheduler.execute();
    stateMachine.update();
}

// Configuration and initialization functions
void configurePins()
{
    pinMode(PAS_PIN_IN, INPUT_PULLUP);
    pinMode(THROTTLE_PIN_OUT, OUTPUT);
    pinMode(LED_PIN_OUT, OUTPUT);
    pinMode(BRAKE_PIN_IN, INPUT_PULLUP);
    pinMode(BRAKE_PIN_OUT, OUTPUT);
    pinMode(UP_BUTTON_PIN, INPUT_PULLUP);
    pinMode(DOWN_BUTTON_PIN, INPUT_PULLUP);
}

void attachInterrupts()
{
    attachInterrupt(digitalPinToInterrupt(PAS_PIN_IN), pasPulseISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BRAKE_PIN_IN), brakeISR, CHANGE);
}

void initializeTasks()
{
    taskScheduler.init();
    taskScheduler.addTask(taskMonitorPAS);
    taskScheduler.addTask(taskUpdateLED);
    taskScheduler.addTask(taskMonitorButtons);
    taskMonitorPAS.enable();
    taskUpdateLED.enable();
    taskMonitorButtons.enable();
}

void initializeButtons()
{
    upButton.attach(UP_BUTTON_PIN);
    downButton.attach(DOWN_BUTTON_PIN);
    upButton.interval(25);
    downButton.interval(25);
}

void initializeVariables()
{
    speedChangeTimestamp = millis(); 
}

// State Action Functions
void onDetectEnter()
{
    Log.info("Entering DETECT state\n");
    setThrottle(PWM_MIN);
    setBrake(false);
}

void onDetectUpdate()
{
    int throttleValue = analogRead(THROTTLE_PIN_IN);
    if (!isThrottleVoltageValid(throttleValue))
    {
        stateMachine.transitionTo(erroredState);
        return;
    }
    throttleMedian.add(throttleValue);
    throttleValue = throttleMedian.getMedian();
    updatePWMOutput(throttleValue);
}

void onBrakeActiveEnter()
{
    Log.info("Entering BRAKE_ACTIVE state\n");
    setThrottle(PWM_MIN);
    setBrake(true);
}

void onErrorEnter()
{
    Log.error("Entering ERROR state\n");
    errorHasOccurred = true;
    setThrottle(PWM_MIN);
    resetBrake();
}

// Helper Functions
void setThrottle(int pwmValue)
{
    float proportionalThrottle = pwmValue / 1023.0;
    referencePWM = proportionalThrottle * speedLevels[currentSpeedLevel].maxPWMFactor * PWM_MAX;
    analogWrite(THROTTLE_PIN_OUT, constrain(referencePWM, PWM_MIN, PWM_MAX));
}

void setBrake(bool active)
{
    digitalWrite(BRAKE_PIN_OUT, active ? HIGH : LOW);
}

void resetBrake()
{
    digitalWrite(BRAKE_PIN_OUT, LOW);
}

bool isThrottleVoltageValid(int throttleValue)
{
    float voltage = throttleValue * (SYSTEM_VOLTAGE / 1023.0);
    return voltage >= THROTTLE_MIN_VOLTAGE && voltage <= THROTTLE_MAX_VOLTAGE;
}

// Task Functions
void monitorPASActivity()
{
    if (millis() - lastPasPulseTime > PAS_TIMEOUT_MS)
    {
        pulseCount = 0;
        Log.info("PAS timeout, pulse count reset\n");
    }
}

void updateLEDState()
{
    bool isActive = referencePWM > 0;
    digitalWrite(LED_PIN_OUT, isActive ? HIGH : LOW);
}

void monitorButtonActivity()
{
    upButton.update();
    downButton.update();

    // Check for button presses
    int delta = (upButton.fell() ? 1 : 0) - (downButton.fell() ? 1 : 0);
    if (delta == 0)
        return; // No button press

    lastSpeedLevel = currentSpeedLevel;
    currentSpeedLevel = constrain(currentSpeedLevel + delta, 0, 5);
    speedChangeTimestamp = millis();
    Log.info("Speed level changed: %d -> %d\n", lastSpeedLevel, currentSpeedLevel);
}

void updatePWMOutput(int rawThrottle)
{
    unsigned long now = millis();
    float factor; // Factor to multiply with max PWM value
    float lastFactor = speedLevels[lastSpeedLevel].maxPWMFactor;
    float currentFactor = speedLevels[currentSpeedLevel].maxPWMFactor;

    if (now - speedChangeTimestamp <= THROTTLE_SMOOTH_DURATION)
    {
        float elapsedTime = (now - speedChangeTimestamp) / (float)THROTTLE_SMOOTH_DURATION;
        factor = lastFactor + (currentFactor - lastFactor) * elapsedTime;
    }
    else
    {
        factor = currentFactor;
    }

    referencePWM = rawThrottle * factor * PWM_MAX;
    analogWrite(THROTTLE_PIN_OUT, constrain(referencePWM, PWM_MIN, PWM_MAX));
}

// Interrupt Service Routines
void pasPulseISR()
{
    if (errorHasOccurred)
    {
        return;
    }

    unsigned long now = millis();
    if (digitalRead(PAS_PIN_IN) == HIGH)
    {
        periodLow = now - isrOldTime;
        pulseCount++;
    }
    else
    {
        periodHigh = now - isrOldTime;
    }
    period = periodHigh + periodLow;
    isrOldTime = now;
    pedalingForward = periodHigh >= periodLow;
}

void brakeISR()
{
    if (errorHasOccurred)
    {
        return;
    }

    brakeActive = digitalRead(BRAKE_PIN_IN) == LOW;

    // Transition to BRAKE_ACTIVE state if brake is active
    stateMachine.transitionTo(brakeActive ? brakeActiveState : detectionState);
}
