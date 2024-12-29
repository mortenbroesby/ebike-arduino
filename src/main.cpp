#include <Deque.h>           // For smoothing PAS pulses
#include <RunningMedian.h>   // For throttle input filtering
#include <SoftwareSerial.h>  // For VESC UART communication

/**
 * CONFIGURATION - MODIFY HERE
 * ---------------------------
 */

// Enums for Configuration
enum VoltageMode { Voltage3V3, Voltage5V };   // Voltage modes: 3.3V or 5V

// User Configuration
#define SELECTED_OUTPUT_MODE Throttle     // Options: Throttle, VESC
#define SELECTED_VOLTAGE_MODE Voltage5V   // Options: Voltage3V3, Voltage5V
#define USE_PROPORTIONAL 1                // Enable proportional assistance

// Voltage-to-PWM Map
const float SYSTEM_VOLTAGE_MAP[] = {3.3, 5.0};          // System voltage for each mode
const float THROTTLE_MIN_VOLTAGE_MAP[] = {0.6, 0.8};    // Min throttle voltage for each mode
const float THROTTLE_MAX_VOLTAGE_MAP[] = {2.8, 4.4};    // Max throttle voltage for each mode

// Derived Voltage Configuration
const float SYSTEM_VOLTAGE = SYSTEM_VOLTAGE_MAP[SELECTED_VOLTAGE_MODE];               // Selected system voltage
const float THROTTLE_MIN_VOLTAGE = THROTTLE_MIN_VOLTAGE_MAP[SELECTED_VOLTAGE_MODE];   // Min throttle voltage
const float THROTTLE_MAX_VOLTAGE = THROTTLE_MAX_VOLTAGE_MAP[SELECTED_VOLTAGE_MODE];   // Max throttle voltage
const int PWM_MIN = int((THROTTLE_MIN_VOLTAGE / SYSTEM_VOLTAGE) * 255);               // Minimum PWM value
const int PWM_MAX = int((THROTTLE_MAX_VOLTAGE / SYSTEM_VOLTAGE) * 255);               // Maximum PWM value

// PAS Configuration
const int MAGNET_COUNT = 12;                  // Number of magnets on PAS
const int START_PULSES = 2;                   // Minimum pulses to start assistance
const unsigned long PAS_TIMEOUT_MS = 500;     // Timeout for PAS inactivity
const unsigned long PAS_DEBOUNCE_TIME = 2;    // Debounce time in ms for PAS pulses

// Proportional Assistance
const long MS_PER_MINUTE = 60000;
const long MAGNET_TO_RPM_FACTOR = MS_PER_MINUTE / MAGNET_COUNT;
const long MS_SLOW = MS_PER_MINUTE / 25 / MAGNET_COUNT; // Slow RPM period for proportional assistance
const long MS_FAST = MS_PER_MINUTE / 65 / MAGNET_COUNT; // Fast RPM period for proportional assistance

/**
 * END CONFIGURATION
 * -----------------
 */

// Pin Definitions
const int PAS_PIN = 2;          // PAS input (interrupt pin)
const int THROTTLE_PIN = A0;    // Analog pin for throttle input
const int THROTTLE_OUT = 9;     // PWM output for throttle signal
const int LED_PIN = 13;         // Built-in LED for status indication
const int BRAKE_PIN = 3;        // Digital pin for brake signal

// Global Variables
volatile unsigned long lastPasPulseTime = 0; ///< Last PAS pulse time
volatile unsigned long brakeStartTime = 0;      // Timestamp when the brake was activated
volatile unsigned long isrOldTime = 0;       ///< Previous interrupt timestamp
volatile unsigned int periodHigh = 0, periodLow = 0, period = 0;
volatile unsigned int pulseCount = 0;        ///< PAS pulse count

bool pedalingForward = false;                ///< Tracks pedaling direction
bool brakeActive = false;              // Tracks whether the brake is active
bool errorState = false;                     ///< Tracks system error state

Deque<int> pwmHistory(10);                   ///< Smoothing for PAS periods
RunningMedian throttleMedian(10);            ///< Median filter for throttle input

/**
 * @brief Arduino setup function.
 *        Initializes pins, interrupts, and idle state.
 */
void setup() {
    Serial.begin(115200);

    pinMode(PAS_PIN, INPUT_PULLUP);    ///< PAS pin with pull-up resistor
    pinMode(THROTTLE_OUT, OUTPUT);    ///< PWM output for motor control
    pinMode(LED_PIN, OUTPUT);         ///< LED for status indication
    pinMode(BRAKE_PIN, INPUT_PULLUP); ///< Brake pin with pull-up resistor
    attachInterrupt(digitalPinToInterrupt(PAS_PIN), pasPulseISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BRAKE_PIN), brakeISR, CHANGE);

    setThrottleIdle();
    Serial.println("System Initialized");
}

/**
 * @brief Main loop function.
 *        Handles throttle and PAS logic, and updates the LED state.
 */
void loop() {
    if (errorState || brakeActive) {
        setThrottleIdle();
        return;
    }

    // Read and filter throttle value
    int throttleValue = analogRead(THROTTLE_PIN);
    throttleMedian.add(throttleValue);
    throttleValue = throttleMedian.getMedian();

    // Enter error state if throttle voltage is invalid
    if (!isThrottleVoltageValid(throttleValue)) {
        errorState = true;
        setThrottleIdle();
        return;
    }

    monitorPASActivity();

    // Determine PWM output based on throttle or PAS
    int pwmOutput;
    if (throttleValue > 0) {
        pwmOutput = calculateThrottlePWM(throttleValue);
    } else {
        pwmOutput = calculatePASPWM();
    }

    analogWrite(THROTTLE_OUT, pwmOutput);
    updateLEDState(pwmOutput > PWM_MIN);
}

/**
 * @brief Sets the throttle output to idle.
 */
void setThrottleIdle() {
    analogWrite(THROTTLE_OUT, PWM_MIN);
    updateLEDState(false);
}

/**
 * @brief Monitors PAS activity and stops assistance if no pulses are detected.
 */
void monitorPASActivity() {
    if (millis() - lastPasPulseTime > PAS_TIMEOUT_MS) {
        setThrottleIdle();
    }
}

/**
 * @brief Calculates PWM output based on throttle input.
 * @param throttleValue Raw analog value from throttle (0-1023).
 * @return Mapped PWM value.
 */
int calculateThrottlePWM(int throttleValue) {
    return map(throttleValue, 0, 1023, PWM_MIN, PWM_MAX);
}

/**
 * @brief Calculates PWM output based on PAS input.
 * @return Calculated PWM value based on RPM or maximum PWM.
 */
int calculatePASPWM() {
    if (pulseCount >= START_PULSES && pedalingForward) {
        if (USE_PROPORTIONAL) {
            int pwmOutput = map(period, MS_SLOW, MS_FAST, PWM_MIN, PWM_MAX);
            return constrain(pwmOutput, PWM_MIN, PWM_MAX);
        } else {
            return PWM_MAX;
        }
    }
    return PWM_MIN;
}

/**
 * @brief Validates throttle voltage range.
 * @param throttleValue Raw analog value from throttle (0-1023).
 * @return True if voltage is valid, false otherwise.
 */
bool isThrottleVoltageValid(int throttleValue) {
    float voltage = throttleValue * (SYSTEM_VOLTAGE / 1023.0);
    return voltage >= THROTTLE_MIN_VOLTAGE && voltage <= THROTTLE_MAX_VOLTAGE;
}

/**
 * @brief Updates the LED state to reflect motor activity.
 * @param state True for active (ON), false for idle (OFF).
 */
void updateLEDState(bool state) {
    digitalWrite(LED_PIN, state ? HIGH : LOW);
}

/**
 * @brief Interrupt Service Routine for PAS pulses.
 *        Tracks pedaling direction and measures pulse periods.
 */
void pasPulseISR() {
    unsigned long now = millis();
    if (digitalRead(PAS_PIN) == HIGH) {
        periodLow = now - isrOldTime;
        pulseCount++;
    } else {
        periodHigh = now - isrOldTime;
    }
    period = periodHigh + periodLow;
    isrOldTime = now;

    pedalingForward = periodHigh >= periodLow;
}

/**
 * @brief Interrupt Service Routine for Brake Signal.
 *        Sets the brake state based on the brake pin signal.
 */
void brakeISR() {
    brakeActive = digitalRead(BRAKE_PIN) == LOW;
    if (brakeActive) {
        brakeStartTime = millis();
    }
}
