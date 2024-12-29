#include <RunningMedian.h>
#include <SoftwareSerial.h>

// Enum for voltage modes
enum VoltageMode
{
    Voltage3V3,
    Voltage5V
};

// Configuration settings
#define SELECTED_VOLTAGE_MODE Voltage5V // Choose system voltage mode
#define USE_PROPORTIONAL 1              // Enable proportional pedal assistance

// Voltage mapping for different modes
const float SYSTEM_VOLTAGE_MAP[] = {3.3, 5.0};
const float THROTTLE_MIN_VOLTAGE_MAP[] = {0.6, 0.8};
const float THROTTLE_MAX_VOLTAGE_MAP[] = {2.8, 4.4};

// Derived voltage and PWM values based on selected mode
const float SYSTEM_VOLTAGE = SYSTEM_VOLTAGE_MAP[SELECTED_VOLTAGE_MODE];
const float THROTTLE_MIN_VOLTAGE = THROTTLE_MIN_VOLTAGE_MAP[SELECTED_VOLTAGE_MODE];
const float THROTTLE_MAX_VOLTAGE = THROTTLE_MAX_VOLTAGE_MAP[SELECTED_VOLTAGE_MODE];
const int PWM_MIN = int((THROTTLE_MIN_VOLTAGE / SYSTEM_VOLTAGE) * 255);
const int PWM_MAX = int((THROTTLE_MAX_VOLTAGE / SYSTEM_VOLTAGE) * 255);

// Pin configuration
const int THROTTLE_PIN_IN = A0; // Throttle input (analog)
const int THROTTLE_PIN_OUT = 9; // Throttle output (PWM)
const int PAS_PIN_IN = 2;       // Pedal Assist Sensor (PAS) input
const int BRAKE_PIN_IN = 3;     // Brake signal input
const int BRAKE_PIN_OUT = 10;   // Brake signal output
const int LED_PIN_OUT = 13;     // Built-in LED for status indication

// PAS configuration
const int MAGNET_COUNT = 12;               // Number of PAS magnets
const int START_PULSES = 2;                // Minimum pulses to activate assistance
const unsigned long PAS_TIMEOUT_MS = 1500; // Timeout for no PAS activity
const unsigned long PAS_DEBOUNCE_TIME = 2; // Debounce time for PAS pulses

// Timing constants
const long MS_PER_MINUTE = 60000;
const long MAGNET_TO_RPM_FACTOR = MS_PER_MINUTE / MAGNET_COUNT;
const long MS_SLOW = MS_PER_MINUTE / 25 / MAGNET_COUNT; // Slow RPM threshold
const long MS_FAST = MS_PER_MINUTE / 65 / MAGNET_COUNT; // Fast RPM threshold

// Global variables
volatile unsigned long lastPasPulseTime = 0; // Time of last PAS pulse
volatile unsigned long brakeStartTime = 0;   // Time when brake was activated
volatile unsigned long isrOldTime = 0;       // Last ISR time for PAS
volatile unsigned int periodHigh = 0, periodLow = 0, period = 0;
volatile unsigned int pulseCount = 0; // PAS pulse count

bool pedalingForward = false; // True if pedaling forward
bool brakeActive = false;     // True if brake is engaged
bool errorState = false;      // True if system is in error state

RunningMedian throttleMedian(3); // Median filter for throttle input

/**
 * @brief Arduino setup function.
 *        Initializes pins, interrupts, and system state.
 */
void setup()
{
    Serial.begin(115200);
    pinMode(PAS_PIN_IN, INPUT_PULLUP);
    pinMode(THROTTLE_PIN_OUT, OUTPUT);
    pinMode(LED_PIN_OUT, OUTPUT);
    pinMode(BRAKE_PIN_IN, INPUT_PULLUP);
    pinMode(BRAKE_PIN_OUT, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(PAS_PIN_IN), pasPulseISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BRAKE_PIN_IN), brakeISR, CHANGE);
    setThrottleIdle();
    Serial.println("System Initialized");
}

/**
 * @brief Main loop function.
 *        Handles brake, throttle, and PAS logic.
 */
void loop()
{
    if (errorState)
    {
        setThrottleIdle();
        return;
    }

    if (brakeActive)
    {
        handleBrakeOutput();
        return;
    }

    handleThrottleOutput();
}

/**
 * @brief Processes throttle input and generates appropriate output.
 */
void handleThrottleOutput()
{
    int throttleValue = analogRead(THROTTLE_PIN_IN);

    if (!isThrottleVoltageValid(throttleValue))
    {
        errorState = true;
        setThrottleIdle();
        return;
    }

    throttleMedian.add(throttleValue);
    throttleValue = throttleMedian.getMedian();

    monitorPASActivity();
    int pwmOutput = getPWMOutput(throttleValue);

    analogWrite(THROTTLE_PIN_OUT, pwmOutput);
    updateLEDState(pwmOutput > PWM_MIN);
}

/**
 * @brief Calculates the PWM output value based on throttle or PAS.
 * @param throttleValue Raw throttle input (0-1023).
 * @return PWM output value.
 */
int getPWMOutput(int throttleValue)
{
    return (throttleValue > 0) ? calculateThrottlePWM(throttleValue) : calculatePASPWM();
}

/**
 * @brief Sets the throttle to idle state.
 */
void setThrottleIdle()
{
    analogWrite(THROTTLE_PIN_OUT, PWM_MIN);
    updateLEDState(false);
}

/**
 * @brief Monitors PAS activity and resets pulse count on timeout.
 */
void monitorPASActivity()
{
    if (millis() - lastPasPulseTime > PAS_TIMEOUT_MS)
    {
        pulseCount = 0;
    }
}

/**
 * @brief Maps throttle input to PWM output.
 * @param throttleValue Raw throttle input (0-1023).
 * @return Mapped PWM value.
 */
int calculateThrottlePWM(int throttleValue)
{
    return map(throttleValue, 0, 1023, PWM_MIN, PWM_MAX);
}

/**
 * @brief Calculates PWM output based on PAS input.
 * @return PWM value based on RPM or max PWM.
 */
int calculatePASPWM()
{
    if (pulseCount >= START_PULSES && pedalingForward)
    {
        int pwmOutput = (USE_PROPORTIONAL) ? map(period, MS_SLOW, MS_FAST, PWM_MIN, PWM_MAX) : PWM_MAX;
        return constrain(pwmOutput, PWM_MIN, PWM_MAX);
    }
    return PWM_MIN;
}

/**
 * @brief Validates throttle voltage range.
 * @param throttleValue Raw throttle input (0-1023).
 * @return True if voltage is within valid range, false otherwise.
 */
bool isThrottleVoltageValid(int throttleValue)
{
    float voltage = throttleValue * (SYSTEM_VOLTAGE / 1023.0);
    return voltage >= THROTTLE_MIN_VOLTAGE && voltage <= THROTTLE_MAX_VOLTAGE;
}

/**
 * @brief Updates the LED state based on motor activity.
 * @param state True if active, false otherwise.
 */
void updateLEDState(bool state)
{
    digitalWrite(LED_PIN_OUT, state ? HIGH : LOW);
}

/**
 * @brief ISR for PAS pulses.
 *        Tracks direction and measures pulse periods.
 */
void pasPulseISR()
{
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

/**
 * @brief ISR for brake signal.
 *        Updates brake state based on input signal.
 */
void brakeISR()
{
    brakeActive = digitalRead(BRAKE_PIN_IN) == LOW;
    if (brakeActive)
    {
        brakeStartTime = millis();
    }
}

/**
 * @brief Handles brake output and disables throttle when active.
 */
void handleBrakeOutput()
{
    digitalWrite(BRAKE_PIN_OUT, brakeActive ? HIGH : LOW);
    setThrottleIdle();
}
