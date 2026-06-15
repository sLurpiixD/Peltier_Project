/**
 * TEC Test Bench — MASTER FIRMWARE (PID Control + Multi-Sensor Data)
 * Target   : ESP32-S3-N16R8 (DevKitC-1)
 * Protocol : RX <- Target RPM (e.g., "2000\n")
 *            TX -> Current_A, Fan_RPM, PWM_Duty, Tamb_C, Thot_C, Tcold_C
 *
 * Requires arduino-esp32 v2.x (espressif32 ~6.5.0)
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include "max6675.h"
#include "INA226.h"
#include <OneWire.h>
#include <DallasTemperature.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================
#define MAX31855_CLK   7
#define MAX31855_CS    11
#define MAX31855_MISO  14

#define MAX6675_CLK    12
#define MAX6675_CS     10
#define MAX6675_MISO   13

#define I2C_SDA        8
#define I2C_SCL        9
#define ONE_WIRE_BUS   4
#define FAN_PWM_PIN    5
#define FAN_TACH_PIN   6

// ============================================================
// CONSTANTS
// ============================================================
static constexpr float    SHUNT_OHMS     = 0.01f;
static constexpr uint8_t  TACH_PPR       = 2;
static constexpr float    FAULT_SENTINEL = -999.0f;

// Fan PWM Settings
static constexpr uint32_t PWM_FREQ       = 25000;
static constexpr uint8_t  PWM_RESOLUTION = 8;
static constexpr float    MAX_SAFE_PWM   = 245.0f;
static constexpr float    MIN_SPIN_PWM   = 10.0f;
static constexpr float    MIN_RPM        = 1400.0f;

// ============================================================
// CONTROL VARIABLES
// ============================================================
float targetRPM   = MIN_RPM;
float measuredRPM = 0.0f;
float currentPWM  = MIN_SPIN_PWM;
float prevError   = 0.0f;

// PID Tuning
static constexpr float Kp = 0.030f;
static constexpr float Ki = 0.005f;

// Timing
unsigned long lastControlMillis   = 0;
unsigned long lastTelemetryMillis = 0;
unsigned long lastSensorMillis    = 0;

static constexpr uint32_t TELEMETRY_MS = 1000;
static constexpr uint32_t SENSOR_MS    = 250;

// Oversampling Accumulators
float   sum_Thot   = 0.0f;
float   sum_Tcold  = 0.0f;
uint8_t valid_hot  = 0;
uint8_t valid_cold = 0;

// ============================================================
// PERIPHERALS
// ============================================================
Adafruit_MAX31855 max31855(MAX31855_CLK, MAX31855_CS, MAX31855_MISO);
MAX6675           max6675 (MAX6675_CLK,  MAX6675_CS,  MAX6675_MISO);
INA226            ina(0x40);
OneWire           oneWire(ONE_WIRE_BUS);
DallasTemperature ambientSensor(&oneWire);

bool ina_ok = false;

// ============================================================
// TACHOMETER — MEDIAN FILTER (ISR-driven)
// ============================================================
#define FILTER_SIZE 5
volatile uint32_t periodHistory[FILTER_SIZE] = {0, 0, 0, 0, 0};
volatile uint8_t  filterIdx    = 0;
volatile uint32_t lastPulse_us = 0;

void IRAM_ATTR tachISR() {
    uint32_t now     = micros();
    uint32_t elapsed = now - lastPulse_us;

    // 3ms debounce — limits max readable to ~10,000 RPM, filters noise
    if (elapsed > 3000) {
        periodHistory[filterIdx] = elapsed;
        filterIdx = (filterIdx + 1) % FILTER_SIZE;
        lastPulse_us = now;
    }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL);

    ina_ok = ina.begin();

    ambientSensor.begin();
    ambientSensor.setWaitForConversion(false);
    ambientSensor.requestTemperatures();

    // arduino-esp32 v2.x channel-based LEDC API
    ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(FAN_PWM_PIN, 0);
    ledcWrite(0, (uint32_t)currentPWM);

    pinMode(FAN_TACH_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), tachISR, FALLING);

    delay(500);
}

// ============================================================
// LOOP
// ============================================================
void loop() {
    unsigned long now = millis();

    // 1. RECEIVE TARGET RPM
    if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() > 0 && isDigit(cmd.charAt(0))) {
            float newTarget = cmd.toFloat();
            if (newTarget < MIN_RPM)  newTarget = MIN_RPM;
            if (newTarget > 4000.0f)  newTarget = 4000.0f;
            targetRPM = newTarget;
        }
    }

    // 2. CONTROL LOOP — every 100 ms
    if (now - lastControlMillis >= 100) {
        lastControlMillis = now;

        // A. Copy tachometer history atomically
        uint32_t historyCopy[FILTER_SIZE];
        uint32_t age;

        noInterrupts();
        for (int i = 0; i < FILTER_SIZE; i++) {
            historyCopy[i] = periodHistory[i];
        }
        age = micros() - lastPulse_us;
        interrupts();

        // B. Compute median-filtered RPM
        if (age > 500000) {
            // No pulse for 500 ms → fan has stopped
            measuredRPM = 0;
        } else {
            // Bubble-sort 5-element history, pick median at index 2
            for (int i = 0; i < FILTER_SIZE - 1; i++) {
                for (int j = 0; j < FILTER_SIZE - i - 1; j++) {
                    if (historyCopy[j] > historyCopy[j + 1]) {
                        uint32_t t         = historyCopy[j];
                        historyCopy[j]     = historyCopy[j + 1];
                        historyCopy[j + 1] = t;
                    }
                }
            }
            uint32_t medianPeriod = historyCopy[2];
            if (medianPeriod > 0) {
                float rawRPM = 60000000.0f / (medianPeriod * TACH_PPR);
                if (rawRPM < 6000.0f) {
                    measuredRPM = 0.2f * rawRPM + 0.8f * measuredRPM;
                }
            }
        }

        // C. PI controller
        float error = targetRPM - measuredRPM;
        currentPWM += (Kp * (error - prevError)) + (Ki * error);
        prevError = error;

        currentPWM = constrain(currentPWM, MIN_SPIN_PWM, MAX_SAFE_PWM);
        ledcWrite(0, (uint32_t)currentPWM);
    }

    // 3. SENSOR OVERSAMPLING — every 250 ms
    if (now - lastSensorMillis >= SENSOR_MS) {
        lastSensorMillis = now;

        float raw_t_hot  = max6675.readCelsius();
        float raw_t_cold = max31855.readCelsius();

        if (!isnan(raw_t_hot) && raw_t_hot > 0.0f && raw_t_hot <= 400.0f) {
            sum_Thot += raw_t_hot;
            valid_hot++;
        }
        if (!isnan(raw_t_cold)) {
            sum_Tcold += raw_t_cold;
            valid_cold++;
        }
    }

    // 4. TELEMETRY TRANSMIT — every 1000 ms
    if (now - lastTelemetryMillis >= TELEMETRY_MS) {
        lastTelemetryMillis = now;

        float t_amb = ambientSensor.getTempCByIndex(0);
        ambientSensor.requestTemperatures();
        if (t_amb == DEVICE_DISCONNECTED_C) t_amb = FAULT_SENTINEL;

        float t_hot  = (valid_hot  == 0) ? FAULT_SENTINEL : (sum_Thot  / valid_hot);
        float t_cold = (valid_cold == 0) ? FAULT_SENTINEL : (sum_Tcold / valid_cold);

        sum_Thot   = 0.0f;
        sum_Tcold  = 0.0f;
        valid_hot  = 0;
        valid_cold = 0;

        float current_A = ina_ok ? (ina.getShuntVoltage() / SHUNT_OHMS) : FAULT_SENTINEL;

        // CSV: current_A, Fan_RPM, PWM_Duty, Tamb_C, Thot_C, Tcold_C
        Serial.print(current_A,   3); Serial.print(',');
        Serial.print(measuredRPM, 0); Serial.print(',');
        Serial.print(currentPWM,  0); Serial.print(',');
        Serial.print(t_amb,       3); Serial.print(',');
        Serial.print(t_hot,       3); Serial.print(',');
        Serial.println(t_cold,    3);
    }
}