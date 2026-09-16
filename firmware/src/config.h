#pragma once

// ---------------------------------------------------------------------------
// GPIO map -- ESP32-S3-DevKitC-1, WROOM-1 N16R8 (16MB flash, 8MB octal PSRAM)
//
// The S3's usable GPIOs are 0-21 and 26-48. GPIO 22-25 DO NOT EXIST on this
// chip -- 22 + 23 = the 45 pins the datasheet claims. Off-limits here:
//   22-25   not bonded out         -- silently do nothing if you assign them
//   26-32   SPI flash              -- instant crash
//   33-37   OCTAL PSRAM            -- the R8 in N16R8. Reserved whether or not
//                                     PSRAM is enabled in software; they are
//                                     physically bonded inside the module.
//   0,3,45,46  strapping           -- boot mode, JTAG select, VDD_SPI voltage
//   19,20   USB D-/D+              -- using these kills native USB
//   43,44   UART0 TX/RX            -- serial monitor
//   48      onboard RGB LED        -- on the DevKitC-1
//
// Unlike the original ESP32, the S3 has no boot-time PWM burst pins, so the
// old 5/14/15 prohibition does not apply here. Every pin below is a plain,
// unreserved GPIO -- verified with scripts/validate_pinmap.py for esp32s3.
// ---------------------------------------------------------------------------

#define PIN_RELAY     5   // -> relay module IN -> contactor coil
#define PIN_FLOW      4   // <- YF-DN25 hall pulse (interrupt)
#define PIN_US_TRIG   6   // -> AJ-SR04M TRIG
#define PIN_US_ECHO   7   // <- AJ-SR04M ECHO (via 20k/10k divider, 5V -> 3.3V)
#define PIN_FLOAT    15   // <- high-level float switch (the real interlock is
                          //    the 5V float loop breaking the contactor coil;
                          //    this pin is only the telemetry copy)
#define PIN_BUTTON   16   // <- momentary button to GND
#define PIN_LED      17   // -> status LED (+220R to GND)

// GPIO 8/9 left free for a future I2C OLED (the S3's conventional SDA/SCL).

// Most import relay boards are ACTIVE-LOW. An external 10k pull-up from
// PIN_RELAY to 3V3 holds the relay OFF through the ~200ms boot window while
// the GPIO is still floating. Do not omit that resistor.
#define RELAY_ON     LOW
#define RELAY_OFF    HIGH

// Float switch: NC contact, opens when the tank is full.
// Wired switch-to-GND with INPUT_PULLUP, so LOW = not full, HIGH = full.
#define FLOAT_IS_FULL(level)  ((level) == HIGH)

// ---------------------------------------------------------------------------
// Defaults -- every one of these is overridable at runtime over MQTT and
// persisted to NVS. Nothing here needs a reflash to change.
// ---------------------------------------------------------------------------

#define DEF_POLL_MIN         15     // minutes between test pulses
#define DEF_TEST_S            8     // test pulse length; a 1HP pump primes in 3-4s
#define DEF_COOLDOWN_H        6     // rest after a real fill -- supply won't repeat
#define DEF_FLOW_THRESH_LPM   5.0f  // below this we call it "no water"
#define DEF_LOWFLOW_WIN_S    60     // municipal supply sputters; don't cut on a dip
#define DEF_MAX_RUNTIME_MIN 360     // 6h. Tank is ~8h of pumping when bone dry.
#define DEF_PULSES_PER_L    135.0f  // YF-DN25 nominal. CALIBRATE THIS (see README).
#define DEF_TANK_EMPTY_CM   300     // sensor face -> tank floor
#define DEF_TANK_FULL_CM     35     // sensor face -> water at full (>=30, blind zone)
#define DEF_TANK_FULL_L   28317UL   // 10x10x10 ft
#define DEF_MIN_SESSION_L   200     // under this, the stop was a sputter not a fill

// Sampling cadence
#define US_PERIOD_PUMPING_MS   5000UL
#define US_PERIOD_IDLE_MS     60000UL
#define US_FAIL_ALERT_MS     600000UL  // 10 min of no valid echo -> sensor fault

// Ultrasonic physical limits (AJ-SR04M)
#define US_MIN_CM   20
#define US_MAX_CM  600

// MQTT
#define MQTT_BASE   "wateriot/motor2"
#define MQTT_HA_PREFIX "homeassistant"
#define DEVICE_ID   "wateriot_motor2"
#define DEVICE_NAME "Water Tank Motor 2"

#define TZ_INFO "IST-5:30"
