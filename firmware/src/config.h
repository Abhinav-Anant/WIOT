#pragma once

// ---------------------------------------------------------------------------
// GPIO map -- ESP32-WROOM-32 (30-pin DevKit V1)
//
// Deliberately avoided:
//   6-11   SPI flash          -- instant crash
//   12     MTDI strap         -- sets flash voltage to 1.8V, bricks the module
//   5,14,15  boot PWM burst   -- these emit pulses during boot; a pulse on the
//                               relay pin would kick a 1HP contactor at power-up
//   1,3    UART0              -- USB serial
//   34-39  input-only, no internal pull-ups
// ---------------------------------------------------------------------------

#define PIN_RELAY    23   // -> relay module IN -> contactor coil
#define PIN_FLOW     27   // <- YF-DN25 hall pulse (interrupt)
#define PIN_US_TRIG  26   // -> AJ-SR04M TRIG
#define PIN_US_ECHO  25   // <- AJ-SR04M ECHO (via 20k/10k divider, 5V -> 3.3V)
#define PIN_FLOAT    33   // <- high-level float switch (also wired in series
                          //    with the contactor coil as a hardware interlock)
#define PIN_BUTTON   18   // <- momentary button to GND
#define PIN_LED      19   // -> status LED (+220R to GND)

// GPIO 21/22 left free for a future I2C OLED.

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
