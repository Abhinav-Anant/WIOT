# WIOT — Municipal Water Auto-Fill (Motor 2)

ESP32 controller that detects when the municipal water supply arrives and fills the
ground tank without anyone having to wake up at 6am.

The municipal line here is **dead until the pump pulls on it** — there is no standing
pressure to sense. So arrival is detected by briefly running the pump and watching a
flow sensor. If water is there, the pump keeps running until the supply dies or the
tank fills.

All control logic runs locally on the ESP32. WiFi and MQTT are telemetry and remote
override only — **if the router is down at 3am, the pump still works.**

---

## Contents

- [How it works](#how-it-works)
- [Bill of materials](#bill-of-materials)
- [Safety — read this before touching the panel](#safety--read-this-before-touching-the-panel)
- [Wiring: mains side](#wiring-mains-side)
- [Wiring: low-voltage side](#wiring-low-voltage-side)
- [Sensor placement](#sensor-placement)
- [Build and flash](#build-and-flash)
- [Commissioning checklist](#commissioning-checklist)
- [Calibration](#calibration)
- [Mosquitto + Home Assistant on the RB5009](#mosquitto--home-assistant-on-the-rb5009)
- [Tunable settings](#tunable-settings)
- [Troubleshooting](#troubleshooting)

---

## How it works

```
IDLE ──poll timer (default 15 min)──► TESTING
                                        │ relay ON, 8 s
                                        ├─ flow ≥ threshold ──► PUMPING
                                        └─ no flow ──────────► relay OFF → IDLE
                                                                (dry start logged)

PUMPING ─┬─ flow < threshold for 60 s straight ──► "supply ended"  ─┐
         ├─ float HIGH  or  level ≥ 99%         ──► "tank full"    ─┤
         └─ runtime > 6 h                       ──► "max runtime"  ─┤
                                                                    ▼
                                                        delivered ≥ 200 L?
                                                         ├─ yes → COOLDOWN 6 h
                                                         └─ no  → IDLE (it was a sputter)
```

Three details that are load-bearing:

**The 4-second priming grace.** A 1HP suction pump has to pull the water column up
before any flow appears. Judging the line before 4s would report every start as dry.

**The 60-second low-flow window.** Municipal supply sputters as it arrives and as it
dies. Cutting the pump on the first dip would chatter the contactor to death.

**The sputter rule.** A 6-hour cooldown after a 20-litre session would make you miss
the real supply half an hour later. Only a session that delivered ≥ 200 L earns the
long rest; anything less returns to normal polling.

### Why the tank never fills in one session

Your sump is 10×10×10 ft = **28,317 L**, 305 cm deep, so **~93 L per cm of depth**.
At a 1HP booster's ~60 L/min, empty→full is **7.9 hours**. Municipal supply that runs
1–2 hours a day will never fill it in one go — so in practice the stop reason will
almost always be *"supply ended"*, and *"tank full"* will be rare. That is normal.

---

## Bill of materials

| # | Item | Suggested part | ~₹ |
|---|---|---|---|
| 1 | MCU | ESP32-WROOM-32 DevKit V1, 30-pin | 350 |
| 2 | Flow sensor | **YF-DN25** — G1" brass, 2–100 L/min | 700 |
| 3 | Level sensor | **AJ-SR04M** waterproof ultrasonic (separate probe on a cable) | 400 |
| 4 | Overflow float | Vertical float switch, NC, ½" side-mount (any voltage — it runs on 5V here) | 200 |
| 5 | Interposing relay | **SRD-05VDC-SL-C** sugar-cube relay + 8-pin base | 60 |
| 6 | Flyback diode | 1N4007 | 5 |
| 7 | Control relay | 1-channel 5V opto-isolated relay module | 80 |
| 8 | Motor switching | **AC contactor, 20A, 230V AC coil** (L&T MNX 9 / Schneider LC1D09 / Havells) | 900 |
| 9 | Protection | **30mA RCBO, 10A** (see safety note — do not skip this) | 1200 |
| 10 | Power | Hi-Link HLK-5M05 (230V→5V 1A) or a spare 5V 2A phone charger | 250 |
| 11 | Enclosure | IP65 ABS box, ~200×150×75mm, + ½" cable glands | 500 |
| 12 | Misc | 10k ×2, 20k ×1, 220R ×1 resistors; terminal blocks; 2.5mm² wire; ferrules; momentary button; LED | 350 |
| | | **Total** | **~₹5,000** |

**Why YF-DN25 and not the YF-S201 everyone uses.** The S201 is a ½" plastic body rated
to 30 L/min. A 1HP booster pushes 50–70 L/min — you would be throttling the pump
through its own sensor and the reading would sit pinned at the top of its range. The
DN25 matches a 1" outlet.

**Why a contactor and not just a relay module.** A 1HP motor draws ~6–7 A running but
~30 A on inrush. That inrush welds the contacts of a cheap 10 A relay module shut —
and a welded relay on a water pump means it runs until someone notices the flood. The
ESP32's relay only ever switches the contactor's **coil** (~10 VA), which it handles
comfortably. The contactor does the real work.

---

## Safety — read this before touching the panel

You are working on a 230 V motor circuit. This is the part where being careless hurts.

1. **Kill the MCB, then verify dead.** Test at the actual motor terminals with a
   contact tester or multimeter — not at the switch, and not by trusting the label.
   Circuits get mislabelled.
2. **Tape the breaker off and tell the household.** Someone flipping it back on while
   your hands are in the box is the realistic accident.
3. **Fit a 30 mA RCBO.** A pump sitting in a wet sump feeding a metal pipe is precisely
   the fault scenario an RCD exists for. If you install nothing else from this list,
   install this.
4. **Earth everything.** The motor body, the contactor's DIN rail, and the enclosure if
   it has any metal. 2.5 mm² earth wire.
5. **Separate the voltages inside the box.** 230 V on one side, 5 V on the other, their
   own cable glands, no shared terminal strip. Cable-tie them apart.
6. **No mains in the water tank.** The float switch in this design runs on 5 V for
   exactly this reason — see below.
7. **Wire sizing:** 2.5 mm² for the motor run, 1.5 mm² minimum for the contactor coil
   circuit.

If any step feels beyond you, stop and call an electrician for the mains side. The
low-voltage side is all yours and completely safe to build.

### About the float switch

The standard Indian water-level controller puts the float switch directly in the 230 V
contactor coil circuit. It works, and sealed 250 VAC-rated floats exist for it. This
design **does not do that**, because it means mains voltage on a component submerged in
your drinking water — if the float body ever cracks, the tank is live.

Instead the float sits in a **5 V loop** driving a small interposing relay, and *that
relay's contact* breaks the contactor coil. The interlock stays entirely independent of
the ESP32's software, but the tank only ever sees 5 V.

It also fails safe: if the 5 V supply dies, the interposing relay drops out and the
contactor cannot energise at all.

---

## Wiring: mains side

```
                            ┌─────────────────────────────────┐
  230V LIVE ──[ 10A RCBO ]──┤                                 │
                            │                            ┌────┴────┐
                            │                            │ CONTACT │
                            ├──────────────────────────► │   OR    ├──► MOTOR 2 LIVE
                            │                     L1/T1  │  20A    │
                            │                            └────┬────┘
                            │                                 │ coil A1
                            │   ┌──────────────┐    ┌─────────┴────────┐
                            └──►│ ESP32 RELAY  ├────┤ INTERPOSING RLY  │
                                │  (NO contact)│    │   (NO contact)   │
                                └──────┬───────┘    └─────────┬────────┘
                                       │                      │
                                  ┌────┴─────┐                │
                                  │ MANUAL   │                │
                                  │ SWITCH   │  (parallel)    │
                                  └────┬─────┘                │
                                       └──────────────────────┘
                                                              │
  230V NEUTRAL ───────────────────────┬───────────────────────┘ coil A2
                                      └──────────────────────────► MOTOR 2 NEUTRAL

  EARTH ──────────────────────────────────────► MOTOR BODY + DIN RAIL
```

Read the coil path as a chain — **every link must be closed for the motor to run**:

```
LIVE → [ESP32 relay NO  OR  manual switch] → [interposing relay NO] → coil A1
                                                        ▲
                                              closed only while the
                                              5V float loop says
                                              "tank is not full"
```

What that buys you:

| Failure | What happens |
|---|---|
| ESP32 crashes with the relay latched on | Float opens → interposing relay drops → coil breaks → pump stops |
| Firmware bug commands the pump forever | Same. Software cannot overflow the tank. |
| ESP32 unplugged entirely | Manual switch still runs the pump, exactly as it does today |
| 5 V supply dies | Interposing relay drops → pump cannot run (fail safe) |
| WiFi / MQTT / Home Assistant down | Nothing changes. All logic is local. |

The manual switch sits **in parallel with the ESP32's relay contact**, so your existing
wall switch keeps working as a bypass — including when the ESP32 is powered off. It
still respects the float interlock, which is what you want.

---

## Wiring: low-voltage side

```
  5V SMPS ──┬── ESP32 VIN            (do NOT also power via USB at the same time)
            ├── AJ-SR04M  VCC
            ├── YF-DN25   VCC (red)
            ├── Relay module VCC
            └── FLOAT LOOP:  5V ──[FLOAT NC]──┬── SRD-05VDC coil ── GND
                                              └── 1N4007 across the coil
                                                  (cathode/stripe to +5V)

  GND ──── common star point: ESP32 GND, sensors, relay module, SMPS −
```

| ESP32 GPIO | Connects to | Notes |
|---|---|---|
| **23** | Relay module `IN` | **Add a 10 kΩ pull-up from GPIO23 to 3V3.** Not optional — see below. |
| **27** | YF-DN25 signal (yellow) | 10 kΩ pull-up to 3V3 |
| **26** | AJ-SR04M `TRIG` | — |
| **25** | AJ-SR04M `ECHO` | **Via 20k/10k divider** — the module echoes 5 V, the ESP32 is 3.3 V only |
| **33** | Float switch sense | Tap the junction of the float and the interposing relay coil, through a 10 kΩ series resistor. Telemetry only. |
| **18** | Momentary button → GND | Short press = check now / stop. Long press (3 s) = clear fault. |
| **19** | Status LED anode (+220 Ω → GND) | Slow blink idle · fast blink testing · solid pumping · stutter fault |
| 21 / 22 | *left free* | For an I2C OLED later |

**The 10 kΩ pull-up on GPIO23 is the single most important low-voltage component.**
Import relay boards are active-LOW, and during the ~200 ms ESP32 boot window GPIO23 is
still a floating input. Without the pull-up holding it high, the relay can click on at
power-up and start a 1HP pump with nobody watching. The firmware also drives it OFF on
the very first line of `setup()`, before `Serial.begin()` — belt and braces.

**GPIO 5, 14 and 15 are deliberately unused.** They emit a PWM burst during boot on the
ESP32, which is exactly the glitch you do not want anywhere near a pump contactor.
GPIO 12 is avoided too — it is the flash-voltage strapping pin and pulling it high at
boot can brick the module.

**Voltage divider for ECHO** (20 kΩ / 10 kΩ):

```
  AJ-SR04M ECHO ──[ 20k ]──┬──► GPIO25
                           │
                         [ 10k ]
                           │
                          GND
```

---

## Sensor placement

**Flow sensor (YF-DN25)** — in the pump's **outlet** pipe, arrow pointing in the
direction of flow. Keep ~10 cm of straight pipe before it so turbulence off an elbow
doesn't corrupt the reading. Mount it with the shaft horizontal if you can; it silts up
less.

**Ultrasonic (AJ-SR04M)** — on the tank lid, probe pointing straight down at the water.

> **The blind zone will bite you if you ignore it.** The AJ-SR04M cannot measure
> anything closer than ~25 cm. In a 305 cm deep sump, if your maximum water level sits
> within 25 cm of the sensor, it reads garbage *exactly when the tank is nearly full* —
> the worst possible moment.

Mount it so there is **≥ 30 cm of air between the sensor face and the highest water
level**. If your lid geometry won't allow that, raise it on a 4–6" PVC riser above the
lid. Point it away from the inlet stream (falling water scatters the echo) and away
from the walls (a 3 m path picks up side echoes — the firmware takes a median of 5
readings to reject them, but don't make it work harder than necessary).

The float switch covers you regardless: it is the hardware interlock at the full mark
and it does not care about blind zones. That is exactly why it is in the design.

**Float switch** — side-mounted at your intended full level, NC contact. Test it by
hand before sealing the tank: lifting the float must open the circuit.

---

## Build and flash

```bash
cd firmware && cp src/secrets.h.example src/secrets.h
```

Edit `src/secrets.h` with your WiFi and MQTT credentials. It is gitignored, so it never
reaches GitHub.

```bash
python -m platformio run -e esp32dev --target upload
```

Then watch it come up:

```bash
python -m platformio device monitor -b 115200
```

Host-side unit tests for the level and flow math (needs gcc on PATH):

```bash
cd firmware && python -m platformio test -e native
```

---

## Commissioning checklist

Do these **in order**. Steps 1–5 have the motor circuit disconnected.

1. **Bench test with the motor disconnected.** Power the ESP32, confirm it joins WiFi,
   open `http://<esp32-ip>/` and see the status page.
2. **Relay boot test.** Power-cycle the ESP32 five times with a multimeter on the relay
   contact. It must never close, not even momentarily. If it clicks, your pull-up is
   missing or wrong.
3. **Flow sensor.** Blow gently through the YF-DN25. The web page's `flow_lpm` must move
   off zero.
4. **Ultrasonic.** Hold a flat board at a measured distance. `distance_cm` on the web
   page should match within a couple of cm. Anything closer than 25 cm reads 0 — that
   is the blind zone working as designed.
5. **Float interlock.** Lift the float by hand. The interposing relay must audibly drop
   out, and `float_full` must go to 1 on the web page.
6. **Connect the motor. Now the interlock test that matters:** with the pump running in
   manual, lift the float. **The motor must stop.** If it doesn't, stop and fix the
   wiring before going any further — this is the only thing standing between a software
   bug and a flooded room.
7. **Manual bypass.** Confirm the wall switch still runs the pump with the ESP32
   powered off.
8. **First supervised auto run.** Press "Check for water now" and watch. Confirm it
   either detects flow and goes to `pumping`, or stops cleanly after 8 s.
9. **Calibrate** (next section), then leave it in auto.

---

## Calibration

Two things need real numbers. Both are settable from Home Assistant or the web page —
**you never reflash to calibrate.**

### Tank geometry

With the tank as empty as it gets, read `distance_cm` off the web page → that is
**`tank_empty_cm`**.

Measure from the sensor face down to your intended full level → that is
**`tank_full_cm`**. It must be ≥ 30 (blind zone). Set both, and the percentage and
litre readings follow.

Sanity check the volume: 10×10×10 ft = 28,317 L and 305 cm deep ≈ **93 L per cm**. If
a 1 cm change in `distance_cm` doesn't move `tank_litres` by roughly 93, your
empty/full distances are wrong.

### Flow K-factor

The default `pulses_per_litre = 135` is the YF-DN25 datasheet nominal. Real sensors run
several percent off, and the error compounds over a 20,000 litre fill.

1. Note `today_litres` on the web page.
2. Run the pump into a **known volume** — a 20 L bucket is fine, a 200 L drum is better.
3. Note `today_litres` again. The difference is what the sensor *thinks* it delivered.
4. New K-factor:

   ```
   new_ppl = current_ppl × (measured_litres / actual_litres)
   ```

   Example: default 135, sensor reported 22.4 L into a true 20 L bucket →
   `135 × (22.4 / 20) = 151.2`

5. Set `pulses_per_litre` to that. Repeat once to confirm.

This only affects the litre counters and the flow threshold, not the safety
interlocks — so an approximate value is fine to start with.

---

## Mosquitto + Home Assistant on the RB5009

The RB5009 is ARM64 and supports RouterOS containers, so Mosquitto runs on it natively.
Home Assistant needs several GB — **attach USB storage for it**; the RB5009's onboard
1 GB NAND is not enough.

Enabling containers requires a physical confirmation: set `device-mode`, then
power-cycle the router and press the reset button when it asks. Plan for the downtime.

```routeros
/system/device-mode/update container=yes
# power-cycle and press reset when prompted

/interface/veth/add name=veth-mqtt address=172.17.0.2/24 gateway=172.17.0.1
/interface/bridge/add name=docker
/interface/bridge/port/add bridge=docker interface=veth-mqtt
/ip/address/add address=172.17.0.1/24 interface=docker

/container/config/set registry-url=https://registry-1.docker.io tmpdir=usb1/pull
/container/add remote-image=eclipse-mosquitto:2 interface=veth-mqtt \
    root-dir=usb1/mosquitto start-on-boot=yes

# forward 1883 from the LAN to the container
/ip/firewall/nat/add chain=dstnat dst-port=1883 protocol=tcp \
    action=dst-nat to-addresses=172.17.0.2 to-ports=1883
```

Mosquitto needs a config file in `usb1/mosquitto/config/mosquitto.conf`:

```
listener 1883
allow_anonymous false
password_file /mosquitto/config/passwd
```

Create the `passwd` entry with `mosquitto_passwd -c passwd wateriot` on any machine and
copy it across. Put that username and password into `firmware/src/secrets.h`.

Once the ESP32 connects, **every entity appears in Home Assistant automatically** via
MQTT discovery — no YAML. You get sensors for level, volume, flow, today's delivery,
runtime and dry starts; binary sensors for pump running, float, and sensor health;
number entities for every tunable; an Automatic Mode switch; and buttons for check-now,
stop, and clear-fault.

A useful first automation — notify on arrival:

```yaml
automation:
  - alias: Water arrived
    trigger:
      - platform: state
        entity_id: sensor.water_tank_motor_2_pump_state
        to: "pumping"
    action:
      - service: notify.mobile_app
        data:
          message: >
            Municipal water detected — filling.
            Tank at {{ states('sensor.water_tank_motor_2_tank_level') }}%.
```

---

## Tunable settings

All persisted to NVS, all changeable from Home Assistant or MQTT without reflashing.

| Setting | Default | What it does |
|---|---|---|
| `poll_interval_min` | 15 | Minutes between test pulses |
| `test_duration_s` | 8 | Test pulse length (4 s of that is priming grace) |
| `cooldown_hours` | 6 | Rest after a real fill |
| `flow_threshold` | 5.0 | L/min below which the line counts as dry |
| `lowflow_window_s` | 60 | How long flow must stay low before stopping |
| `max_runtime_min` | 360 | Hard runtime cap (6 h) |
| `pulses_per_litre` | 135 | Flow K-factor — **calibrate this** |
| `tank_empty_cm` | 300 | Sensor→floor distance — **calibrate this** |
| `tank_full_cm` | 35 | Sensor→full-water distance — **calibrate this**, keep ≥ 30 |
| `auto_enabled` | on | Master switch for automatic polling |

### A note on 24/7 polling

At the default 15-minute interval this does **~96 test starts a day**, nearly all of
them dry. A 1HP pump's mechanical seal is water-cooled, so dry running is what wears it.
Three things already limit the damage: the 8-second pulse (not 20), the 6-hour cooldown
after a real fill, and skipping the test entirely when the tank already reads full.

If you find the pump getting noisy, raise `poll_interval_min` to 30 — you would still
catch the supply within half an hour of it arriving, at half the wear. Watch
`dry_starts_today` for a week before deciding.

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Relay clicks on at power-up | Missing 10 kΩ pull-up on GPIO23, or the relay module is active-HIGH — swap `RELAY_ON`/`RELAY_OFF` in `config.h` |
| `distance_cm` always 0 | Water is inside the 25 cm blind zone; ECHO divider wired wrong; or the probe isn't pointing at the water |
| `flow_lpm` always 0 while pumping | Flow sensor installed backwards (check the arrow), missing pull-up on GPIO27, or the sensor is powered from 3V3 instead of 5V |
| Every test reports a dry line | `flow_threshold` too high, or `pulses_per_litre` far too high — calibrate |
| Pump stops and restarts repeatedly | `lowflow_window_s` too short for a sputtering supply — raise it to 120 |
| State stuck in `fault` | Float reads full while the level reads under 50% — one of the two sensors is lying. Check the float isn't stuck up. Long-press the button or use Clear Fault. |
| Entities missing in Home Assistant | MQTT discovery prefix isn't `homeassistant`, or the broker rejected the credentials — check the serial monitor |
| ESP32 reboots every 30 s | Watchdog firing. Check the serial monitor for the stack trace. |

---

## Not built (and when to add it)

- **Motor 1 and Motor 3, and the ball valve** — out of scope by choice. Motor 1
  (tank→overhead) is the obvious next one; it needs an overhead tank level sensor and a
  low-level cutoff on the ground tank so it can't run the sump dry.
- **Inlet pressure sensing** — would remove dry test-starts entirely, but your municipal
  line has no standing pressure so it cannot work here. If that ever changes, an inlet
  pressure switch makes the whole test-pulse mechanism unnecessary.
- **OTA updates** — flash over USB for now. Add `ArduinoOTA` when climbing to the pump
  with a laptop gets old.
