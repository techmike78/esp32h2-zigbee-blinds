# ESP32-H2 Zigbee Window Covering Controller

A DIY motorized window blind/curtain controller built on the **ESP32-H2**, running as a
battery-powered **Zigbee sleepy end device** with real light sleep between actions. Fully
custom firmware on pure `framework = espidf` (no Arduino layer) — designed to be paired
directly with **Zigbee2MQTT** and run for weeks/months on a 2S Li-ion pack.

## Features

- DC gear motor (DRV8871 driver) with soft speed ramp near the travel limits.
- Pulse encoder for absolute position tracking (0–100% reported to Zigbee).
- Physical buttons (UP / DOWN / SET) with single-click, long-press and combo actions
  (calibration, motor direction invert, pairing mode, factory reset).
- WS2812 status LED and buzzer feedback.
- Battery voltage monitoring (2S Li-ion) via ADC, reported as a percentage over Zigbee.
- Real ESP32-H2 light sleep (~0.1 mA measured) driven entirely by the Zigbee stack's
  `ESP_ZB_COMMON_SIGNAL_CAN_SLEEP` signal — no separate sleep timer, no polling loop.
- **Firmware updates over the air**, entirely through Zigbee (Zigbee OTA cluster) via
  Zigbee2MQTT — no physical access needed once the device is installed. See
  [`src/main.c`](src/main.c) (top-of-file comment) for the exact release process, and
  `tools/make_ota.py` / `tools/post_build_ota.py` for the packaging tooling.
- Motor tuning (min/max PWM duty, direction invert) exposed as a custom Zigbee cluster and
  editable straight from the Zigbee2MQTT UI — no reflash needed for routine tuning.

## Hardware

Custom PCB (`pcb/`, Altium project) around an ESP32-H2 module, DRV8871 motor driver, a
battery charger/2S protection circuit, and a pulse encoder tapped off the gear motor.
3D-printable enclosure and mounting parts are under `frame/` (STEP/STL). Pinout is defined at
the top of [`src/main.c`](src/main.c); the key GPIOs:

| Signal | GPIO |
|---|---|
| Motor driver IN1 / IN2 (PWM) | GPIO10 / GPIO11 |
| Pulse encoder | GPIO5 |
| Buttons UP / DOWN / SET | GPIO1 / GPIO2 / GPIO3 |
| WS2812 status LED | GPIO14 |
| Battery ADC | GPIO4 |
| Buzzer | GPIO12 |
| Load switch (LED + encoder power) | GPIO13 |

## Building and flashing

PlatformIO project, pure `framework = espidf` (ESP-IDF 5.5.x under the hood via
`pioarduino`/`platform-espressif32` — used only for ESP32-H2 board support, there is no
Arduino code in this project).

```
pio run                 # also (re)generates the OTA image under tools/ota_out/
pio run -t upload        # first flash must be by USB cable — see "First flash" below
pio device monitor
```

### First flash (one-time, by cable)

The very first firmware flashed to a device **must** be built with
`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y` (already set in `sdkconfig.defaults`) — this is a
bootloader-level option and cannot itself be updated over the air. Every subsequent update can
go entirely over Zigbee OTA (see below); a bad OTA image that never rejoins the network is
rolled back automatically.

## Pairing with Zigbee2MQTT

Copy [`my_blind_controller.js`](my_blind_controller.js) into Z2M's external converters
directory and restart Z2M, then pair the device as usual. The converter exposes: cover
position/state, battery percentage/voltage, OTA support, and the custom motor-tuning
attributes (`min_pwm_duty`, `max_pwm_duty`, `motor_invert`).

**Note:** an external converter is only read from disk at Z2M startup — after editing it you
need to restart Z2M, re-running the pairing interview alone does not reload it.

## OTA updates

Full step-by-step release process (bumping the version, packaging, deploying to Z2M, rollback
behavior) is documented at the top of [`src/main.c`](src/main.c) and in
`esp32h2_light_sleep_test/roadmap.md` (Stage 21) / that project's
`.claude/skills/esp32-zigbee-idf-sleep-migration/SKILL.md` for the hardware-verified gotchas
found along the way (attribute registration crash, OTA sub-element framing, sleepy-device
delivery, etc.).

## Repository layout

- `src/main.c` — the entire firmware (single file).
- `my_blind_controller.js` — Zigbee2MQTT external converter.
- `tools/make_ota.py`, `tools/post_build_ota.py` — OTA image packaging (auto-run on build).
- `pcb/` — Altium PCB design + manufacturing outputs (Gerbers).
- `frame/` — 3D-printable enclosure/mounting parts (STEP/STL) and reference part models.
- `zigbee.csv` — flash partition table (dual-slot, sized for OTA).

## License

MIT — see [`LICENSE`](LICENSE).
