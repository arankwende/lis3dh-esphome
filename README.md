# lis3dh-esphome

ESPHome external component for the ST **LIS3DH** 3-axis accelerometer (the
accelerometer used in the [OMOTE](https://github.com/CoretechR/OMOTE) remote).

It provides:

- `accel_x` / `accel_y` / `accel_z` sensors (m/s², ±2g high-resolution mode)
- `temperature` sensor (the LIS3DH's internal relative temperature sensor)
- A **wake-on-motion interrupt on INT1** (latched, high-pass filtered so
  gravity/orientation doesn't matter) — ideal as an `esp32_ext1_wakeup`
  deep-sleep wake source or a GPIO binary sensor.

## Usage

```yaml
external_components:
  - source: github://arankwende/lis3dh-esphome@main

i2c:
  sda: GPIO20
  scl: GPIO19

sensor:
  - platform: lis3dh_motion
    id: accel
    address: 0x19        # 0x18 if SDO/SA0 is pulled low
    threshold: 16        # motion threshold, 1 LSB = 16mg at ±2g (16 ≈ 256mg)
    duration: 0          # minimum event duration in ODR cycles (10Hz ODR)
    update_interval: 60s
    accel_x:
      name: "Accel X"
    accel_y:
      name: "Accel Y"
    accel_z:
      name: "Accel Z"
    temperature:
      name: "LIS3DH Temperature"
```

### Motion interrupt / wake from deep sleep

INT1 goes **high** on motion and stays latched until cleared. Wire it to an
RTC-capable GPIO and use it both as a binary sensor and a deep-sleep wake
source:

```yaml
deep_sleep:
  esp32_ext1_wakeup:
    pins:
      - number: GPIO02
        allow_other_uses: true
    mode: ANY_HIGH

binary_sensor:
  - platform: gpio
    pin:
      number: GPIO02
      allow_other_uses: true
    on_press:
      - logger.log: "Motion!"
      # Clear the latch so the next motion can retrigger
      - lambda: 'id(accel).clear_interrupt();'
```

### Lambda API

| Method | Purpose |
|---|---|
| `id(accel).clear_interrupt();` | Clear the latched INT1 so it can fire again |
| `id(accel).disable_motion_interrupt();` | Stop INT1 events (e.g. before a deep sleep that should ignore motion) |
| `id(accel).enable_motion_interrupt();` | Re-arm INT1 motion events (also re-armed automatically at boot) |

## Configuration variables

- **address** (Optional, default `0x19`): I2C address.
- **threshold** (Optional, 1–127, default `16`): INT1 motion threshold,
  16mg per LSB at ±2g.
- **duration** (Optional, 0–127, default `0`): INT1 minimum duration in
  ODR cycles (ODR is 10Hz, so 1 = 100ms).
- **accel_x / accel_y / accel_z / temperature** (Optional): standard sensor
  schemas.
- **update_interval** (Optional, default `60s`).
