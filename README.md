# lis3dh-esphome

ESPHome external component for the ST **LIS3DH** 3-axis accelerometer (the
accelerometer used in the [OMOTE](https://github.com/CoretechR/OMOTE) remote).

It provides:

- `accel_x` / `accel_y` / `accel_z` sensors (m/s², selectable ±2/4/8/16g,
  high-resolution mode)
- `temperature` sensor (the LIS3DH's internal relative temperature sensor)
- A **wake-on-motion interrupt on INT1** (latched, high-pass filtered so
  gravity/orientation doesn't matter) — ideal as an `esp32_ext1_wakeup`
  deep-sleep wake source or a GPIO binary sensor.

## Usage

```yaml
external_components:
  - source: github://arankwende/lis3dh-esphome@main
    # For production, replace main with a release tag or commit SHA so ESPHome
    # upgrades cannot silently change the component.

i2c:
  sda: GPIO20
  scl: GPIO19

sensor:
  - platform: lis3dh_motion
    id: accel
    address: 0x19        # 0x18 if SDO/SA0 is pulled low
    range: 2G            # ±2G / 4G / 8G / 16G
    data_rate: 10HZ      # 1/10/25/50/100/200/400 HZ
    threshold: 16        # motion threshold, 1 LSB = 16mg at ±2g (16 ≈ 256mg)
    duration: 0          # minimum event duration in ODR cycles
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

The `main` reference tracks current development. Production installations
should pin a published release tag or commit SHA and update that reference
deliberately.

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
      - lis3dh_motion.clear_interrupt: accel
```

### Automation actions

| Action | Purpose |
|---|---|
| `lis3dh_motion.clear_interrupt: accel` | Clear the latched INT1 so it can fire again |
| `lis3dh_motion.disable_motion_interrupt: accel` | Stop INT1 events (e.g. before a deep sleep that should ignore motion) |
| `lis3dh_motion.enable_motion_interrupt: accel` | Re-arm INT1 motion events (also re-armed automatically at boot) |

The same operations are available from lambdas as
`id(accel).clear_interrupt();`, `id(accel).disable_motion_interrupt();` and
`id(accel).enable_motion_interrupt();`.

### Motion binary sensor (no extra wire)

If you don't need deep-sleep wake, you can read the interrupt over I2C instead
of wiring INT1 to a GPIO. Add a `binary_sensor` that references the accelerometer:

```yaml
binary_sensor:
  - platform: lis3dh_motion
    lis3dh_motion_id: accel
    name: "Motion"
```

It polls `INT1_SRC` a few times a second and reports motion. Because reading
`INT1_SRC` clears the latch, use **either** this binary sensor **or** the INT1
GPIO wake pin above — not both at once, as they consume the same latch.

## Configuration variables

- **address** (Optional, default `0x19`): I2C address.
- **range** (Optional, default `2G`): full-scale range — `2G`, `4G`, `8G` or
  `16G`. Larger ranges measure stronger accelerations but reduce resolution.
- **data_rate** (Optional, default `10HZ`): output data rate — `1HZ`, `10HZ`,
  `25HZ`, `50HZ`, `100HZ`, `200HZ` or `400HZ`. Also sets the `duration` tick.
- **threshold** (Optional, 1–127, default `16`): INT1 motion threshold. LSB
  weight depends on `range` (16mg at ±2g, 32mg at ±4g, 62mg at ±8g, 186mg at ±16g).
- **duration** (Optional, 0–127, default `0`): INT1 minimum duration in
  ODR cycles (at 10Hz, 1 = 100ms).
- **accel_x / accel_y / accel_z / temperature** (Optional): standard sensor
  schemas.
- **update_interval** (Optional, default `60s`).

## Development

Regression configurations live in `tests/`. They cover a sensor-only build
and the complete sensor, motion binary sensor, and automation-action build.
Both configurations are compiled for the Arduino and ESP-IDF frameworks in
GitHub Actions.

To validate them locally with ESPHome 2026.5.1:

```sh
esphome -s framework arduino compile tests/sensor-only.yaml
esphome -s framework arduino compile tests/full.yaml
esphome -s framework esp-idf compile tests/sensor-only.yaml
esphome -s framework esp-idf compile tests/full.yaml
```

Third-party code attribution is recorded in
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md).
