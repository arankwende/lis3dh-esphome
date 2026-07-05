import esphome.codegen as cg
from esphome.components import i2c, sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_TEMPERATURE,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_METER_PER_SECOND_SQUARED,
)

from . import lis3dh_motion_ns

CONF_ACCEL_X = "accel_x"
CONF_ACCEL_Y = "accel_y"
CONF_ACCEL_Z = "accel_z"
CONF_THRESHOLD = "threshold"
CONF_DURATION = "duration"

ICON_ACCELERATION = "mdi:acceleration"

LIS3DHMotionComponent = lis3dh_motion_ns.class_(
    "LIS3DHMotionComponent", cg.PollingComponent, i2c.I2CDevice
)

accel_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_METER_PER_SECOND_SQUARED,
    icon=ICON_ACCELERATION,
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT,
)

temperature_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_CELSIUS,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_TEMPERATURE,
    state_class=STATE_CLASS_MEASUREMENT,
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(LIS3DHMotionComponent),
            cv.Optional(CONF_ACCEL_X): accel_schema,
            cv.Optional(CONF_ACCEL_Y): accel_schema,
            cv.Optional(CONF_ACCEL_Z): accel_schema,
            cv.Optional(CONF_TEMPERATURE): temperature_schema,
            cv.Optional(CONF_THRESHOLD, default=16): cv.int_range(min=1, max=127),
            cv.Optional(CONF_DURATION, default=0): cv.int_range(min=0, max=127),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x19))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_threshold(config[CONF_THRESHOLD]))
    cg.add(var.set_duration(config[CONF_DURATION]))

    for d in ["x", "y", "z"]:
        key = f"accel_{d}"
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, f"set_accel_{d}_sensor")(sens))

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))
