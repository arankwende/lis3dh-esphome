import esphome.codegen as cg
from esphome import automation
from esphome.components import i2c, sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_RANGE,
    CONF_TEMPERATURE,
    DEVICE_CLASS_TEMPERATURE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_METER_PER_SECOND_SQUARED,
)

from . import LIS3DHMotionComponent, lis3dh_motion_ns

CONF_ACCEL_X = "accel_x"
CONF_ACCEL_Y = "accel_y"
CONF_ACCEL_Z = "accel_z"
CONF_THRESHOLD = "threshold"
CONF_DURATION = "duration"
CONF_DATA_RATE = "data_rate"

ICON_ACCELERATION = "mdi:acceleration"

LIS3DHRange = lis3dh_motion_ns.enum("LIS3DHRange")
RANGES = {
    "2G": LIS3DHRange.LIS3DH_RANGE_2G,
    "4G": LIS3DHRange.LIS3DH_RANGE_4G,
    "8G": LIS3DHRange.LIS3DH_RANGE_8G,
    "16G": LIS3DHRange.LIS3DH_RANGE_16G,
}

LIS3DHDataRate = lis3dh_motion_ns.enum("LIS3DHDataRate")
DATA_RATES = {
    "1HZ": LIS3DHDataRate.LIS3DH_ODR_1HZ,
    "10HZ": LIS3DHDataRate.LIS3DH_ODR_10HZ,
    "25HZ": LIS3DHDataRate.LIS3DH_ODR_25HZ,
    "50HZ": LIS3DHDataRate.LIS3DH_ODR_50HZ,
    "100HZ": LIS3DHDataRate.LIS3DH_ODR_100HZ,
    "200HZ": LIS3DHDataRate.LIS3DH_ODR_200HZ,
    "400HZ": LIS3DHDataRate.LIS3DH_ODR_400HZ,
}

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
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
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
            cv.Optional(CONF_RANGE, default="2G"): cv.enum(RANGES, upper=True),
            cv.Optional(CONF_DATA_RATE, default="10HZ"): cv.enum(
                DATA_RATES, upper=True
            ),
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
    cg.add(var.set_range(config[CONF_RANGE]))
    cg.add(var.set_data_rate(config[CONF_DATA_RATE]))

    for d in ["x", "y", "z"]:
        key = f"accel_{d}"
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, f"set_accel_{d}_sensor")(sens))

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))


# --- Automation actions -----------------------------------------------------

ClearInterruptAction = lis3dh_motion_ns.class_("ClearInterruptAction", automation.Action)
EnableMotionInterruptAction = lis3dh_motion_ns.class_(
    "EnableMotionInterruptAction", automation.Action
)
DisableMotionInterruptAction = lis3dh_motion_ns.class_(
    "DisableMotionInterruptAction", automation.Action
)

LIS3DH_ACTION_SCHEMA = automation.maybe_simple_id(
    {
        cv.GenerateID(): cv.use_id(LIS3DHMotionComponent),
    }
)


@automation.register_action(
    "lis3dh_motion.clear_interrupt", ClearInterruptAction, LIS3DH_ACTION_SCHEMA
)
@automation.register_action(
    "lis3dh_motion.enable_motion_interrupt",
    EnableMotionInterruptAction,
    LIS3DH_ACTION_SCHEMA,
)
@automation.register_action(
    "lis3dh_motion.disable_motion_interrupt",
    DisableMotionInterruptAction,
    LIS3DH_ACTION_SCHEMA,
)
async def lis3dh_action_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, parent)
