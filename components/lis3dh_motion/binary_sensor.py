import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import DEVICE_CLASS_MOTION

from . import CONF_LIS3DH_MOTION_ID, LIS3DHMotionComponent

DEPENDENCIES = ["lis3dh_motion"]

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema(
    device_class=DEVICE_CLASS_MOTION,
).extend(
    {
        cv.GenerateID(CONF_LIS3DH_MOTION_ID): cv.use_id(LIS3DHMotionComponent),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_LIS3DH_MOTION_ID])
    var = await binary_sensor.new_binary_sensor(config)
    cg.add(parent.set_motion_binary_sensor(var))
