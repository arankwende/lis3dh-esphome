import esphome.codegen as cg
from esphome.components import i2c
import esphome.config_validation as cv

CODEOWNERS = ["@arankwende"]
DEPENDENCIES = ["i2c"]

lis3dh_motion_ns = cg.esphome_ns.namespace("lis3dh_motion")
LIS3DHMotionComponent = lis3dh_motion_ns.class_(
    "LIS3DHMotionComponent", cg.PollingComponent, i2c.I2CDevice
)

CONF_LIS3DH_MOTION_ID = "lis3dh_motion_id"
