import esphome.codegen as cg
import esphome.config_validation as cv

CODEOWNERS = ["@arankwende"]
DEPENDENCIES = ["i2c"]

lis3dh_motion_ns = cg.esphome_ns.namespace("lis3dh_motion")
