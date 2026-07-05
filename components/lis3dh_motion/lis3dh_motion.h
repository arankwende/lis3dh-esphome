#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome::lis3dh_motion {

// LIS3DH register addresses (from gschorcht/lis3dh-esp-idf driver)
static const uint8_t REG_WHO_AM_I     = 0x0F;
static const uint8_t REG_TEMP_CFG     = 0x1F;
static const uint8_t REG_CTRL1        = 0x20;
static const uint8_t REG_CTRL2        = 0x21;
static const uint8_t REG_CTRL3        = 0x22;
static const uint8_t REG_CTRL4        = 0x23;
static const uint8_t REG_CTRL5        = 0x24;
static const uint8_t REG_CTRL6        = 0x25;
static const uint8_t REG_REFERENCE    = 0x26;
static const uint8_t REG_STATUS       = 0x27;
static const uint8_t REG_OUT_X_L      = 0x28;
static const uint8_t REG_OUT_ADC3_L   = 0x0C;
static const uint8_t REG_FIFO_CTRL    = 0x2E;
static const uint8_t REG_INT1_CFG     = 0x30;
static const uint8_t REG_INT1_SRC     = 0x31;
static const uint8_t REG_INT1_THS     = 0x32;
static const uint8_t REG_INT1_DUR     = 0x33;

static const uint8_t LIS3DH_CHIP_ID   = 0x33;
// INT1_CFG for wake-on-motion: OR combination of X/Y/Z high events
static const uint8_t INT1_CFG_MOTION  = 0x2A;
static const float GRAVITY_EARTH      = 9.80665f;
// At +/-2g, 12-bit mode: 1 mg/digit = 0.001g per digit
// Raw 16-bit left-justified: divide by 16 to get 12-bit, then * 0.001g
static const float ACCEL_SCALE_2G     = (GRAVITY_EARTH * 0.001f);

class LIS3DHMotionComponent : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;

  void set_accel_x_sensor(sensor::Sensor *s) { accel_x_sensor_ = s; }
  void set_accel_y_sensor(sensor::Sensor *s) { accel_y_sensor_ = s; }
  void set_accel_z_sensor(sensor::Sensor *s) { accel_z_sensor_ = s; }
  void set_temperature_sensor(sensor::Sensor *s) { temperature_sensor_ = s; }

  void set_threshold(uint8_t threshold) { threshold_ = threshold; }
  void set_duration(uint8_t duration) { duration_ = duration; }

  /// Clear the latched INT1 interrupt so it can trigger again.
  void clear_interrupt();
  /// Stop generating motion interrupts on INT1 (e.g. before a deep sleep
  /// that should only wake on a key press). Re-armed by enable_motion_interrupt()
  /// or automatically on the next boot via setup().
  void disable_motion_interrupt();
  /// Re-arm motion interrupt generation on INT1.
  void enable_motion_interrupt();

 protected:
  bool reset_();

  sensor::Sensor *accel_x_sensor_{nullptr};
  sensor::Sensor *accel_y_sensor_{nullptr};
  sensor::Sensor *accel_z_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};

  uint8_t threshold_{16};
  uint8_t duration_{0};
};

}  // namespace esphome::lis3dh_motion
