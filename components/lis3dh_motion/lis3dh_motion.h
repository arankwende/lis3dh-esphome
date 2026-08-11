#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
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
static const uint8_t REG_OUT_X_L      = 0x28;
static const uint8_t REG_OUT_ADC3_L   = 0x0C;
static const uint8_t REG_FIFO_CTRL    = 0x2E;
static const uint8_t REG_INT1_CFG     = 0x30;
static const uint8_t REG_INT1_SRC     = 0x31;
static const uint8_t REG_INT1_THS     = 0x32;
static const uint8_t REG_INT1_DUR     = 0x33;

static const uint8_t LIS3DH_CHIP_ID   = 0x33;

// --- Register bit flags (named so setup() reads without a hex decoder) ---
static const uint8_t CTRL1_XYZ_EN         = 0x07;  // Xen | Yen | Zen
static const uint8_t CTRL2_HPIS1          = 0x01;  // high-pass filter enabled for INT1
static const uint8_t CTRL3_I1_AOI1        = 0x40;  // route AOI1 event to INT1 pin
static const uint8_t CTRL4_BDU            = 0x80;  // block data update
static const uint8_t CTRL4_HR             = 0x08;  // high-resolution (12-bit) mode
static const uint8_t CTRL5_BOOT           = 0x80;  // reboot memory content
static const uint8_t CTRL5_LIR_INT1       = 0x08;  // latch INT1 until INT1_SRC is read
static const uint8_t TEMP_CFG_ADC_TEMP_EN = 0xC0;  // ADC_EN | TEMP_EN

// INT1_CFG for direction-independent wake-on-motion: OR combination of the
// high and low events on all three axes.
static const uint8_t INT1_CFG_MOTION  = 0x3F;
static const uint8_t INT1_SRC_IA      = 0x40;
static const float GRAVITY_EARTH      = 9.80665f;

// Full-scale range. Values match the CTRL_REG4 FS[1:0] field (bits 5:4).
enum LIS3DHRange : uint8_t {
  LIS3DH_RANGE_2G  = 0,
  LIS3DH_RANGE_4G  = 1,
  LIS3DH_RANGE_8G  = 2,
  LIS3DH_RANGE_16G = 3,
};

// Output data rate. Values match the CTRL_REG1 ODR[3:0] nibble.
enum LIS3DHDataRate : uint8_t {
  LIS3DH_ODR_1HZ   = 0x1,
  LIS3DH_ODR_10HZ  = 0x2,
  LIS3DH_ODR_25HZ  = 0x3,
  LIS3DH_ODR_50HZ  = 0x4,
  LIS3DH_ODR_100HZ = 0x5,
  LIS3DH_ODR_200HZ = 0x6,
  LIS3DH_ODR_400HZ = 0x7,
};

namespace detail {

constexpr uint8_t accel_mg_per_digit(LIS3DHRange range) {
  switch (range) {
    case LIS3DH_RANGE_4G:
      return 2;
    case LIS3DH_RANGE_8G:
      return 4;
    case LIS3DH_RANGE_16G:
      return 12;
    case LIS3DH_RANGE_2G:
    default:
      return 1;
  }
}

constexpr int16_t decode_accel_raw(uint8_t low, uint8_t high) {
  return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low) >> 4;
}

constexpr int16_t decode_temperature_raw(uint8_t low, uint8_t high) {
  return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low) >> 6;
}

constexpr float decode_temperature_celsius(uint8_t low, uint8_t high) {
  return 25.0f + (decode_temperature_raw(low, high) / 4.0f);
}

}  // namespace detail

class LIS3DHMotionComponent : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  void update() override;

  void set_accel_x_sensor(sensor::Sensor *s) { accel_x_sensor_ = s; }
  void set_accel_y_sensor(sensor::Sensor *s) { accel_y_sensor_ = s; }
  void set_accel_z_sensor(sensor::Sensor *s) { accel_z_sensor_ = s; }
  void set_temperature_sensor(sensor::Sensor *s) { temperature_sensor_ = s; }
#ifdef USE_BINARY_SENSOR
  void set_motion_binary_sensor(binary_sensor::BinarySensor *s) { motion_binary_sensor_ = s; }
#endif

  void set_threshold(uint8_t threshold) { threshold_ = threshold; }
  void set_duration(uint8_t duration) { duration_ = duration; }
  void set_range(LIS3DHRange range) { range_ = range; }
  void set_data_rate(LIS3DHDataRate data_rate) { data_rate_ = data_rate; }

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
  /// Write INT1 threshold/duration/config. Returns false if any write failed.
  bool arm_motion_interrupt_();
  /// mg per LSB of INT1_THS for the configured full-scale range.
  uint16_t threshold_mg_per_lsb_() const;

  sensor::Sensor *accel_x_sensor_{nullptr};
  sensor::Sensor *accel_y_sensor_{nullptr};
  sensor::Sensor *accel_z_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
#ifdef USE_BINARY_SENSOR
  binary_sensor::BinarySensor *motion_binary_sensor_{nullptr};
  // Throttle for the INT1_SRC poll in loop() (ms).
  uint32_t last_int_poll_{0};
#endif

  uint8_t threshold_{16};
  uint8_t duration_{0};
  LIS3DHRange range_{LIS3DH_RANGE_2G};
  LIS3DHDataRate data_rate_{LIS3DH_ODR_10HZ};
  // m/s² per digit for the configured range; computed in setup().
  float accel_scale_{GRAVITY_EARTH * 0.001f};
};

template<typename... Ts> class ClearInterruptAction : public Action<Ts...> {
 public:
  explicit ClearInterruptAction(LIS3DHMotionComponent *parent) : parent_(parent) {}
  void play(const Ts &...x) override { this->parent_->clear_interrupt(); }

 protected:
  LIS3DHMotionComponent *parent_;
};

template<typename... Ts> class EnableMotionInterruptAction : public Action<Ts...> {
 public:
  explicit EnableMotionInterruptAction(LIS3DHMotionComponent *parent) : parent_(parent) {}
  void play(const Ts &...x) override { this->parent_->enable_motion_interrupt(); }

 protected:
  LIS3DHMotionComponent *parent_;
};

template<typename... Ts> class DisableMotionInterruptAction : public Action<Ts...> {
 public:
  explicit DisableMotionInterruptAction(LIS3DHMotionComponent *parent) : parent_(parent) {}
  void play(const Ts &...x) override { this->parent_->disable_motion_interrupt(); }

 protected:
  LIS3DHMotionComponent *parent_;
};

}  // namespace esphome::lis3dh_motion
