// LIS3DH accelerometer + motion-wake component for ESPHome
// Based on gschorcht/lis3dh-esp-idf driver (BSD-3-Clause)
// Provides accel_x/y/z and temperature sensors like MPU6050,
// plus configures INT1 for motion-wake from deep sleep.

#include "lis3dh_motion.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::lis3dh_motion {

static const char *const TAG = "lis3dh_motion";

bool LIS3DHMotionComponent::reset_() {
  // CTRL_REG5: set BOOT bit to reboot memory content
  if (!this->write_byte(REG_CTRL5, CTRL5_BOOT))
    return false;
  delay(10);

  // Set all CTRL registers to default
  if (!this->write_byte(REG_CTRL1, 0x07))
    return false;
  if (!this->write_byte(REG_CTRL2, 0x00))
    return false;
  if (!this->write_byte(REG_CTRL3, 0x00))
    return false;
  if (!this->write_byte(REG_CTRL4, 0x00))
    return false;
  if (!this->write_byte(REG_CTRL5, 0x00))
    return false;
  if (!this->write_byte(REG_CTRL6, 0x00))
    return false;

  // Clear FIFO
  if (!this->write_byte(REG_FIFO_CTRL, 0x00))
    return false;

  // Clear interrupt config
  if (!this->write_byte(REG_INT1_CFG, 0x00))
    return false;
  if (!this->write_byte(REG_INT1_THS, 0x00))
    return false;
  if (!this->write_byte(REG_INT1_DUR, 0x00))
    return false;

  // Clear any latched interrupt
  uint8_t dummy;
  this->read_byte(REG_INT1_SRC, &dummy);

  return true;
}

uint16_t LIS3DHMotionComponent::threshold_mg_per_lsb_() const {
  // INT1_THS LSB weight per full-scale range (datasheet Table 4).
  switch (this->range_) {
    case LIS3DH_RANGE_4G:
      return 32;
    case LIS3DH_RANGE_8G:
      return 62;
    case LIS3DH_RANGE_16G:
      return 186;
    case LIS3DH_RANGE_2G:
    default:
      return 16;
  }
}

bool LIS3DHMotionComponent::arm_motion_interrupt_() {
  // Write every register even if an earlier one fails, then report.
  bool ok = true;
  ok = this->write_byte(REG_INT1_THS, this->threshold_) && ok;
  ok = this->write_byte(REG_INT1_DUR, this->duration_) && ok;
  ok = this->write_byte(REG_INT1_CFG, INT1_CFG_MOTION) && ok;
  return ok;
}

void LIS3DHMotionComponent::setup() {
  ESP_LOGD(TAG, "Setting up LIS3DH...");

  // mg/digit in high-resolution mode per full-scale range (2/4/8/16g).
  static const float MG_PER_DIGIT_HR[4] = {1.0f, 2.0f, 4.0f, 12.0f};
  this->accel_scale_ = MG_PER_DIGIT_HR[this->range_] * 0.001f * GRAVITY_EARTH;

  // 1. Verify chip ID
  uint8_t who_am_i;
  if (!this->read_byte(REG_WHO_AM_I, &who_am_i)) {
    ESP_LOGE(TAG, "Failed to read WHO_AM_I!");
    this->mark_failed();
    return;
  }
  if (who_am_i != LIS3DH_CHIP_ID) {
    ESP_LOGE(TAG, "WHO_AM_I=0x%02X, expected 0x%02X", who_am_i, LIS3DH_CHIP_ID);
    this->mark_failed();
    return;
  }
  ESP_LOGD(TAG, "  WHO_AM_I: 0x%02X (OK)", who_am_i);

  // 2. Reset to known state
  if (!this->reset_()) {
    ESP_LOGE(TAG, "Reset failed!");
    this->mark_failed();
    return;
  }

  // 3. CTRL_REG4: BDU=1, HR=1 (high-resolution 12-bit), FS from config
  if (!this->write_byte(REG_CTRL4, CTRL4_BDU | CTRL4_HR | (this->range_ << 4))) {
    this->mark_failed();
    return;
  }

  // 4. Enable ADC and temperature sensor only if a temperature sensor is used
  //    (the ADC draws extra current, so leave it off otherwise).
  if (this->temperature_sensor_ != nullptr) {
    if (!this->write_byte(REG_TEMP_CFG, TEMP_CFG_ADC_TEMP_EN)) {
      ESP_LOGW(TAG, "Failed to enable temperature sensor");
    }
  }

  // 5. Configure high-pass filter for interrupt (filters out gravity)
  //    CTRL_REG2: HPM=00 (normal), HPCF=00, HPIS1=1
  if (!this->write_byte(REG_CTRL2, CTRL2_HPIS1)) {
    this->mark_failed();
    return;
  }

  // 6. Read REFERENCE to reset HPF
  uint8_t ref;
  this->read_byte(REG_REFERENCE, &ref);

  // 7. Configure motion interrupt on INT1. This is the whole point of the
  //    component, so a failure here must mark the device failed.
  bool int_ok = this->arm_motion_interrupt_();
  int_ok = this->write_byte(REG_CTRL5, CTRL5_LIR_INT1) && int_ok;  // latch INT1
  int_ok = this->write_byte(REG_CTRL3, CTRL3_I1_AOI1) && int_ok;   // route AOI1 -> INT1
  if (!int_ok) {
    ESP_LOGE(TAG, "Failed to configure motion interrupt!");
    this->mark_failed();
    return;
  }

  // 8. Clear pending interrupt
  uint8_t int_src;
  this->read_byte(REG_INT1_SRC, &int_src);

  // 9. CTRL_REG1: ODR from config, LPen=0 (normal/HR mode), all axes enabled.
  //    Set LAST so measurements start after all config is done.
  if (!this->write_byte(REG_CTRL1, (this->data_rate_ << 4) | CTRL1_XYZ_EN)) {
    this->mark_failed();
    return;
  }

  // Wait for sensor startup
  delay(20);

  // Reset HPF baseline with current orientation
  this->read_byte(REG_REFERENCE, &ref);
  // Clear interrupt after startup
  this->read_byte(REG_INT1_SRC, &int_src);

  // Verify register readback
  uint8_t ctrl1, ctrl3, ctrl4, int1_cfg, int1_ths;
  this->read_byte(REG_CTRL1, &ctrl1);
  this->read_byte(REG_CTRL3, &ctrl3);
  this->read_byte(REG_CTRL4, &ctrl4);
  this->read_byte(REG_INT1_CFG, &int1_cfg);
  this->read_byte(REG_INT1_THS, &int1_ths);

  ESP_LOGD(TAG, "  CTRL1=0x%02X CTRL3=0x%02X CTRL4=0x%02X INT1_CFG=0x%02X THS=0x%02X",
           ctrl1, ctrl3, ctrl4, int1_cfg, int1_ths);
  ESP_LOGD(TAG, "  LIS3DH configured successfully!");
}

void LIS3DHMotionComponent::loop() {
  if (this->motion_binary_sensor_ == nullptr)
    return;
  // Poll INT1_SRC a few times a second. Reading it also clears the latch
  // (LIR_INT1=1), so the sensor pulses true for the poll after a motion
  // event. NOTE: this consumes the same latch used for GPIO/deep-sleep wake,
  // so use the binary sensor OR the INT1 wake pin, not both at once.
  const uint32_t now = millis();
  if (now - this->last_int_poll_ < 100)
    return;
  this->last_int_poll_ = now;

  uint8_t src;
  if (!this->read_byte(REG_INT1_SRC, &src))
    return;
  // Bit 6 (IA) = interrupt active.
  this->motion_binary_sensor_->publish_state((src & 0x40) != 0);
}

void LIS3DHMotionComponent::update() {
  // Only read acceleration if at least one axis is published.
  if (this->accel_x_sensor_ != nullptr || this->accel_y_sensor_ != nullptr ||
      this->accel_z_sensor_ != nullptr) {
    // Read 6 bytes of acceleration data (X_L, X_H, Y_L, Y_H, Z_L, Z_H)
    // Register auto-increments when MSB of sub-address is set
    uint8_t raw[6];
    if (!this->read_bytes(REG_OUT_X_L | 0x80, raw, 6)) {
      this->status_set_warning();
      return;
    }

    // Data is 16-bit left-justified two's complement, in high-resolution
    // (12-bit) mode. Shift right by 4 to get the 12-bit value.
    int16_t raw_x = (int16_t)((raw[1] << 8) | raw[0]) >> 4;
    int16_t raw_y = (int16_t)((raw[3] << 8) | raw[2]) >> 4;
    int16_t raw_z = (int16_t)((raw[5] << 8) | raw[4]) >> 4;

    // Convert to m/s² using the scale for the configured range.
    float accel_x = raw_x * this->accel_scale_;
    float accel_y = raw_y * this->accel_scale_;
    float accel_z = raw_z * this->accel_scale_;

    ESP_LOGD(TAG, "Accel: x=%.2f m/s², y=%.2f m/s², z=%.2f m/s²",
             accel_x, accel_y, accel_z);

    if (this->accel_x_sensor_ != nullptr)
      this->accel_x_sensor_->publish_state(accel_x);
    if (this->accel_y_sensor_ != nullptr)
      this->accel_y_sensor_->publish_state(accel_y);
    if (this->accel_z_sensor_ != nullptr)
      this->accel_z_sensor_->publish_state(accel_z);
  }

  // Read temperature (ADC3 when temp sensor enabled)
  // LIS3DH temp is relative: output 0 = 25°C, 1 digit = 1°C
  if (this->temperature_sensor_ != nullptr) {
    uint8_t temp_raw[2];
    if (this->read_bytes(REG_OUT_ADC3_L | 0x80, temp_raw, 2)) {
      int16_t temp_val = (int16_t)((temp_raw[1] << 8) | temp_raw[0]);
      // In high-resolution mode, 10-bit left-justified: shift right by 6
      temp_val >>= 6;
      float temperature = 25.0f + (temp_val / 4.0f);
      ESP_LOGD(TAG, "Temp: %.1f°C (raw=%d)", temperature, temp_val);
      this->temperature_sensor_->publish_state(temperature);
    }
  }

  this->status_clear_warning();
}

void LIS3DHMotionComponent::clear_interrupt() {
  uint8_t src;
  if (!this->read_byte(REG_INT1_SRC, &src)) {
    ESP_LOGW(TAG, "Failed to clear INT1 latch");
  }
}

void LIS3DHMotionComponent::disable_motion_interrupt() {
  // Stop the event generator, then clear any already-latched interrupt
  // so INT1 drops and stays low.
  if (!this->write_byte(REG_INT1_CFG, 0x00)) {
    ESP_LOGW(TAG, "Failed to disable motion interrupt");
    return;
  }
  this->clear_interrupt();
  ESP_LOGD(TAG, "Motion interrupt disabled");
}

void LIS3DHMotionComponent::enable_motion_interrupt() {
  if (!this->arm_motion_interrupt_()) {
    ESP_LOGW(TAG, "Failed to enable motion interrupt");
    return;
  }
  this->clear_interrupt();
  ESP_LOGD(TAG, "Motion interrupt enabled");
}

void LIS3DHMotionComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "LIS3DH Accelerometer:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  static const uint8_t RANGE_G[4] = {2, 4, 8, 16};
  ESP_LOGCONFIG(TAG, "  Range: +/-%ug", RANGE_G[this->range_]);
  uint16_t mg = this->threshold_mg_per_lsb_();
  ESP_LOGCONFIG(TAG, "  Motion threshold: %u (~%umg)", this->threshold_, this->threshold_ * mg);
  ESP_LOGCONFIG(TAG, "  Motion duration: %u ODR cycles", this->duration_);
  LOG_SENSOR("  ", "Acceleration X", this->accel_x_sensor_);
  LOG_SENSOR("  ", "Acceleration Y", this->accel_y_sensor_);
  LOG_SENSOR("  ", "Acceleration Z", this->accel_z_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_BINARY_SENSOR("  ", "Motion", this->motion_binary_sensor_);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "  Communication failed!");
  }
}

}  // namespace esphome::lis3dh_motion
