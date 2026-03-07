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
  // CTRL_REG5: set BOOT bit (bit 7) to reboot memory content
  if (!this->write_byte(REG_CTRL5, 0x80))
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

void LIS3DHMotionComponent::setup() {
  ESP_LOGD(TAG, "Setting up LIS3DH...");

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

  // 3. CTRL_REG4: BDU=1, FS=00 (+/-2g), HR=1 (high-resolution 12-bit)
  if (!this->write_byte(REG_CTRL4, 0x88)) {
    this->mark_failed();
    return;
  }

  // 4. Enable ADC and temperature sensor
  //    TEMP_CFG_REG: ADC_EN=1, TEMP_EN=1 (bits 7:6 = 11)
  if (!this->write_byte(REG_TEMP_CFG, 0xC0)) {
    ESP_LOGW(TAG, "Failed to enable temperature sensor");
  }

  // 5. Configure high-pass filter for interrupt (filters out gravity)
  //    CTRL_REG2: HPM=00 (normal), HPCF=01, HPIS1=1
  if (!this->write_byte(REG_CTRL2, 0x01)) {
    this->mark_failed();
    return;
  }

  // 6. Read REFERENCE to reset HPF
  uint8_t ref;
  this->read_byte(REG_REFERENCE, &ref);

  // 7. Configure motion interrupt on INT1
  this->write_byte(REG_INT1_THS, this->threshold_);
  this->write_byte(REG_INT1_DUR, this->duration_);
  // INT1_CFG: OR combination of X/Y/Z high events (wake-up mode)
  this->write_byte(REG_INT1_CFG, 0x2A);
  // CTRL_REG5: LIR_INT1=1 (latch interrupt)
  this->write_byte(REG_CTRL5, 0x08);
  // CTRL_REG3: I1_AOI1=1 (route event generator 1 to INT1 pin)
  this->write_byte(REG_CTRL3, 0x40);

  // 8. Clear pending interrupt
  uint8_t int_src;
  this->read_byte(REG_INT1_SRC, &int_src);

  // 9. CTRL_REG1: ODR=0010 (10Hz), LPen=0 (normal/HR mode), all axes
  //    0x27 = 0b00100111 = 10Hz, normal mode, XYZ enabled
  //    Set LAST so measurements start after all config is done
  if (!this->write_byte(REG_CTRL1, 0x27)) {
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

void LIS3DHMotionComponent::update() {
  // Read 6 bytes of acceleration data (X_L, X_H, Y_L, Y_H, Z_L, Z_H)
  // Register auto-increments when MSB of sub-address is set
  uint8_t raw[6];
  if (!this->read_bytes(REG_OUT_X_L | 0x80, raw, 6)) {
    this->status_set_warning();
    return;
  }

  // Data is 16-bit left-justified two's complement, in high-resolution (12-bit) mode
  // Shift right by 4 to get the 12-bit value
  int16_t raw_x = (int16_t)((raw[1] << 8) | raw[0]) >> 4;
  int16_t raw_y = (int16_t)((raw[3] << 8) | raw[2]) >> 4;
  int16_t raw_z = (int16_t)((raw[5] << 8) | raw[4]) >> 4;

  // Convert to m/s² (at +/-2g, 12-bit: 1 mg/digit)
  float accel_x = raw_x * ACCEL_SCALE_2G;
  float accel_y = raw_y * ACCEL_SCALE_2G;
  float accel_z = raw_z * ACCEL_SCALE_2G;

  ESP_LOGD(TAG, "Accel: x=%.2f m/s², y=%.2f m/s², z=%.2f m/s²",
           accel_x, accel_y, accel_z);

  if (this->accel_x_sensor_ != nullptr)
    this->accel_x_sensor_->publish_state(accel_x);
  if (this->accel_y_sensor_ != nullptr)
    this->accel_y_sensor_->publish_state(accel_y);
  if (this->accel_z_sensor_ != nullptr)
    this->accel_z_sensor_->publish_state(accel_z);

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
  this->read_byte(REG_INT1_SRC, &src);
}

void LIS3DHMotionComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "LIS3DH Accelerometer:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  ESP_LOGCONFIG(TAG, "  Motion threshold: %u (~%umg)", this->threshold_, this->threshold_ * 16);
  ESP_LOGCONFIG(TAG, "  Motion duration: %u ODR cycles", this->duration_);
  LOG_SENSOR("  ", "Acceleration X", this->accel_x_sensor_);
  LOG_SENSOR("  ", "Acceleration Y", this->accel_y_sensor_);
  LOG_SENSOR("  ", "Acceleration Z", this->accel_z_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "  Communication failed!");
  }
}

}  // namespace esphome::lis3dh_motion
