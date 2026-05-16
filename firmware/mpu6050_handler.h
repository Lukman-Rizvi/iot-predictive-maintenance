/*
 * mpu6050_handler.h — MPU6050 initialisation and raw read
 * IoT Predictive Maintenance | University of Peradeniya
 * Dept: Electrical & Electronic Engineering
 * Team: E22215, E22028, E22177, E22287, E22199
 */

#pragma once
#include <Wire.h>
#include "config.h"

// MPU6050 register addresses
#define MPU_REG_PWR_MGMT_1  0x6B
#define MPU_REG_ACCEL_CFG   0x1C
#define MPU_REG_DLPF_CFG    0x1A
#define MPU_REG_SMPLRT_DIV  0x19
#define MPU_REG_ACCEL_XOUT  0x3B
#define MPU_REG_WHO_AM_I    0x75

// Sensitivity scale factors (LSB/g)
static const float ACCEL_SCALE[] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };

// ── Write one byte to MPU6050 register ─────────────────────────────
static void mpu_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

// ── Read n bytes starting at reg ───────────────────────────────────
static void mpu_read(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_I2C_ADDR, len, (uint8_t)true);
  for (uint8_t i = 0; i < len && Wire.available(); i++) buf[i] = Wire.read();
}

// ── Initialise MPU6050 — returns true on success ───────────────────
static bool mpu_init() {
  // Check WHO_AM_I register
  uint8_t who = 0;
  mpu_read(MPU_REG_WHO_AM_I, &who, 1);
  if (who != 0x68 && who != 0x72) return false;   // not an MPU6050/6500

  mpu_write(MPU_REG_PWR_MGMT_1,  0x00);           // wake up, use internal oscillator
  delay(100);
  mpu_write(MPU_REG_SMPLRT_DIV,  0x00);           // sample rate = gyro rate / (1+0)
  mpu_write(MPU_REG_DLPF_CFG,    MPU_DLPF_BW);    // digital low-pass filter
  mpu_write(MPU_REG_ACCEL_CFG,   MPU_ACCEL_RANGE << 3); // set full-scale range
  delay(50);
  return true;
}

// ── Read calibrated accelerometer values in g ─────────────────────
// Called from ISR — must be fast; no dynamic allocation
static void mpu_read_accel(float *ax, float *ay, float *az) {
  uint8_t raw[6];
  mpu_read(MPU_REG_ACCEL_XOUT, raw, 6);

  int16_t raw_ax = (int16_t)((raw[0] << 8) | raw[1]);
  int16_t raw_ay = (int16_t)((raw[2] << 8) | raw[3]);
  int16_t raw_az = (int16_t)((raw[4] << 8) | raw[5]);

  float scale = ACCEL_SCALE[MPU_ACCEL_RANGE];
  *ax = (float)raw_ax / scale;
  *ay = (float)raw_ay / scale;
  *az = (float)raw_az / scale;
}
