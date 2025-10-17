//
// Created by 创鸿 on 2025/10/12.
//

#ifndef DEMO5_BMI_H
#define DEMO5_BMI_H
#include "spi.h"

struct IMU {
  float acc_x;
  float acc_y;
  float acc_z;
  float angle_x;
  float angle_y;
  float angle_z;
};

// 全局变量声明
extern uint8_t raw_range[2];
extern uint8_t rx_acc_data[7];
extern uint8_t rx_gyro_data[6];
extern struct IMU imu;
#endif //DEMO5_BMI_H

uint8_t raw_range[2];
uint8_t rx_acc_data[7];
uint8_t rx_gyro_data[6];
struct IMU imu;

inline void bmi088_write_byte(uint8_t tx_data) {
  HAL_SPI_Transmit(&hspi1, &tx_data, 1, 1000);
  while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
}

inline void bmi088_read_byte(uint8_t *rx_data, uint8_t length) {
  HAL_SPI_Receive(&hspi1, rx_data, length, 1000);
  while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX);
}

inline void bmi088_write_reg(uint8_t reg, uint8_t data) {
  bmi088_write_byte(reg&0x7F);      // 寄存器地址
  bmi088_write_byte(data);     // 要写入的数据
}




inline void BMI088_ACCEL_NS_L(void) {
  HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}

inline void BMI088_ACCEL_NS_H(void) {
  HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

inline void BMI088_GYRO_NS_L(void) {
  HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}

inline void BMI088_GYRO_NS_H(void) {
  HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}





// 数据读取接口
// 参考: acc写入，相当于加上片选的 bmi088_write_reg 函数
inline void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data) {
  BMI088_GYRO_NS_H();
  BMI088_ACCEL_NS_L();

  bmi088_write_byte(reg & 0x7F);
  bmi088_write_byte(data);

  BMI088_ACCEL_NS_H();
}

void bmi088_accel_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length) {
  BMI088_GYRO_NS_H();
  BMI088_ACCEL_NS_L();
  HAL_Delay(1);

  bmi088_write_byte(reg | 0x80);  // 设置读取标志位
  bmi088_read_byte(rx_data, length);
  HAL_Delay(1);

  BMI088_ACCEL_NS_H();
}

void bmi088_gyro_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length) {
  BMI088_ACCEL_NS_H();
  BMI088_GYRO_NS_L();

  bmi088_write_byte(reg | 0x80);  // 设置读取标志位
  bmi088_read_byte(rx_data, length);

  BMI088_GYRO_NS_H();
}

inline void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t tx_data) {
  BMI088_ACCEL_NS_H();
  BMI088_GYRO_NS_L();

  bmi088_write_byte(reg & 0x7F);  // 寄存器地址
  bmi088_write_byte(tx_data);     // 要写入的数据

  BMI088_GYRO_NS_H();
}


