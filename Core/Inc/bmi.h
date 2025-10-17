//
// Created by 创鸿 on 2025/10/12.
//

#ifndef DEMO5_BMI_H
#define DEMO5_BMI_H
#include "spi.h"


// 全局变量声明
extern uint8_t raw_range[2];
extern uint8_t rx_acc_data[7];
extern uint8_t rx_gyro_data[6];
extern struct IMU imu;


uint8_t raw_range[2];
uint8_t rx_acc_data[7];
uint8_t rx_gyro_data[6];


void bmi088_write_byte(uint8_t tx_data) ;

void bmi088_read_byte(uint8_t *rx_data, uint8_t length) ;

void bmi088_write_reg(uint8_t reg, uint8_t data) ;




void BMI088_ACCEL_NS_L(void) ;

void BMI088_ACCEL_NS_H(void) ;

void BMI088_GYRO_NS_L(void) ;

void BMI088_GYRO_NS_H(void) ;





// 数据读取接口
// 参考: acc写入，相当于加上片选的 bmi088_write_reg 函数
void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data);

void bmi088_accel_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length);

void bmi088_gyro_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length) ;

void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t tx_data) ;
#endif //DEMO5_BMI_H

