#pragma once
#ifndef IMU_H
#define IMU_H

#include "SparkFun_BNO08x_Arduino_Library.h"

#define IMU_SDA 21
#define IMU_SCL 22
#define IMU_ADDR 0x4A

extern BNO08x imu;

extern unsigned long previous_millis_imu;
extern unsigned long current_millis_imu;
extern int millis_since_last_ts;

extern uint64_t ts_acc, ts_ori;

extern uint8_t report_id;
extern float ax, ay, az;
extern float qx, qy, qz, qw;

extern bool hasAcc, hasOri;

void start_imu();
void enable_reading();
void imu_handler();

#endif
