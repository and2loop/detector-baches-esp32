// IMU MPU-6050 (módulo GY-521) por I2C: 200 muestras por segundo leídas de su FIFO.
#pragma once
#include <stdbool.h>

bool imu_iniciar(void);        // prepara el bus, comprueba que el sensor contesta y lo configura
void imu_task(void *arg);      // cada 20 ms vacía la FIFO y manda las muestras a q_imu_log
