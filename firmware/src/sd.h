// Tarjeta microSD por SPI: una carpeta por sesión con los CSV del IMU y del GPS.
#pragma once
#include <stdbool.h>

bool sd_iniciar(void);         // monta la tarjeta (sin formatearla nunca) y abre la sesión nueva
void log_task(void *arg);      // escribe todo lo que llega; cada segundo (tmr_sync) guarda de verdad
