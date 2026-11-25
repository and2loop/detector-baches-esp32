// GPS NEO-6M (módulo GY-GPS6MV2) por UART: posición, velocidad y hora UTC una vez por segundo.
#pragma once
#include <stdbool.h>

bool gps_iniciar(void);        // configura el UART1 a 9600 baudios en los pines de pines.h
void gps_task(void *arg);      // lee las frases NMEA y actualiza g_gps (con el mutex m_gps)
