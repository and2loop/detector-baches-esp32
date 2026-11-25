// Lo que comparten las tareas: tipos de datos, colas, mutex y bits de estado.
// Las variables se crean en main.c; aquí solo se anuncian con "extern" para que
// cualquier fichero que incluya comun.h pueda usarlas.
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/stream_buffer.h"

// Una muestra del IMU con su hora. Es lo que viaja por las colas del IMU.
typedef struct {
    int64_t t_us;          // momento de la muestra, en µs desde el arranque
    int16_t v[6];          // ax, ay, az, gx, gy, gz en bruto (4096 = 1 g, 65,5 = 1 °/s)
} muestra_t;

// Lo último que se sabe del GPS. Lo escribe gps_task y lo leen las demás con el mutex m_gps.
typedef struct {
    bool    fix;               // hay posición válida
    double  lat, lon;          // grados; NAN si no hay fix
    float   vel_kmh, hdop;
    int     satelites;         // los que usa para calcular la posición
    char    hora[11], fecha[7];// "hhmmss.ss" y "ddmmaa", en UTC
    int64_t t_us;              // cuándo llegó la última frase RMC
    double  km;                // km recorridos desde el arranque (para el latido de la Fase 5)
} gps_t;

// Avisos para el buzzer
typedef enum { AV_LISTO, AV_CANDIDATO, AV_BACHE, AV_BADEN, AV_OTRO } aviso_t;

// Bits del event group "estado": cada bit es una bandera que cualquier tarea puede mirar
#define EV_IMU_OK   BIT0    // el IMU contesta y está configurado
#define EV_SD_OK    BIT1    // la tarjeta está montada y la sesión abierta
#define EV_GPS_FIX  BIT2    // el GPS tiene posición
#define EV_WIFI_OK  BIT3    // (Fase 5)
#define EV_MQTT_OK  BIT4    // (Fase 5)

extern QueueHandle_t q_imu_log;       // IMU → tarjeta SD (400 muestras = 2 s de margen)
extern QueueHandle_t q_marcas;        // ISR del pulsador → tarjeta SD (horas de las pulsaciones)
extern QueueHandle_t q_aviso;         // quien sea → buzzer
extern SemaphoreHandle_t m_gps;       // candado de g_gps
extern gps_t g_gps;                   // última posición conocida
extern EventGroupHandle_t estado;     // bits EV_*
extern StreamBufferHandle_t sb_nmea;  // texto NMEA en bruto: gps_task → log_task

// Identificadores de las tareas: tmr_sync los usa para avisar a log_task, y log_task
// para imprimir cuánta pila le sobra a cada una
extern TaskHandle_t h_imu, h_gps, h_log, h_led, h_buzzer;

// Contadores para comprobar que no se pierde nada (los imprime log_task cada segundo)
extern volatile uint32_t imu_perdidas;    // muestras que no cupieron en q_imu_log
extern volatile uint32_t imu_desbordes;   // veces que se llenó la FIFO del sensor
