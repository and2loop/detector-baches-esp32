// Registrador del nodo: lee el IMU (200 Hz) y el GPS (1 Hz) y lo graba todo en la microSD.
// app_main crea lo que comparten las tareas, inicializa cada módulo y lanza las tareas.
// Después app_main termina: a partir de ahí todo lo hacen las tareas.

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "comun.h"
#include "imu.h"
#include "gps.h"
#include "sd.h"
#include "pulsador.h"
#include "led.h"
#include "buzzer.h"

// Las variables compartidas que comun.h anuncia con "extern" se crean aquí
QueueHandle_t q_imu_log, q_marcas, q_aviso;
SemaphoreHandle_t m_gps;
gps_t g_gps;
EventGroupHandle_t estado;
StreamBufferHandle_t sb_nmea;
TaskHandle_t h_imu, h_gps, h_log, h_led, h_buzzer;
volatile uint32_t imu_perdidas, imu_desbordes;

// Se ejecuta cada segundo en la tarea de servicio de timers de FreeRTOS. Si aquí se hiciera
// algo lento, retrasaría a los demás timers: solo avisa a log_task, que es quien hace el trabajo
static void cb_sync(TimerHandle_t t)
{
    xTaskNotifyGive(h_log);
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000));            // da tiempo a abrir el monitor antes de los primeros mensajes
    printf("\n=== Nodo de baches: registrador ===\n");

    // La memoria NVS de la flash: la usarán el WiFi y los contadores de eventos (Fases 4 y 5)
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();                      // NVS de otra versión o llena: empezar de cero
        nvs_flash_init();
    }

    // Lo que comparten las tareas, antes de que ninguna arranque
    estado    = xEventGroupCreate();
    q_imu_log = xQueueCreate(400, sizeof(muestra_t));   // 2 s de muestras de margen si la tarjeta se retrasa
    q_marcas  = xQueueCreate(8, sizeof(int64_t));
    q_aviso   = xQueueCreate(4, sizeof(aviso_t));
    m_gps     = xSemaphoreCreateMutex();
    sb_nmea   = xStreamBufferCreate(1024, 1);  // ~2 s de frases; el 1 = log_task puede leer en cuanto haya 1 byte
    g_gps.lat = g_gps.lon = NAN;                // sin posición todavía

    // Cada módulo. Si uno falla, se sigue con los demás: el LED (apagado) y el monitor lo dicen
    bool imu_ok = imu_iniciar();
    if (imu_ok) xEventGroupSetBits(estado, EV_IMU_OK);
    if (!gps_iniciar()) printf("GPS: no se ha podido configurar el UART\n");
    if (sd_iniciar()) xEventGroupSetBits(estado, EV_SD_OK);
    pulsador_iniciar();

    //                         función      nombre    pila  arg  prio  handle      núcleo
    if (imu_ok)
        xTaskCreatePinnedToCore(imu_task,    "imu",    4096, NULL, 10, &h_imu,    1);
    xTaskCreatePinnedToCore(gps_task,    "gps",    4096, NULL,  5, &h_gps,    0);
    xTaskCreatePinnedToCore(log_task,    "log",    8192, NULL,  4, &h_log,    0);
    xTaskCreatePinnedToCore(buzzer_task, "buzzer", 2048, NULL,  2, &h_buzzer, 0);
    xTaskCreatePinnedToCore(led_task,    "led",    2048, NULL,  1, &h_led,    0);

    // Timer de 1 s que se repite (pdTRUE) y despierta a log_task para guardar y escribir gps.csv
    TimerHandle_t tmr_sync = xTimerCreate("sync", pdMS_TO_TICKS(1000), pdTRUE, NULL, cb_sync);
    xTimerStart(tmr_sync, 0);
}
