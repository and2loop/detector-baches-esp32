// Buzzer: duerme hasta que llega un aviso por q_aviso y toca su patrón.
// Solo confirma cosas buenas (GPS listo, golpe detectado); los errores los indica el LED.
// Se silencia poniendo BUZZER_ACTIVO a 0 en pines.h.

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "comun.h"
#include "pines.h"
#include "buzzer.h"

static void pitar(int ms)
{
    gpio_set_level(PIN_BUZZER, 1);  vTaskDelay(pdMS_TO_TICKS(ms));
    gpio_set_level(PIN_BUZZER, 0);  vTaskDelay(pdMS_TO_TICKS(120));    // silencio entre pitidos
}

void buzzer_task(void *arg)
{
    gpio_reset_pin(PIN_BUZZER);
    gpio_set_direction(PIN_BUZZER, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_BUZZER, 0);
    aviso_t a;
    while (1) {
        xQueueReceive(q_aviso, &a, portMAX_DELAY);      // dormida hasta que llegue un aviso
        if (!BUZZER_ACTIVO) continue;
        switch (a) {
        case AV_BACHE: pitar(80); pitar(80); break;     // dos cortos
        case AV_BADEN: pitar(300);           break;     // uno largo
        default:       pitar(80);            break;     // listo, candidato, otro: uno corto
        }
    }
}
