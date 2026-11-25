// LED de estado: dice de un vistazo cómo está el nodo, sin tener que mirar el monitor.
//   apagado          error: el IMU no contesta o la tarjeta no graba
//   parpadeo rápido  todo bien, esperando a que el GPS tenga posición
//   encendido fijo   grabando con posición
// (Fase 5: dos destellos cada 2 s = grabando y conectado a MQTT)

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "comun.h"
#include "pines.h"
#include "led.h"

void led_task(void *arg)
{
    gpio_reset_pin(PIN_LED);
    gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);
    bool encendido = false;
    while (1) {
        EventBits_t b = xEventGroupGetBits(estado);
        if (!(b & EV_IMU_OK) || !(b & EV_SD_OK))
            encendido = false;
        else if (b & EV_GPS_FIX)
            encendido = true;
        else
            encendido = !encendido;              // cambia cada 100 ms: parpadeo rápido
        gpio_set_level(PIN_LED, encendido ? LED_ON : !LED_ON);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
