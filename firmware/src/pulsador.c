// Pulsador por interrupción (ISR).
// log_task puede pasar un buen rato escribiendo en la tarjeta: si mirase el pin en cada vuelta,
// la hora de la marca llegaría tarde (a 50 km/h, 300 ms son 4 m) y una pulsación corta podría
// perderse. La ISR se ejecuta en el mismo instante de pulsar, apunta la hora y se la pasa a log_task.

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "comun.h"
#include "pines.h"
#include "pulsador.h"

#define REBOTE_US 200000     // un pulsador "rebota" unos ms al pulsarlo: lo que llegue en menos de 200 ms se ignora

// IRAM_ATTR: la función va en la RAM interna, para que se pueda ejecutar aunque
// en ese momento se esté escribiendo en la flash
static void IRAM_ATTR isr_pulsador(void *arg)
{
    static int64_t ultima = 0;
    int64_t t = esp_timer_get_time();
    if (t - ultima < REBOTE_US) return;
    ultima = t;
    BaseType_t despertar = pdFALSE;
    // Una ISR no puede esperar, ni imprimir, ni tocar la tarjeta: solo apunta la hora y la pasa
    xQueueSendFromISR(q_marcas, &t, &despertar);
    if (despertar) portYIELD_FROM_ISR();    // si eso despierta a una tarea más importante, cambiar ya a ella
}

void pulsador_iniciar(void)
{
    gpio_config_t c = {
        .pin_bit_mask = 1ULL << PIN_PULSADOR,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,         // suelto = 1, pulsado = 0 (el pulsador va a GND)
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,           // avisar al pasar de 1 a 0, es decir, al pulsar
    };
    gpio_config(&c);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_PULSADOR, isr_pulsador, NULL);
}
