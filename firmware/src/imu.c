// IMU MPU-6050 (módulo GY-521) por I2C.
// El sensor mide solo, a ritmo exacto (200 veces por segundo), y guarda las muestras en su FIFO.
// imu_task las recoge cada 20 ms, les pone hora y las manda a la tarjeta por la cola q_imu_log.

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "comun.h"
#include "pines.h"
#include "imu.h"

#define IMU_DIR      0x68    // dirección del MPU-6050 cuando su pin AD0 está sin conectar
#define MAX_LOTE     32      // como mucho, muestras que se sacan del sensor en cada lectura
#define PERIODO_US   5000    // 200 muestras por segundo = una cada 5 ms

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t imu;

// En la tarea no se usa ESP_ERROR_CHECK: un fallo suelto del bus reiniciaría la placa
// y cortaría la grabación. Se devuelve si ha ido bien y quien llama decide.
static bool imu_escribir(uint8_t reg, uint8_t valor)
{
    uint8_t b[2] = { reg, valor };
    return i2c_master_transmit(imu, b, 2, 100) == ESP_OK;   // 100 ms de tiempo máximo
}

static bool imu_leer(uint8_t reg, uint8_t *dest, size_t n)
{
    return i2c_master_transmit_receive(imu, &reg, 1, dest, n, 100) == ESP_OK;
}

static void imu_vaciar_fifo(void)
{
    imu_escribir(0x6A, 0x04);   // borra lo que haya guardado
    imu_escribir(0x6A, 0x40);   // y vuelve a activar la FIFO
}

static bool imu_configurar(void)
{
    // Reinicio completo del sensor: el botón RST solo reinicia el ESP32, y el sensor
    // seguía midiendo con la configuración anterior y la FIFO llena
    if (!imu_escribir(0x6B, 0x80)) return false;
    vTaskDelay(pdMS_TO_TICKS(100));
    bool ok = imu_escribir(0x6B, 0x01);   // despertarlo con el reloj del giroscopio, más estable
    vTaskDelay(pdMS_TO_TICKS(100));
    ok = ok && imu_escribir(0x1A, 0x02);  // filtro interno a unos 94 Hz: quita vibraciones más rápidas de lo que se mide
    ok = ok && imu_escribir(0x19, 4);     // 1000 Hz / (1 + 4) = 200 muestras por segundo
    ok = ok && imu_escribir(0x1C, 0x10);  // rango ±8 g: un bache fuerte no satura la medida (4096 = 1 g)
    ok = ok && imu_escribir(0x1B, 0x08);  // rango ±500 grados/s (65,5 = 1 grado/s)
    ok = ok && imu_escribir(0x23, 0x78);  // guardar en la FIFO acelerómetro y giroscopio: 12 bytes por muestra
    imu_vaciar_fifo();
    uint8_t s;
    imu_leer(0x3A, &s, 1);                // leer este registro borra sus avisos: así no cuenta un desborde de antes de empezar
    return ok;
}

bool imu_iniciar(void)
{
    i2c_master_bus_config_t bc = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,   // el GY-521 ya trae las suyas; estas no molestan
    };
    if (i2c_new_master_bus(&bc, &bus) != ESP_OK) return false;

    esp_log_level_set("i2c.master", ESP_LOG_NONE);   // si no contesta, el driver llenaría el monitor de avisos
    bool hay = i2c_master_probe(bus, IMU_DIR, 100) == ESP_OK;
    esp_log_level_set("i2c.master", ESP_LOG_WARN);
    if (!hay) {
        printf("IMU: nadie contesta en 0x68 (revisa VCC, GND, SDA %d y SCL %d)\n", PIN_SDA, PIN_SCL);
        return false;
    }

    i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_DIR,
        .scl_speed_hz = 400000,                 // modo rápido de I2C: sobra para 200 muestras/s
    };
    if (i2c_master_bus_add_device(bus, &dc, &imu) != ESP_OK) return false;

    uint8_t quien = 0;
    imu_leer(0x75, &quien, 1);
    printf("IMU: WHO_AM_I = 0x%02X\n", quien);
    return imu_configurar();
}

// Devuelve cuántas muestras ha leído de la FIFO (0 si no había ninguna o falló la lectura)
static int imu_leer_fifo(int16_t m[][6])
{
    uint8_t c[2];
    if (!imu_leer(0x72, c, 2)) return 0;        // cuántos bytes hay guardados
    int n = ((c[0] << 8) | c[1]) / 12;          // muestras completas
    if (n > MAX_LOTE) n = MAX_LOTE;             // las que sobren se leen en la vuelta siguiente
    if (n == 0) return 0;
    uint8_t buf[MAX_LOTE * 12];
    if (!imu_leer(0x74, buf, n * 12)) return 0;
    for (int i = 0; i < n; i++)
        for (int k = 0; k < 6; k++)             // ax, ay, az, gx, gy, gz; byte alto primero
            m[i][k] = (int16_t)((buf[i * 12 + 2 * k] << 8) | buf[i * 12 + 2 * k + 1]);
    return n;
}

// Si la FIFO se llena antes de leerla, el sensor tira datos y lo marca en este registro
static bool imu_fifo_desbordada(void)
{
    uint8_t s = 0;
    imu_leer(0x3A, &s, 1);
    return s & 0x10;
}

void imu_task(void *arg)
{
    int16_t m[MAX_LOTE][6];
    TickType_t ultimo = xTaskGetTickCount();
    while (1) {
        // Despertar cada 20 ms exactos: la FIFO se llena en unos 0,4 s, así que hay margen de sobra
        xTaskDelayUntil(&ultimo, pdMS_TO_TICKS(20));
        if (imu_fifo_desbordada()) {
            imu_desbordes++;
            imu_vaciar_fifo();
            continue;                           // lo que hubiera ya no es continuo: se descarta
        }
        int n = imu_leer_fifo(m);
        // La última muestra del lote acaba de llegar; las anteriores van 5 ms por detrás cada una
        int64_t ahora = esp_timer_get_time();
        for (int i = 0; i < n; i++) {
            muestra_t s = { .t_us = ahora - (int64_t)(n - 1 - i) * PERIODO_US };
            memcpy(s.v, m[i], sizeof s.v);
            // 0 = no esperar nunca: si la tarjeta va lenta, se cuenta la pérdida
            // en vez de retrasar la lectura del sensor
            if (xQueueSend(q_imu_log, &s, 0) != pdTRUE) imu_perdidas++;
            // Fase 4: xQueueSend(q_imu_det, &s, 0);
        }
    }
}
