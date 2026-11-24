// Prueba del IMU (MPU-6050 en un módulo GY-521) por I2C.
// Comprueba que el sensor contesta, lo configura para medir 200 veces por segundo
// y cada segundo cuenta cuántas muestras llegan y si se ha perdido alguna.
// A los 10 minutos imprime un resumen con el resultado.

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#define PIN_SDA     GPIO_NUM_8
#define PIN_SCL     GPIO_NUM_9
#define IMU_DIR     0x68    // dirección del MPU-6050 cuando su pin AD0 está sin conectar
#define MAX_LOTE    32      // como mucho, muestras que se sacan del sensor en cada lectura
#define DURACION_S  600     // 10 minutos

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t imu;

// Si algo impide seguir, lo dice y se queda parado en vez de reiniciarse sin parar
static void parar(const char *motivo)
{
    printf("\n*** PRUEBA PARADA: %s ***\n", motivo);
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}

static void imu_escribir(uint8_t reg, uint8_t valor)
{
    uint8_t b[2] = { reg, valor };
    ESP_ERROR_CHECK(i2c_master_transmit(imu, b, 2, 100));   // 100 ms de tiempo máximo
}

static void imu_leer(uint8_t reg, uint8_t *dest, size_t n)
{
    ESP_ERROR_CHECK(i2c_master_transmit_receive(imu, &reg, 1, dest, n, 100));
}

// Pregunta a todas las direcciones posibles: así se sabe si el sensor está aunque no conteste en 0x68
static void escanear_bus(void)
{
    esp_log_level_set("i2c.master", ESP_LOG_NONE);   // el driver avisaría por cada dirección vacía
    printf("Direcciones que contestan en el bus I2C:");
    int encontradas = 0;
    for (uint16_t d = 0x08; d < 0x78; d++) {
        if (i2c_master_probe(bus, d, 50) == ESP_OK) {
            printf(" 0x%02X", (unsigned)d);
            encontradas++;
        }
    }
    printf("%s\n", encontradas ? "" : " ninguna");
    esp_log_level_set("i2c.master", ESP_LOG_WARN);
}

static void i2c_iniciar(void)
{
    i2c_master_bus_config_t bc = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,   // el GY-521 ya trae las suyas; estas no molestan
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bc, &bus));
    escanear_bus();
    if (i2c_master_probe(bus, IMU_DIR, 100) != ESP_OK)
        parar("nadie contesta en 0x68: revisa VCC, GND, SDA, SCL y las soldaduras");

    i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_DIR,
        .scl_speed_hz = 400000,                 // modo rápido de I2C: sobra para 200 muestras/s
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dc, &imu));
}

static void imu_vaciar_fifo(void)
{
    imu_escribir(0x6A, 0x04);   // borra lo que haya guardado
    imu_escribir(0x6A, 0x40);   // y vuelve a activar la memoria FIFO
}

static void imu_configurar(void)
{
    // Reinicio completo del sensor: el botón RST solo reinicia el ESP32, y el sensor
    // seguía midiendo con la configuración anterior y la FIFO llena
    imu_escribir(0x6B, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));
    imu_escribir(0x6B, 0x01);   // despertarlo con el reloj del giroscopio, más estable
    vTaskDelay(pdMS_TO_TICKS(100));
    imu_escribir(0x1A, 0x02);   // filtro interno a unos 94 Hz: quita vibraciones más rápidas de lo que se mide
    imu_escribir(0x19, 4);      // 1000 Hz / (1 + 4) = 200 muestras por segundo
    imu_escribir(0x1C, 0x10);   // rango ±8 g: un bache fuerte no satura la medida (4096 = 1 g)
    imu_escribir(0x1B, 0x08);   // rango ±500 grados/s (65,5 = 1 grado/s)
    imu_escribir(0x23, 0x78);   // guardar en la FIFO acelerómetro y giroscopio: 12 bytes por muestra
    imu_vaciar_fifo();
    uint8_t s;
    imu_leer(0x3A, &s, 1);      // leer este registro borra sus avisos: así no cuenta un desborde de antes de empezar
}
// El sensor guarda las muestras en su FIFO a ritmo exacto; aquí se recogen todas de golpe.
// Devuelve cuántas ha leído.
static int imu_leer_fifo(int16_t m[][6])
{
    uint8_t c[2];
    imu_leer(0x72, c, 2);                       // cuántos bytes hay guardados
    int n = ((c[0] << 8) | c[1]) / 12;          // muestras completas
    if (n > MAX_LOTE) n = MAX_LOTE;             // las que sobren se leen en la vuelta siguiente
    if (n == 0) return 0;
    uint8_t buf[MAX_LOTE * 12];
    imu_leer(0x74, buf, n * 12);
    for (int i = 0; i < n; i++)
        for (int k = 0; k < 6; k++)             // ax, ay, az, gx, gy, gz; byte alto primero
            m[i][k] = (int16_t)((buf[i * 12 + 2 * k] << 8) | buf[i * 12 + 2 * k + 1]);
    return n;
}

// Si la FIFO se llena antes de leerla, el sensor tira datos y lo marca en este registro
static bool imu_fifo_desbordada(void)
{
    uint8_t s;
    imu_leer(0x3A, &s, 1);
    return s & 0x10;
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000));            // da tiempo a abrir el monitor antes de los primeros mensajes
    printf("\n=== Prueba del IMU ===\n");
    i2c_iniciar();

    uint8_t quien;
    imu_leer(0x75, &quien, 1);
    printf("WHO_AM_I = 0x%02X %s\n", quien,
           quien == 0x68 ? "(MPU-6050 original)" : "(otro chip o un clon: sigue si las medidas tienen sentido)");
    imu_configurar();

    int16_t m[MAX_LOTE][6];
    int16_t ult[6] = { 0 };
    int vueltas = 0, en_este_segundo = 0, segundos = 0;
    int min_s = 100000, max_s = 0;
    long total_util = 0;                        // muestras sin contar el primer segundo
    unsigned long desbordes = 0;
    TickType_t ultimo = xTaskGetTickCount();

    while (1) {
        // Leer cada 20 ms: la FIFO se llena en unos 0,4 s, así que hay margen de sobra
        xTaskDelayUntil(&ultimo, pdMS_TO_TICKS(20));
        if (imu_fifo_desbordada()) {
            desbordes++;
            imu_vaciar_fifo();
        }
        int n = imu_leer_fifo(m);
        if (n > 0) memcpy(ult, m[n - 1], sizeof ult);
        en_este_segundo += n;

        if (++vueltas < 50) continue;           // 50 vueltas de 20 ms = 1 segundo exacto
        vueltas = 0;
        segundos++;
        if (segundos > 1) {                     // el primer segundo no cuenta: el sensor acaba de arrancar
            total_util += en_este_segundo;
            if (en_este_segundo < min_s) min_s = en_este_segundo;
            if (en_este_segundo > max_s) max_s = en_este_segundo;
        }

        float ax = ult[0] / 4096.0f, ay = ult[1] / 4096.0f, az = ult[2] / 4096.0f;
        float gx = ult[3] / 65.5f,   gy = ult[4] / 65.5f,   gz = ult[5] / 65.5f;
        float modulo = sqrtf(ax * ax + ay * ay + az * az);
        printf("t=%3d s | %3d muestras/s | desbordes %lu | a = %+.2f %+.2f %+.2f g |a|=%.2f | giro = %+5.1f %+5.1f %+5.1f dps\n",
               segundos, en_este_segundo, desbordes, ax, ay, az, modulo, gx, gy, gz);
        en_este_segundo = 0;

        if (segundos == DURACION_S) {
            float media = (float)total_util / (DURACION_S - 1);
            bool ok = desbordes == 0 && media >= 198.0f && media <= 202.0f;
            printf("\n=== 10 minutos: media %.1f muestras/s (entre %d y %d cada segundo), %lu desbordes -> %s ===\n\n",
                   media, min_s, max_s, desbordes, ok ? "PRUEBA SUPERADA" : "NO SUPERADA");
        }
    }
}
