// Prueba del GPS (módulo GY-GPS6MV2, chip u-blox NEO-6M) por UART.
// Los primeros segundos enseña las frases NMEA tal cual llegan, para ver que el módulo habla.
// Después, cada segundo, resume lo que ha entendido: si hay posición (fix), dónde,
// la calidad de la posición (HDOP), cuántos satélites usa y la hora UTC.
// Mientras no hay fix, también dice cuántos satélites ve y con qué fuerza le llegan,
// para saber si el sitio tiene señal suficiente.

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define GPS_UART     UART_NUM_1
// Pines solo para esta prueba: se han reaprovechado los cables del IMU, que estaban en 8 y 9.
// En el montaje final el GPS va en 17 y 18, como dice pines.h.
#define GPS_PIN_RX   GPIO_NUM_8   // el ESP32 recibe aquí: va el TX del GPS
#define GPS_PIN_TX   GPIO_NUM_9   // el ESP32 envía por aquí: va el RX del GPS (en esta prueba no se le envía nada)
#define GPS_BAUDIOS  9600     // velocidad de fábrica del NEO-6M
#define CRUDO_S      5        // segundos en los que se imprimen las frases tal cual llegan
#define MAX_LINEA    100      // una frase NMEA ocupa como mucho 82 caracteres

typedef struct {
    bool   fix;
    double lat, lon;
    float  vel_kmh, hdop;
    int    satelites;            // los que usa para calcular la posición
    int    a_la_vista;           // los que dice que hay en el cielo (aunque no los oiga)
    int    con_senal;            // los que oye de verdad (fuerza mayor que 0)
    int    fuertes;              // los que oye con 30 dB o más
    int    mejor_snr;            // el que mejor se oye, en dB
    char   hora[11], fecha[7];   // "hhmmss.ss" y "ddmmaa"
    int64_t t_us;                // esp_timer_get_time() cuando llegó la última frase RMC
} gps_t;

static gps_t g_gps;

#define SNR_BUENA 30   // a partir de aquí la señal da para descargar la órbita del satélite sin cortes

static void gps_iniciar(void)
{
    uart_config_t c = {
        .baud_rate = GPS_BAUDIOS,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // 2048 bytes de buffer de recepción: unos 2 s de frases, de sobra aunque el printf tarde
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART, &c));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART, GPS_PIN_TX, GPS_PIN_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

// Comprueba la suma de control: XOR de los caracteres entre '$' y '*'
static bool nmea_ok(const char *s)
{
    const char *p = strchr(s, '$'), *ast = strchr(s, '*');
    if (!p || !ast) return false;
    uint8_t x = 0;
    for (p++; p < ast; p++) x ^= (uint8_t)*p;
    return x == (uint8_t)strtol(ast + 1, NULL, 16);
}

// Parte la línea por comas SIN saltarse los campos vacíos (strtok sí se los salta,
// y sin fix casi todos los campos vienen vacíos)
static int nmea_campos(char *s, char *c[], int max)
{
    int n = 0;
    c[n++] = s;
    for (char *p = s; *p && n < max; p++)
        if (*p == ',' || *p == '*') { *p = '\0'; c[n++] = p + 1; }
    return n;
}

// "3959.6470","N" -> 39.994117   (el formato NMEA es gggmm.mmmm: grados y minutos juntos)
static double nmea_grados(const char *v, const char *h)
{
    if (!*v) return NAN;
    double x = atof(v);
    int g = (int)(x / 100);
    double r = g + (x - g * 100) / 60.0;
    return (*h == 'S' || *h == 'W') ? -r : r;
}

// Interesan tres frases: RMC (posición, velocidad, hora y fecha), GGA (calidad y satélites usados)
// y GSV (satélites a la vista y fuerza de cada uno). GSV viene partida en varias frases
// seguidas, con 4 satélites cada una, así que se va sumando y se guarda al llegar la última.
// La línea ya tiene que venir con la suma de control comprobada.
static void nmea_procesar(char *linea, gps_t *g)
{
    char *c[24];
    int n = nmea_campos(linea, c, 24);
    if (strstr(c[0], "RMC") && n >= 10) {           // $GPRMC o $GNRMC
        g->fix = (c[2][0] == 'A');
        g->lat = nmea_grados(c[3], c[4]);
        g->lon = nmea_grados(c[5], c[6]);
        g->vel_kmh = atof(c[7]) * 1.852f;           // nudos -> km/h
        strncpy(g->hora, c[1], sizeof g->hora - 1);
        strncpy(g->fecha, c[9], sizeof g->fecha - 1);
        g->t_us = esp_timer_get_time();
    } else if (strstr(c[0], "GGA") && n >= 9) {     // $GPGGA o $GNGGA
        g->satelites = atoi(c[7]);
        g->hdop = atof(c[8]);
    } else if (strstr(c[0], "GSV") && n >= 4) {     // $GPGSV
        static int con_senal, fuertes, mejor;
        int total = atoi(c[1]), num = atoi(c[2]);
        if (num == 1) con_senal = fuertes = mejor = 0;
        // Cada satélite son 4 campos: número, elevación, azimut y fuerza; el último campo es la suma de control
        for (int i = 4; i + 3 < n - 1; i += 4) {
            int snr = atoi(c[i + 3]);               // vacío = no lo oye
            if (snr > 0) con_senal++;
            if (snr >= SNR_BUENA) fuertes++;
            if (snr > mejor) mejor = snr;
        }
        if (num == total) {
            g->a_la_vista = atoi(c[3]);
            g->con_senal = con_senal;
            g->fuertes = fuertes;
            g->mejor_snr = mejor;
        }
    }
}

// "hhmmss.ss" -> "hh:mm:ss", o "--:--:--" si el GPS todavía no sabe la hora
static void formatear_hora(const char *h, char *out, size_t tam)
{
    if (strlen(h) >= 6) snprintf(out, tam, "%.2s:%.2s:%.2s", h, h + 2, h + 4);
    else snprintf(out, tam, "--:--:--");
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000));            // da tiempo a abrir el monitor antes de los primeros mensajes
    printf("\n=== Prueba del GPS ===\n");
    printf("UART1 a %d baudios. Cables: TX del GPS -> GPIO%d, RX del GPS -> GPIO%d\n",
           GPS_BAUDIOS, GPS_PIN_RX, GPS_PIN_TX);
    gps_iniciar();
    printf("Frases tal cual durante %d s:\n", CRUDO_S);

    char linea[MAX_LINEA];
    int largo = 0;
    bool larga = false;                         // la línea no cabía: basura o velocidad equivocada
    uint8_t buf[256];
    unsigned long bytes = 0, frases_ok = 0, frases_mal = 0;
    bool superada = false;
    int64_t inicio = esp_timer_get_time();
    int64_t siguiente = inicio + 1000000;       // próximo resumen, una vez por segundo

    while (1) {
        int n = uart_read_bytes(GPS_UART, buf, sizeof buf, pdMS_TO_TICKS(100));
        if (n > 0) bytes += n;

        // Juntar los bytes en líneas: cada frase acaba en "\r\n"
        for (int i = 0; i < n; i++) {
            char ch = (char)buf[i];
            if (ch == '\r') continue;
            if (ch != '\n') {
                if (largo < MAX_LINEA - 1) linea[largo++] = ch;
                else larga = true;
                continue;
            }
            linea[largo] = '\0';
            if (largo > 0) {
                if (esp_timer_get_time() - inicio < CRUDO_S * 1000000LL)
                    printf("  %s\n", linea);
                if (!larga && nmea_ok(linea)) {
                    frases_ok++;
                    nmea_procesar(linea, &g_gps);
                } else {
                    frases_mal++;
                }
            }
            largo = 0;
            larga = false;
        }

        int64_t ahora = esp_timer_get_time();
        if (ahora < siguiente) continue;
        siguiente += 1000000;
        int t = (int)((ahora - inicio) / 1000000);

        if (bytes == 0) {
            printf("t=%3d s | no llega nada: revisa que el TX del GPS vaya al GPIO%d, GND y VCC\n",
                   t, GPS_PIN_RX);
            continue;
        }
        if (frases_ok == 0) {
            printf("t=%3d s | llegan %lu bytes pero ninguna frase valida: ¿velocidad distinta de %d?\n",
                   t, bytes, GPS_BAUDIOS);
            continue;
        }

        char hora[12];
        formatear_hora(g_gps.hora, hora, sizeof hora);
        if (!g_gps.fix) {
            printf("t=%3d s | sin fix | usa %2d | ve %2d, oye %2d, fuertes (>= %d dB) %2d, el mejor %2d dB | hora UTC %s | frases ok %lu, mal %lu\n",
                   t, g_gps.satelites, g_gps.a_la_vista, g_gps.con_senal, SNR_BUENA, g_gps.fuertes,
                   g_gps.mejor_snr, hora, frases_ok, frases_mal);
            continue;
        }

        printf("t=%3d s | FIX | %.6f, %.6f | %5.1f km/h | HDOP %.2f | satelites %2d | UTC %s %.2s/%.2s/%.2s\n",
               t, g_gps.lat, g_gps.lon, g_gps.vel_kmh, g_gps.hdop, g_gps.satelites,
               hora, g_gps.fecha, g_gps.fecha + 2, g_gps.fecha + 4);

        // Lo que se pide para dar el GPS por bueno: posición válida, HDOP < 2 y hora UTC
        if (!superada && g_gps.hdop > 0 && g_gps.hdop < 2.0f && strlen(g_gps.hora) >= 6) {
            superada = true;
            printf("\n=== PRUEBA SUPERADA: fix a los %d s, HDOP %.2f, %d satelites, hora UTC %s"
                   " (en Espana ahora es una hora mas) ===\n\n", t, g_gps.hdop, g_gps.satelites, hora);
        }
    }
}
