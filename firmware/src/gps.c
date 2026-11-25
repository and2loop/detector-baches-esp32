// GPS NEO-6M por UART.
// gps_task junta los bytes que llegan en líneas (frases NMEA), comprueba cada una,
// actualiza la última posición conocida (g_gps) y pasa el texto en bruto a log_task
// por el stream buffer sb_nmea, que lo guarda en gps_raw.nmea.

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "comun.h"
#include "pines.h"
#include "gps.h"

#define GPS_UART     UART_NUM_1
#define GPS_BAUDIOS  9600     // velocidad de fábrica del NEO-6M
#define MAX_LINEA    100      // una frase NMEA ocupa como mucho 82 caracteres

bool gps_iniciar(void)
{
    uart_config_t c = {
        .baud_rate = GPS_BAUDIOS,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // 2048 bytes de buffer de recepción: unos 2 s de frases por si la tarea se retrasa
    if (uart_driver_install(GPS_UART, 2048, 0, 0, NULL, 0) != ESP_OK) return false;
    if (uart_param_config(GPS_UART, &c) != ESP_OK) return false;
    // Los nombres son desde el ESP32: su TX va al RX del GPS y su RX al TX del GPS
    return uart_set_pin(GPS_UART, PIN_GPS_TX, PIN_GPS_RX,
                        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) == ESP_OK;
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

// Interesan dos frases: RMC (posición, velocidad, hora y fecha) y GGA (calidad y satélites).
// La línea ya viene con la suma de control comprobada. Devuelve true si era una RMC.
static bool nmea_procesar(char *linea, gps_t *g)
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
        return true;
    }
    if (strstr(c[0], "GGA") && n >= 9) {            // $GPGGA o $GNGGA
        g->satelites = atoi(c[7]);
        g->hdop = atof(c[8]);
    }
    return false;
}

// Se llama con cada línea completa que llega del GPS
static void linea_completa(char *linea, int largo)
{
    // Copia en bruto para gps_raw.nmea. Solo se manda si cabe entera: un trozo de frase
    // en el fichero no serviría de nada. Hay un solo escritor (esta tarea), así que
    // el hueco no puede cambiar entre la comprobación y el envío.
    if (xStreamBufferSpacesAvailable(sb_nmea) >= (size_t)largo + 1) {
        xStreamBufferSend(sb_nmea, linea, largo, 0);
        xStreamBufferSend(sb_nmea, "\n", 1, 0);
    }

    if (!nmea_ok(linea)) return;

    static bool primer_fix = true;
    bool fix = false, avisar = false;
    xSemaphoreTake(m_gps, portMAX_DELAY);
    if (nmea_procesar(linea, &g_gps)) {
        // RMC llega una vez por segundo: velocidad (km/h) × 1/3600 h = km recorridos en ese segundo
        if (g_gps.fix) g_gps.km += g_gps.vel_kmh / 3600.0;
    }
    fix = g_gps.fix;
    xSemaphoreGive(m_gps);

    if (fix) {
        xEventGroupSetBits(estado, EV_GPS_FIX);
        if (primer_fix) { primer_fix = false; avisar = true; }
    } else {
        xEventGroupClearBits(estado, EV_GPS_FIX);
    }
    if (avisar) {                                   // un pitido corto: "listo para grabar"
        aviso_t a = AV_LISTO;
        xQueueSend(q_aviso, &a, 0);
    }
}

void gps_task(void *arg)
{
    char linea[MAX_LINEA];
    int largo = 0;
    bool larga = false;                             // la línea no cabía: basura, se descarta
    uint8_t buf[128];
    while (1) {
        int n = uart_read_bytes(GPS_UART, buf, sizeof buf, pdMS_TO_TICKS(100));
        for (int i = 0; i < n; i++) {
            char ch = (char)buf[i];
            if (ch == '\r') continue;
            if (ch != '\n') {
                if (largo < MAX_LINEA - 1) linea[largo++] = ch;
                else larga = true;
                continue;
            }
            linea[largo] = '\0';
            if (largo > 0 && !larga) linea_completa(linea, largo);
            largo = 0;
            larga = false;
        }
    }
}
