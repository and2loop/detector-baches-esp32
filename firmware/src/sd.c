// Tarjeta microSD por SPI y log_task.
// Cada arranque crea una carpeta de sesión nueva (/sd/S0001, /sd/S0002...) con:
//   imu_000.csv, imu_001.csv...  las muestras del IMU, un fichero nuevo cada 5 minutos
//   gps.csv                      una línea por segundo con la última posición
//   gps_raw.nmea                 las frases del GPS tal cual llegan (copia en bruto)
//   marcas.csv                   la hora de cada pulsación del pulsador
// Escribir no es guardar: los datos se quedan en memoria hasta el fsync, que se hace
// cada segundo cuando avisa tmr_sync. Si se corta la corriente, se pierde como mucho 1 s.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "comun.h"
#include "pines.h"
#include "sd.h"

#define RAIZ            "/sd"
#define SEG_POR_FICHERO 300      // 5 minutos: si un fichero se daña al cortar la corriente, solo se pierde ese
#define BUF_IMU         8192     // memoria intermedia del CSV del IMU (unos 200 ms de datos)

static bool sd_ok = false;
static char sesion[16];          // "/sd/S0001"
static FILE *f_imu, *f_gps, *f_raw, *f_marcas;
static int n_fichero_imu = 0;

static bool sd_montar(void)
{
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    // 10 MHz en vez de los 20 por defecto: con cables Dupont largos la señal llega peor,
    // y para guardar unos 8 KB por segundo sobra con mucho menos
    host.max_freq_khz = 10000;

    spi_bus_config_t bus = {
        .mosi_io_num = PIN_SD_MOSI, .miso_io_num = PIN_SD_MISO, .sclk_io_num = PIN_SD_SCK,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 4096,
    };
    if (spi_bus_initialize(host.slot, &bus, SDSPI_DEFAULT_DMA) != ESP_OK) return false;

    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = PIN_SD_CS;
    slot.host_id = host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mc = {
        .format_if_mount_failed = false,    // nunca formatear sola: borraría lo que tenga la tarjeta
        .max_files = 8,
        .allocation_unit_size = 16 * 1024,
    };
    sdmmc_card_t *tarjeta;
    esp_err_t r = esp_vfs_fat_sdspi_mount(RAIZ, &host, &slot, &mc, &tarjeta);
    if (r != ESP_OK) {
        printf("SD: no se ha podido montar (%s). ", esp_err_to_name(r));
        printf(r == ESP_FAIL ? "La tarjeta contesta pero no esta en FAT32.\n"
                             : "No contesta: revisa la tarjeta, VCC a 5V, GND, SCK 14, MOSI 11, MISO 13 y CS 10.\n");
        return false;
    }
    sdmmc_card_print_info(stdout, tarjeta);
    return true;
}

// Abre un fichero de la sesión y le pone la cabecera (si la hay)
static FILE *abrir(const char *nombre, const char *cabecera)
{
    char ruta[40];
    snprintf(ruta, sizeof ruta, "%s/%s", sesion, nombre);
    FILE *f = fopen(ruta, "w");
    if (f && cabecera) fputs(cabecera, f);
    return f;
}

static bool abrir_imu(void)
{
    char nombre[16];
    snprintf(nombre, sizeof nombre, "imu_%03d.csv", n_fichero_imu);
    f_imu = abrir(nombre, "t_us,ax,ay,az,gx,gy,gz\n");
    if (!f_imu) return false;
    // Por defecto la librería escribe en la tarjeta en trozos muy pequeños; con un buffer
    // de 8 KB escribe menos veces y más de golpe, que es mucho más rápido en una SD
    setvbuf(f_imu, NULL, _IOFBF, BUF_IMU);
    return true;
}

bool sd_iniciar(void)
{
    if (!sd_montar()) return false;

    // Carpeta de sesión nueva: la primera S0001, S0002... que todavía no exista
    struct stat st;
    int n;
    for (n = 1; n <= 9999; n++) {
        snprintf(sesion, sizeof sesion, RAIZ "/S%04d", n);
        if (stat(sesion, &st) != 0) break;
    }
    if (n > 9999 || mkdir(sesion, 0777) != 0) {
        printf("SD: no se ha podido crear la carpeta de la sesion\n");
        return false;
    }

    f_gps    = abrir("gps.csv", "t_us,hora,fecha,fix,lat,lon,vel_kmh,hdop,sats\n");
    f_marcas = abrir("marcas.csv", "t_us\n");
    f_raw    = abrir("gps_raw.nmea", NULL);
    if (!f_gps || !f_marcas || !f_raw || !abrir_imu()) {
        printf("SD: no se han podido crear los ficheros en %s\n", sesion);
        return false;
    }
    printf("SD: grabando en %s\n", sesion);
    sd_ok = true;
    return true;
}

// Pasa a la tarjeta de verdad lo que haya en memoria. Devuelve false si la tarjeta falla
static bool guardar(FILE *f)
{
    if (fflush(f) != 0) return false;           // de la memoria del programa al sistema de ficheros
    return fsync(fileno(f)) == 0;               // del sistema de ficheros a la tarjeta
}

// -1 si la tarea no se llegó a crear (por ejemplo, el IMU no contestaba al arrancar)
static int pila_libre(TaskHandle_t h)
{
    return h ? (int)uxTaskGetStackHighWaterMark(h) : -1;
}

static void cada_segundo(int seg, int muestras)
{
    // Copia de la última posición: el mutex se suelta enseguida, antes de escribir en la tarjeta
    gps_t g;
    xSemaphoreTake(m_gps, portMAX_DELAY);
    g = g_gps;
    xSemaphoreGive(m_gps);

    int64_t t0 = esp_timer_get_time();
    if (sd_ok) {
        fprintf(f_gps, "%lld,%s,%s,%d,%.6f,%.6f,%.1f,%.2f,%d\n", (long long)esp_timer_get_time(),
                g.hora, g.fecha, g.fix, g.lat, g.lon, g.vel_kmh, g.hdop, g.satelites);
        bool ok = guardar(f_imu) && guardar(f_gps) && guardar(f_raw) && guardar(f_marcas);
        if (!ok) {                               // tarjeta sacada o estropeada: se deja de escribir
            sd_ok = false;
            xEventGroupClearBits(estado, EV_SD_OK);
            printf("SD: error al guardar, se deja de grabar\n");
        } else if (seg % SEG_POR_FICHERO == 0) { // cada 5 minutos, fichero del IMU nuevo
            fclose(f_imu);
            n_fichero_imu++;
            if (!abrir_imu()) {
                sd_ok = false;
                xEventGroupClearBits(estado, EV_SD_OK);
                printf("SD: no se ha podido abrir el fichero %d del IMU\n", n_fichero_imu);
            }
        }
    }
    int ms = (int)((esp_timer_get_time() - t0) / 1000);

    printf("t=%4d s | %3d muestras/s | perdidas %lu | desbordes %lu | %s | GPS %s sat %d HDOP %.1f | guardar %d ms\n",
           seg, muestras, (unsigned long)imu_perdidas, (unsigned long)imu_desbordes,
           sd_ok ? "grabando" : "SIN SD", g.fix ? "fix" : "sin fix", g.satelites, g.hdop, ms);

    // Cada minuto, la memoria de pila que le sobra a cada tarea: si baja de unos 500 bytes, hay que darle más
    if (seg % 60 == 0)
        printf("Pila libre (bytes): imu %d, gps %d, log %d, led %d, buzzer %d\n",
               pila_libre(h_imu), pila_libre(h_gps), pila_libre(h_log), pila_libre(h_led), pila_libre(h_buzzer));
}

void log_task(void *arg)
{
    muestra_t s;
    int64_t t_marca;
    char nmea[256];
    int seg = 0, muestras = 0;
    while (1) {
        // Esperar como mucho 100 ms a que llegue una muestra, y después sacar todas las que haya
        if (xQueueReceive(q_imu_log, &s, pdMS_TO_TICKS(100)) == pdTRUE) {
            do {
                muestras++;
                if (sd_ok)
                    fprintf(f_imu, "%lld,%d,%d,%d,%d,%d,%d\n", (long long)s.t_us,
                            s.v[0], s.v[1], s.v[2], s.v[3], s.v[4], s.v[5]);
            } while (xQueueReceive(q_imu_log, &s, 0) == pdTRUE);
        }

        // Pulsaciones que ha apuntado la ISR del pulsador
        while (xQueueReceive(q_marcas, &t_marca, 0) == pdTRUE) {
            printf("Marca del pulsador en t_us=%lld\n", (long long)t_marca);
            if (sd_ok) fprintf(f_marcas, "%lld\n", (long long)t_marca);
        }

        // Texto NMEA en bruto que ha dejado gps_task
        size_t n;
        while ((n = xStreamBufferReceive(sb_nmea, nmea, sizeof nmea, 0)) > 0)
            if (sd_ok) fwrite(nmea, 1, n, f_raw);

        // ¿Ha avisado tmr_sync? Los avisos se acumulan como un contador: pdFALSE resta solo uno,
        // así que si un fsync lento hace que lleguen dos avisos seguidos, no se pierde ningún segundo.
        // El 0 es "no esperar": si no ha avisado, se sigue con las muestras.
        if (ulTaskNotifyTake(pdFALSE, 0)) {
            seg++;
            cada_segundo(seg, muestras);
            muestras = 0;
        }
    }
}
