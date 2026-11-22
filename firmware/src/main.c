#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "secrets.h"

static EventGroupHandle_t estado;
#define WIFI_OK BIT0
#define MQTT_OK BIT1
static esp_mqtt_client_handle_t cliente;

static void on_wifi(void *arg, esp_event_base_t base, int32_t id, void *datos)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *d = datos;
        printf("WiFi desconectado, motivo %d\n", d->reason);    // DIAGNOSTICO
        xEventGroupClearBits(estado, WIFI_OK);
        vTaskDelay(pdMS_TO_TICKS(1000));                         // no reintentar en bucle loco
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        printf("WiFi conectado: ya tengo IP\n");
        xEventGroupSetBits(estado, WIFI_OK);
    }
}

static void wifi_iniciar(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi, NULL);
    wifi_config_t wc = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS } };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wc);
    esp_wifi_start();
}

static void on_mqtt(void *arg, esp_event_base_t base, int32_t id, void *datos)
{
    esp_mqtt_event_handle_t ev = datos;
    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        printf("MQTT conectado al broker\n");
        xEventGroupSetBits(estado, MQTT_OK);
        break;
    case MQTT_EVENT_DISCONNECTED:
        printf("MQTT desconectado\n");
        xEventGroupClearBits(estado, MQTT_OK);
        break;
    case MQTT_EVENT_PUBLISHED:
        printf("Confirmado el mensaje %d\n", ev->msg_id);
        break;
    case MQTT_EVENT_ERROR:                                        // DIAGNOSTICO
        printf("MQTT error\n");
        if (ev->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED)
            printf("  el broker rechaza la conexion, codigo %d\n",
                   ev->error_handle->connect_return_code);
        else if (ev->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
            printf("  error de red, errno %d\n",
                   ev->error_handle->esp_transport_sock_errno);
        break;
    default:
        break;
    }
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000));            // DIAGNOSTICO: da tiempo a abrir el monitor
    printf("Arrancando nodo1...\n");
    ESP_ERROR_CHECK(nvs_flash_init());
    estado = xEventGroupCreate();
    wifi_iniciar();
    printf("Conectando al WiFi %s...\n", WIFI_SSID);
    xEventGroupWaitBits(estado, WIFI_OK, false, true, portMAX_DELAY);

    esp_mqtt_client_config_t mc = {
        .broker.address.uri = MQTT_URI,
        .credentials.username = MQTT_USER,
        .credentials.client_id = "nodo1",
        .credentials.authentication.password = MQTT_PASS,
    };
    printf("Conectando a %s...\n", MQTT_URI);
    cliente = esp_mqtt_client_init(&mc);
    esp_mqtt_client_register_event(cliente, ESP_EVENT_ANY_ID, on_mqtt, NULL);
    esp_mqtt_client_start(cliente);

    int n = 0;
    char txt[64];
    while (1) {
        if (xEventGroupGetBits(estado) & MQTT_OK) {
            snprintf(txt, sizeof txt, "{\"dev\":\"nodo1\",\"n\":%d}", n++);
            int id = esp_mqtt_client_publish(cliente, "carretera/nodo1/prueba", txt, 0, 1, 0);
            printf("Publicado %s (id %d)\n", txt, id);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}