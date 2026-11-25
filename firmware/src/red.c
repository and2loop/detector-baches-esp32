// WiFi y MQTT: el código de la prueba que publicaba en carretera/nodo1/prueba,
// separado en su propio fichero. Nadie lo llama todavía: en la Fase 5 lo usará net_task.

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "mqtt_client.h"
#include "comun.h"
#include "secrets.h"
#include "red.h"

static esp_mqtt_client_handle_t cliente;

// ESP-IDF avisa de lo que pasa con el WiFi mediante eventos; esta función los atiende
static void on_wifi(void *arg, esp_event_base_t base, int32_t id, void *datos)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *d = datos;
        printf("WiFi desconectado, motivo %d\n", d->reason);
        xEventGroupClearBits(estado, EV_WIFI_OK);
        // Pendiente para la Fase 5: reintentar desde net_task con una espera, no desde aquí
        // (un vTaskDelay dentro de este gestor bloquearía todos los demás eventos)
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        printf("WiFi conectado: ya tengo IP\n");
        xEventGroupSetBits(estado, EV_WIFI_OK);
    }
}

static void on_mqtt(void *arg, esp_event_base_t base, int32_t id, void *datos)
{
    esp_mqtt_event_handle_t ev = datos;
    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        printf("MQTT conectado al broker\n");
        xEventGroupSetBits(estado, EV_MQTT_OK);
        break;
    case MQTT_EVENT_DISCONNECTED:
        printf("MQTT desconectado\n");
        xEventGroupClearBits(estado, EV_MQTT_OK);
        break;
    case MQTT_EVENT_PUBLISHED:                       // el broker confirmó un mensaje QoS 1
        printf("Confirmado el mensaje %d\n", ev->msg_id);
        break;
    case MQTT_EVENT_ERROR:
        if (ev->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED)
            printf("MQTT: el broker rechaza la conexion, codigo %d\n", ev->error_handle->connect_return_code);
        else if (ev->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
            printf("MQTT: error de red, errno %d\n", ev->error_handle->esp_transport_sock_errno);
        break;
    default:
        break;
    }
}

void red_iniciar(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi, NULL);
    wifi_config_t wc = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASS } };
    esp_wifi_set_mode(WIFI_MODE_STA);                // modo "cliente" de una red
    esp_wifi_set_config(WIFI_IF_STA, &wc);
    esp_wifi_start();

    esp_mqtt_client_config_t mc = {
        .broker.address.uri = MQTT_URI,
        .credentials.username = MQTT_USER,
        .credentials.client_id = "nodo1",
        .credentials.authentication.password = MQTT_PASS,
    };
    cliente = esp_mqtt_client_init(&mc);
    esp_mqtt_client_register_event(cliente, ESP_EVENT_ANY_ID, on_mqtt, NULL);
    esp_mqtt_client_start(cliente);                  // conecta y reconecta solo
}
