// WiFi (zona Wi-Fi del móvil) y MQTT hacia el broker de Azure.
// Todavía no se usa: el envío desde el coche (net_task) llega en la Fase 5.
#pragma once

void red_iniciar(void);        // conecta al WiFi y arranca el cliente MQTT (reconecta solo)
