#include "mqtt.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include "wifi_module.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// --- Internal Module Variables ---
static WiFiClient g_wifi_client;
static PubSubClient g_mqtt_client(g_wifi_client);

static char g_mqtt_host[64];
static uint16_t g_mqtt_port;
static char g_client_id[64];
static char g_mqtt_user[64];
static char g_mqtt_pass[64];
static SemaphoreHandle_t g_mqtt_mutex = NULL;

static unsigned long g_last_reconnect_attempt = 0;

void handleCallback(char *topic, byte *payload, unsigned int length)
{
    char *message = new char[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0';
    Serial.print("Nachricht empfangen [");
    Serial.print(topic);
    Serial.print("]: ");
    Serial.println(message);
    delete[] message;
}
void mqtt_connect()
{
    while (!g_mqtt_client.connected())
    {
        bool connected = false;
        connected = g_mqtt_client.connect(g_client_id, g_mqtt_user, g_mqtt_pass);

        if (connected)
        {
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "Connected to MQTT Broker %s, state: %d", g_mqtt_host, g_mqtt_client.state());
            Serial.println(buffer);
            // _mqttClient.subscribe("/test", _qos);
        }
        else
        {
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "Connection to Broker %s failed, state: %d", g_mqtt_host, g_mqtt_client.state());
            Serial.println(buffer);
            delay(5);
        }
    }
}

// --- Public Functions ---

void mqtt_init(const char *host, uint16_t port, const char *client_id, const char *user, const char *pass)
{
    strncpy(g_mqtt_host, host, sizeof(g_mqtt_host) - 1);
    g_mqtt_port = port;
    strncpy(g_client_id, client_id, sizeof(g_client_id) - 1);
    strncpy(g_mqtt_user, user, sizeof(g_mqtt_user) - 1);
    strncpy(g_mqtt_pass, pass, sizeof(g_mqtt_pass) - 1);

    g_mqtt_client.setServer(g_mqtt_host, g_mqtt_port);
    g_mqtt_client.setBufferSize(2048);
    g_mqtt_client.setCallback(handleCallback);
    g_mqtt_client.setKeepAlive(180);

    if (g_mqtt_mutex == NULL)
    {
        g_mqtt_mutex = xSemaphoreCreateMutex();

        if (g_mqtt_mutex == NULL)
        {
            Serial.println("FEHLER: Konnte MQTT-Mutex nicht erstellen!");
        }
    }
}

void mqtt_loop()
{
    if (!g_mqtt_client.connected())
    {
        mqtt_connect();
        return;
    }
    g_mqtt_client.loop();
}

bool mqtt_publish(const char *topic, const char *payload)
{
    bool publish_result = false;
    if (xSemaphoreTake(g_mqtt_mutex, portMAX_DELAY) == pdTRUE)
    {
        if (g_mqtt_client.connected())
        {
            // Die eigentliche, unsichere Operation
            publish_result = g_mqtt_client.publish(topic, payload);
        }
        else
        {
            publish_result = false;
        }
        xSemaphoreGive(g_mqtt_mutex);
    }
    return publish_result;
}

bool mqtt_is_connected()
{
    return g_mqtt_client.connected();
}
