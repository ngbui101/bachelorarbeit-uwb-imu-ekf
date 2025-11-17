#include "uwb.h"
#include "imu.h"
#include "wifi_module.h"
#include "mqtt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

TaskHandle_t imuTaskHandle;
TaskHandle_t uwbTaskHandle;
TaskHandle_t mqttTaskHandle;

String inputString = "";
bool stringComplete = false;

#define WIFI_SSID ""
#define WIFI_PASS ""
#define MQTT_BROKER ""
#define MQTT_PORT 1883
#define MQTT_USER ""
#define MQTT_PASS ""
#define CLIENT_ID "esp32-sensor-logger"
const TickType_t xImuPeriod = pdMS_TO_TICKS(20);

void imu_loop(void *pvParameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        imu_handler();
        vTaskDelayUntil(&xLastWakeTime, xImuPeriod);
    }
    vTaskDelete(NULL); 
}
void uwb_loop(void *pvParameters)
{
  for (;;)
  {
    initiator_loop();
  }
  vTaskDelete(NULL);
}
void mqtt_handle(void *pvParameters)
{
  for (;;)
  {
    mqtt_loop();
  }
  vTaskDelete(NULL);
}


void on_wifi_connected()
{
  Serial.println("Main: WiFi is connected.");
}

void on_wifi_disconnected()
{
  Serial.println("Main: WiFi is disconnected.");
}

void setup()
{
  Serial.begin(115200);

  wifi_on_connected(on_wifi_connected);
  wifi_on_disconnected(on_wifi_disconnected);
  wifi_init(WIFI_SSID, WIFI_PASS);
  mqtt_init(MQTT_BROKER, MQTT_PORT, CLIENT_ID, MQTT_USER, MQTT_PASS);
  mqtt_connect();

  start_uwb();
  start_imu();
  xTaskCreatePinnedToCore(imu_loop, "IMU Handler",
                          CONFIG_ARDUINO_LOOP_STACK_SIZE, NULL, 1,
                          &imuTaskHandle, 0);
  // xTaskCreatePinnedToCore(mqtt_handle, "MQTT Handler",
  //                         CONFIG_ARDUINO_LOOP_STACK_SIZE, NULL, 1,
  //                         &mqttTaskHandle, 0);
  // xTaskCreatePinnedToCore(uwb_loop, "UWB Handler",
  //                         CONFIG_ARDUINO_LOOP_STACK_SIZE, NULL, 1,
  //                         &uwbTaskHandle, 1);
}

void loop()
{
  mqtt_loop();
  initiator_loop();
}
