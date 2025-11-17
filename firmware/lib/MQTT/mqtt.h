#ifndef MQTT_MODULE_H
#define MQTT_MODULE_H

#include <functional>

/**
 * @brief Initializes the MQTT client.
 *
 * @param host MQTT broker host name or IP address.
 * @param port MQTT broker port.
 * @param client_id Unique client ID for this connection.
 * @param user (New) MQTT username (can be "" if not needed).
 * @param pass (New) MQTT password (can be "" if not needed).
 */
void mqtt_init(const char* host, uint16_t port, const char* client_id, const char* user, const char* pass);

/**
 * @brief Keeps the MQTT connection alive.
 *
 * Must be called regularly. Handles connection,
 * sending/receiving packets, and automatic reconnections.
 */
void mqtt_loop();

void mqtt_connect();

/**
 * @brief Publishes a message to an MQTT topic.
 *
 * @param topic The topic to publish to.
 * @param payload The message to send.
 * @return true on success, false on failure.
 */
bool mqtt_publish(const char* topic, const char* payload);

/**
 * @brief Checks if the client is currently connected to the broker.
 *
 * @return true if connected, false otherwise.
 */
bool mqtt_is_connected();

#endif // MQTT_MODULE_H