#ifndef WIFI_MODULE_H
#define WIFI_MODULE_H

// Ein einfacher Funktionstyp für unsere Callbacks
typedef void (*wifi_callback_t)(void);

/**
 * @brief Initialisiert das WiFi-Modul und startet den Verbindungsvorgang.
 * Diese Funktion ist nicht-blockierend.
 *
 * @param ssid Die SSID (Name) des WLAN-Netzwerks.
 * @param pass Das Passwort für das WLAN-Netzwerk.
 */
void wifi_init(const char* ssid, const char* pass);

/**
 * @brief Prüft, ob das Gerät aktuell mit dem WLAN verbunden ist.
 *
 * @return true, wenn verbunden, sonst false.
 */
bool wifi_is_connected();

/**
 * @brief Registriert eine Callback-Funktion, die aufgerufen wird,
 * wenn die WLAN-Verbindung (genauer: IP-Adresse) hergestellt wurde.
 */
void wifi_on_connected(wifi_callback_t callback);

/**
 * @brief Registriert eine Callback-Funktion, die aufgerufen wird,
 * wenn die WLAN-Verbindung getrennt wird.
 */
void wifi_on_disconnected(wifi_callback_t callback);

#endif // WIFI_MODULE_H
