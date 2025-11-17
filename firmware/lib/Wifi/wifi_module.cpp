#include "wifi_module.h"
#include <Arduino.h>
#include <WiFi.h> // Verwendet die Standard-Arduino-WiFi-Bibliothek

// --- Interne (statische) Variablen ---
static wifi_callback_t g_on_connected_cb = NULL;
static wifi_callback_t g_on_disconnected_cb = NULL;

// Speichert die Anmeldedaten für die automatische Wiederverbindung
static char g_wifi_ssid[64];
static char g_wifi_pass[64];

/**
 * @brief Der zentrale Event-Handler für alle WiFi-Ereignisse.
 * Wird von der Arduino WiFi-Bibliothek aufgerufen.
 */
static void wifi_event_handler(WiFiEvent_t event, WiFiEventInfo_t info) {
    switch(event) {
        // ARDUINO_EVENT_... ist der neue Name für SYSTEM_EVENT_...
        case ARDUINO_EVENT_WIFI_STA_START:
            Serial.println("WiFi: Station-Modus gestartet. Verbinde...");
            // Der Verbindungsversuch (WiFi.begin) wurde bereits in wifi_init() gestartet
            break;

        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            Serial.print("WiFi: IP-Adresse erhalten: ");
            // KORREKTUR FÜR FEHLER 1:
            // Verwendet die Arduino IPAddress-Klasse, um die IP zu drucken.
            Serial.println(IPAddress(info.got_ip.ip_info.ip.addr));
            
            // Callback aufrufen, falls registriert
            if (g_on_connected_cb) {
                g_on_connected_cb();
            }
            break;

        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            Serial.println("WiFi: Verbindung getrennt. Versuche Neuverbindung...");
            
            if (g_on_disconnected_cb) {
                g_on_disconnected_cb();
            }
            // Das Arduino-Framework versucht normalerweise, sich selbst
            // wieder zu verbinden. Ein expliziter Aufruf kann helfen.
            WiFi.begin(g_wifi_ssid, g_wifi_pass);
            break;

        default:
            // Andere Ereignisse ignorieren
            break;
    }
}

// --- Öffentliche Funktionen (aus wifi_module.h) ---

void wifi_init(const char* ssid, const char* pass) {
    // Anmeldedaten für spätere Wiederverbindungen speichern
    strncpy(g_wifi_ssid, ssid, sizeof(g_wifi_ssid) - 1);
    strncpy(g_wifi_pass, pass, sizeof(g_wifi_pass) - 1);
    
    // KORREKTUR FÜR FEHLER 2:
    // Entfernt alle Low-Level ESP-IDF-Aufrufe (esp_wifi_init_once, etc.)
    
    // Registriert unseren Event-Handler bei der Arduino-Bibliothek
    WiFi.onEvent(wifi_event_handler);
    
    // Setzt den Modus (dies initialisiert das WiFi-System sicher)
    WiFi.mode(WIFI_STA);
    
    // Startet den (nicht-blockierenden) Verbindungsversuch
    WiFi.begin(g_wifi_ssid, g_wifi_pass);
}

bool wifi_is_connected() {
    return (WiFi.status() == WL_CONNECTED);
}

void wifi_on_connected(wifi_callback_t callback) {
    g_on_connected_cb = callback;
}

void wifi_on_disconnected(wifi_callback_t callback) {
    g_on_disconnected_cb = callback;
}

