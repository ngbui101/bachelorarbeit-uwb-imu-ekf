import paho.mqtt.client as mqtt
import csv
import os
import sys

# --- MQTT Konfiguration ---
MQTT_BROKER = ""
MQTT_PORT = 1883
MQTT_USER = ""
MQTT_PASS = ""

# Topics
UWB_TOPIC = "uwb/data"
IMU_TOPIC = "imu/data"

# --- CSV Konfiguration ---
UWB_CSV_FILE = 'uwb_data_1.csv'
IMU_CSV_FILE = 'imu_data_1.csv'
UWB_HEADERS = ['timestamp_ns', 'mac_address', 'distance']
IMU_HEADERS = ['timestamp_ns', 'qw', 'qx', 'qy', 'qz', 'ax', 'ay', 'az']


def check_and_write_headers():
    """Überprüft, ob die CSV-Dateien existieren, und schreibt die Header, falls nicht."""
    try:
        with open(UWB_CSV_FILE, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(UWB_HEADERS)
        print(f"Header in {UWB_CSV_FILE} geschrieben.")
    except IOError as e:
        print(f"FEHLER: Konnte Header nicht in {UWB_CSV_FILE} schreiben: {e}")
        sys.exit(1)
            
    try:
        with open(IMU_CSV_FILE, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(IMU_HEADERS)
        print(f"Header in {IMU_CSV_FILE} geschrieben.")
    except IOError as e:
        print(f"FEHLER: Konnte Header nicht in {IMU_CSV_FILE} schreiben: {e}")
        sys.exit(1)

def parse_uwb_data(payload_str):
    """
    Parst den UWB-Payload (Format: "timestamp;mac,dist;mac,dist;...")
    und wandelt ihn in mehrere CSV-Zeilen um.
    """
    parts = payload_str.strip().split(';')
    if len(parts) < 2:
        return []
        
    timestamp = parts[0]
    rows_to_write = []
    
    for device_data in parts[1:]:
        if not device_data: 
            continue
        try:
            mac, dist = device_data.split(',')
            rows_to_write.append([timestamp, mac, dist])
        except ValueError:
            print(f"Überspringe fehlerhafte UWB-Daten: {device_data}")
            
    return rows_to_write

def parse_imu_data(payload_str):
    """
    Parst den IMU-Payload basierend auf dem C-Code-Format: 
    "timestamp;ax,ay,az;qw,qx,qy,qz"
    und ordnet ihn entsprechend der IMU_HEADERS an.
    """
    parts = payload_str.strip().split(';')
    
    if len(parts) != 3: 
        print(f"Überspringe fehlerhafte IMU-Daten (falsche Anzahl an Teilen): {payload_str}")
        return None
        
    timestamp = parts[0]
    try:
        group1_values = parts[1].split(',')
        group2_values = parts[2].split(',')

        if len(group1_values) != 3 or len(group2_values) != 4:
            print(f"Überspringe fehlerhafte IMU-Daten (falsche Anzahl an Werten): {payload_str}")
            return None
            
        # Gruppe 1: ax, ay, az
        ax = group1_values[0]
        ay = group1_values[1]
        az = group1_values[2]
        
        # Gruppe 2: qw, qx, qy, qz
        qw = group2_values[0]
        qx = group2_values[1]
        qy = group2_values[2]
        qz = group2_values[3]
            
        # Die Daten in der Reihenfolge zurückgeben, die in IMU_HEADERS definiert ist:
        # ['timestamp_ns', 'qw', 'qx', 'qy', 'qz', 'ax', 'ay', 'az']
        return [timestamp, qw, qx, qy, qz, ax, ay, az]
        
    except Exception as e:
        print(f"Fehler beim Parsen der IMU-Daten: {e} -- Payload: {payload_str}")
        return None

# --- MQTT Callback-Funktionen ---

def on_connect(client, userdata, flags, rc):
    """Wird aufgerufen, wenn die Verbindung zum Broker hergestellt wurde."""
    if rc == 0:
        print("Erfolgreich mit MQTT-Broker verbunden.")
        # Abonniere beide Topics
        client.subscribe([(UWB_TOPIC, 0), (IMU_TOPIC, 0)])
        print(f"Abonniert: {UWB_TOPIC} und {IMU_TOPIC}")
    else:
        print(f"Verbindung fehlgeschlagen, return code {rc}")

def on_message(client, userdata, msg):
    """Wird aufgerufen, wenn eine Nachricht auf einem abonnierten Topic empfangen wird."""
    try:
        payload_str = msg.payload.decode('utf-8')
        
        if msg.topic == UWB_TOPIC:
            csv_rows = parse_uwb_data(payload_str)
            if csv_rows:
                with open(UWB_CSV_FILE, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerows(csv_rows) 
                            
        elif msg.topic == IMU_TOPIC:
            csv_row = parse_imu_data(payload_str)
            if csv_row:
                with open(IMU_CSV_FILE, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(csv_row)
                    
    except Exception as e:
        print(f"Ein Fehler ist in on_message aufgetreten: {e}")

def main():
    
    if os.path.exists(UWB_CSV_FILE):
        try:
            os.remove(UWB_CSV_FILE)
            print(f"Alte Datei '{UWB_CSV_FILE}' erfolgreich gelöscht.")
        except OSError as e:
            print(f"FEHLER: Konnte '{UWB_CSV_FILE}' nicht löschen: {e}")
            
    if os.path.exists(IMU_CSV_FILE):
        try:
            os.remove(IMU_CSV_FILE)
            print(f"Alte Datei '{IMU_CSV_FILE}' erfolgreich gelöscht.")
        except OSError as e:
            print(f"FEHLER: Konnte '{IMU_CSV_FILE}' nicht löschen: {e}")
    check_and_write_headers()

    # MQTT-Client initialisieren
    client = mqtt.Client()
    client.username_pw_set(MQTT_USER, MQTT_PASS)
    client.on_connect = on_connect
    client.on_message = on_message

    print(f"Verbinde mit {MQTT_BROKER}...")
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
    except Exception as e:
        print(f"Konnte nicht mit MQTT-Broker verbinden: {e}")
        sys.exit(1)
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print("\nLogger gestoppt. Auf Wiedersehen.")
        client.disconnect()

if __name__ == "__main__":
    main()