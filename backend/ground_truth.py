import paho.mqtt.client as mqtt
import math
import pandas as pd
import numpy as np
import time
import os 

# --- Ihre MQTT-Konfiguration ---
MQTT_BROKER = ""
MQTT_PORT = 1883
MQTT_USER = ""
MQTT_PASS = ""
UWB_TOPIC = "uwb/data"
IMU_TOPIC = "imu/data"
OUTPUT_CSV_FILE = 'mqtt_ground_truth.csv'

# --- Experiment-Parameter ---
ACCEL_CHANGE_THRESHOLD = 0.5   # Schwellenwert für Bewegung
MIN_STOP_DURATION_S = 2.0      # Mindestdauer for einen gültigen Stopp

# --- Ground Truth Definition (Y = 0.7 bis 2.4) ---
X_CONST = 2.07
Y_START = 0.7
Y_END = 2.4
STEP_SIZE_M = 0.05
EXPECTED_NUM_STOPS = int(round((Y_END - Y_START) / STEP_SIZE_M)) + 1 
y_positions = np.linspace(Y_START, Y_END, EXPECTED_NUM_STOPS)
gt_positions = list(zip([X_CONST] * EXPECTED_NUM_STOPS, y_positions))

# --- Globale Variablen für die Live-Verarbeitung ---
last_accel = None 
all_data_records = []
current_state = 'STOP' 
stop_start_timestamp_ns = None 
valid_stop_counter = 0

# --- Nachverarbeitungs-Funktion ---
def process_and_save_data(records, gt_positions_list):
    print(f"\nVerarbeite {len(records)} gesammelte IMU-Datensätze...")
    if not records:
        print("Keine Daten gesammelt. CSV-Datei wird nicht erstellt.")
        return

    # 1. Daten in DataFrame umwandeln
    df = pd.DataFrame(records)
    df = df.sort_values('timestamp_ns')

    # 2. Stopp-Perioden aus den Live-Daten extrahieren
    df['is_stationary'] = ~df['is_moving'] 
    df['stop_group'] = (df['is_stationary'] != df['is_stationary'].shift()).cumsum()
    
    stationary_groups = df[df['is_stationary'] == True]
    if stationary_groups.empty:
        print("Keine stationären Phasen in den Daten gefunden.")
        return

    # 3. Gültige Stopps filtern
    stop_periods = stationary_groups.groupby('stop_group')['timestamp_ns'].agg(['first', 'last'])
    stop_periods['duration_ns'] = stop_periods['last'] - stop_periods['first']
    min_duration_ns = MIN_STOP_DURATION_S * 1_000_000_000
    detected_stops_df = stop_periods[stop_periods['duration_ns'] >= min_duration_ns].reset_index(drop=True)
    
    print(f"Nachverarbeitung: {len(detected_stops_df)} gültige Stopps gefunden.")

    # 4. Erwartete Stopps vorbereiten
    expected_stops_df = pd.DataFrame(gt_positions_list, columns=['gt_pos_x', 'gt_pos_y'])
    
    # 5. Validieren und Zuweisen
    if len(detected_stops_df) != len(expected_stops_df):
        print(f"WARNUNG: {len(detected_stops_df)} Stopps erkannt, aber {len(expected_stops_df)} erwartet!")
    
    num_stops_to_assign = min(len(detected_stops_df), len(expected_stops_df))
    ground_truth_stops = pd.concat([detected_stops_df.iloc[:num_stops_to_assign], expected_stops_df.iloc[:num_stops_to_assign]], axis=1)

    # 6. Vollständiges DataFrame füllen (Zustand und Positionen)
    df['time_s'] = (df['timestamp_ns'] - df['timestamp_ns'].min()) / 1e9
    df['state'] = 'Moving'
    df['gt_pos_x'] = np.nan
    df['gt_pos_y'] = np.nan
    df['stop_number'] = np.nan 

    # 'Stop'-Phasen füllen
    for stop in ground_truth_stops.itertuples():
        mask = (df['timestamp_ns'] >= stop.first) & (df['timestamp_ns'] <= stop.last)
        df.loc[mask, 'state'] = 'Stop'
        df.loc[mask, 'gt_pos_x'] = stop.gt_pos_x
        df.loc[mask, 'gt_pos_y'] = stop.gt_pos_y
        df.loc[mask, 'stop_number'] = stop.Index + 1

    # 7. 'Moving'-Phasen interpolieren
    for i in range(len(ground_truth_stops) - 1):
        stop_A = ground_truth_stops.iloc[i]
        stop_B = ground_truth_stops.iloc[i+1]
        
        move_start_ns = stop_A['last']
        start_pos_y = stop_A['gt_pos_y']
        move_end_ns = stop_B['first']
        end_pos_y = stop_B['gt_pos_y']

        move_duration_s = (move_end_ns - move_start_ns) / 1e9
        dist_y = end_pos_y - start_pos_y
        
        vel_y = dist_y / move_duration_s if move_duration_s > 0 else 0
        
        move_mask = (df['timestamp_ns'] > move_start_ns) & (df['timestamp_ns'] < move_end_ns)
        time_in_move_s = (df.loc[move_mask, 'timestamp_ns'] - move_start_ns) / 1e9
        
        df.loc[move_mask, 'gt_pos_x'] = X_CONST
        df.loc[move_mask, 'gt_pos_y'] = start_pos_y + (vel_y * time_in_move_s)

    # 8. Speichern 
    final_columns = ['timestamp_ns', 'time_s', 'state', 'gt_pos_x', 'gt_pos_y', 'stop_number']

    df[final_columns].to_csv(OUTPUT_CSV_FILE, index=False, float_format='%.6f')
    print(f"Ground Truth Datei erfolgreich gespeichert: {OUTPUT_CSV_FILE}")
    print(f"Enthaltene Spalten: {', '.join(final_columns)}")

# --- MQTT Callbacks ---

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"Erfolgreich verbunden mit {MQTT_BROKER}")
        client.subscribe(UWB_TOPIC)
        client.subscribe(IMU_TOPIC)
        print(f"Abonniert: {UWB_TOPIC} und {IMU_TOPIC}")
        print("Starte Ground Truth Aufzeichnung...")
        print(f"Erwarte {len(gt_positions)} Stopps (Mindestdauer: {MIN_STOP_DURATION_S}s).")
    else:
        print(f"Verbindung fehlgeschlagen mit Code: {rc}")

def on_message(client, userdata, msg):
    global last_accel, all_data_records, current_state, stop_start_timestamp_ns, valid_stop_counter
    
    payload_str = msg.payload.decode()
    
    if msg.topic == UWB_TOPIC:
        return

    try:
        if msg.topic == IMU_TOPIC:
            # 1. Payload parsen
            parts = payload_str.split(';')
            if len(parts) != 3: return
            timestamp_ns = int(parts[0])

            # IMU-Parsing (3+4 Format)
            accel_parts = parts[1].split(',')
            if len(accel_parts) != 3: return
            current_accel = {
                'x': float(accel_parts[0]), 'y': float(accel_parts[1]), 'z': float(accel_parts[2])
            }

            quat_parts = parts[2].split(',')
            if len(quat_parts) != 4: return
            current_quat = {
                'w': float(quat_parts[0]), 'x': float(quat_parts[1]),
                'y': float(quat_parts[2]), 'z': float(quat_parts[3])
            }
            
            # 2. Bewegung berechnen
            if last_accel is None:
                last_accel = current_accel
                if stop_start_timestamp_ns is None:
                    stop_start_timestamp_ns = timestamp_ns 
                    print(f"Warte auf Stopp #1 (Ziel: {gt_positions[0]}). Timer gestartet...")
                return
            
            ax_diff = current_accel['x'] - last_accel['x']
            ay_diff = current_accel['y'] - last_accel['y']
            az_diff = current_accel['z'] - last_accel['z']
            accel_change_mag = math.sqrt(ax_diff**2 + ay_diff**2 + az_diff**2)
            
            is_moving = accel_change_mag > ACCEL_CHANGE_THRESHOLD
            
            # 3. Alle Daten speichern
            record = {
                'timestamp_ns': timestamp_ns,
                'is_moving': is_moving,
                'accel_change_mag': accel_change_mag,
                'ax': current_accel['x'], 'ay': current_accel['y'], 'az': current_accel['z'],
                'qw': current_quat['w'], 'qx': current_quat['x'], 'qy': current_quat['y'], 'qz': current_quat['z']
            }
            all_data_records.append(record)

            # 4. Live-Feedback-Statusmaschine
            if is_moving:
                if current_state == 'STOP':
                    if stop_start_timestamp_ns is None:
                        stop_start_timestamp_ns = timestamp_ns 

                    stop_duration_s = (timestamp_ns - stop_start_timestamp_ns) / 1e9
                    if stop_duration_s >= MIN_STOP_DURATION_S:
                        valid_stop_counter += 1
                        if valid_stop_counter <= len(gt_positions):
                            pos = gt_positions[valid_stop_counter-1]
                            print(f"BEWEGUNG. Gültiger Stopp #{valid_stop_counter} (Pos: {pos}) beendet (Dauer: {stop_duration_s:.2f}s).")
                        else:
                            print(f"BEWEGT. Gültiger Stopp #{valid_stop_counter} (Extra) beendet (Dauer: {stop_duration_s:.2f}s).")
                    else:
                        print(f"BEWEGUNG. (Stopp war zu kurz: {stop_duration_s:.2f}s).")
                current_state = 'MOVING'
            else: # not is_moving
                if current_state == 'MOVING':
                    # Dies ist ein neuer Stopp
                    if valid_stop_counter < len(gt_positions):
                        pos = gt_positions[valid_stop_counter] 
                        print(f"STILLSTAND. Warte auf Stopp #{valid_stop_counter + 1} (Ziel: {pos}). Timer gestartet...")
                    else:
                         print(f"STILLSTAND. (Alle {len(gt_positions)} erwarteten Stopps bereits erfasst). Timer gestartet...")
                    stop_start_timestamp_ns = timestamp_ns
                
                current_state = 'STOP'

            last_accel = current_accel

    except (ValueError, IndexError):
        pass 
    except Exception as e:
        print(f"Ein unerwarteter Fehler ist aufgetreten: {e}")

# --- Hauptprogramm ---
def main():
    #Alte CSV-Datei beim Start löschen ***
    if os.path.exists(OUTPUT_CSV_FILE):
        try:
            os.remove(OUTPUT_CSV_FILE)
            print(f"Alte Datei '{OUTPUT_CSV_FILE}' erfolgreich gelöscht.")
        except OSError as e:
            print(f"FEHLER: Konnte '{OUTPUT_CSV_FILE}' nicht löschen: {e}")
            # Wir fahren trotzdem fort, das Überschreiben am Ende funktioniert vielleicht noch
    
    print(f"--- Ground Truth Logger gestartet ---")
    print(f"Erwartete Stopps: {EXPECTED_NUM_STOPS} (von {Y_START}m bis {Y_END}m)")
    print(f"Bewegungsschwelle (Accel Change): {ACCEL_CHANGE_THRESHOLD}g") 
    print(f"Mindest-Stoppdauer: {MIN_STOP_DURATION_S}s")
    print("-------------------------------------")
    print(f"Skript startet im 'STOP'-Zustand.")
    print(f"Warte auf erste IMU-Nachricht, um Timer für Stopp #1 zu starten...")
    
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.username_pw_set(MQTT_USER, MQTT_PASS)

    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
    except Exception as e:
        print(f"Konnte nicht verbinden: {e}")
        return

    try:
        client.loop_forever()
    except KeyboardInterrupt:
        client.disconnect()
        print("\nAufzeichnung gestoppt. Verarbeite Daten und speichere CSV...")
        process_and_save_data(all_data_records, gt_positions)
        print("Verarbeitung abgeschlossen.")

if __name__ == "__main__":
    main()