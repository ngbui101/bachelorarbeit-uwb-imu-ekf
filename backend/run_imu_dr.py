import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
import os 

# --- 1. Konfigurationen & Konstanten ---
INPUT_FILENAME = 'merged_imu_uwb_data.csv'
OUTPUT_FILENAME = 'imu_dead_reckoning.csv' 

try:
    df = pd.read_csv(INPUT_FILENAME)
except FileNotFoundError:
    print(f"FEHLER: '{INPUT_FILENAME}' nicht gefunden.")
    exit()

# --- 2. Initialisierung ---
x_est = np.array([2.070000, 0.700000, 0.0, 0.0])

if os.path.exists(OUTPUT_FILENAME):
    try:
        os.remove(OUTPUT_FILENAME)
        print(f"Alte Datei '{OUTPUT_FILENAME}' erfolgreich gelöscht.")
    except OSError as e:
        print(f"FEHLER beim Löschen der Datei '{OUTPUT_FILENAME}': {e}")
# ----------------------------------------

results = []
prev_timestamp = 0
print("Starte IMU Dead Reckoning (mit 90-Grad-Korrektur)...")

# --- 3. Dead Reckoning Hauptschleife ---
for i, row in df.iterrows():
    timestamp = row['timestamp_ns']

    if i == 0:
        results.append({'timestamp_ns': timestamp, 'pos_x': x_est[0], 'pos_y': x_est[1]})
        prev_timestamp = timestamp
        continue

    dt = (timestamp - prev_timestamp) / 1e9
    prev_timestamp = timestamp
    if dt <= 0: continue

    # --- Prädiktionsschritt (IMU) ---
    
    q = [row['qx'], row['qy'], row['qz'], row['qw']]
    r = R.from_quat(q)

    acc_body = np.array([row['ay'], -row['ax'], row['az']])

    acc_global = r.apply(acc_body)
    ax_global = acc_global[0]
    ay_global = acc_global[1]

    # 2. Zustandsprädiktion (Bewegungsmodell)
    A = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]])
    B = np.array([[0.5 * dt**2, 0], [0, 0.5 * dt**2], [dt, 0], [0, dt]])
    u = np.array([ax_global, ay_global])
    x_est = A @ x_est + B @ u
    
    results.append({'timestamp_ns': timestamp, 'pos_x': x_est[0], 'pos_y': x_est[1]})

# --- 4. Ergebnisse speichern ---
df_results = pd.DataFrame(results)
df_results.to_csv(OUTPUT_FILENAME, index=False)

print(f"Verarbeitung abgeschlossen. {len(df_results)} Zeitschritte verarbeitet.")
print(f"Ergebnisse in '{OUTPUT_FILENAME}' gespeichert.")