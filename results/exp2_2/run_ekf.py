import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
import os 

# --- 1. Konfigurationen & Konstanten ---
try:
    df = pd.read_csv('merged_imu_uwb_data.csv')
except FileNotFoundError:
    print("FEHLER: 'merged_imu_uwb_data.csv' nicht gefunden.")
    exit()

ANCHOR_POSITIONS_3D = {
    "dist_e05a1": np.array([2.8, 0, 1.31]),
    "dist_48e72": np.array([0.1, 0, 2.0]),
    "dist_83a8d": np.array([1.86, 4.1, 2.10])
}
TAG_HEIGHT = 0.015 
OUTPUT_FILENAME = 'ekf_results.csv' 

ACCEL_THRESHOLD = 0.5 # m/s^2

ANCHOR_POSITIONS_2D = {}
ANCHOR_HEIGHTS = {}
for col, pos3d in ANCHOR_POSITIONS_3D.items():
    ANCHOR_POSITIONS_2D[col] = pos3d[:2] 
    ANCHOR_HEIGHTS[col] = pos3d[2]      

# --- 2. EKF Initialisierung ---
x_est = np.array([2.4, 4.1, 0.0, 0.0])
P_est = np.eye(4) * 1.0

# --- TUNING ---
sigma_acc = 0.1   
Q_scale = sigma_acc**2
sigma_uwb = 0.5   
R_uwb = sigma_uwb**2
# -----------------------------------------------------------------

if os.path.exists(OUTPUT_FILENAME):
    try:
        os.remove(OUTPUT_FILENAME)
        print(f"Alte Datei '{OUTPUT_FILENAME}' erfolgreich gelöscht.")
    except OSError as e:
        print(f"FEHLER beim Löschen der Datei '{OUTPUT_FILENAME}': {e}")
# ----------------------------------------

results = []
prev_timestamp = 0
print("Starte EKF-Verarbeitung (mit ZUPT, ohne Glättung)...")

# --- 3. EKF Hauptschleife ---
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

    # 90-Grad-Korrektur 
    acc_body = np.array([row['ay'], -row['ax'], row['az']])
    
    acc_global = r.apply(acc_body)
    ax_global = acc_global[0]
    ay_global = acc_global[1]

    A = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]])
    B = np.array([[0.5 * dt**2, 0], [0, 0.5 * dt**2], [dt, 0], [0, dt]])
    
    # --- ZUPT-Logik---

    acc_norm = np.linalg.norm(acc_global) 
    
    is_stationary = acc_norm < ACCEL_THRESHOLD
    
    if is_stationary:
        # STILLSTAND ERKANNT
        # Prädiziere keine Bewegung (ignoriere 'u')
        u = np.array([0.0, 0.0]) 
        x_pred = A @ x_est + B @ u # u ist [0,0]
        x_pred[2] = 0.0 # vx = 0
        x_pred[3] = 0.0 # vy = 0
        
    else:
        # BEWEGUNG ERKANNT
        # Nutze die IMU-Daten für die Prädiktion
        u = np.array([ax_global, ay_global])
        x_pred = A @ x_est + B @ u
    # --- Ende ZUPT-Logik ---

    # Kovarianz-Prädiktion 
    G = np.array([[0.5 * dt**2, 0], [0, 0.5 * dt**2], [dt, 0], [0, dt]])
    Q = G @ G.T * Q_scale 
    P_pred = A @ P_est @ A.T + Q

    x_est = x_pred
    P_est = P_pred

    # --- Korrekturschritt (UWB) ---
    uwb_data_available = False
    for anchor_col in ['dist_83a8d', 'dist_48e72', 'dist_e05a1']:
        if not np.isnan(row[anchor_col]):
            uwb_data_available = True
            break
    
    if uwb_data_available:
        for anchor_col in ['dist_83a8d', 'dist_48e72', 'dist_e05a1']:
            dist_3d = row[anchor_col]
            if np.isnan(dist_3d): continue

            anchor_pos_2d = ANCHOR_POSITIONS_2D[anchor_col]
            anchor_h = ANCHOR_HEIGHTS[anchor_col]

            height_diff = abs(anchor_h - TAG_HEIGHT)
            dist_2d_meas = 0.01
            if dist_3d > height_diff:
                dist_2d_meas = np.sqrt(dist_3d**2 - height_diff**2)

            dx = x_est[0] - anchor_pos_2d[0] 
            dy = x_est[1] - anchor_pos_2d[1] 
            dist_pred = np.sqrt(dx**2 + dy**2)
            if dist_pred < 1e-3: dist_pred = 1e-3 

            H = np.array([[dx / dist_pred, dy / dist_pred, 0, 0]]) 
            innovation = dist_2d_meas - dist_pred
            S = H @ P_est @ H.T + R_uwb
            K = P_est @ H.T @ np.linalg.inv(S)
            x_est = x_est + K.flatten() * innovation
            P_est = (np.eye(4) - K @ H) @ P_est

    # --- ZUSÄTZLICHE ZUPT-KORREKTUR ---
    if is_stationary:
        P_est[2, 2] = 0.001 
        P_est[3, 3] = 0.001 
    # --------------------------------------------------------

    results.append({'timestamp_ns': timestamp, 'pos_x': x_est[0], 'pos_y': x_est[1]})

# --- 4. Ergebnisse speichern ---
df_results = pd.DataFrame(results)
df_results.to_csv(OUTPUT_FILENAME, index=False) 
print(f"Verarbeitung abgeschlossen. Ergebnisse in '{OUTPUT_FILENAME}' gespeichert.")