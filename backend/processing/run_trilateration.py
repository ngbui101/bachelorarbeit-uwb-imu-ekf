import numpy as np
import pandas as pd
from scipy.optimize import least_squares
import os 

ANCHOR_POSITIONS_3D = {
    "dist_e05a1": np.array([2.8, 0, 1.31]),
    "dist_48e72": np.array([0.1, 0, 2.0]),
    "dist_83a8d": np.array([1.86, 4.1, 2.10])
}
TAG_HEIGHT = 0.015 
OUTPUT_FILENAME = 'trilat_results.csv' 

try:
    df = pd.read_csv('merged_imu_uwb_data.csv')
except FileNotFoundError:
    print("FEHLER: 'merged_imu_uwb_data.csv' nicht gefunden.")
    print("Bitte stellen Sie sicher, dass die Datei im selben Ordner liegt.")
    exit()

def project_to_2d(dist_3d, anchor_h, tag_h):
    """Projiziert 3D-Distanz auf 2D-Ebene basierend auf Höhendifferenz."""
    h_diff = abs(anchor_h - tag_h)
    if dist_3d < h_diff:
        return 0.01 
    return np.sqrt(dist_3d**2 - h_diff**2)

def trilateration_residuals(pos_2d, anchor_positions_2d, distances_2d):
    """Residuen-Funktion für Least-Squares Optimierung."""
    residuals = []
    for i, anchor_pos in enumerate(anchor_positions_2d):
        dist_pred = np.linalg.norm(pos_2d - anchor_pos)
        residuals.append(dist_pred - distances_2d[i])
    return residuals

if os.path.exists(OUTPUT_FILENAME):
    try:
        os.remove(OUTPUT_FILENAME)
        print(f"Alte Datei '{OUTPUT_FILENAME}' erfolgreich gelöscht.")
    except OSError as e:
        print(f"FEHLER beim Löschen der Datei '{OUTPUT_FILENAME}': {e}")

trilat_results = []
last_pos = np.array([2.07, 0.70]) 

print("Starte Trilateration (fülle Lücken mit letzter Position)...")

for i, row in df.iterrows():
    timestamp = row['timestamp_ns']
    
    active_anchors_2d = []
    active_dists_2d = []

    for col, anchor_pos_3d in ANCHOR_POSITIONS_3D.items():
        dist_raw = row[col]
        if not np.isnan(dist_raw):
            dist_2d = project_to_2d(dist_raw, anchor_pos_3d[2], TAG_HEIGHT)
            active_anchors_2d.append(anchor_pos_3d[:2])
            active_dists_2d.append(dist_2d)
    
    if len(active_dists_2d) >= 1:
        try:
            res = least_squares(
                trilateration_residuals, 
                last_pos, 
                args=(np.array(active_anchors_2d), np.array(active_dists_2d)),
                method='lm'
            )
            
            if res.success:
                pos_est = res.x
                last_pos = pos_est 
                trilat_results.append({'timestamp_ns': timestamp, 'pos_x': pos_est[0], 'pos_y': pos_est[1]})
            else:
                # Optimierung fehlgeschlagen, letzte Position halten
                trilat_results.append({'timestamp_ns': timestamp, 'pos_x': last_pos[0], 'pos_y': last_pos[1]})
        
        except ValueError:
             # Fehler bei least_squares, letzte Position halten
             trilat_results.append({'timestamp_ns': timestamp, 'pos_x': last_pos[0], 'pos_y': last_pos[1]})
            
    else:
        # Fallback: Wenn 0 Ankerdaten, letzte Position halten
        trilat_results.append({'timestamp_ns': timestamp, 'pos_x': last_pos[0], 'pos_y': last_pos[1]})

# 5. Speichern
df_trilat = pd.DataFrame(trilat_results)
df_trilat.to_csv(OUTPUT_FILENAME, index=False) # HINZUGEFÜGT: Nutzt die Variable
print(f"Trilateration abgeschlossen. Ergebnisse in '{OUTPUT_FILENAME}' gespeichert.")