import pandas as pd
import numpy as np

# === KONFIGURATION ===
IMU_FILE = 'imu_data_1.csv'
UWB_FILE = 'uwb_data_1.csv'
OUTPUT_FILE = 'merged_imu_uwb_data.csv'
TIME_TOLERANCE_NS = 1e8 

# === 1. DATEN LADEN ===
print("Lade Daten...")
df_imu = pd.read_csv(IMU_FILE)
df_uwb = pd.read_csv(UWB_FILE)

df_imu['timestamp_ns'] = df_imu['timestamp_ns'].astype(np.int64)
df_uwb['timestamp_ns'] = df_uwb['timestamp_ns'].astype(np.int64)

t0 = min(df_imu['timestamp_ns'].iloc[0], df_uwb['timestamp_ns'].min())
df_imu['t_sec'] = (df_imu['timestamp_ns'] - t0) / 1e9

# === 2. UWB DATEN VORBEREITEN ===
uwb_macs = df_uwb['mac_address'].unique()
print(f"Gefundene UWB Anker: {uwb_macs}")

df_merged = df_imu.copy()

# === 3. SPARSE MERGE (DER KERN) ===
print("Starte Merge-Vorgang...")
for mac in uwb_macs: 
    col_name = f"dist_{mac.replace(':', '')[:5]}"

    df_merged[col_name] = np.nan
    
    uwb_subset = df_uwb[df_uwb['mac_address'] == mac]
    
    idx_closest_imu = df_merged['timestamp_ns'].searchsorted(uwb_subset['timestamp_ns'])
    
    idx_closest_imu = np.clip(idx_closest_imu, 0, len(df_merged) - 1)
    
    imu_timestamps = df_merged['timestamp_ns'].values[idx_closest_imu]
    uwb_timestamps = uwb_subset['timestamp_ns'].values
    time_diffs = np.abs(imu_timestamps - uwb_timestamps)
    
    valid_mask = time_diffs < TIME_TOLERANCE_NS
    
    valid_imu_indices = idx_closest_imu[valid_mask]
    valid_uwb_distances = uwb_subset['distance'].values[valid_mask]
    
    df_merged.loc[df_merged.index[valid_imu_indices], col_name] = valid_uwb_distances
    
    print(f"  -> Anker {col_name}: {len(valid_uwb_distances)} Messungen zugeordnet.")

# === 4. SPEICHERN ===
df_merged.to_csv(OUTPUT_FILE, index=False)
print(f"Fertig! Gemergte Datei gespeichert als: {OUTPUT_FILE}")
print(df_merged.head())