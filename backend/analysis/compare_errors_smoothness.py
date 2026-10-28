import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

# --- HILFSFUNKTIONEN ---
def calculate_errors(res_df, gt_df):
    res_df = res_df.dropna(subset=['pos_x', 'pos_y'])

    if res_df.empty: 
        return np.array([]), np.array([])
    gt_interp_x = interp1d(gt_df['timestamp_ns'], gt_df['gt_pos_x'], kind='linear', fill_value="extrapolate")
    gt_interp_y = interp1d(gt_df['timestamp_ns'], gt_df['gt_pos_y'], kind='linear', fill_value="extrapolate")
    
    timestamps = res_df['timestamp_ns'].values
    gt_x_at_ts = gt_interp_x(timestamps)
    gt_y_at_ts = gt_interp_y(timestamps)
    
    errors = np.sqrt((res_df['pos_x'].values - gt_x_at_ts)**2 + 
                     (res_df['pos_y'].values - gt_y_at_ts)**2)
                     
    return timestamps, errors

def calculate_smoothness(df):
    df = df.dropna(subset=['pos_x', 'pos_y'])

    if df.empty or len(df) < 3: 
        return {'path_len': 0, 'mean_jerk': 0}

    pos = df[['pos_x', 'pos_y']].values
    t = df['timestamp_ns'].values * 1e-9 # in Sekunden

    # 1. Path Length
    diffs = np.diff(pos, axis=0)
    dists = np.sqrt((diffs**2).sum(axis=1))
    path_len = np.sum(dists)

    # 2. Mean Jerk (Proxy)
    dt = np.diff(t)
    dt[dt == 0] = 1e-9 
    vel = diffs / dt[:, None] 

    vel_diffs = np.diff(vel, axis=0)
    dt_a = dt[1:] 
    acc = vel_diffs / dt_a[:, None]

    acc_norm = np.sqrt((acc**2).sum(axis=1))
    mean_jerk_proxy = np.mean(acc_norm) 

    return {'path_len': path_len, 'mean_jerk': mean_jerk_proxy}

def get_stats(errors):
    if len(errors) == 0: return {}
    return {
        'mean': np.mean(errors), 'median': np.median(errors),
        'rmse': np.sqrt(np.mean(errors**2)), 'p95': np.percentile(errors, 95)
    }

# --- HAUPTPROGRAMM ---
print("1. Lade CSV-Dateien...")
try:
    gt_df = pd.read_csv('mqtt_ground_truth.csv')
    ekf_df = pd.read_csv('ekf_results.csv') 
    trilat_df = pd.read_csv('trilat_results.csv')
except FileNotFoundError as e:
    print(f"FEHLER: Datei nicht gefunden: {e.filename}")
    exit()

print("2. Berechne Fehler & Smoothness...")
t_ekf, err_ekf = calculate_errors(ekf_df, gt_df)
t_tri, err_tri = calculate_errors(trilat_df, gt_df)

stats_ekf = get_stats(err_ekf)
stats_tri = get_stats(err_tri)
smooth_ekf = calculate_smoothness(ekf_df)
smooth_tri = calculate_smoothness(trilat_df)
smooth_gt = calculate_smoothness(gt_df.rename(columns={'gt_pos_x':'pos_x', 'gt_pos_y':'pos_y'}))

print("\n--- VERGLEICH: GENAUIGKEIT & GLÄTTE ---")
print(f"{'Metric':<15} | {'EKF Fusion':<15} | {'Trilateration':<15} | {'Ground Truth':<15}")
print("-" * 66)
print(f"{'RMSE (Error)':<15} | {stats_ekf.get('rmse',0):.3f} m {'':<7} | {stats_tri.get('rmse',0):.3f} m {'':<7} | {'0.000 m':<15}")
print(f"{'Mean Error':<15} | {stats_ekf.get('mean',0):.3f} m {'':<7} | {stats_tri.get('mean',0):.3f} m {'':<7} | {'0.000 m':<15}")
print(f"{'P95 Error (CDF)':<15} | {stats_ekf.get('p95',0):.3f} m {'':<7} | {stats_tri.get('p95',0):.3f} m {'':<7} | {'0.000 m':<15}")
print("-" * 66)
print(f"{'Path Length':<15} | {smooth_ekf['path_len']:.3f} m {'':<7} | {smooth_tri['path_len']:.3f} m {'':<7} | {smooth_gt['path_len']:.3f} m")
print(f"{'Jerk (Unruhe)':<15} | {smooth_ekf['mean_jerk']:.2f} m/s²{'':<6} | {smooth_tri['mean_jerk']:.2f} m/s²{'':<6} | {smooth_gt['mean_jerk']:.2f} m/s²")
print("-" * 66)
print("(Hinweis: 'Jerk' ist hier die mittlere Beschleunigungsnorm. Niedriger = Glatter)")

# --- PLOTTING ---
fig, ax = plt.subplots(1, 1, figsize=(10, 8))

# --- Plot Pfad-Glätte ---
ax.plot(gt_df['gt_pos_x'], gt_df['gt_pos_y'], 'g-', lw=3, alpha=0.5, label='Ground Truth (Len: 1,7m)')
ax.plot(trilat_df['pos_x'], trilat_df['pos_y'], 'gray', lw=1, alpha=0.3, label=f'Trilat (Len: {smooth_tri["path_len"]:.1f}m)')
ax.plot(ekf_df['pos_x'], ekf_df['pos_y'], 'b-', lw=2, label=f'EKF (Len: {smooth_ekf["path_len"]:.1f}m)')
ax.set_title('Pfad-Glätte (Zoom)')
ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.legend(); ax.grid(True); ax.axis('equal')

# ---Metriken-Textbox auf dem Plot ---
metrics_text = (
    f"--- Genauigkeitsvergleich ---\n"
    f"Metrik    | EKF     | Trilateration\n"
    f"-----------------------------------\n"
    f"RMSE      | {stats_ekf.get('rmse', 0):.3f} m | {stats_tri.get('rmse', 0):.3f} m\n"
    f"Mean Err  | {stats_ekf.get('mean', 0):.3f} m | {stats_tri.get('mean', 0):.3f} m\n"
    f"CDF 95%   | {stats_ekf.get('p95', 0):.3f} m | {stats_tri.get('p95', 0):.3f} m"
)

ax.text(0.05, 0.95, metrics_text,
        transform=ax.transAxes,
        fontsize=9, 
        verticalalignment='top',
        fontfamily='monospace',
        bbox=dict(boxstyle='round,pad=0.5', facecolor='white', alpha=0.8))
# ---------------------------------------------------

plt.tight_layout()
plt.savefig('smoothness_comparison_with_metrics.png', dpi=150)
plt.show()