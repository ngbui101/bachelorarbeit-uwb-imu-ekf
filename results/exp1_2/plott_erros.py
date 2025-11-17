import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

def calculate_errors(res_df, gt_df):

    res_df = res_df.dropna(subset=['pos_x', 'pos_y'])

    if res_df.empty: 
        return np.array([]), np.array([])

    gt_interp_x = interp1d(gt_df['timestamp_ns'], gt_df['gt_pos_x'],
                           kind='linear', fill_value="extrapolate")
    gt_interp_y = interp1d(gt_df['timestamp_ns'], gt_df['gt_pos_y'],
                           kind='linear', fill_value="extrapolate")

    timestamps = res_df['timestamp_ns'].values
    gt_x_at_ts = gt_interp_x(timestamps)
    gt_y_at_ts = gt_interp_y(timestamps)

    errors = np.sqrt((res_df['pos_x'].values - gt_x_at_ts)**2 +
                     (res_df['pos_y'].values - gt_y_at_ts)**2)

    return timestamps, errors

def get_stats(errors):
    """ Berechnet wichtige Fehlerstatistiken """
    if len(errors) == 0: 
        return {'mean': np.nan, 'median': np.nan, 'rmse': np.nan, 'p95': np.nan, 'max': np.nan}
    return {
        'mean': np.mean(errors),
        'median': np.median(errors),
        'rmse': np.sqrt(np.mean(errors**2)),
        'p95': np.percentile(errors, 95), 
        'max': np.max(errors)
    }

print("1. Lade CSV-Dateien...")
try:
    gt_df = pd.read_csv('mqtt_ground_truth.csv')
    ekf_df = pd.read_csv('ekf_results.csv')
    trilat_df = pd.read_csv('trilat_results.csv')
except FileNotFoundError as e:
    print(f"\nFEHLER: Konnte Datei nicht finden: {e.filename}")
    exit()

gt_df = gt_df.dropna(subset=['timestamp_ns', 'gt_pos_x', 'gt_pos_y'])

print("2. Berechne Fehler (mit Zeit-Interpolation)...")
t_ekf, err_ekf = calculate_errors(ekf_df, gt_df)
t_tri, err_tri = calculate_errors(trilat_df, gt_df)

stats_ekf = get_stats(err_ekf)
stats_tri = get_stats(err_tri)

print("\n--- FEHLER STATISTIKEN ---")
print(f"{'Metric':<10} | {'EKF Fusion':<15} | {'Trilateration':<15}")
print("-" * 46)
print(f"{'Mean':<10} | {stats_ekf.get('mean',0):.3f} m {'':<7} | {stats_tri.get('mean',0):.3f} m")
print(f"{'Median':<10} | {stats_ekf.get('median',0):.3f} m {'':<7} | {stats_tri.get('median',0):.3f} m")
print(f"{'RMSE':<10} | {stats_ekf.get('rmse',0):.3f} m {'':<7} | {stats_tri.get('rmse',0):.3f} m")
print(f"{'95% Pctl':<10} | {stats_ekf.get('p95',0):.3f} m {'':<7} | {stats_tri.get('p95',0):.3f} m")
print("-" * 46)

print("3. Erstelle Vergleichsplots...")
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7)) 
fig.suptitle('Vergleich von EKF-Fusion und reiner Trilateration', fontsize=16, y=1.02)


if not gt_df.empty:
    start_time = gt_df['timestamp_ns'].iloc[0]
    t_tri_s = (t_tri - start_time) * 1e-9
    t_ekf_s = (t_ekf - start_time) * 1e-9
    ax1.plot(t_tri_s, err_tri, 'gray', alpha=0.4, label='Trilateration (Roh)')
    ax1.plot(t_ekf_s, err_ekf, 'b-', linewidth=2, label='EKF Fusion (Gefiltert)')
else:
    ax1.text(0.5, 0.5, 'Keine Ground-Truth-Daten', horizontalalignment='center', verticalalignment='center')

ax1.set_title('Positionsfehler über die Zeit')
ax1.set_xlabel('Zeit (s)'); ax1.set_ylabel('Fehler (m)')
ax1.legend(); ax1.grid(True)

def plot_cdf(ax, data, label, color, style='-'):
    if len(data) == 0: return
    sorted_data = np.sort(data)
    yvals = np.arange(len(sorted_data)) / float(len(sorted_data) - 1)
    ax.plot(sorted_data, yvals, color=color, linestyle=style, linewidth=2, label=label)

plot_cdf(ax2, err_tri, 'Trilateration', 'gray', '--')
plot_cdf(ax2, err_ekf, 'EKF Fusion', 'blue', '-')
ax2.set_title('CDF: Verteilung der Fehlergenauigkeit')
ax2.set_xlabel('Fehler (m)'); ax2.set_ylabel('Wahrscheinlichkeit (0.0 - 1.0)')
ax2.legend(loc='lower right'); ax2.grid(True)
ax2.set_xlim(0, 1.0) 

stats_text = (
    f"--- Statistiken (m) ---\n"
    f"{'Metric':<8} | {'EKF':<7} | {'Trilat':<7}\n"
    f"{'-'*28}\n"
    f"{'RMSE':<8} | {stats_ekf['rmse']:.3f} | {stats_tri['rmse']:.3f}\n"
    f"{'Mean':<8} | {stats_ekf['mean']:.3f} | {stats_tri['mean']:.3f}\n"
    f"{'Median':<8} | {stats_ekf['median']:.3f} | {stats_tri['median']:.3f}\n"
    f"{'95% Pctl':<8} | {stats_ekf['p95']:.3f} | {stats_tri['p95']:.3f}"
)

props = dict(boxstyle='round', facecolor='white', alpha=0.8)
ax2.text(0.95, 0.95, stats_text, transform=ax2.transAxes, 
         fontsize=9, verticalalignment='top', horizontalalignment='right', 
         bbox=props, family='monospace')
# ---------------------------------------------

plt.tight_layout(rect=[0, 0, 1, 0.95]) 
plt.savefig('error_comparison.png', dpi=300)
print("\nFertig! Plot gespeichert als 'error_comparison.png'.")
plt.show()