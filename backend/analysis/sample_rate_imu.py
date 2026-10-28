import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# --- Parameter ---
EXPECTED_RATE_HZ = 50.0
EXPECTED_DELTA_S = 1.0 / EXPECTED_RATE_HZ
DROPPED_PACKET_THRESHOLD_S = EXPECTED_DELTA_S * 1.5 

try:
    # --- 1. Daten laden ---
    file_path = 'imu_data_1.csv'
    df = pd.read_csv(file_path)
    
    if df.empty or len(df) < 2:
        print(f"Fehler: Datei '{file_path}' ist leer oder enthält nicht genügend Daten.")
    else:
        print(f"--- Analyse der Vollständigkeit für '{file_path}' ---")
        print(f"Erwartete Samplerate: {EXPECTED_RATE_HZ} Hz (oder {EXPECTED_DELTA_S:.4f} s pro Sample)\n")
        df = df.sort_values('timestamp_ns')
        df['delta_ns'] = df['timestamp_ns'].diff()
        df_deltas = df['delta_ns'].dropna() / 1_000_000_000 # In Sekunden
        
        dropped_packets = (df_deltas > DROPPED_PACKET_THRESHOLD_S).sum()
        
        print("--- Check 1: Analyse der Zeitabstände (Delta-t) ---")
        print(f"Mittlerer Zeitabstand: {df_deltas.mean():.6f} s")
        print(f"Median Zeitabstand:  {df_deltas.median():.6f} s")
        print(f"Min. Zeitabstand:    {df_deltas.min():.6f} s")
        print(f"Max. Zeitabstand:    {df_deltas.max():.6f} s")
        print(f"Anzahl vermuteter 'Dropped Packets' (Abstand > {DROPPED_PACKET_THRESHOLD_S:.4f}s): {dropped_packets}")
        t_start = df['timestamp_ns'].min()
        t_end = df['timestamp_ns'].max()
        num_samples = len(df)
        duration_s = (t_end - t_start) / 1_000_000_000
        overall_avg_rate = (num_samples - 1) / duration_s
        
        print("\n--- Check 2: Gesamt-Durchschnittsrate ---")
        print(f"Gesamte Dauer:      {duration_s:.2f} s")
        print(f"Anzahl aller Samples: {num_samples}")
        print(f"Durchschnittsrate:  {overall_avg_rate:.2f} Hz")
        df['datetime'] = pd.to_datetime(df['timestamp_ns'], unit='ns')
        df = df.set_index('datetime')
        samples_per_second = df['ax'].resample('1S').count()
        if len(samples_per_second) > 2:
            middle_seconds = samples_per_second[1:-1]
        else:
            middle_seconds = samples_per_second

        print("\n--- Check 3: Analyse der Samples pro Sekunde (Binning) ---")
        if not middle_seconds.empty:
            print(f"Median Samples pro Sekunde:   {middle_seconds.median():.0f}")
            print(f"Min. Samples pro Sekunde:     {middle_seconds.min():.0f}")
            print(f"Max. Samples pro Sekunde:     {middle_seconds.max():.0f}")
            
            deviating_seconds = middle_seconds[middle_seconds != EXPECTED_RATE_HZ].count()
            total_middle_seconds = len(middle_seconds)
            completeness_perc = (1 - (deviating_seconds / total_middle_seconds)) * 100
            
            print(f"Sekunden mit Abweichungen: {deviating_seconds} von {total_middle_seconds} (vollen) Sekunden")
            print(f"Vollständigkeit (volle Sekunden): {completeness_perc:.2f}%")
        else:
            print("Daten umfassen weniger als 3 Sekunden, 'volle Sekunden'-Analyse übersprungen.")

        plt.figure(figsize=(10, 6))
        plt.hist(df_deltas, bins=100, range=(0, 0.05), label='Tatsächliche Zeitabstände')
        plt.axvline(EXPECTED_DELTA_S, color='r', linestyle='--', 
                    label=f'Erwartet ({EXPECTED_DELTA_S:.4f}s / {EXPECTED_RATE_HZ} Hz)')
        plt.title('Histogramm der Zeitabstände zwischen Samples (Delta-t)')
        plt.xlabel('Zeitabstand (s)')
        plt.ylabel('Anzahl der Samples')
        plt.legend()
        plt.grid(True)
        
        hist_path = 'sample_rate_histogram.png'
        plt.savefig(hist_path)
        print(f"\nHistogramm der Zeitabstände gespeichert: {hist_path}")

except FileNotFoundError:
    print(f"Fehler: Die Datei '{file_path}' wurde nicht gefunden.")
except Exception as e:
    print(f"Ein Fehler ist aufgetreten: {e}")