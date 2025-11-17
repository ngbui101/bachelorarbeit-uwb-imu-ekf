import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

try:
    gt_df = pd.read_csv('mqtt_ground_truth.csv')
    trilat_df = pd.read_csv('trilat_results.csv')

    trilat_df['timestamp_ns'] = trilat_df['timestamp_ns'].astype('int64')

    merged_df = pd.merge(gt_df, trilat_df, on='timestamp_ns', how='inner')

    stop_df = merged_df[merged_df['state'] == 'Stop'].copy()
    
    if not stop_df.empty and 'stop_number' in stop_df.columns:
        stop_df['stop_number'] = stop_df['stop_number'].astype(int)
    
    if stop_df.empty:
        print("Keine überlappenden 'Stop'-Daten nach dem Zusammenführen gefunden.")
        print(f"Größe merged_df: {len(merged_df)}")
        print(f"Größe gt_df: {len(gt_df)}, Größe trilat_df: {len(trilat_df)}")

    else:
        stop_df['euclidean_error'] = np.sqrt(
            (stop_df['pos_x'] - stop_df['gt_pos_x'])**2 +
            (stop_df['pos_y'] - stop_df['gt_pos_y'])**2
        )

        print("--- Statistische Auswertung des Euklidischen Fehlers (nur 'Stop'-Phase) ---")
        
        print("\nGesamtstatistik (alle Stopps):")
        print(stop_df['euclidean_error'].describe().to_markdown(numalign="left", stralign="left"))
        
        print("\nStatistik pro Stopp-Nummer:")
        print(stop_df.groupby('stop_number')['euclidean_error'].describe().to_markdown(numalign="left", stralign="left"))

        sns.set(style='whitegrid')
        
        plt.figure(figsize=(12, 7))
        sns.boxplot(data=stop_df, x='stop_number', y='euclidean_error')
        plt.title('Fehlerverteilung (euklidische Distanz) pro Stopp', fontsize=16)
        plt.xlabel('Stopp-Nummer', fontsize=12)
        plt.ylabel('Euklidischer Fehler [m]', fontsize=12) 
        plt.tight_layout()
        plt.savefig('stop_phase_error_boxplot.png')

        plt.figure(figsize=(10, 10))
        
        gt_stops = stop_df.drop_duplicates('stop_number')[['stop_number', 'gt_pos_x', 'gt_pos_y']]
        
        sns.scatterplot(
            data=stop_df, 
            x='pos_x', 
            y='pos_y', 
            hue='stop_number', 
            palette='deep', 
            s=15, 
            alpha=0.7
        )
        
        sns.scatterplot(
            data=gt_stops, 
            x='gt_pos_x', 
            y='gt_pos_y', 
            color='red', 
            marker='X', 
            s=200, 
            edgecolor='black', 
            label='Ground Truth Position'
        )
        
        plt.title('Gemessene Positionen vs. Ground Truth (Stopp-Phasen)', fontsize=16)
        plt.xlabel('X-Position [m]', fontsize=12)
        plt.ylabel('Y-Position [m]', fontsize=12)
        plt.legend(title='Stopp-Nummer / Typ', bbox_to_anchor=(1.05, 1), loc='upper left')
        plt.axis('equal') 
        plt.tight_layout()
        plt.savefig('stop_phase_position_scatter.png')

        plt.figure(figsize=(15, 7))
        sns.scatterplot(
            data=stop_df, 
            x='time_s', 
            y='euclidean_error', 
            hue='stop_number', 
            palette='deep', 
            s=10, 
            alpha=0.6,
            edgecolor=None 
        )
        plt.title('Euklidischer Fehler über die Zeit (nur Stopp-Phasen)', fontsize=16)
        plt.xlabel('Experimentzeit [s]', fontsize=12)
        plt.ylabel('Euklidischer Fehler [m]', fontsize=12)
        plt.legend(title='Stopp-Nummer')
        plt.tight_layout()
        plt.savefig('stop_phase_error_over_time.png')

        print(f"\nAnalyse abgeschlossen. {len(stop_df)} 'Stop'-Datenpunkte analysiert.")
        print("Plots erstellt: 'stop_phase_error_boxplot.png', 'stop_phase_position_scatter.png', 'stop_phase_error_over_time.png'")

except FileNotFoundError as e:
    print(f"Fehler: Datei nicht gefunden. {e}")
except Exception as e:
    print(f"Ein Fehler ist aufgetreten: {e}")