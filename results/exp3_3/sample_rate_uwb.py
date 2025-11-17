import pandas as pd

# --- Konfiguration ---
gt_file = 'mqtt_ground_truth.csv'
uwb_file = 'uwb_data_1.csv'
output_csv = 'uwb_data_1_summary.csv'

# Erwartete Messrate pro Anker
EXPECTED_RATE_HZ = 5
# ---------------------

print(f"Starte Analyse für {uwb_file}...")

try:
    # --- 1. Daten laden ---
    gt_df = pd.read_csv(gt_file)
    uwb_df = pd.read_csv(uwb_file)

    # --- 2. Datentyp-Konvertierung  ---
    if not pd.api.types.is_integer_dtype(gt_df['timestamp_ns']):
        gt_df['timestamp_ns'] = gt_df['timestamp_ns'].astype('int64')
        
    if not pd.api.types.is_integer_dtype(uwb_df['timestamp_ns']):
        uwb_df['timestamp_ns'] = uwb_df['timestamp_ns'].astype('int64')

    # --- 3. Anfangs- und Endzeit des Experiments bestimmen ---
    start_time_ns = gt_df['timestamp_ns'].min()
    end_time_ns = gt_df['timestamp_ns'].max()
    
    duration_ns = end_time_ns - start_time_ns
    duration_s = duration_ns / 1_000_000_000  # Umrechnung in Sekunden

    print(f"Experiment-Dauer (s): {duration_s:.2f}")

    # --- 4. UWB-Daten auf das Zeitfenster filtern ---
    uwb_filtered_df = uwb_df[
        (uwb_df['timestamp_ns'] >= start_time_ns) &
        (uwb_df['timestamp_ns'] <= end_time_ns)
    ].copy()
    
    if uwb_filtered_df.empty:
        print("FEHLER: Keine UWB-Daten im ermittelten Zeitfenster gefunden.")
    else:
        # --- 5. Anzahl Messungen pro Anker (mac_address) berechnen ---
        measurements_per_anchor = uwb_filtered_df.groupby('mac_address').size().to_frame(name='actual_measurements')
        
        unique_anchors = measurements_per_anchor.index.values
        num_anchors = len(unique_anchors)
        
        # --- 6. Erwartete Messungen und Fehlquellen berechnen ---
        expected_measurements_per_anchor = duration_s * EXPECTED_RATE_HZ
        total_expected_measurements = expected_measurements_per_anchor * num_anchors
        total_actual_measurements = measurements_per_anchor['actual_measurements'].sum()

        # --- 7. Summary-DataFrame erstellen ---
        summary_df = measurements_per_anchor
        
        summary_df['expected_measurements'] = expected_measurements_per_anchor
        summary_df['missing_measurements'] = summary_df['expected_measurements'] - summary_df['actual_measurements']
        summary_df['received_percentage'] = (summary_df['actual_measurements'] / summary_df['expected_measurements']) * 100
        
        # --- 8. Gesamt-Zeile hinzufügen ---
        total_row_data = {
            'actual_measurements': total_actual_measurements,
            'expected_measurements': total_expected_measurements,
            'missing_measurements': total_expected_measurements - total_actual_measurements,
            'received_percentage': (total_actual_measurements / total_expected_measurements) * 100
        }
        total_row_df = pd.DataFrame(total_row_data, index=['Total'])
        summary_df = pd.concat([summary_df, total_row_df])
        
        # --- 9. Formatierung ---
        summary_df['expected_measurements'] = summary_df['expected_measurements'].round(0).astype(int)
        summary_df['missing_measurements'] = summary_df['missing_measurements'].round(0).astype(int)
        summary_df['received_percentage'] = summary_df['received_percentage'].round(2)
        
        # --- 10. Als CSV speichern ---
        summary_df.to_csv(output_csv, index_label='mac_address')
        
        print(f"\n--- Zusammenfassung der UWB-Messungen (Szenario 1) ---")
        print(summary_df.to_markdown(numalign="left", stralign="left"))
        print(f"\nErgebnisse erfolgreich in '{output_csv}' gespeichert.")

except FileNotFoundError as e:
    print(f"FEHLER: Datei nicht gefunden. {e}")
except KeyError as e:
    print(f"FEHLER: Spalte nicht gefunden. Stelle sicher, dass die CSVs '{e}' enthalten.")
except Exception as e:
    print(f"Ein unerwarteter Fehler ist aufgetreten: {e}")