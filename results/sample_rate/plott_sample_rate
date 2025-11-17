import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

file_map = {
    'uwb_sample_los.csv': 'LOS',
    'uwb_sample_wlos.csv': 'WLOS',
    'uwb_sample_nlos.csv': 'NLOS'
}

all_anchor_data = []

print("Lese und verarbeite CSV-Dateien für die detaillierte Ansicht...")

try:
    for file_name, scenario_name in file_map.items():
        df = pd.read_csv(file_name, index_col=0)
        df_anchors_only = df.drop('Total', errors='ignore')
        df_anchors_only['Szenario'] = scenario_name
        all_anchor_data.append(df_anchors_only)
        
        print(f"Datei {file_name} verarbeitet (nur Anker-Daten).")

    combined_df = pd.concat(all_anchor_data)
    combined_df = combined_df.reset_index()

    scenario_order = ['LOS', 'WLOS', 'NLOS']
    combined_df['Szenario'] = pd.Categorical(combined_df['Szenario'], categories=scenario_order, ordered=True)
    combined_df = combined_df.sort_values('Szenario')

    print("\n--- Kombinierte Daten für den detaillierten Plot ---")
    print(combined_df.to_markdown(index=False, numalign="left", stralign="left"))

    sns.set(style="whitegrid")
    plt.figure(figsize=(14, 8))
    
    ax = sns.barplot(
        data=combined_df,
        x='Szenario',
        y='received_percentage',
        hue='mac_address',
        palette='muted'
    )
    
    plt.ylim(0, 105)
    
    plt.title('UWB-Datenempfangsrate pro Anker und Szenario', fontsize=16)
    plt.xlabel('Sichtverbindungsszenario', fontsize=12)
    plt.ylabel('Empfangsrate [%]', fontsize=12)
    
    plt.legend(title='Anker MAC-Adresse', bbox_to_anchor=(1.02, 1), loc='upper left')
    
    for p in ax.patches:
        ax.annotate(
            f"{p.get_height():.2f}%", 
            (p.get_x() + p.get_width() / 2., p.get_height()),
            ha='center', 
            va='center',
            xytext=(0, 9),
            textcoords='offset points',
            fontsize=10
        )
        
    plt.tight_layout()
    plt.savefig('uwb_scenario_comparison_by_anchor.png')
    
    print("\nDiagramm 'uwb_scenario_comparison_by_anchor.png' erfolgreich erstellt.")
    
    # Zeigt das Diagramm an, wenn es lokal ausgeführt wird
    # plt.show() 

except FileNotFoundError as e:
    print(f"FEHLER: Datei nicht gefunden. {e}")
except KeyError as e:
    print(f"FEHLER: Spalte {e} oder Zeile 'Total' nicht in einer der CSVs gefunden.")
except Exception as e:
    print(f"Ein unerwarteter Fehler ist aufgetreten: {e}")