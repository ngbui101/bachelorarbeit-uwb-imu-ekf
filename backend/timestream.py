import sys
import threading
from collections import deque
import numpy as np
import paho.mqtt.client as mqtt
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore, QtGui
import csv
import os
import time 

# --- MQTT Konfiguration ---
MQTT_BROKER = ""
MQTT_PORT = 1883
MQTT_USER = ""
MQTT_PASS = ""
UWB_TOPIC = "uwb/data"
IMU_TOPIC = "imu/data"

# --- Plot-Konfiguration ---
MAX_DATA_POINTS = 500
KNOWN_MACS = {
    "e05a1b1fafc4": 0,
    "48e72903b3fc": 1,
    "83a8d3e15c4": 2
}
MAC_LABELS = list(KNOWN_MACS.keys())

COLOR_IMU_W = pg.mkPen(color=(0, 0, 0), width=2)       # Schwarz
COLOR_IMU_X = pg.mkPen(color=(200, 0, 0), width=2)       # Rot
COLOR_IMU_Y = pg.mkPen(color=(0, 150, 0), width=2)       # Grün
COLOR_IMU_Z = pg.mkPen(color=(0, 0, 200), width=2)       # Blau

COLOR_UWB_1 = pg.mkPen(color=(230, 90, 0), width=2)     # Orange
COLOR_UWB_2 = pg.mkPen(color=(0, 100, 255), width=2)   # Dunkleres Blau
COLOR_UWB_3 = pg.mkPen(color=(200, 0, 200), width=2)     # Magenta

COLOR_ACCEL_X_DOT = pg.mkPen(color=(255, 100, 100), width=1, style=QtCore.Qt.PenStyle.DotLine) # Dunkleres Hellrot
COLOR_ACCEL_Y_DOT = pg.mkPen(color=(100, 200, 100), width=1, style=QtCore.Qt.PenStyle.DotLine) # Dunkleres Hellgrün
COLOR_ACCEL_Z_DOT = pg.mkPen(color=(100, 100, 255), width=1, style=QtCore.Qt.PenStyle.DotLine) # Dunkleres Hellblau


# --- Globale Datenpuffer ---
imu_timestamps = deque(maxlen=MAX_DATA_POINTS)
imu_qw_data = deque(maxlen=MAX_DATA_POINTS)
imu_qx_data = deque(maxlen=MAX_DATA_POINTS)
imu_qy_data = deque(maxlen=MAX_DATA_POINTS)
imu_qz_data = deque(maxlen=MAX_DATA_POINTS)
imu_ax_data = deque(maxlen=MAX_DATA_POINTS)
imu_ay_data = deque(maxlen=MAX_DATA_POINTS)
imu_az_data = deque(maxlen=MAX_DATA_POINTS)

uwb_timestamps = deque(maxlen=MAX_DATA_POINTS)
uwb_dist_data = [deque(maxlen=MAX_DATA_POINTS) for _ in KNOWN_MACS]

data_lock = threading.Lock()

# --- MQTT Daten-Handler ---

def parse_imu_data(payload_str):
    """
    Parst den IMU-Payload basierend auf dem C-Code-Format:
    "%llu;%f,%f,%f;%f,%f,%f,%f"
    (timestamp; ax,ay,az; qw,qx,qy,qz)
    """
    try:
        parts = payload_str.strip().split(';')
        if len(parts) != 3: 
            print(f"IMU Format-Fehler: Erwarte 3 Teile, bekam {len(parts)}")
            return

        timestamp_ns = int(parts[0])
        group1_values = parts[1].split(',') # Sollte (ax, ay, az) sein
        group2_values = parts[2].split(',') # Sollte (qw, qx, qy, qz) sein
        
        # *** NEUER Sicherheitscheck ***
        # Gruppe 1 muss 3 Werte haben, Gruppe 2 muss 4 Werte haben
        if len(group1_values) != 3 or len(group2_values) != 4:
            print(f"IMU Längen-Fehler: Grp1={len(group1_values)} (erw. 3), Grp2={len(group2_values)} (erw. 4)")
            return

        # *** WERTE KORREKT ZUORDNEN ***
        
        # Gruppe 1: ax, ay, az
        imu_ax = float(group1_values[0])
        imu_ay = float(group1_values[1])
        imu_az = float(group1_values[2])
        
        # Gruppe 2: qw, qx, qy, qz
        imu_qw = float(group2_values[0])
        imu_qx = float(group2_values[1])
        imu_qy = float(group2_values[2])
        imu_qz = float(group2_values[3])
        
        with data_lock:
            # Daten in der korrekten Reihenfolge an die Puffer anhängen
            imu_timestamps.append(timestamp_ns)
            imu_qw_data.append(imu_qw)
            imu_qx_data.append(imu_qx)
            imu_qy_data.append(imu_qy)
            imu_qz_data.append(imu_qz)
            imu_ax_data.append(imu_ax)
            imu_ay_data.append(imu_ay)
            imu_az_data.append(imu_az)
            
    except Exception as e:
        print(f"IMU Parse-Fehler: {e} -- Payload: {payload_str}")


def parse_uwb_data(payload_str):
    """Parst UWB: 'timestamp;mac,dist;mac,dist;...'"""
    try:
        parts = payload_str.strip().split(';')
        if len(parts) < 2: return

        timestamp_ns = int(parts[0])
        
        macs_in_this_frame = set()

        for device_data in parts[1:]:
            if not device_data: continue
            mac, dist_str = device_data.split(',')
            
            if mac in KNOWN_MACS:
                mac_index = KNOWN_MACS[mac]
                distance = float(dist_str)
                
                with data_lock:
                    uwb_dist_data[mac_index].append(distance)
                
                macs_in_this_frame.add(mac_index)

        with data_lock:
            uwb_timestamps.append(timestamp_ns)
            for i in range(len(KNOWN_MACS)):
                if i not in macs_in_this_frame:
                    if len(uwb_dist_data[i]) > 0:
                        last_val = uwb_dist_data[i][-1] # Nimm den letzten bekannten Wert
                        uwb_dist_data[i].append(last_val)
                    else:
                        uwb_dist_data[i].append(0) # Startwert, falls noch nie gesehen

    except Exception as e:
        print(f"UWB Parse-Fehler: {e} -- Payload: {payload_str}")

# --- MQTT Callback-Funktionen ---
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("MQTT-Plotter: Verbunden.")
        client.subscribe([(UWB_TOPIC, 0), (IMU_TOPIC, 0)])
    else:
        print(f"MQTT-Plotter: Verbindung fehlgeschlagen (Code {rc})")

def on_message(client, userdata, msg):
    try:
        payload_str = msg.payload.decode('utf-8')
        
        if msg.topic == IMU_TOPIC:
            parse_imu_data(payload_str)
        elif msg.topic == UWB_TOPIC:
            parse_uwb_data(payload_str)
            
    except Exception as e:
        print(f"on_message Fehler: {e}")

def mqtt_thread_func():
    """Diese Funktion läuft in einem separaten Thread, um MQTT zu verwalten."""
    client = mqtt.Client() 
    
    client.username_pw_set(MQTT_USER, MQTT_PASS)
    client.on_connect = on_connect
    client.on_message = on_message
    
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_forever()
    except Exception as e:
        print(f"MQTT Thread Fehler: {e}")
    print("MQTT Thread beendet.")


# --- Haupt-GUI-Anwendung ---
if __name__ == '__main__':
    # 1. Starte den MQTT-Thread
    mqtt_thread = threading.Thread(target=mqtt_thread_func, daemon=True)
    mqtt_thread.start()

    # 2. Erstelle die PyQt-Anwendung
    app = QtWidgets.QApplication(sys.argv)
    
    # 3. Erstelle das Hauptfenster
    win = QtWidgets.QMainWindow()
    win.setWindowTitle("ESP32 Sensor-Dashboard")
    win.resize(1200, 800)

    # Haupt-Widget für das Layout
    central_widget = QtWidgets.QWidget()
    win.setCentralWidget(central_widget)
    
    # Vertikales Layout für Plots und Button
    main_layout = QtWidgets.QVBoxLayout(central_widget)

    # GraphicsLayoutWidget für die Plots
    plot_widget = pg.GraphicsLayoutWidget()

    plot_widget.setBackground('w') 
    
    main_layout.addWidget(plot_widget)

    # 4. Erstelle den oberen Plot (IMU)
    plot_imu = plot_widget.addPlot(title="IMU Daten (Quaternion & Linearbeschleunigung)")
    legend_imu = plot_imu.addLegend(labelTextColor='k')


    plot_imu.setLabel('left', 'Wert', color='k')
    plot_imu.setLabel('bottom', 'Zeit (s)', color='k')
    plot_imu.showGrid(x=True, y=True) 
    
    # Text-Elemente der Achsen anpassen
    plot_imu.getAxis('left').setTextPen('k')
    plot_imu.getAxis('bottom').setTextPen('k')
    plot_imu.getAxis('left').setPen('k')
    plot_imu.getAxis('bottom').setPen('k')
    
    # Quaternionen
    imu_curve_qw = plot_imu.plot(pen=COLOR_IMU_W, name='Qw')
    imu_curve_qx = plot_imu.plot(pen=COLOR_IMU_X, name='Qx')
    imu_curve_qy = plot_imu.plot(pen=COLOR_IMU_Y, name='Qy')
    imu_curve_qz = plot_imu.plot(pen=COLOR_IMU_Z, name='Qz')

    # Beschleunigungen
    imu_curve_ax = plot_imu.plot(pen=COLOR_ACCEL_X_DOT, name='Ax')
    imu_curve_ay = plot_imu.plot(pen=COLOR_ACCEL_Y_DOT, name='Ay')
    imu_curve_az = plot_imu.plot(pen=COLOR_ACCEL_Z_DOT, name='Az')

    # Gehe zur nächsten Zeile im Layout
    plot_widget.nextRow() 

    # 5. Erstelle den unteren Plot (UWB)
    plot_uwb = plot_widget.addPlot(title="UWB-Distanzen (Meter)")
    legend_uwb = plot_uwb.addLegend(labelTextColor='k')

    plot_uwb.setLabel('left', 'Distanz (m)', color='k')
    plot_uwb.setLabel('bottom', 'Zeit (s)', color='k')
    plot_uwb.showGrid(x=True, y=True)
    
    plot_uwb.getAxis('left').setTextPen('k')
    plot_uwb.getAxis('bottom').setTextPen('k')
    plot_uwb.getAxis('left').setPen('k')
    plot_uwb.getAxis('bottom').setPen('k')
    
    uwb_curve_1 = plot_uwb.plot(pen=COLOR_UWB_1, name=MAC_LABELS[0])
    uwb_curve_2 = plot_uwb.plot(pen=COLOR_UWB_2, name=MAC_LABELS[1])
    uwb_curve_3 = plot_uwb.plot(pen=COLOR_UWB_3, name=MAC_LABELS[2])
    uwb_curves = [uwb_curve_1, uwb_curve_2, uwb_curve_3]

    # --- Speichern-Funktion ---
    def save_data_to_csv():
        with data_lock:
            local_imu_timestamps = list(imu_timestamps)
            local_imu_qw_data = list(imu_qw_data)
            local_imu_qx_data = list(imu_qx_data)
            local_imu_qy_data = list(imu_qy_data)
            local_imu_qz_data = list(imu_qz_data)
            local_imu_ax_data = list(imu_ax_data)
            local_imu_ay_data = list(imu_ay_data)
            local_imu_az_data = list(imu_az_data)
            
            local_uwb_timestamps = list(uwb_timestamps)
            local_uwb_dist_data = [list(d) for d in uwb_dist_data]
            
        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        
        # IMU-Daten speichern
        imu_filename = f"imu_data_plot_{timestamp_str}.csv"
        try:
            with open(imu_filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp_ns', 'timestamp_s_relative', 'qw', 'qx', 'qy', 'qz', 'ax', 'ay', 'az'])
                if local_imu_timestamps:
                    t0 = local_imu_timestamps[0]
                    min_len = min(len(local_imu_timestamps), len(local_imu_qw_data), len(local_imu_qx_data), 
                                  len(local_imu_qy_data), len(local_imu_qz_data), len(local_imu_ax_data), 
                                  len(local_imu_ay_data), len(local_imu_az_data))
                    
                    if min_len != len(local_imu_timestamps):
                         print("WARNUNG: IMU-Daten-Inkonsistenz beim Speichern entdeckt.")

                    for i in range(min_len):
                        writer.writerow([
                            local_imu_timestamps[i],
                            (local_imu_timestamps[i] - t0) / 1e9,
                            local_imu_qw_data[i],
                            local_imu_qx_data[i],
                            local_imu_qy_data[i],
                            local_imu_qz_data[i],
                            local_imu_ax_data[i],
                            local_imu_ay_data[i],
                            local_imu_az_data[i]
                        ])
            print(f"IMU-Plotdaten gespeichert: {imu_filename}")
        except IOError as e:
            print(f"FEHLER beim Speichern der IMU-Daten: {e}")
            QtWidgets.QMessageBox.warning(win, "Speicherfehler", f"Konnte IMU-Daten nicht speichern: {e}")
            return
        except IndexError:
            print("FEHLER: Inkonsistente IMU-Daten beim Speichern.")
            QtWidgets.QMessageBox.warning(win, "Speicherfehler", "Inkonsistente IMU-Datenlänge beim Speichern.")
            return

        # UWB-Daten speichern
        uwb_filename = f"uwb_data_plot_{timestamp_str}.csv"
        try:
            with open(uwb_filename, 'w', newline='') as f:
                writer = csv.writer(f)
                uwb_header = ['timestamp_ns', 'timestamp_s_relative'] + [f'distance_{mac}' for mac in MAC_LABELS]
                writer.writerow(uwb_header) 
                
                if local_uwb_timestamps:
                    t0 = local_uwb_timestamps[0]
                    # Sicherstellen, dass alle Listen dieselbe Länge haben
                    min_len = len(local_uwb_timestamps)
                    for j in range(len(KNOWN_MACS)):
                        min_len = min(min_len, len(local_uwb_dist_data[j]))
                    
                    if min_len != len(local_uwb_timestamps):
                         print("WARNUNG: UWB-Daten-Inkonsistenz beim Speichern entdeckt.")

                    for i in range(min_len):
                        row = [local_uwb_timestamps[i], (local_uwb_timestamps[i] - t0) / 1e9]
                        for j in range(len(KNOWN_MACS)):
                            row.append(local_uwb_dist_data[j][i])
                        writer.writerow(row)
            print(f"UWB-Plotdaten gespeichert: {uwb_filename}")
        except IOError as e:
            print(f"FEHLER beim Speichern der UWB-Daten: {e}")
            QtWidgets.QMessageBox.warning(win, "Speicherfehler", f"Konnte UWB-Daten nicht speichern: {e}")
            return
        except IndexError:
            print("FEHLER: Inkonsistente UWB-Daten beim Speichern.")
            QtWidgets.QMessageBox.warning(win, "Speicherfehler", "Inkonsistente UWB-Datenlänge beim Speichern.")
            return
        
        QtWidgets.QMessageBox.information(win, "Speichern Erfolgreich", 
                                            f"Daten in '{imu_filename}' und '{uwb_filename}' gespeichert.")

    # 6. Button zum Speichern hinzufügen
    save_button = QtWidgets.QPushButton("Plotdaten als CSV speichern")
    main_layout.addWidget(save_button)
    save_button.clicked.connect(save_data_to_csv)

    # 7. Die Update-Funktion, die die Plots aktualisiert
    def update_plots():
        with data_lock:
            if imu_timestamps:
                imu_ts_list = list(imu_timestamps)
                imu_qw_list = list(imu_qw_data)
                imu_qx_list = list(imu_qx_data)
                imu_qy_list = list(imu_qy_data)
                imu_qz_list = list(imu_qz_data)
                imu_ax_list = list(imu_ax_data)
                imu_ay_list = list(imu_ay_data)
                imu_az_list = list(imu_az_data)
                
                if not imu_ts_list: 
                    return
                
                min_len = min(len(imu_ts_list), len(imu_qw_list), len(imu_qx_list), 
                              len(imu_qy_list), len(imu_qz_list), len(imu_ax_list), 
                              len(imu_ay_list), len(imu_az_list))
                
                if min_len < len(imu_ts_list):
                    imu_ts_list = imu_ts_list[:min_len]
                    imu_qw_list = imu_qw_list[:min_len]
                    imu_qx_list = imu_qx_list[:min_len]
                    imu_qy_list = imu_qy_list[:min_len]
                    imu_qz_list = imu_qz_list[:min_len]
                    imu_ax_list = imu_ax_list[:min_len]
                    imu_ay_list = imu_ay_list[:min_len]
                    imu_az_list = imu_az_list[:min_len]

                if not imu_ts_list: #
                    return

                t0_imu = imu_ts_list[0]
                x_imu = [(t - t0_imu) / 1e9 for t in imu_ts_list]
                
                imu_curve_qw.setData(x=x_imu, y=imu_qw_list)
                imu_curve_qx.setData(x=x_imu, y=imu_qx_list)
                imu_curve_qy.setData(x=x_imu, y=imu_qy_list)
                imu_curve_qz.setData(x=x_imu, y=imu_qz_list)
                imu_curve_ax.setData(x=x_imu, y=imu_ax_list)
                imu_curve_ay.setData(x=x_imu, y=imu_ay_list)
                imu_curve_az.setData(x=x_imu, y=imu_az_list)

            # --- UWB-Plot aktualisieren ---
            if uwb_timestamps:
                uwb_ts_list = list(uwb_timestamps)
                
                if not uwb_ts_list: # Fängt leere Liste ab
                    return
                
                t0_uwb = uwb_ts_list[0]
                x_uwb = [(t - t0_uwb) / 1e9 for t in uwb_ts_list]
                
                # Datenlisten kopieren und auf die Länge der Zeitstempel kürzen
                min_len = len(x_uwb)
                uwb_data_lists = []
                for i in range(len(KNOWN_MACS)):
                    uwb_data_lists.append(list(uwb_dist_data[i]))
                    min_len = min(min_len, len(uwb_data_lists[i]))

                if min_len < len(x_uwb):
                    # print("UWB-Plot-Warnung: Datenlängen inkonsistent, kürze auf Minimum.")
                    x_uwb = x_uwb[:min_len]
                    for i in range(len(KNOWN_MACS)):
                        uwb_data_lists[i] = uwb_data_lists[i][:min_len]
                
                if not x_uwb: # Erneute Prüfung
                    return

                for i, curve in enumerate(uwb_curves):
                    curve.setData(x=x_uwb, y=uwb_data_lists[i])

                        
    # 8. Starte einen Timer, der die 'update_plots' Funktion aufruft
    timer = QtCore.QTimer()
    timer.timeout.connect(update_plots)
    timer.start(33) # Update alle ~30 FPS

    # 9. Zeige das Hauptfenster an
    win.show()
    
    # Beendet das Programm, wenn das Fenster geschlossen wird
    sys.exit(app.exec())