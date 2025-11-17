import sys
import threading
from collections import deque
import numpy as np
import paho.mqtt.client as mqtt
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore, QtGui
from scipy.optimize import least_squares 

# --- MQTT-Konfiguration ---
MQTT_BROKER = ""
MQTT_PORT = 1883
MQTT_USER = ""
MQTT_PASS = ""
UWB_TOPIC = "uwb/data"
IMU_TOPIC = "imu/data"

# --- Raum- und Anker-Konfiguration ---
ANCHOR_POSITIONS = {
    "e05a1b1fafc4": np.array([2.8, 0, 1.31]),
    "48e72903b3fc": np.array([0.1, 0, 2.0]),
    "83a8d3e15c4": np.array([1.86, 4.1, 2.10])
}

KNOWN_MACS = list(ANCHOR_POSITIONS.keys())

ROOM_X_DIM = 2.9
ROOM_Y_DIM = 4.1
TAG_HEIGHT = 0.015 

# --- Globale Variablen ---
POSITION_HISTORY_LENGTH = 200 
current_distances = {}
current_quaternion = None
position_history = deque(maxlen=POSITION_HISTORY_LENGTH)
data_lock = threading.Lock()
circle_plots = {}

last_pos = np.array([2.07, 0.70]) 

# --- Berechnungsfunktionen ---

def project_to_2d(dist_3d, anchor_h, tag_h):
    """Projiziert 3D-Distanz auf 2D-Ebene basierend auf Höhendifferenz."""
    h_diff = abs(anchor_h - tag_h)
    if dist_3d < h_diff:
        return 0.01 # Physikalischer Fehler 
    return np.sqrt(dist_3d**2 - h_diff**2)

def trilateration_residuals(pos_2d, anchor_positions_2d, distances_2d):
    """Residuen-Funktion für Least-Squares Optimierung."""
    residuals = []
    for i, anchor_pos in enumerate(anchor_positions_2d):
        dist_pred = np.linalg.norm(pos_2d - anchor_pos)
        residuals.append(dist_pred - distances_2d[i])
    return residuals

def calculate_position(distances_dict):
    """
    Berechnet die 2D-Position mittels 'least_squares', 
    basierend auf dem Algorithmus von Skript 1.
    """
    global last_pos 

    active_anchors_2d = []
    active_dists_2d = []

    # 1. Daten vorbereiten: 3D-Distanzen auf 2D projizieren
    for mac, dist_raw in distances_dict.items():
        if mac in ANCHOR_POSITIONS:
            anchor_pos_3d = ANCHOR_POSITIONS[mac]

            dist_2d = project_to_2d(dist_raw, anchor_pos_3d[2], TAG_HEIGHT)
            
            active_anchors_2d.append(anchor_pos_3d[:2]) # Nur (x, y)
            active_dists_2d.append(dist_2d)
    if len(active_dists_2d) < 3:
        return None 
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
            return pos_est
        else:
            return None # Optimierung fehlgeschlagen
    except Exception:
        return None

# --- Orientierungsfunktion ---
def get_orientation_vectors(q):
    """ 
    Berechnet die 2D-Projektion der lokalen X-, Y- und Z-Achsen
    aus einem Quaternion (qw, qx, qy, qz).
    """
    qw, qx, qy, qz = q
    
    # Lokale X-Achse (R[0,0], R[1,0])
    vec_x_x = 1.0 - 2.0 * (qy*qy + qz*qz)
    vec_x_y = 2.0 * (qx*qy + qw*qz)
    
    # Lokale Y-Achse (R[0,1], R[1,1])
    vec_y_x = 2.0 * (qx*qy - qw*qz)
    vec_y_y = 1.0 - 2.0 * (qx*qx + qz*qz)
    
    # Lokale Z-Achse (R[0,2], R[1,2])
    vec_z_x = 2.0 * (qx*qz + qw*qy)
    vec_z_y = 2.0 * (qy*qz - qw*qx)
    
    return (vec_x_x, vec_x_y), (vec_y_x, vec_y_y), (vec_z_x, vec_z_y)

# --- MQTT-Datenparser ---

def parse_uwb_data(payload_str):
    global current_distances
    try:
        parts = payload_str.strip().split(';')
        if len(parts) < 2: return
        
        with data_lock:
            for device_data in parts[1:]:
                if not device_data: continue
                mac, dist_str = device_data.split(',')
                if mac in KNOWN_MACS:
                    current_distances[mac] = float(dist_str)
                    
    except Exception:
        pass 

def parse_imu_data(payload_str):
    global current_quaternion
    try:
        parts = payload_str.strip().split(';')
        if len(parts) != 3: return
        q_parts = parts[2].split(',')
        if len(q_parts) != 4: return
        
        qw = float(q_parts[0])
        qx = float(q_parts[1])
        qy = float(q_parts[2])
        qz = float(q_parts[3])
        
        with data_lock:
            norm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
            if norm > 1e-6:
                current_quaternion = [qw/norm, qx/norm, qy/norm, qz/norm]
                
    except Exception:
        pass

# --- MQTT-Callbacks und Thread  ---

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Verbunden mit MQTT Broker.")
        client.subscribe(UWB_TOPIC)
        client.subscribe(IMU_TOPIC)
    else:
        print(f"MQTT Connect failed code {rc}")

def on_message(client, userdata, msg):
    try:
        payload_str = msg.payload.decode('utf-8')
        if msg.topic == UWB_TOPIC:
            parse_uwb_data(payload_str)
        elif msg.topic == IMU_TOPIC:
            parse_imu_data(payload_str)
    except Exception:
        pass 

def mqtt_thread_func():
    # DeprecationWarning behoben
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1) 
    client.username_pw_set(MQTT_USER, MQTT_PASS)
    client.on_connect = on_connect
    client.on_message = on_message
    
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_forever()
    except Exception as e:
        print(f"MQTT Thread Exception: {e}")

# --- Hauptanwendung (PyQtGraph) ---
if __name__ == '__main__':
    mqtt_thread = threading.Thread(target=mqtt_thread_func, daemon=True)
    mqtt_thread.start()

    app = QtWidgets.QApplication(sys.argv)
    
    win = QtWidgets.QMainWindow()
    win.setWindowTitle("ESP32 2D-Position & IMU-Orientierung (Skript 1 Logik)")
    win.resize(800, 750) 

    central_widget = QtWidgets.QWidget()
    win.setCentralWidget(central_widget)
    main_layout = QtWidgets.QVBoxLayout(central_widget)

    plot_widget = pg.GraphicsLayoutWidget()
    plot_widget.setBackground('w') 
    main_layout.addWidget(plot_widget) 
    
    plot_2d = plot_widget.addPlot(title="2D-Position (Top-Down-Ansicht)")
    plot_2d.setLabel('bottom', 'X-Position (m)')
    plot_2d.setLabel('left', 'Y-Position (m)')
    
    plot_2d.setAspectLocked(True)
    plot_2d.showGrid(x=True, y=True)
    plot_2d.addLegend()

    # --- Raum und Anker zeichnen ---
    room_x = [0, ROOM_X_DIM, ROOM_X_DIM, 0, 0]
    room_y = [0, 0, ROOM_Y_DIM, ROOM_Y_DIM, 0]
    plot_2d.plot(room_x, room_y, 
                 pen=pg.mkPen('k', style=QtCore.Qt.PenStyle.DashLine, width=2), 
                 name="Raumgrundriss")
    
    # --- Flur 1 hinzufügen ---
    flur_x = [1.85, 2.9, 2.9, 1.85, 1.85]
    flur_y = [4.1, 4.1, 6.4, 6.4, 4.1]
    plot_2d.plot(flur_x, flur_y,
                 pen=pg.mkPen('k', style=QtCore.Qt.PenStyle.DashLine, width=2),
                 name="Flur 1")
    
    # --- Flur 2 hinzufügen ---
    flur2_x = [2.9, 2.9, 6.58, 6.58, 2.9]
    flur2_y = [4.1, 6.0, 6.0,  4.1, 4.1]
    plot_2d.plot(flur2_x, flur2_y,
                 pen=pg.mkPen('k', style=QtCore.Qt.PenStyle.DashLine, width=2),
                 name="Flur 2")
    # --- Ende Flur ---
    
    anchor_x_coords = []
    anchor_y_coords = []
    
    for mac, pos in ANCHOR_POSITIONS.items():
        x, y = pos[0], pos[1]
        anchor_x_coords.append(x)
        anchor_y_coords.append(y)
        
        label_text = f"...{mac[-6:]}"
        text_label = pg.TextItem(text=label_text, color='k', anchor=(0.5, 0))
        plot_2d.addItem(text_label)
        text_label.setPos(x, y)

    plot_2d.plot(anchor_x_coords, anchor_y_coords, 
                 pen=None, symbol='x', symbolPen='r', 
                 symbolBrush='r', symbolSize=15, name="Anker")

    gt_x = np.arange(2.85, 4.6, 0.1) 
    gt_y = np.full(gt_x.shape, 4.7) 
    
    ground_truth_points = plot_2d.plot(
        gt_x, gt_y, 
        pen=None, 
        symbol='o', 
        symbolPen='g', 
        symbolBrush=(0, 200, 0, 150), 
        symbolSize=5, 
        name="Ground Truth"
    )
    
    # --- Trilaterationskreise ---
    circle_pen = pg.mkPen(color=(100, 100, 100), style=QtCore.Qt.PenStyle.DotLine, width=1)
    for mac in KNOWN_MACS:
        circle = plot_2d.plot(pen=circle_pen, name=f"Distanz {mac[-6:]}")
        circle.setVisible(False) 
        circle_plots[mac] = circle

    # --- Positions-Plot-Elemente ---
    position_trail_curve = plot_2d.plot(pen='c', name="Berechnete Position") 
    current_pos_dot = plot_2d.plot(pen=None, symbol='o', symbolBrush=(0,0,200), symbolPen='k', symbolSize=10)

    # --- IMU Orientierungs-Pfeile ---
    IMU_DISPLAY_ORIGIN = (-1.0, 0.0) # Statische START-Position
    IMU_AXIS_LENGTH = 0.5            # Länge der Linien
    LABEL_OFFSET_PLOT_UNITS = 0.1    # Abstand der Labels "X,Y,Z"
    
    axis_label_font = QtGui.QFont()
    axis_label_font.setPointSize(10)
    axis_label_font.setBold(True)

    # X-Achse (Rot)
    imu_x_line = plot_2d.plot(pen=pg.mkPen('r', width=3), name="IMU X-Achse")
    imu_x_label = pg.TextItem(text="X", color='r', anchor=(0.5, 0.5))
    imu_x_label.setFont(axis_label_font)
    plot_2d.addItem(imu_x_label)

    # Y-Achse (Grün)
    imu_y_line = plot_2d.plot(pen=pg.mkPen('g', width=3), name="IMU Y-Achse")
    imu_y_label = pg.TextItem(text="Y", color='g', anchor=(0.5, 0.5))
    imu_y_label.setFont(axis_label_font)
    plot_2d.addItem(imu_y_label)

    # Z-Achse (Blau)
    imu_z_line = plot_2d.plot(pen=pg.mkPen('b', width=3), name="IMU Z-Achse")
    imu_z_label = pg.TextItem(text="Z", color='b', anchor=(0.5, 0.5))
    imu_z_label.setFont(axis_label_font)
    plot_2d.addItem(imu_z_label)
    
    imu_origin_label = pg.TextItem(text="IMU Referenz", color='k', anchor=(0.5, 1))
    imu_origin_label.setFont(axis_label_font)
    plot_2d.addItem(imu_origin_label)
    
    all_imu_items = [
        imu_x_line, imu_x_label,
        imu_y_line, imu_y_label,
        imu_z_line, imu_z_label,
        imu_origin_label
    ]
    for item in all_imu_items:
        item.hide()
    # --- Ende IMU-Block ---

    pos_text_label = pg.TextItem(text="Position (X, Y): ---", color='k', anchor=(0, 1))
    pos_font = QtGui.QFont()
    pos_font.setPointSize(12)
    pos_font.setBold(True)
    pos_text_label.setFont(pos_font)
    plot_2d.addItem(pos_text_label)
    pos_text_label.setPos(0, ROOM_Y_DIM) 

    # --- Checkboxen ---
    checkbox_gt = QtWidgets.QCheckBox("Ground Truth anzeigen")
    checkbox_gt.setChecked(True) 
    main_layout.addWidget(checkbox_gt) 
    
    def toggle_ground_truth(state):
        is_visible = (state == QtCore.Qt.CheckState.Checked)
        ground_truth_points.setVisible(is_visible)
    
    checkbox_gt.stateChanged.connect(toggle_ground_truth)

    checkbox_circles = QtWidgets.QCheckBox("Trilaterationskreise anzeigen")
    checkbox_circles.setChecked(False) 
    main_layout.addWidget(checkbox_circles) 
    
    def toggle_circles(state):
        is_visible = (state == QtCore.Qt.CheckState.Checked)
        for circle in circle_plots.values():
            circle.setVisible(is_visible)
    
    checkbox_circles.stateChanged.connect(toggle_circles)

    # --- Hilfsvariablen für Kreise ---
    t = np.linspace(0, 2 * np.pi, 100)
    circle_cos = np.cos(t)
    circle_sin = np.sin(t)

    # --- Update-Funktion (Timer) ---
    def update_plots():
        global position_history, current_distances, current_quaternion
        
        raw_position = None
        q = None
        
        with data_lock:
            if len(current_distances) > 0:
                raw_position = calculate_position(current_distances.copy())
            
            if current_quaternion is not None:
                q = current_quaternion
        
        # --- Positions-Plot aktualisieren ---
        if raw_position is not None:
            position_history.append(raw_position)
            
            trail_data = list(position_history)
            x_data = [p[0] for p in trail_data]
            y_data = [p[1] for p in trail_data]
            
            position_trail_curve.setData(x=x_data, y=y_data)
            
            x_pos = x_data[-1]
            y_pos = y_data[-1]
            current_pos_dot.setData(x=[x_pos], y=[y_pos])
            
            pos_text_label.setText(f"Position (X, Y): ({x_pos:.2f}, {y_pos:.2f}) m")
            
            # --- IMU-Orientierung aktualisieren ---
            if q is not None:
                vec_x_2d, vec_y_2d, vec_z_2d = get_orientation_vectors(q)
                
                origin_x, origin_y = IMU_DISPLAY_ORIGIN
                
                # --- X-Achse ---
                end_x_x = origin_x + vec_x_2d[0] * IMU_AXIS_LENGTH
                end_x_y = origin_y + vec_x_2d[1] * IMU_AXIS_LENGTH
                
                imu_x_line.setData(x=[origin_x, end_x_x], y=[origin_y, end_x_y])
                imu_x_label.setPos(end_x_x + vec_x_2d[0] * LABEL_OFFSET_PLOT_UNITS, 
                                   end_x_y + vec_x_2d[1] * LABEL_OFFSET_PLOT_UNITS)

                # --- Y-Achse ---
                end_y_x = origin_x + vec_y_2d[0] * IMU_AXIS_LENGTH
                end_y_y = origin_y + vec_y_2d[1] * IMU_AXIS_LENGTH

                imu_y_line.setData(x=[origin_x, end_y_x], y=[origin_y, end_y_y])
                imu_y_label.setPos(end_y_x + vec_y_2d[0] * LABEL_OFFSET_PLOT_UNITS, 
                                   end_y_y + vec_y_2d[1] * LABEL_OFFSET_PLOT_UNITS)
                
                # --- Z-Achse ---
                end_z_x = origin_x + vec_z_2d[0] * IMU_AXIS_LENGTH
                end_z_y = origin_y + vec_z_2d[1] * IMU_AXIS_LENGTH

                imu_z_line.setData(x=[origin_x, end_z_x], y=[origin_y, end_z_y])
                imu_z_label.setPos(end_z_x + vec_z_2d[0] * LABEL_OFFSET_PLOT_UNITS, 
                                   end_z_y + vec_z_2d[1] * LABEL_OFFSET_PLOT_UNITS)

                # --- Label und Sichtbarkeit ---
                max_y = max(origin_y, end_x_y, end_y_y, end_z_y, 
                            imu_x_label.pos().y(), imu_y_label.pos().y(), imu_z_label.pos().y())
                imu_origin_label.setPos(origin_x, max_y + LABEL_OFFSET_PLOT_UNITS)

                for item in all_imu_items:
                    item.show()
            else:
                for item in all_imu_items:
                    item.hide()
            # --- Ende IMU-Block ---

            if checkbox_circles.isChecked():
                with data_lock:
                    local_distances = current_distances.copy()
                
                for mac, circle_plot in circle_plots.items():
                    if mac in local_distances:
                        anchor_pos = ANCHOR_POSITIONS[mac]
                        slant_dist = local_distances[mac]
                        
                        radius_2d = project_to_2d(slant_dist, anchor_pos[2], TAG_HEIGHT)
                        
                        if radius_2d > 0.01:
                            x_circle = anchor_pos[0] + radius_2d * circle_cos
                            y_circle = anchor_pos[1] + radius_2d * circle_sin
                            circle_plot.setData(x_circle, y_circle)
        else:
            # Pfeile und Labels ausblenden, wenn KEINE Position vorhanden ist
            for item in all_imu_items:
                item.hide()
            
    timer = QtCore.QTimer()
    timer.timeout.connect(update_plots)
    timer.start(50) 

    win.show()
    sys.exit(app.exec())