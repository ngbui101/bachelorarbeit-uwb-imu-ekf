# UWB-IMU-Sensorfusion mit Extended Kalman Filter

Hardware: ESP32+DW3000+BNO085

**➡️ [Link zur Videodemonstration des Versuchsaufbaus in LOS](https://youtu.be/Rbanm2skb94)**

## 🚀Ergebnisse

Der EKF kompensiert Rauschen und Datenverlust, scheitert jedoch an systematischem Bias durch Wände.

| Szenario | Algorithmus | RMSE (m) | 95% CDF (m) |
| :--- | :--- | :--- | :--- |
| **LOS** | Trilateration | 0,078 | 0,139 |
| (Ideal) | **EKF-Fusion** | **0,056** | **0,086** |
| | *Verbesserung* | *28,2%* | *38,1%* |
| --- | --- | --- | --- |
| **WLOS** | Trilateration | 0,520 | 0,858 |
| (1 Wand) | **EKF-Fusion** | **0,518** | **0,817** |
| | *Verbesserung* | *0,4%* | *4,8%* |
| --- | --- | --- | --- |
| **NLOS** | Trilateration¹ | 0,608 | 0,967 |
| (Stark) | **EKF-Fusion**² | **0,219** | **0,330** |
| | *Verbesserung* | *64,0%* | *65,9%* |


## 🛠️ Methodik & Algorithmen

Die Distanzmessung erfolgte mittels DS-TWR. 
Die Fusion der UWB- und IMU-Daten wurde über einen EKF realisiert. 

![DS-TWR Verfahren](figures/DS-TWR.png)
* Schematische Darstellung des DS-TWR-Verfahrens

![EKF Ablaufdiagramm](figures/EKF.png)
* Ablaufdiagramm des implementierten EKF-Algorithmus
## 📁 Repository-Struktur
* `/firmware/`: PlatformIO C++ Code für die ESP32-Geräte (Tag und Anker).
* `/backend/`: Python-Skripte für die Datenverarbeitung, EKF-Implementierung und Analyse.
* `/results/`: Experimentelle Rohdaten (CSV) und generierte Plots, sortiert nach Szenarien (exp1_1, exp2_1, ...).
