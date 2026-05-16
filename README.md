# 🔧 IoT-Based Predictive Maintenance System
### Using Vibration Analysis on Electric Motors

![ESP32](https://img.shields.io/badge/MCU-ESP32-blue)
![Python](https://img.shields.io/badge/Dashboard-Python%203-green)
![MQTT](https://img.shields.io/badge/Protocol-MQTT-orange)
![License](https://img.shields.io/badge/License-MIT-lightgrey)

## 📌 Overview
A low-cost IoT system for real-time predictive maintenance of 
electric motors using triaxial vibration analysis. The system 
detects mechanical faults (bearing wear, imbalance, misalignment) 
using RMS-based anomaly detection — all for under USD 10 in hardware.

Developed as part of **EN2160 — Electronic Design Realization**  
Faculty of Engineering, University of Peradeniya, Sri Lanka.

## ✨ Features
- Real-time 3-axis vibration monitoring at 1 kHz
- On-device RMS computation on ESP32 (256-sample windows)
- Wi-Fi data transmission using MQTT protocol
- Live Python dashboard with trend plots and fault alerts
- Local LED + buzzer fault indication at sensor node
- CSV data logging for post-analysis
- Total BOM cost: ~LKR 3,576 (under USD 10)

## 🛠️ Hardware Components
| Component         | Model          | Purpose                     |
|-------------------|----------------|-----------------------------|
| Microcontroller   | ESP32 Dev Kit  | Data acquisition + Wi-Fi    |
| Vibration Sensor  | MPU6050        | Triaxial MEMS accelerometer |
| Status Indicators | LEDs + Buzzer  | Local fault signalling      |
| Power             | Li-ion 3.7V    | Portable operation          |

## 📂 Project Structure
```
firmware/    → ESP32 Arduino source code
dashboard/   → Python monitoring application
hardware/    → Schematics and wiring diagrams
docs/        → Full project report (PDF + LaTeX)
results/     → Experimental test data and plots
media/       → Photos and demo footage
```

## 🚀 Getting Started

### Prerequisites
- Arduino IDE or PlatformIO
- Python 3.8+
- Mosquitto MQTT Broker

### Firmware Setup
```bash
# 1. Open firmware/main.ino in Arduino IDE
# 2. Edit config.h with your WiFi credentials
# 3. Flash to ESP32 via USB
```

### Dashboard Setup
```bash
cd dashboard
pip install -r requirements.txt
python dashboard.py
```

## 📊 Results
| Condition          | X RMS (g) | Y RMS (g) | Fault Alert |
|--------------------|-----------|-----------|-------------|
| Healthy Baseline   | 0.09      | 0.07      | None        |
| Imbalance Fault    | 0.38      | 0.12      | ✅ ACTIVATED |
| Bearing Fault      | 0.31      | 0.29      | ✅ ACTIVATED |
| Misalignment Fault | 0.25      | 0.11      | ✅ ACTIVATED |


## 📄 License
MIT License — see LICENSE file for details.

## 🎓 Citation
If you use this project, please cite:
> IoT-Based Predictive Maintenance Using Vibration Analysis
