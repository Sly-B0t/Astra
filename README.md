# 🚁 Astra-1 Drone

A custom-built quadcopter powered by an ESP32, designed for real-time control, stabilization, and modular expansion.
This project focuses on embedded systems, control systems (PID), and full hardware–software integration.

PCB design files and 3D-printable frame components are planned for future revisions.

The ESP32 interfaces with any Bluepad32-compatible controller. Ensure your controller is paired before use.

---

## 📌 Features

* ESP32-based flight controller
* PWM control of BLDC motors via ESCs
* PS5 controller input over Bluetooth
* Real-time task management using FreeRTOS
* Modular and scalable architecture
* Planned PID-based stabilization (pitch, roll, yaw)

---

## 🛠️ Hardware

* ESP32 DOIT Dev Board
* 4× BLDC Motors (1000KV)
* 4× ESCs (30A)
* 3S LiPo Battery
* IMU (MPU6050)
* Custom PCB *(in progress)*
* Frame (DJI F450-style)

---

## 💻 Software

* C/C++ (ESP-IDF or Arduino framework)
* FreeRTOS (ESP32 built-in)
* PlatformIO + Visual Studio Code

---

## 📂 Project Structure

```bash
.
├── src/            # Main application code
├── include/        # Header files
├── lib/            # Custom libraries/modules
├── test/           # Unit tests (optional)
├── platformio.ini  # Build configuration
└── README.md
```

---

## 🚀 Getting Started

### 1. Clone the repository

```bash
git clone https://github.com/yourusername/astra-1.git
cd astra-1
```

### 2. Build the project

```bash
pio run
```

### 3. Upload to ESP32

```bash
pio run --target upload
```

### 4. Pair controller

* Power on your controller
* Pair via Bluetooth (handled by Bluepad32)
* Ensure connection before arming motors

---

## 🎮 Controls

* L2 / R2 (analog triggers) → Throttle
* Left Stick → Pitch / Yaw
* L1 / R1 → Roll

---

## ⚙️ Future Improvements

* Full PID tuning for stable flight
* Sensor fusion (Kalman / Complementary filter)
* GPS integration
* Real-time telemetry dashboard (web interface)
* Live PID tuning over WiFi

---

## ⚠️ Disclaimer

This project involves high-speed motors and LiPo batteries.
Improper use can cause serious damage or injury. Test carefully and implement safety precautions.

---

## 📸 Media

Will be added later

---

## 👤 Author

Muhiedin Omar
