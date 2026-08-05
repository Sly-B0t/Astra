# Astra Revision 5.5

Flightcontroller firmware for a custom ESP32S3-N8R8 embedded board, ICM-20948, LPS25HBTR. PCB Gerber files are also within this repo and are easily reproducable.

# Hardware

* ESP32-S3 Module
* LDO
* GPIO18 LED
* ICM-20948 Gyro, Magnetometer, and Accelerometer
* LPS25HBTR Barometer
* Fully Custom PCB
* Couple Resistors, Capacitors, Buttons and 2.54mm Pin Headers
* Usb-C 2.0 Port

---

# Software

* C++
* FreeRTOS 
* PlatformIO + Visual Studio Code


# Project Structure

```bash
.
├── src/            # Main application code
├── include/        # Header files
├── lib/            # Custom libraries/modules
├── test/           # Unit tests (optional)
├── platformio.ini  # Build configuration
├── README.md
└── Gerber.zip      # Gerber files for PCB production
```

---

# Controls

* L2 / R2 (analog triggers) → Throttle
* Left Stick → Pitch
* R1/L1 Yaw
* A -> Arm Motors
* B -> Disarm Motors

---

# Getting Started

### 1. Clone the repository

```bash
git clone https://github.com/yourusername/astra-1.git
cd astra-1
```

### 2. Build the project

```bash
pio run
```

### 3. Upload to ESP32-S3-N8R8

```bash
pio run --target upload
```

### 4. Pair controller

* Power on your controller
* Pair via Bluetooth 
* Ensure connection before arming motors

---



