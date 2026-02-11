# Pet Space - Environment-Responsive Robotic Desk Lamp

Pet Space is an interactive remote pet monitoring and companionship system. It utilizes non-intrusive sensing technologies to alleviate occupational burnout by providing users with a real-time emotional connection to their pets through movement data, audio interaction, and historical activity analysis.

---

## System Architecture

The system operates across three hardware tiers and a Python-based data processing backend:

1. **Detection Device Nodes**: ESP32-C3 units that combine LD2420 radar and HC-SR501 PIR sensors to detect pet presence, distance, and angle.
2. **Central Server (Gateway)**: An ESP32-C3 that bridges the local ESP-NOW sensor network to the internet via MQTT over SSL/TLS.
3. **Portable Mobile Terminal**: A handheld device with a TJC serial screen for real-time visualization and remote audio triggering.
4. **Python Backend**: A dual-script suite for persistent data logging and advanced trajectory visualization.

---

## Software Implementation

### 1. Hardware Firmware (C++/Arduino)

* **ESP-NOW**: Used for low-latency, low-power communication between sensor nodes and the gateway.
* **MQTT**: Gateway publishes node status to `home/catTracker` and subscribes to `home/catCommand` for remote interactions.

### 2. Python Data Suite

The repository includes two specialized Python scripts to handle the cloud-side data lifecycle:

* **`real-time-acquisition.py`**: A persistent MQTT subscriber that connects to the broker, parses incoming JSON payloads from the Central Server, and logs pet activity (timestamp, room, distance, detection status) into a local CSV database.
* **`track-visualization.py`**: An analysis tool that reads the logged historical data to generate pet activity route graphs. It maps categorical room data to numerical values to visualize movement patterns and identify behavioral abnormalities.

---

## Installation and Setup

### Hardware Setup

1. Flash the detection node firmware to the ESP32-C3, ensuring unique `NODE_ID` and `ROOM_NAME` values are assigned.
2. Flash the Central Server firmware with valid WiFi and MQTT credentials.

### Python Environment

Ensure you have Python 3.x installed along with the necessary libraries:

```bash
pip install paho-mqtt pandas matplotlib

```

### Running the Data Pipeline

1. **Start Data Collection**: Run the acquisition script to begin logging MQTT traffic:
```bash
python real-time-acquisition.py

```


2. **Generate Reports**: Once sufficient data is collected, run the visualization script to view the pet's activity route:
```bash
python track-visualization.py

```



---

## Hardware Specifications

* **Microcontroller**: ESP32-C3.
* **Sensors**: LD2420 Millimeter-wave Radar, HC-SR501 PIR.
* **Audio**: MAX98357A I2S Amplifier with 3W Speaker.
* **Display**: TJC Serial Port Screen.
