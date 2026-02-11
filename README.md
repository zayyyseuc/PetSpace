# PetSpace
An Environment-Responsive Robotic Interaction System

The Pet Space project is an interactive remote pet monitoring and companionship system designed to alleviate occupational burnout. By sensing a pet's presence and activity in a space, it provides remote users with a low-interference channel for emotional connection, reducing the sense of disconnection commonly experienced during long work hours.

---

## System Architecture

The system is composed of three primary functional modules: detection nodes, a central server, and a mobile terminal.

### 1. Detection Device Node

* Utilizes an ESP32-C3 microcontroller to integrate HC-SR501 infrared sensors and LD2420 radar sensors.
* Detects the presence, distance, and angle of a pet in real-time.
* Transmits sensor data to the central server using the ESP-NOW protocol.
* Includes an I2S audio playback module to play WAV files from a Micro SD card upon receiving remote commands.

### 2. Central Server (Gateway)

* Acts as a communication bridge, receiving local data from detection nodes via ESP-NOW.
* Connects to a WiFi network and uploads formatted JSON data to a cloud broker using MQTT over a secure SSL/TLS connection.
* Maintains a registry of node IDs and their corresponding MAC addresses for targeted command forwarding.

### 3. Portable Mobile Terminal

* Equipped with a TJC serial port screen for real-time status display and user interaction.
* Visualizes the pet's current location, distance, and activity status.
* Allows users to select specific nodes and audio clips to trigger remote playback.

---

## Hardware Components

| Module | Component | Purpose |
| --- | --- | --- |
| **Main Controller** | ESP32-C3 (AiM2M CORE) | Core logic and wireless communication |
| **Motion Detection** | LD2420 Radar & HC-SR501 PIR | Dual-sensor verification for presence detection |
| **Audio Output** | MAX98357A I2S Amp & 3W Speaker | Remote voice interaction and sound playback |
| **User Interface** | TJC Serial Port Display | Information display and command input |
| **Feedback Lighting** | WS2812B LEDs (FastLED) | Visual response based on pet activity |
| **External Storage** | Micro SD Card | Storage for WAV audio resources |

---

## Software Implementation

### Communication Protocols

* **Local Network**: ESP-NOW is used for low-latency communication between detection nodes and the gateway.
* **Cloud Connectivity**: MQTT (via PubSubClient) manages upstream data publishing and downstream command subscription.

### Data Management

* **Upstream Topic**: `home/catTracker` - Gateway publishes JSON arrays containing node IDs, room names, distances, and detection flags.
* **Downstream Topic**: `home/catCommand` - Mobile terminal publishes commands to trigger audio on specific nodes.
* **Python Integration**: While the core system runs on C++, Python is utilized for historical data analysis and generating activity trajectory graphs to check for pet health abnormalities.

---

## Quick Setup

### Node Configuration

1. Define a unique `NODE_ID` and `ROOM_NAME` for each detection node.
2. Set the `gatewayMac` to match the MAC address of your Central Server.

### Gateway Configuration

1. Enter your WiFi `ssid` and `password`.
2. Provide the MQTT server address and ensure the `root_ca` SSL certificate is correctly formatted for the secure broker.

### Terminal Operation

1. Power on the mobile terminal to automatically subscribe to node updates.
2. Use the touch interface to send `PlayCommand` structures to specific nodes.
