# 🤖 Autonomous Navigation Robot

A modular MicroPython-based robotic platform built to simulate indoor autonomous navigation using MQTT-based communication, ultrasonic and IR sensor feedback, and decentralized control logic. Designed for adaptability in dynamic environments with support for occupancy detection and real-time coordination.

---

## 📦 Project Structure

```

├── boot.py                # System startup configuration
├── compilepubsub.py       # MQTT handler (main orchestrator)
├── oldcompilepubsub.py    # Legacy pubsub script for reference
├── display_test.py        # OLED/Display test script
├── hcsr04.py              # Ultrasonic sensor interface (HC-SR04)
├── IRLineNav.py           # IR line navigation logic
├── laptopsubpaho.py       # MQTT subscriber on external laptop
├── Npublisher.py          # MQTT publisher for room navigation
├── publisher.py           # MQTT publisher (generic)
├── publisherbutton.py     # MQTT publisher triggered via button
├── subscriber.py          # MQTT subscriber (robot-side)
├── r2auto_move.py         # Autonomous movement logic
├── r2auto_nav.py          # Autonomous room-to-room pathing
├── r2mover.py             # Basic directional motion controller
├── r2moverotate.py        # Motion with angle rotation
├── r_backupmove.py        # Fallback motion strategy
├── r2occupancy.py         # Occupancy detection (standard)
├── r2occupancy2.py        # Occupancy detection (enhanced logic)
├── r2scanner.py           # Environmental scanner (room state)
├── setup.py               # Deployment setup script
├── users.txt              # User mapping or device IDs

```

---

## 🚀 Features

### 🤖 Autonomous Navigation
- IR-guided movement via `IRLineNav.py`
- Obstacle detection and avoidance using ultrasonic sensors (`hcsr04.py`)
- Real-time room-based pathfinding in `r2auto_nav.py`

### 🌐 MQTT-Based Control System
- Full MQTT Publisher/Subscriber framework (`compilepubsub.py`, `subscriber.py`)
- Multi-node communication across devices and environments
- Laptop and ESP32-compatible setup

### 🧠 Occupancy-Aware Logic
- Scans and adjusts to room occupancy (`r2occupancy.py`, `r2occupancy2.py`)
- Supports re-routing and fallback behavior via `r_backupmove.py`

### 🧩 Modular & Testable
- Decoupled architecture: sensor, motor control, and communication layers
- Test utilities (`display_test.py`) and legacy support (`oldcompilepubsub.py`)

---

## 🧠 System Overview

The system is designed around three core layers:
1. **Sensor Layer**
   - IR sensors (for line navigation)
   - HC-SR04 ultrasonic sensor (for distance and obstacle detection)

2. **Communication Layer**
   - MQTT publishers (on ESP32 and laptop)
   - MQTT subscribers (robot, laptop for debugging)
   - Topics include room state, movement commands, and occupancy updates

3. **Control Layer**
   - Modular movement: forward/back/turn via `r2mover.py`, `r2moverotate.py`
   - Navigation logic maps rooms to destinations dynamically
   - Occupancy-aware pathing for realistic use-cases

---

## 🛠️ Requirements

### Hardware
- ESP32 board
- HC-SR04 Ultrasonic Sensor
- IR Line Sensors
- Push buttons / OLED display (optional)
- Laptop with MQTT broker (Mosquitto)

### Software
- MicroPython (flashed on ESP32)
- Paho MQTT (for laptop-side scripts)
- Thonny IDE / Visual Studio Code
- Mosquitto Broker installed locally or over network

---

## 🧪 Getting Started

1. **Flash MicroPython** on your ESP32 board.
2. Upload the scripts using Thonny or ampy.
3. Connect to your Wi-Fi network and set MQTT broker IP in:
   - `compilepubsub.py`, `laptopsubpaho.py`, `publisher.py`, etc.
4. Start the broker (Mosquitto) on your laptop or server.
5. Run `compilepubsub.py` on the ESP32 to activate robot logic.
6. Monitor/control using `laptopsubpaho.py` or `subscriber.py`.

---

## 🧭 Example MQTT Topics

| Topic                  | Purpose                                |
|------------------------|----------------------------------------|
| `r2/room/state`        | Room occupancy updates                 |
| `r2/move/command`      | Movement instructions (FWD/BWD/STOP)   |
| `r2/scan`              | Environmental scan trigger             |
| `r2/feedback/ultra`    | Ultrasonic distance data               |
| `r2/control/override`  | Manual override commands               |

---

## 📈 Potential Applications

- Smart Indoor Mobility Systems
- Autonomous Delivery Robots
- Wheelchair Navigation Prototypes
- Educational Robotics Platforms
- Research in Multi-Agent Navigation
