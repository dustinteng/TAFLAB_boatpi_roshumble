# TAFLAB Swarm Boat System

An integrated multi-robot platform for oceanic data collection using autonomous sailboats. The system is composed of three main components:

- A **ROS2-based control system** running on Raspberry Pi (boat side)
- A **Flask-based backend** for communication, data collection, and storage (shore side)
- A **React frontend dashboard** for real-time visualization, control, and analysis

---

## 🛍️ Repositories Overview

### 🔹 [TAFLAB_boatpi_roshumble](https://github.com/dustinteng/TAFLAB_boatpi_roshumble)
> ROS2 Humble stack deployed on each Raspberry Pi-equipped sailboat

- Nodes control rudder, sail, and motor
- Publishes GPS, IMU, and sensor data
- Communicates with the backend via XBee
- Launch file structure under `captain/launch/`

### 🔹 [TAFLAB_backend](https://github.com/dustinteng/TAFLAB_backend)
> Flask backend to receive, store, and serve telemetry data

- Accepts data from boats and appends to a `large_table`
- Serves REST and GraphQL endpoints
- Includes endpoints:
  - `/upload`, `/download/<source>`, `/sources`, `/delete/<source>`
- Modular GraphQL API for advanced queries
- Supports ELT model (extract + load, then transform in-database)

### 🔹 [TAFLAB_frontend](https://github.com/dustinteng/TAFLAB_frontend)
> React.js frontend for live control, visual feedback, and data analysis

- Live boat map with route setting and telemetry overlays
- Manual control with joystick and slider
- Heatmap visualization of environmental data (temperature/wind)
- CSV file management, data previews, and time-slider control
- Responsive UI with drag-and-drop legends and dropdowns

---

## 🚀 Quick Start Guide

### 1. Boat (Raspberry Pi) Setup
```bash
# On each Raspberry Pi
cd TAFLAB_boatpi_roshumble
# Setup ROS2 workspace and dependencies
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
# Launch boat control stack
ros2 launch captain captain.launch.py
```

### 2. Backend (Shore Station)
```bash
cd TAFLAB_backend
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python3 app.py
```

### 3. Frontend (Dashboard)
```bash
cd TAFLAB_frontend
npm install
npm start
```

---

## 📊 Features Summary

### Autonomy
- Waypoint navigation (click on map)
- Live telemetry trail and target position popup

### Manual Control
- Joystick: rudder
- Slider: propeller/motor
- Dropdown to switch boats in GUI

### Data Collection & Management
- CSV files saved and categorized by source name
- GraphQL query builder (from frontend)
- Optional upload/download/delete actions

### Visualization
- Leaflet-based map (OpenStreetMap tiles)
- Heatmap display with color-scaled data (temperature / wind)
- Draggable legend box and marker toggles
- Snapshot time-slider to scroll back in time

---

## 🧰 Architecture Diagram
```mermaid
graph LR
  subgraph BOAT [Onboard: Raspberry Pi + ROS2 Humble]
    A1[Rudder/Sail/Motor Control Node]
    A2[Sensor Publishers: GPS, IMU, Wind]
    A3[XBee Communication Node]
  end

  subgraph SHORE [Shore Station]
    B1[TAFLAB_backend (Flask)]
    B2[TAFLAB_frontend (React)]
  end

  A3 -- XBee --> B1
  B1 -- REST/GraphQL --> B2
```

---

## 🧱 Tech Stack

| Layer         | Technology                        |
|---------------|------------------------------------|
| Onboard Boat | ROS2 Humble, Python, XBee         |
| Backend      | Flask, SQLite, REST & GraphQL     |
| Frontend     | React.js, Leaflet, WebSocket, CSS |

---

## ⚡ Future Work
- Real-time mesh networking across boat nodes
- Predictive wave modeling from aggregated data
- Integration with external APIs for weather overlays
- Dashboard export to PDF or data dashboarding libraries

---

## ✅ License
MIT License

---

## 🙏 Acknowledgements
This system is developed as part of the TAFLAB project at UC Berkeley. Special thanks to collaborators, testers, and the ocean robotics community.

