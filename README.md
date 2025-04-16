# TAFLAB: Telemetry-Driven Autonomous Fleet Lab

TAFLAB is a modular, full-stack robotics system for collecting, analyzing, and visualizing data from autonomous marine robots. It supports swarm data telemetry, command control, real-time heatmap rendering, and an extensible backend architecture.

---

## 🚀 System Overview

TAFLAB is composed of three major components:

1. **Frontend GUI** - [TAFLAB_frontend](https://github.com/dustinteng/TAFLAB_frontend)
2. **Backend API Server** - [TAFLAB_backend](https://github.com/dustinteng/TAFLAB_backend/tree/modular_v3)
3. **Boat Onboard Stack** - [TAFLAB_boatpi_roshumble](https://github.com/dustinteng/TAFLAB_boatpi_roshumble/tree/version4)

Each part communicates via HTTP and XBee mesh network to form a reliable and scalable robotic data infrastructure.

---

## 1. Frontend - TAFLAB_frontend

- **Framework**: React.js with React Leaflet for geospatial visualization
- **Features**:
  - Map-based boat tracking
  - Real-time joystick + slider control for rudder/propeller
  - Heatmap rendering from historical CSV logs
  - File source selector, telemetry viewer, and draggable legend

### Run the Frontend

```bash
git clone https://github.com/dustinteng/TAFLAB_frontend
cd TAFLAB_frontend
npm install
npm start
```

Default: `http://localhost:3000`

---

## 2. Backend - TAFLAB_backend

- **Branch**: [`modular_v3`](https://github.com/dustinteng/TAFLAB_backend/tree/modular_v3)
- **Purpose**: Shore station database API for uploading, downloading, and querying telemetry logs

### Key Features

- CSV ingest and parse
- ETL and ELT hybrid design
- GraphQL for flexible data queries
- SQLite database with dynamic column detection

### API Endpoints

| Method | Route                | Description                     |
| ------ | -------------------- | ------------------------------- |
| POST   | `/upload`            | Upload a CSV telemetry file     |
| GET    | `/sources`           | List all known CSV file sources |
| GET    | `/download/<source>` | Download a file's data          |
| DELETE | `/delete/<source>`   | Remove all rows from a source   |
| POST   | `/query`             | Execute GraphQL database query  |

### Run the Backend

```bash
git clone -b modular_v3 https://github.com/dustinteng/TAFLAB_backend.git
cd TAFLAB_backend
python3 app.py

don't be a kiddo, debug it yourself, its probably just library installations...
```

---

## 3. Boat Stack - TAFLAB_boatpi_roshumble

- **Branch**: [`version4`](https://github.com/dustinteng/TAFLAB_boatpi_roshumble/tree/version4)
- **Platform**: Raspberry Pi with ROS2 Humble
- **Functionality**:
  - Rudder and sail actuation
  - Sensor node publishing (GPS, IMU, etc.)
  - Listens for `/boatcontrol` topic and sends XBee data
  - Periodic heartbeat and telemetry broadcast

### Launch Example

````bash
source /opt/ros/humble/setup.bash
colcon build
ros2 launch boat_launch boat_launch.py```

---

## 📊 Data Flow

1. **Boats** periodically log and/or transmit sensor data
2. **Backend** accepts batch CSV upload or live POST (WIP)
3. **Frontend** requests available files, visualizes them as heatmaps
4. **Users** can control boats or review parsed data

---

## 🤝 Contributing

This system is in active development. Feel free to fork, contribute via PRs, or raise issues on GitHub.

---

## ⚙️ Roadmap

- [x] Modular Flask backend with GraphQL
- [x] ROS2 Humble boat integration
- [x] CSV-based batch heatmap rendering
- [ ] Live MQTT stream support
- [ ] Mission planning via map GUI
- [ ] PostgreSQL upgrade

---

## ✨ License

MIT License. © 2025 Jan Dustin Tengdyantono
````
