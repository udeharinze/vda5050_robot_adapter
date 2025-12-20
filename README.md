# VDA 5050 Fleet Management System

A VDA 5050 v2.1.0 compliant fleet management adapter that enables consumer vacuum robots (Dreame L10S Ultra with Valetudo firmware) to be controlled by industrial master control systems like Flexus.

![VDA 5050](https://img.shields.io/badge/VDA%205050-v2.1.0-blue)
![Python](https://img.shields.io/badge/Python-3.8+-green)

## Overview

This project implements a complete VDA 5050 communication interface that translates between the standardized VDA 5050 protocol and the Valetudo HTTP API. It enables robots that aren't natively VDA 5050 compatible to be integrated into industrial fleet management systems.

### What is VDA 5050?

VDA 5050 is a standardized interface for communication between Automated Guided Vehicles (AGVs) and a central Master Control system. Developed by the German Automotive Industry Association (VDA) and VDMA, it allows robots from different manufacturers to be controlled by a single fleet management system.

### System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                  MASTER CONTROL (Flexus)                    │
│              Fleet Management & Traffic Control             │
└─────────────────────────┬───────────────────────────────────┘
                          │ MQTT (VDA 5050 JSON)
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                      MQTT BROKER                            │
│           (Mosquitto local / Flexus Cloud WSS)              │
└─────────────────────────┬───────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                   VDA 5050 BACKEND                          │
│                   (vda_backend.py)                          │
│  • Receives VDA 5050 orders                                 │
│  • Manages node/edge state tracking                         │
│  • Implements base/horizon logic                            │
│  • Publishes VDA 5050 state messages                        │
└─────────────────────────┬───────────────────────────────────┘
                          │ Internal MQTT
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                       BRIDGE                                │
│                   (bridge.py)                           │
│  • Translates VDA 5050 → Valetudo HTTP API                  │
│  • Polls robot position (e.g 2s interval)                     │
│  • Executes navigation commands                             │
└─────────────────────────┬───────────────────────────────────┘
                          │ HTTP REST API
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                    DREAME ROBOT                             │
│              (Valetudo Firmware @ 192.168.x.x)              │
└─────────────────────────────────────────────────────────────┘
```

## Features

### VDA 5050 Compliance
- Full v2.1.0 message format support
- Order handling with nodes and edges
- Base/Horizon traffic control mechanism
- State publishing at 2-second intervals
- Connection state management (ONLINE/OFFLINE/CONNECTIONBROKEN)
- Instant actions support (pause, resume, stop)

### Navigation
- Sequential waypoint navigation
- Seamless node-to-node movement
- Arrival detection with configurable tolerance
- Stall detection and recovery

### Actions
| Action | Description |
|--------|-------------|
| `beep` | Play test sound |
| `locate` | Play "find me" sound |
| `setFanSpeed` | Set vacuum power (low/medium/high/max) |
| `setVolume` | Set speaker volume (0-100) |
| `dock` | Return to charging station |
| `startPause` | Pause current movement |
| `stopPause` | Resume movement |

### GUI Dashboard
- Real-time map visualization
- Robot position tracking
- Mission selection and execution
- Action execution panel
- Log viewer

## Requirements

### Hardware
- Dreame L10S Ultra (or compatible vacuum robot)
- Valetudo firmware installed ([Valetudo Installation Guide](https://valetudo.cloud/))
- Robot accessible on local network

### Software
- Python 3.8+
- MQTT Broker (Mosquitto recommended)
- Required Python packages:
  ```
  paho-mqtt>=2.0.0
  requests>=2.28.0
  ```

## Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/yourusername/vda5050-fleet-management.git
   cd vda5050-fleet-management
   ```

2. **Install dependencies**
   ```bash
   pip install paho-mqtt requests
   ```

3. **Install MQTT broker** (if running locally)
   ```bash
   # Ubuntu/Debian
   sudo apt install mosquitto mosquitto-clients
   
   # Start broker
   sudo systemctl start mosquitto
   ```

4. **Configure the system**
   
   Edit `vda_config.py`:
   ```python
   # MQTT Broker settings
   MQTT_BROKER = "localhost"  # or Flexus cloud broker
   MQTT_PORT = 1883
   
   # Robot identity
   MANUFACTURER = "Dreame"
   SERIAL_NUMBER = "robot001"
   ```

5. **Configure robot IP**
   
   Edit `bridge.py`:
   ```python
   VALETUDO_HOST = "192.168.178.69"  # Your robot's IP
   ```

## Usage

### Quick Start

1. **Start the bridge** (connects to robot)
   ```bash
   python bridge.py
   ```

2. **Start the backend** (in another terminal)
   ```bash
   python vda_backend.py
   ```

3. **Start the GUI** (optional, for visualization)
   ```bash
   python vda_gui.py
   ```

### Sending Orders via MQTT

You can send VDA 5050 orders using any MQTT client (e.g., MQTTX, mosquitto_pub):

```bash
mosquitto_pub -t "uagv/v2/Dreame/robot001/order" -m '{
  "headerId": 1,
  "timestamp": "2025-01-01T12:00:00.000Z",
  "version": "2.1.0",
  "manufacturer": "Dreame",
  "serialNumber": "robot001",
  "orderId": "order_001",
  "orderUpdateId": 0,
  "nodes": [
    {"nodeId": "home", "sequenceId": 0, "released": true, "actions": []},
    {"nodeId": "node1", "sequenceId": 2, "released": true, "actions": []}
  ],
  "edges": [
    {"edgeId": "e1", "sequenceId": 1, "startNodeId": "home", "endNodeId": "node1", "released": true, "actions": []}
  ]
}'
```

### Monitoring State

Subscribe to state messages:
```bash
mosquitto_sub -t "uagv/v2/Dreame/robot001/state" -v
```

## Configuration

### Node Positions

Define your warehouse/lab layout in `vda_config.py`:

```python
NODES = {
    "home":  {"x": 3269, "y": 3352, "id": "home"},
    "node1": {"x": 3228, "y": 3143, "id": "n1"},
    "node2": {"x": 3252, "y": 2918, "id": "n2"},
    # Add more nodes...
}

EDGES = [
    {"start": "home",  "end": "node1", "id": "e_home_node1"},
    {"start": "node1", "end": "home",  "id": "e_node1_home"},
    # Add more edges...
]
```

### Flexus Cloud Integration

To connect to Flexus's cloud MQTT broker:

```python
# In vda_config.py
MQTT_BROKER = "flexusag-cf-ft-demo-flexus-flexguide-mqtt-broker.cfapps.eu20-001.hana.ondemand.com"
MQTT_PORT = 443
MQTT_TRANSPORT = "websockets"
MQTT_WS_PATH = "/api_broker/mqtt/connect"
MQTT_USE_TLS = True
```

See `WEBSOCKET_UPDATE_GUIDE.md` for detailed instructions.

## VDA 5050 Message Examples

### Order Message
```json
{
  "orderId": "order_001",
  "orderUpdateId": 0,
  "nodes": [
    {"nodeId": "home", "sequenceId": 0, "released": true},
    {"nodeId": "node1", "sequenceId": 2, "released": true},
    {"nodeId": "node2", "sequenceId": 4, "released": false}
  ],
  "edges": [
    {"edgeId": "e1", "sequenceId": 1, "released": true},
    {"edgeId": "e2", "sequenceId": 3, "released": false}
  ]
}
```

### State Message
```json
{
  "orderId": "order_001",
  "lastNodeId": "node1",
  "driving": true,
  "newBaseRequest": false,
  "nodeStates": [{"nodeId": "node2", "sequenceId": 4}],
  "edgeStates": [{"edgeId": "e2", "sequenceId": 3}],
  "agvPosition": {"x": 3228, "y": 3143, "theta": 1.57, "mapId": "lab"},
  "batteryState": {"batteryCharge": 85.0, "charging": false},
  "operatingMode": "AUTOMATIC",
  "safetyState": {"eStop": "NONE", "fieldViolation": false}
}
```

## Base/Horizon Mechanism

The base/horizon concept enables traffic control in multi-robot environments:

- **BASE nodes** (`released: true`): Robot can navigate immediately
- **HORIZON nodes** (`released: false`): Robot knows the path but waits for permission
- **Decision Point**: Last BASE node where robot stops if no update arrives

When `newBaseRequest: true` appears in state, the master control should send an order update to release more nodes.

## File Structure

```
vda5050-fleet-management/
├── vda_backend.py          # Main VDA 5050 backend controller
├── bridge.py               # Valetudo HTTP API bridge
├── vda_config.py           # Configuration (nodes, edges, MQTT)
├── vda_gui.py              # Tkinter GUI dashboard
├── actions_basic.py        # Action handlers
├── actions_navigation.py   # goTo handler
├── mqtt_connection.py      # WebSocket MQTT helper
├── vda5050.db              # SQLite database (optional)
├── requirements.txt        # Python dependencies
└── README.md               # This file
```

## Troubleshooting

### Robot not responding
- Check robot IP address in `bridge.py`
- Verify Valetudo is running: `http://ROBOT_IP/api/v2/robot/state`
- Ensure robot is on same network

### MQTT connection failed
- Check if broker is running: `systemctl status mosquitto`
- Verify broker address in `vda_config.py`
- For Flexus: ensure WebSocket settings are correct

### Navigation issues
- Verify node coordinates match your map
- Check arrival tolerance in `vda_config.py`
- Review logs for stall detection

## Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.


## Acknowledgments

- [VDA 5050 Specification](https://github.com/VDA5050/VDA5050)
- [Valetudo](https://valetudo.cloud/) - Open source cloud replacement for vacuum robots
- Flexus GmbH - Industrial fleet management partner
- THWS, Schweinfurt - Project Initiator

## Contact

For questions about this implementation, please open an issue on GitHub.
