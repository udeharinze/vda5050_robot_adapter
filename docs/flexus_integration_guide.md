# ============================================================================
# VDA 5050 Integration Guide for Flexus Master Control
# ============================================================================
#
# This document describes how to integrate with our VDA 5050 compliant
# robot system using MQTT.
#
# ============================================================================

## 1. CONNECTION DETAILS

### MQTT Broker
- **Host**: [YOUR_SERVER_IP] (e.g., 192.168.178.100)
- **TCP Port**: 1883 (standard MQTT)
- **WebSocket Port**: 9001 (MQTT over WebSocket)
- **WebSocket URL**: ws://[YOUR_SERVER_IP]:9001

### Robot Identity
- **Manufacturer**: Dreame
- **Serial Number**: robot001
- **VDA Version**: 2.1.0
- **Interface Name**: uagv


## 2. MQTT TOPICS (VDA 5050 v2.1.0)

Topic format: `uagv/v2/{manufacturer}/{serialNumber}/{topic}`

### Topics Master Control SUBSCRIBES to (Robot → Master):

| Topic | Description | QoS |
|-------|-------------|-----|
| `uagv/v2/Dreame/robot001/state` | Robot state (position, battery, order status) | 0 |
| `uagv/v2/Dreame/robot001/visualization` | Visualization data (position, velocity) | 0 |
| `uagv/v2/Dreame/robot001/connection` | Connection state (ONLINE/OFFLINE) | 1 |

### Topics Master Control PUBLISHES to (Master → Robot):

| Topic | Description | QoS |
|-------|-------------|-----|
| `uagv/v2/Dreame/robot001/order` | Navigation orders | 0 |
| `uagv/v2/Dreame/robot001/instantActions` | Immediate actions (pause, resume, stop) | 0 |


## 3. MESSAGE FORMATS

### 3.1 Order Message (Master → Robot)

```json
{
    "headerId": 1,
    "timestamp": "2024-12-17T12:00:00.000Z",
    "version": "2.1.0",
    "manufacturer": "Dreame",
    "serialNumber": "robot001",
    "orderId": "order_001",
    "orderUpdateId": 0,
    "nodes": [
        {
            "nodeId": "home",
            "sequenceId": 0,
            "released": true,
            "nodePosition": {"x": 3269, "y": 3352, "mapId": "ERP_lab"},
            "actions": []
        },
        {
            "nodeId": "node1",
            "sequenceId": 2,
            "released": true,
            "nodePosition": {"x": 3228, "y": 3143, "mapId": "ERP_lab"},
            "actions": []
        }
    ],
    "edges": [
        {
            "edgeId": "e_home_node1",
            "sequenceId": 1,
            "startNodeId": "home",
            "endNodeId": "node1",
            "released": true,
            "actions": []
        }
    ]
}
```

### 3.2 State Message (Robot → Master)

```json
{
    "headerId": 123,
    "timestamp": "2024-12-17T12:00:01.000Z",
    "version": "2.1.0",
    "manufacturer": "Dreame",
    "serialNumber": "robot001",
    "orderId": "order_001",
    "orderUpdateId": 0,
    "lastNodeId": "home",
    "lastNodeSequenceId": 0,
    "driving": true,
    "paused": false,
    "newBaseRequest": false,
    "nodeStates": [
        {"nodeId": "node1", "sequenceId": 2, "released": true}
    ],
    "edgeStates": [
        {"edgeId": "e_home_node1", "sequenceId": 1, "released": true}
    ],
    "agvPosition": {
        "x": 3250.0,
        "y": 3200.0,
        "theta": 1.57,
        "mapId": "ERP_lab",
        "positionInitialized": true
    },
    "velocity": {"vx": 0.0, "vy": 0.0, "omega": 0.0},
    "batteryState": {
        "batteryCharge": 85.0,
        "charging": false
    },
    "operatingMode": "AUTOMATIC",
    "errors": [],
    "actionStates": []
}
```

### 3.3 Connection Message (Robot → Master)

```json
{
    "headerId": 1,
    "timestamp": "2024-12-17T12:00:00.000Z",
    "version": "2.1.0",
    "manufacturer": "Dreame",
    "serialNumber": "robot001",
    "connectionState": "ONLINE"
}
```

Values: `ONLINE`, `OFFLINE`, `CONNECTIONBROKEN`

### 3.4 Instant Actions (Master → Robot)

```json
{
    "headerId": 1,
    "timestamp": "2024-12-17T12:00:00.000Z",
    "version": "2.1.0",
    "manufacturer": "Dreame",
    "serialNumber": "robot001",
    "actions": [
        {
            "actionId": "pause_001",
            "actionType": "startPause",
            "blockingType": "HARD",
            "actionParameters": []
        }
    ]
}
```


## 4. AVAILABLE NODES (Map: ERP_lab)

| Node ID | X (mm) | Y (mm) | Type |
|---------|--------|--------|------|
| home | 3269 | 3352 | charging_station |
| node1 | 3228 | 3143 | waypoint |
| node2 | 3252 | 2918 | waypoint |
| node3 | 3093 | 2932 | waypoint |
| node4 | 3086 | 3141 | waypoint |
| node5 | 3096 | 3303 | waypoint |


## 5. AVAILABLE EDGES

| Edge ID | Start | End |
|---------|-------|-----|
| e_home_node1 | home | node1 |
| e_node1_home | node1 | home |
| e_home_node5 | home | node5 |
| e_node5_home | node5 | home |
| e_node1_node2 | node1 | node2 |
| e_node2_node1 | node2 | node1 |
| e_node1_node4 | node1 | node4 |
| e_node4_node1 | node4 | node1 |
| e_node2_node3 | node2 | node3 |
| e_node3_node2 | node3 | node2 |
| e_node3_node4 | node3 | node4 |
| e_node4_node3 | node4 | node3 |
| e_node4_node5 | node4 | node5 |
| e_node5_node4 | node5 | node4 |


## 6. SUPPORTED ACTIONS

### Navigation Actions
| Action Type | Description | Parameters |
|-------------|-------------|------------|
| goTo | Navigate to position | x, y, targetNodeId |
| dock | Return to charging station | none |

### Instant Actions
| Action Type | Description | Parameters |
|-------------|-------------|------------|
| startPause / pauseOrder | Pause current order | none |
| stopPause / resumeOrder | Resume paused order | none |
| cancelOrder | Cancel current order | none |

### Node/Edge Actions
| Action Type | Description | Parameters |
|-------------|-------------|------------|
| beep | Play test sound | none |
| locate | Play "find me" sound | none |
| setFanSpeed | Set vacuum fan speed | speed: low/medium/high/turbo |
| setVolume | Set speaker volume | volume: 0-100 |


## 7. ORDER FLOW

### Simple Order (No Horizon)
1. Master sends order with all nodes `released: true`
2. Robot executes nodes in sequence
3. Robot publishes state updates
4. When complete, robot publishes state with empty nodeStates

### Order with Horizon (Base + Horizon)
1. Master sends order with BASE nodes (`released: true`) + HORIZON nodes (`released: false`)
2. Robot executes BASE nodes
3. Robot stops at decision point (last BASE node)
4. Robot publishes state with `newBaseRequest: true`
5. Master sends order UPDATE (same orderId, higher orderUpdateId) releasing horizon
6. Robot continues with newly released nodes


## 8. WEBSOCKET CONNECTION EXAMPLE (JavaScript)

```javascript
// Connect to MQTT broker via WebSocket
const client = mqtt.connect('ws://192.168.178.100:9001');

client.on('connect', () => {
    console.log('Connected to MQTT broker');
    
    // Subscribe to robot state
    client.subscribe('uagv/v2/Dreame/robot001/state');
    client.subscribe('uagv/v2/Dreame/robot001/connection');
});

client.on('message', (topic, message) => {
    const data = JSON.parse(message.toString());
    console.log(`${topic}:`, data);
});

// Send an order
function sendOrder(order) {
    client.publish('uagv/v2/Dreame/robot001/order', JSON.stringify(order));
}

// Pause robot
function pauseRobot() {
    const action = {
        headerId: Date.now(),
        timestamp: new Date().toISOString(),
        version: "2.1.0",
        manufacturer: "Dreame",
        serialNumber: "robot001",
        actions: [{
            actionId: `pause_${Date.now()}`,
            actionType: "startPause",
            blockingType: "HARD",
            actionParameters: []
        }]
    };
    client.publish('uagv/v2/Dreame/robot001/instantActions', JSON.stringify(action));
}
```


## 9. TESTING

### Test Connection
```bash
# Subscribe to all robot topics
mosquitto_sub -h localhost -t "uagv/v2/Dreame/robot001/#" -v

# In another terminal, check connection state
mosquitto_sub -h localhost -t "uagv/v2/Dreame/robot001/connection"
```

### Test Order
```bash
mosquitto_pub -h localhost -t "uagv/v2/Dreame/robot001/order" -m '{
    "headerId": 1,
    "timestamp": "2024-12-17T12:00:00.000Z",
    "version": "2.1.0",
    "manufacturer": "Dreame",
    "serialNumber": "robot001",
    "orderId": "test_001",
    "orderUpdateId": 0,
    "nodes": [
        {"nodeId": "home", "sequenceId": 0, "released": true, "actions": []},
        {"nodeId": "node1", "sequenceId": 2, "released": true, "actions": []}
    ],
    "edges": [
        {"edgeId": "e_home_node1", "sequenceId": 1, "startNodeId": "home", "endNodeId": "node1", "released": true, "actions": []}
    ]
}'
```


## 10. CONTACT

For questions or issues, contact the robot integration team.

============================================================================
