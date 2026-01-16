# ============================================================================
# bridge_refactored.py - VDA 5050 Bridge (REFACTORED)
# ============================================================================
#
# FIXES IMPLEMENTED:
#   1. Proper order update stitching detection
#   2. No navigation if already at node
#   3. No dock command if already docked
#   4. Proper sequenceId tracking
#   5. cancelOrder support
#
# ============================================================================

import time
import json
import threading
import paho.mqtt.client as mqtt
import requests
import math

# --- IMPORT YOUR ACTION MODULES ---
try:
    import actions_basic
    import actions_navigation
except ImportError:
    print("ERROR: Could not find actions_basic.py or actions_navigation.py")
    print("Please make sure all .py files are in the same folder.")
    exit()

# --- IMPORT FROM CENTRAL CONFIG ---
from vda_config import (
    MQTT_BROKER, MQTT_PORT,
    MQTT_USERNAME, MQTT_PASSWORD, MQTT_WEBSOCKET, MQTT_WS_PATH, MQTT_VERSION,
    MANUFACTURER, SERIAL_NUMBER
)


# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def normalize_theta(theta_degrees: float) -> float:
    """
    Convert theta from degrees to radians in -π to +π range (VDA 5050 / Flexus convention).

    Input: Valetudo theta in degrees (0 to 360)
    Output: VDA 5050 theta in radians (-π to +π, i.e., -3.14 to +3.14)
    """
    # Step 1: Convert degrees to radians
    theta_rad = math.radians(theta_degrees)

    # Step 2: Normalize to -π to +π range
    if theta_rad > math.pi:
        theta_rad = theta_rad - (2 * math.pi)

    return round(theta_rad, 5)


# --- VDA 5050 TOPICS ---
TOPIC_VISUALIZATION = f"uagv/v2/{MANUFACTURER}/{SERIAL_NUMBER}/visualization"
TOPIC_INSTANT_ACTIONS = f"uagv/v2/{MANUFACTURER}/{SERIAL_NUMBER}/instantActions"
TOPIC_ORDER = f"uagv/v2/{MANUFACTURER}/{SERIAL_NUMBER}/order"

# --- CONFIGURATION ---
ROBOT_IP = "192.168.178.69"  # UPDATE with your robot's IP
STATE_INTERVAL = 5
ARRIVAL_TOLERANCE_MM = 75
AT_NODE_THRESHOLD_MM = 50  # Tighter threshold for "already at node" detection

print(f"Bridge is configured for Robot IP: {ROBOT_IP}")


# ============================================================================
# ROBOT DATA FETCHING
# ============================================================================

def get_robot_data():
    """Fetches and parses all necessary data from the robot's API."""
    try:
        map_url = f"http://{ROBOT_IP}/api/v2/robot/state/map"
        state_url = f"http://{ROBOT_IP}/api/v2/robot/state"

        map_response = requests.get(map_url, timeout=5)
        state_response = requests.get(state_url, timeout=5)

        if map_response.status_code != 200:
            print(f"Error: Could not get map data. Status code: {map_response.status_code}")
            return None
        if state_response.status_code != 200:
            print(f"Error: Could not get state data. Status code: {state_response.status_code}")
            return None

        map_data = map_response.json()
        state_data = state_response.json()

        # --- Extract Position ---
        x, y = None, None
        angle_deg = 0.0
        for entity in map_data.get('entities', []):
            if entity.get('type') == 'robot_position':
                points = entity.get('points', [])
                if len(points) >= 2:
                    x, y = points[0], points[1]
                    angle_deg = entity.get("metaData", {}).get("angle", 0.0)
                break

        if x is None:
            print("Warning: Could not find 'robot_position' in map data.")
            return None

        # --- Extract State Attributes ---
        battery_level = 0.0
        battery_flag = "unknown"
        status_value = "unknown"

        for attribute in state_data.get('attributes', []):
            attr_class = attribute.get('__class') or attribute.get('__class__') or ""

            if 'BatteryStateAttribute' in attr_class:
                battery_level = attribute.get('level', 0)
                battery_flag = attribute.get('flag', 'unknown')
            elif 'StatusStateAttribute' in attr_class:
                status_value = attribute.get('value', 'unknown')

        # --- Derive VDA 5050 Fields ---
        is_driving = status_value in ['cleaning', 'returning', 'moving', 'manual_control', 'going_to_target']
        is_charging = (battery_flag == 'charging') or (status_value == 'docked')
        is_paused = (status_value == 'paused')
        is_docked = (status_value == 'docked')

        return {
            "x": x,
            "y": y,
            "theta_deg": angle_deg,
            "battery": battery_level,
            "charging": is_charging,
            "driving": is_driving,
            "paused": is_paused,
            "docked": is_docked,
            "status": status_value
        }

    except requests.exceptions.RequestException as e:
        print(f"Error fetching robot data: {e}")
        return None
    except Exception as e:
        print(f"Error parsing robot data: {e}")
        import traceback
        traceback.print_exc()
        return None


# ============================================================================
# ORDER STATE - REFACTORED
# ============================================================================

ORDER_STATE = {
    "order_id": "",  # String order ID (VDA 5050 uses strings)
    "order_update_id": 0,
    "nodes": [],  # List of all nodes in current order
    "edges": [],  # List of all edges
    "current_target": None,  # {"nodeId": str, "sequenceId": int, "x": float, "y": float}
    "active": False,
    # CRITICAL: Track traversed nodes by sequenceId
    "traversed_sequence_ids": set(),
    # Track last position for "already at node" detection
    "robot_x": 0,
    "robot_y": 0,
    "is_docked": False,
}

# Track when bridge started (to ignore old retained messages)
BRIDGE_START_TIME = None


# ============================================================================
# ARRIVAL DETECTION & NAVIGATION
# ============================================================================

def check_arrival_and_advance(robot_x, robot_y, is_driving, is_docked):
    """
    Check if robot has arrived at current target node.
    If arrived and not driving:
    1. Execute any node actions
    2. Mark node as traversed
    3. Advance to next node
    """
    global ORDER_STATE

    # Update robot position for "already at node" checks
    ORDER_STATE["robot_x"] = robot_x
    ORDER_STATE["robot_y"] = robot_y
    ORDER_STATE["is_docked"] = is_docked

    if not ORDER_STATE["active"] or ORDER_STATE["current_target"] is None:
        return

    target = ORDER_STATE["current_target"]
    target_x = target.get("x", 0)
    target_y = target.get("y", 0)
    target_id = target.get("nodeId", "?")
    target_seq = target.get("sequenceId", 0)
    target_actions = target.get("actions", [])

    # Calculate distance to target
    distance = math.sqrt((robot_x - target_x) ** 2 + (robot_y - target_y) ** 2)

    # Check if arrived (within tolerance and not driving)
    if distance < ARRIVAL_TOLERANCE_MM and not is_driving:
        print(f"\n✅ ARRIVED at node: {target_id} (seq: {target_seq}, distance: {distance:.0f}mm)")

        # Execute node actions if any (VDA 5050: actions are part of order, not instantActions)
        if target_actions:
            print(f"   🎬 Executing {len(target_actions)} node action(s)")
            execute_node_actions(target_actions)

        # Mark as traversed
        ORDER_STATE["traversed_sequence_ids"].add(target_seq)
        ORDER_STATE["current_target"] = None

        # Navigate to next node
        navigate_to_next_node()


def is_already_at_node(node_x, node_y, node_id):
    """
    Check if robot is already at the specified node position.
    Uses tighter threshold than arrival detection.
    """
    robot_x = ORDER_STATE.get("robot_x", 0)
    robot_y = ORDER_STATE.get("robot_y", 0)

    distance = math.sqrt((robot_x - node_x) ** 2 + (robot_y - node_y) ** 2)

    # For home/dock node (node "0"), also check docked status
    if node_id == "0" and ORDER_STATE.get("is_docked", False):
        return True

    return distance < AT_NODE_THRESHOLD_MM


def handle_order_message(vda_message):
    """
    Handle order messages from /order topic with proper VDA 5050 stitching.

    VDA 5050 SPEC:
    "The last node of the previous base is the first base node in the updated order.
     The other nodes and edges from the previous base are NOT RESENT."

    This means order updates only contain:
    - Stitch node (last base node = first node in update)
    - New/updated nodes after stitch point
    - Already traversed nodes are NOT in the update
    """
    global ORDER_STATE

    order_id = str(vda_message.get("orderId", ""))
    order_update_id = vda_message.get("orderUpdateId", 0)
    update_nodes = vda_message.get("nodes", [])
    update_edges = vda_message.get("edges", [])

    # Check if this is an echo (same order, same update ID) - silently ignore
    is_echo = (order_id == ORDER_STATE["order_id"] and
               order_update_id == ORDER_STATE["order_update_id"])
    if is_echo:
        return

    print(f"\n📥 ORDER received: #{order_id} (updateId: {order_update_id})")

    # Determine order type
    is_new_order = (order_id != ORDER_STATE["order_id"])
    is_update = (order_id == ORDER_STATE["order_id"] and
                 order_update_id > ORDER_STATE["order_update_id"])
    is_deprecated = (order_id == ORDER_STATE["order_id"] and
                     order_update_id < ORDER_STATE["order_update_id"])

    if is_deprecated:
        print(f"   ⚠ Deprecated update ignored (have: {ORDER_STATE['order_update_id']}, got: {order_update_id})")
        return

    if is_new_order:
        prev_id = ORDER_STATE['order_id'] if ORDER_STATE['order_id'] else 'none'
        print(f"   🆕 New order (replacing: #{prev_id})")

        # RESET STATE for new order
        ORDER_STATE["order_id"] = order_id
        ORDER_STATE["order_update_id"] = order_update_id
        ORDER_STATE["nodes"] = update_nodes
        ORDER_STATE["edges"] = update_edges
        ORDER_STATE["active"] = True
        ORDER_STATE["current_target"] = None
        ORDER_STATE["traversed_sequence_ids"] = set()  # CRITICAL: Clear traversed set

    elif is_update:
        print(f"   📝 Order update: {ORDER_STATE['order_update_id']} → {order_update_id}")
        ORDER_STATE["order_update_id"] = order_update_id

        # VDA 5050: Update only contains stitch node onwards
        # We need to MERGE with our existing state, not replace

        if not update_nodes:
            print(f"   ⚠ Empty update, ignoring")
            return

        # Find stitch point (first node in update)
        stitch_node = update_nodes[0]
        stitch_seq_id = stitch_node.get("sequenceId", 0)
        stitch_node_id = stitch_node.get("nodeId", "")

        print(f"   Stitch point: {stitch_node_id} (seq:{stitch_seq_id})")

        # Keep traversed nodes (sequenceId < stitch_seq_id)
        # These are NOT in the update per VDA 5050
        preserved_nodes = [n for n in ORDER_STATE["nodes"] if n.get("sequenceId", 0) < stitch_seq_id]
        preserved_edges = [e for e in ORDER_STATE["edges"] if e.get("sequenceId", 0) < stitch_seq_id]

        print(f"   Preserved {len(preserved_nodes)} traversed nodes")

        # Merge: preserved + update
        ORDER_STATE["nodes"] = preserved_nodes + update_nodes
        ORDER_STATE["edges"] = preserved_edges + update_edges

        # Sort by sequenceId
        ORDER_STATE["nodes"].sort(key=lambda n: n.get("sequenceId", 0))
        ORDER_STATE["edges"].sort(key=lambda e: e.get("sequenceId", 0))

        # Reactivate navigation (for horizon release case)
        ORDER_STATE["active"] = True
        print(f"   ▶ Reactivating navigation (traversed: {len(ORDER_STATE['traversed_sequence_ids'])} nodes)")

    # Log nodes
    released_nodes = [n for n in ORDER_STATE["nodes"] if n.get("released", True)]
    horizon_nodes = [n for n in ORDER_STATE["nodes"] if not n.get("released", True)]
    print(f"   Total nodes: {len(ORDER_STATE['nodes'])} ({len(released_nodes)} base, {len(horizon_nodes)} horizon)")
    print(f"   Traversed sequences: {ORDER_STATE['traversed_sequence_ids']}")

    # Navigate to next released node we haven't visited yet
    navigate_to_next_node()


def navigate_to_next_node():
    """
    Navigate to the next released node in the order.

    VDA 5050 Flow:
    1. Find next released node not yet traversed
    2. Navigate to it
    3. When arrived, execute any node actions
    4. Then continue to next node
    """
    global ORDER_STATE

    if not ORDER_STATE["active"]:
        return

    nodes = ORDER_STATE["nodes"]
    traversed = ORDER_STATE["traversed_sequence_ids"]

    # Find next released node that hasn't been traversed
    for node in nodes:
        seq_id = node.get("sequenceId", 0)

        # CRITICAL: Skip already traversed nodes
        if seq_id in traversed:
            continue

        # Skip horizon (unreleased) nodes
        if not node.get("released", True):
            print(f"   ⏸ Reached horizon at seq {seq_id} - waiting for release")
            ORDER_STATE["active"] = False
            ORDER_STATE["current_target"] = None
            return

        node_id = node.get("nodeId", "")
        node_pos = node.get("nodePosition", {})
        node_actions = node.get("actions", [])

        x = node_pos.get("x")
        y = node_pos.get("y")

        if x is None or y is None:
            print(f"   ⚠ Node {node_id} (seq: {seq_id}) has no position, skipping")
            traversed.add(seq_id)  # Mark as traversed to skip
            continue

        # CRITICAL: Check if already at this node
        if is_already_at_node(x, y, node_id):
            print(f"   ✓ Already at node: {node_id} (seq: {seq_id})")
            traversed.add(seq_id)

            # Execute node actions if any
            if node_actions:
                print(f"   🎬 Executing {len(node_actions)} node action(s)")
                execute_node_actions(node_actions)

            continue

        # Navigate to this node
        print(f"   🎯 Navigating to node: {node_id} (seq: {seq_id}) at ({x}, {y})")

        ORDER_STATE["current_target"] = {
            "nodeId": node_id,
            "sequenceId": seq_id,
            "x": x,
            "y": y,
            "actions": node_actions  # Store actions to execute on arrival
        }

        # Handle home/dock node (node "0") specially
        if node_id == "0":
            # Double-check not already docked
            if ORDER_STATE.get("is_docked", False):
                print(f"   🏠 Already docked - skipping dock command")
                traversed.add(seq_id)
                ORDER_STATE["current_target"] = None
                continue
            else:
                print(f"   🏠 Home node - sending dock command")
                actions_basic.handle_dock(ROBOT_IP)
        else:
            # Send goTo command
            params = [
                {"key": "x", "value": x},
                {"key": "y", "value": y},
                {"key": "targetNodeId", "value": node_id}
            ]
            actions_navigation.handle_goto(ROBOT_IP, params)

        return  # Wait for arrival before continuing

    # All released nodes processed
    print(f"   ✅ Order #{ORDER_STATE['order_id']} - all released nodes complete")
    ORDER_STATE["active"] = False
    ORDER_STATE["current_target"] = None


def execute_node_actions(actions):
    """
    Execute actions attached to a node.

    These are NOT instant actions - they're part of the order.
    The bridge executes them directly when reaching the node.
    """
    for action in actions:
        action_type = action.get("actionType", "")
        action_id = action.get("actionId", "")

        print(f"      Executing: {action_type} (id: {action_id})")

        if action_type == "beep":
            actions_basic.handle_beep(ROBOT_IP)
        elif action_type == "locate":
            actions_basic.handle_locate(ROBOT_IP)
        elif action_type == "dock" or action_type == "quickCharge":
            if not ORDER_STATE.get("is_docked", False):
                actions_basic.handle_dock(ROBOT_IP)
        elif action_type == "startCleaning":
            actions_basic.handle_start_cleaning(ROBOT_IP)
        elif action_type == "stopCleaning":
            actions_basic.handle_stop_cleaning(ROBOT_IP)
        elif action_type == "pauseCleaning":
            actions_basic.handle_pause_cleaning(ROBOT_IP)
        elif action_type == "setFanSpeed":
            speed = "medium"
            for param in action.get("actionParameters", []):
                if param.get("key") == "speed":
                    speed = param.get("value", "medium")
            actions_basic.handle_set_fan_speed(ROBOT_IP, speed)
        else:
            print(f"      ⚠ Unknown node action: {action_type}")

        time.sleep(0.2)  # Small delay between actions


# ============================================================================
# INSTANT ACTIONS HANDLING
# ============================================================================

def handle_instant_actions(vda_message):
    """Handle instant actions from master control."""
    for vda_action in vda_message.get('actions', []):
        action_type = vda_action.get('actionType')
        print(f"\nReceived VDA action: {action_type}")

        if action_type == 'beep':
            actions_basic.handle_beep(ROBOT_IP)
        elif action_type == 'locate':
            actions_basic.handle_locate(ROBOT_IP)
        elif action_type == 'startCleaning':
            actions_basic.handle_start_cleaning(ROBOT_IP)
        elif action_type == 'stopCleaning':
            actions_basic.handle_stop_cleaning(ROBOT_IP)
        elif action_type == 'pauseCleaning':
            actions_basic.handle_pause_cleaning(ROBOT_IP)
        elif action_type == 'dock' or action_type == 'quickCharge':
            # CRITICAL: Check if already docked
            if ORDER_STATE.get("is_docked", False):
                print("   Already docked - skipping dock command")
            else:
                actions_basic.handle_dock(ROBOT_IP)
        elif action_type == 'setFanSpeed':
            speed = "medium"
            for param in vda_action.get('actionParameters', []):
                if param['key'] == 'speed':
                    speed = param['value']
            actions_basic.handle_set_fan_speed(ROBOT_IP, speed)
        elif action_type == 'goTo':
            actions_navigation.handle_goto(ROBOT_IP, vda_action.get('actionParameters', []))
        elif action_type == 'stopMovement':
            handle_stop(ROBOT_IP)
        elif action_type == 'pauseMovement':
            handle_pause(ROBOT_IP)
        elif action_type == 'resumeMovement':
            handle_resume(ROBOT_IP)
        elif action_type == 'resumeOrder':
            handle_resume_order()
        elif action_type == 'cancelOrder':
            handle_cancel_order()
        else:
            print(f"Unknown VDA action type: {action_type}")

        time.sleep(0.1)


def handle_cancel_order():
    """Handle cancelOrder instant action - stop and clear order state."""
    global ORDER_STATE

    print("🛑 CANCEL ORDER received - stopping robot")

    # Stop the robot
    handle_stop(ROBOT_IP)

    # Clear navigation state but keep order ID
    ORDER_STATE["active"] = False
    ORDER_STATE["current_target"] = None
    ORDER_STATE["nodes"] = []
    ORDER_STATE["edges"] = []
    ORDER_STATE["traversed_sequence_ids"] = set()

    print(f"   Order #{ORDER_STATE['order_id']} cancelled")


def handle_resume_order():
    """
    Handle resumeOrder instant action - resume navigation after stop.

    This is called when the GUI's Resume button is pressed.
    It reactivates navigation and continues to the next node.
    """
    global ORDER_STATE

    print("▶ RESUME ORDER received")

    if not ORDER_STATE["order_id"]:
        print("   ⚠ No order to resume")
        return

    if not ORDER_STATE["nodes"]:
        print("   ⚠ No nodes in order")
        return

    # Reactivate navigation
    ORDER_STATE["active"] = True

    # If we have a current target, re-navigate to it
    if ORDER_STATE["current_target"]:
        target = ORDER_STATE["current_target"]
        node_id = target.get("nodeId", "")
        x = target.get("x")
        y = target.get("y")

        print(f"   Resuming navigation to: {node_id} at ({x}, {y})")

        if node_id == "0":
            actions_basic.handle_dock(ROBOT_IP)
        else:
            params = [
                {"key": "x", "value": x},
                {"key": "y", "value": y},
                {"key": "targetNodeId", "value": node_id}
            ]
            actions_navigation.handle_goto(ROBOT_IP, params)
    else:
        # No current target, try to find next node
        print("   No current target, finding next node...")
        navigate_to_next_node()


def handle_stop(robot_ip):
    """Stop robot movement using Valetudo BasicControlCapability"""
    try:
        url = f"http://{robot_ip}/api/v2/robot/capabilities/BasicControlCapability"
        response = requests.put(url, json={"action": "stop"}, timeout=5)
        if response.status_code == 200:
            print("✓ Robot STOPPED")
        else:
            print(f"✗ Stop failed: {response.status_code}")
    except Exception as e:
        print(f"✗ Stop error: {e}")


def handle_pause(robot_ip):
    """Pause robot movement using Valetudo BasicControlCapability"""
    try:
        url = f"http://{robot_ip}/api/v2/robot/capabilities/BasicControlCapability"
        response = requests.put(url, json={"action": "pause"}, timeout=5)
        if response.status_code == 200:
            print("✓ Robot PAUSED")
        else:
            print(f"✗ Pause failed: {response.status_code}")
    except Exception as e:
        print(f"✗ Pause error: {e}")


def handle_resume(robot_ip):
    """Resume robot movement using Valetudo BasicControlCapability"""
    try:
        url = f"http://{robot_ip}/api/v2/robot/capabilities/BasicControlCapability"
        response = requests.put(url, json={"action": "start"}, timeout=5)
        if response.status_code == 200:
            print("✓ Robot RESUMED")
        else:
            print(f"✗ Resume failed: {response.status_code}")
    except Exception as e:
        print(f"✗ Resume error: {e}")


# ============================================================================
# STATE LOOP
# ============================================================================

def loop_state_and_visualization(mqtt_client):
    """Publishes the robot's full state AND visualization data, with arrival detection."""
    header_id_counter = 0
    debug_printed = False

    while True:
        data = get_robot_data()

        if data:
            header_id_counter += 1
            timestamp = time.strftime('%Y-%m-%dT%H:%M:%S.000Z', time.gmtime())
            theta_vda = normalize_theta(data["theta_deg"])

            agv_position = {
                "positionInitialized": True,
                "localizationScore": 1.0,
                "deviationRange": 0.0,
                "x": data["x"],
                "y": data["y"],
                "theta": theta_vda,
                "mapId": "lab"
            }
            velocity = {"vx": 0.0, "vy": 0.0, "omega": 0.0}

            # Send visualization to VDA 5050 topic
            vda_vis = {
                "headerId": header_id_counter,
                "timestamp": timestamp,
                "version": "2.1.0",
                "manufacturer": MANUFACTURER,
                "serialNumber": SERIAL_NUMBER,
                "agvPosition": agv_position,
                "velocity": velocity,
                "driving": data["driving"],
                "batteryState": {
                    "batteryCharge": float(data["battery"]),
                    "charging": data["charging"]
                }
            }
            mqtt_client.publish(TOPIC_VISUALIZATION, json.dumps(vda_vis))

            # ARRIVAL DETECTION - pass docked status
            check_arrival_and_advance(data["x"], data["y"], data["driving"], data["docked"])

            if not debug_printed:
                debug_printed = True
                print("--- Debug messages above will stop after first successful parse ---")

            print(f"STATE: Pos({data['x']}, {data['y']}) | Theta {data['theta_deg']}° | "
                  f"Batt {data['battery']}% | Status: {data['status']} | Docked: {data['docked']}")

        time.sleep(STATE_INTERVAL)


# ============================================================================
# MQTT CALLBACKS
# ============================================================================

def on_connect(client, userdata, flags, reason_code, properties):
    global BRIDGE_START_TIME
    if reason_code.is_failure:
        print(f"Failed to connect: {reason_code}. Retrying...")
    else:
        print(f"✓ Connected to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")

        from datetime import datetime, timezone
        BRIDGE_START_TIME = datetime.now(timezone.utc)
        print(f"⏰ Bridge start time: {BRIDGE_START_TIME.isoformat()}")

        client.subscribe(TOPIC_ORDER)
        print(f"✓ Subscribed to {TOPIC_ORDER}")

        client.subscribe(TOPIC_INSTANT_ACTIONS)
        print(f"✓ Subscribed to {TOPIC_INSTANT_ACTIONS}")


def on_message(client, userdata, msg):
    global BRIDGE_START_TIME

    try:
        payload = msg.payload.decode()
        vda_message = json.loads(payload)

        # Check message timestamp - ignore old/retained messages
        # BUT: Always process instant actions regardless of timestamp
        msg_timestamp = vda_message.get('timestamp', '')
        if msg_timestamp and BRIDGE_START_TIME and msg.topic != TOPIC_INSTANT_ACTIONS:
            try:
                from datetime import datetime
                msg_time = datetime.fromisoformat(msg_timestamp.replace('Z', '+00:00'))
                if msg_time < BRIDGE_START_TIME:
                    print(f"⏭ Ignoring old message (timestamp: {msg_timestamp})")
                    return
            except:
                pass

        # Route messages based on topic
        if msg.topic == TOPIC_ORDER:
            handle_order_message(vda_message)
        elif msg.topic == TOPIC_INSTANT_ACTIONS:
            handle_instant_actions(vda_message)

    except Exception as e:
        print(f"Error processing message: {e}")
        import traceback
        traceback.print_exc()


# ============================================================================
# MAIN
# ============================================================================

print("=== VDA 5050 Bridge (REFACTORED) Starting ===")

# Determine MQTT protocol version
if MQTT_VERSION == "3.1":
    protocol = mqtt.MQTTv31
    print("📋 Using MQTT protocol v3.1")
elif MQTT_VERSION == "3.1.1":
    protocol = mqtt.MQTTv311
    print("📋 Using MQTT protocol v3.1.1")
else:
    protocol = mqtt.MQTTv5
    print("📋 Using MQTT protocol v5.0")

# Create MQTT client
if MQTT_WEBSOCKET:
    client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2,
        "Bridge_V1",
        transport="websockets",
        protocol=protocol
    )
    print("🌐 Using WebSocket transport")
else:
    client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2,
        "Bridge_V1",
        protocol=protocol
    )

client.on_connect = on_connect
client.on_message = on_message

try:
    if MQTT_USERNAME and MQTT_PASSWORD:
        client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        print(f"🔐 MQTT auth enabled for user: {MQTT_USERNAME}")

    if MQTT_WEBSOCKET:
        import ssl

        ssl_context = ssl.create_default_context()
        ssl_context.check_hostname = False
        ssl_context.verify_mode = ssl.CERT_NONE
        client.tls_set_context(ssl_context)
        print("🔒 TLS encryption enabled")

    if MQTT_WEBSOCKET and MQTT_WS_PATH:
        client.ws_set_options(path=MQTT_WS_PATH)
        print(f"🌐 WebSocket path: {MQTT_WS_PATH}")

    print(f"📡 Connecting to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}...")
    client.connect(MQTT_BROKER, MQTT_PORT, 60)

    state_thread = threading.Thread(target=loop_state_and_visualization, args=(client,), daemon=True)
    state_thread.start()

    print(f"Bridge running. Publishing state every {STATE_INTERVAL} seconds.")
    print("Bridge is READY!")

    client.loop_forever()

except Exception as e:
    print(f"Failed to start bridge: {e}")
except KeyboardInterrupt:
    print("\nBridge stopped by user.")