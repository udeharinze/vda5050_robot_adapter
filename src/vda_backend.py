# ============================================================================
# vda_backend.py - VDA 5050 v2.1.0 Backend Controller
# ============================================================================
#
# Features:
#   - VDA5050 orders with Base/Horizon support
#   - Node and Edge actions (beep, locate, etc.)
#   - Action state tracking (WAITING → RUNNING → FINISHED/FAILED)
#   - Pause/Resume via Valetudo API
#   - Stop cancels current order
#   - Connection topic (ONLINE/OFFLINE)
#   - Home node uses dock command
#
# ============================================================================

import json
import time
import math
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any
import paho.mqtt.client as mqtt
from vda_config import (
    MQTT_BROKER, MQTT_PORT, MANUFACTURER, SERIAL_NUMBER, VDA_VERSION, MAP_ID,
    NODES, EDGES, NAV
)

# ============================================================================
# TUNABLE CONSTANTS
# ============================================================================

COMMAND_COOLDOWN_SEC = NAV.COMMAND_COOLDOWN_SEC
STALLED_TIMEOUT_SEC = NAV.STALL_TIMEOUT_SEC
ARRIVAL_TOLERANCE_MM = NAV.POSITION_TOLERANCE_MM
STATE_PUBLISH_INTERVAL = 1.0  # Publish state every second
CONNECTION_PUBLISH_INTERVAL = 1.0  # Publish connection every second
ACTION_TIMEOUT_SEC = 10.0  # Timeout for actions like beep/locate

# ============================================================================
# MQTT TOPICS
# ============================================================================

TOPIC_ORDER = f"uagv/v2/{MANUFACTURER}/{SERIAL_NUMBER}/order"
TOPIC_STATE = f"uagv/v2/{MANUFACTURER}/{SERIAL_NUMBER}/state"
TOPIC_INSTANT_ACTIONS = f"uagv/v2/{MANUFACTURER}/{SERIAL_NUMBER}/instantActions"
TOPIC_CONNECTION = f"uagv/v2/{MANUFACTURER}/{SERIAL_NUMBER}/connection"

TOPIC_BRIDGE_STATE = "vda5050/robot/state"
TOPIC_BRIDGE_ACTION = "vda5050/robot/instantAction"


# ============================================================================
# DATA CLASSES
# ============================================================================

@dataclass
class ActionState:
    """VDA5050 Action State"""
    actionId: str
    actionType: str
    actionStatus: str = "WAITING"  # WAITING, INITIALIZING, RUNNING, PAUSED, FINISHED, FAILED
    actionDescription: str = ""
    resultDescription: str = ""

    def to_dict(self) -> Dict:
        return {
            "actionId": self.actionId,
            "actionType": self.actionType,
            "actionStatus": self.actionStatus,
            "actionDescription": self.actionDescription,
            "resultDescription": self.resultDescription
        }


@dataclass
class NodeState:
    nodeId: str
    sequenceId: int
    released: bool = True
    nodePosition: Optional[Dict] = None
    actions: List[Dict] = field(default_factory=list)

    def to_dict(self) -> Dict:
        result = {
            "nodeId": self.nodeId,
            "sequenceId": self.sequenceId,
            "released": self.released
        }
        if self.nodePosition:
            result["nodePosition"] = self.nodePosition
        return result


@dataclass
class EdgeState:
    edgeId: str
    sequenceId: int
    released: bool = True
    startNodeId: str = ""
    endNodeId: str = ""
    actions: List[Dict] = field(default_factory=list)

    def to_dict(self) -> Dict:
        return {
            "edgeId": self.edgeId,
            "sequenceId": self.sequenceId,
            "released": self.released,
            "startNodeId": self.startNodeId,
            "endNodeId": self.endNodeId
        }


# ============================================================================
# ROBOT BACKEND CLASS
# ============================================================================

class RobotBackend:
    def __init__(self, broker: str, port: int, log_callback, planner=None):
        self.broker = broker
        self.port = port
        self.log = log_callback
        self.planner = planner
        self.client = None
        self.running = False
        self._connected = False

        # Robot state - Initialize at home position
        home_x = NODES.get("home", {}).get("x", 0.0)
        home_y = NODES.get("home", {}).get("y", 0.0)

        self.pose = {"x": home_x, "y": home_y, "theta": 0.0}
        self.display_pose = {"x": home_x, "y": home_y, "theta": 0.0}
        self.anim_start_pose = {"x": home_x, "y": home_y, "theta": 0.0}
        self.anim_start_time = 0.0

        self.battery = 1.0
        self.charging = False
        self.is_driving = False
        self.paused = False

        # VDA5050 Order State
        self.current_order: Optional[Dict] = None
        self.order_id = ""
        self.order_update_id = 0
        self.last_node_id = None  # Will be set on first command based on actual position
        self.last_node_sequence_id = 0

        # Node/Edge states
        self.node_states: List[NodeState] = []
        self.edge_states: List[EdgeState] = []
        self.traversed_node_indices: int = 0

        # Action states - track all actions from current order
        self.action_states: List[ActionState] = []
        self.current_action: Optional[ActionState] = None
        self.action_start_time: float = 0.0

        # Mission control
        self.mission_active = False
        self.mission_queue: List[str] = []
        self.current_target_node: Optional[str] = None
        self.waiting_at_decision_point = False
        self.new_base_request = False  # True when APPROACHING decision point
        self.last_command_ts = 0.0
        self.last_state_publish_ts = 0.0
        self.last_connection_publish_ts = 0.0  # Track connection topic publishing

        # Pending actions for current node (execute after arrival)
        self.pending_node_actions: List[Dict] = []

        # GUI state
        self.gui_node_states = {n: "idle" for n in NODES}

        # Header counter
        self._header_id = 0

        # Threading
        self.lock = threading.RLock()

        # Error tracking
        self.errors: List[Dict] = []

        # Initial position sync flag - to detect robot's actual position on startup
        self._initial_position_synced = False

    # ========================================================================
    # MQTT SETUP
    # ========================================================================

    def start(self):
        """Start MQTT client and main loop"""
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, f"vda_backend_{SERIAL_NUMBER}")
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message

        # Set last will for connection topic (OFFLINE when disconnected unexpectedly)
        last_will = {
            "headerId": 0,
            "timestamp": self._get_timestamp(),
            "version": VDA_VERSION,
            "manufacturer": MANUFACTURER,
            "serialNumber": SERIAL_NUMBER,
            "connectionState": "CONNECTIONBROKEN"
        }
        self.client.will_set(TOPIC_CONNECTION, json.dumps(last_will), qos=1, retain=True)

        self.log(f"🎯 VDA5050 Backend starting...", "info")

        try:
            self.client.connect(self.broker, self.port, 60)
            self.running = True
            self.client.loop_start()
            threading.Thread(target=self._main_loop, daemon=True).start()
            self.log("✓ Backend started", "success")
        except Exception as e:
            self.log(f"✗ MQTT connection failed: {e}", "error")

    def connect(self):
        """Alias for start() - for GUI compatibility"""
        self.start()

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        """MQTT connection callback"""
        if reason_code == 0:
            if not self._connected:
                self._connected = True
                self.log("✓ MQTT connected", "success")
                self.log(f"📡 Order topic: {TOPIC_ORDER}", "info")
                self.log(f"📡 State topic: {TOPIC_STATE}", "info")
                self.log(f"📡 Connection topic: {TOPIC_CONNECTION}", "info")
                self.log(f"📡 Instant Actions topic: {TOPIC_INSTANT_ACTIONS}", "info")

                # Subscribe to topics
            client.subscribe(TOPIC_ORDER)
            client.subscribe(TOPIC_BRIDGE_STATE)
            client.subscribe(TOPIC_INSTANT_ACTIONS)

            # Publish ONLINE connection state (with logging)
            self._publish_connection_state("ONLINE", log=True)
        else:
            self._connected = False
            self.log(f"✗ MQTT connect failed: {reason_code}", "error")

    def _on_disconnect(self, client, userdata, flags, reason_code, properties):
        """MQTT disconnect callback"""
        self._connected = False
        self.log("⚠ MQTT disconnected", "warning")

    def _on_message(self, client, userdata, msg):
        """MQTT message callback"""
        try:
            payload = json.loads(msg.payload.decode())

            if msg.topic == TOPIC_ORDER:
                self._handle_order(payload)
            elif msg.topic == TOPIC_BRIDGE_STATE:
                self._handle_bridge_state(payload)
            elif msg.topic == TOPIC_INSTANT_ACTIONS:
                self._handle_instant_actions(payload)

        except json.JSONDecodeError as e:
            self.log(f"✗ JSON error: {e}", "error")
        except Exception as e:
            self.log(f"✗ Message error: {e}", "error")

    # ========================================================================
    # CONNECTION STATE
    # ========================================================================

    def _publish_connection_state(self, state: str, log: bool = False):
        """Publish connection state (ONLINE, OFFLINE, CONNECTIONBROKEN)"""
        if not self.client:
            return

        connection_msg = {
            "headerId": self._next_header_id(),
            "timestamp": self._get_timestamp(),
            "version": VDA_VERSION,
            "manufacturer": MANUFACTURER,
            "serialNumber": SERIAL_NUMBER,
            "connectionState": state
        }
        self.client.publish(TOPIC_CONNECTION, json.dumps(connection_msg), qos=1, retain=True)
        if log:
            self.log(f"📡 Connection: {state}", "info")

    # ========================================================================
    # BRIDGE STATE HANDLING
    # ========================================================================

    def _handle_bridge_state(self, payload: Dict):
        """Update internal state from Valetudo bridge"""
        pos = payload.get("agvPosition", {})
        if pos:
            new_x = pos.get("x")
            new_y = pos.get("y")
            new_theta = pos.get("theta")

            if new_x is not None and new_y is not None:
                self.anim_start_pose = self.display_pose.copy()
                self.anim_start_time = time.time()

                self.pose["x"] = new_x
                self.pose["y"] = new_y
                if new_theta is not None:
                    self.pose["theta"] = new_theta

                # Mark that we've received at least one position update
                self._initial_position_synced = True

        self.is_driving = payload.get("driving", False)

        batt = payload.get("batteryState", {})
        if batt:
            battery_charge = batt.get("batteryCharge")
            if battery_charge is not None:
                self.battery = battery_charge / 100.0
            self.charging = batt.get("charging", False)

    def _update_animation(self):
        """Smooth animation for display pose"""
        if NAV.ANIMATION_DURATION_SEC <= 0:
            self.display_pose = self.pose.copy()
            return

        elapsed = time.time() - self.anim_start_time
        t = min(1.0, elapsed / NAV.ANIMATION_DURATION_SEC)

        self.display_pose["x"] = (
                self.anim_start_pose["x"] +
                (self.pose["x"] - self.anim_start_pose["x"]) * t
        )
        self.display_pose["y"] = (
                self.anim_start_pose["y"] +
                (self.pose["y"] - self.anim_start_pose["y"]) * t
        )
        self.display_pose["theta"] = self.pose["theta"]

    # ========================================================================
    # INITIAL POSITION SYNC & PATH FINDING
    # ========================================================================

    def _sync_initial_position(self):
        """Sync last_node_id with robot's actual position on startup"""
        nearest = self._find_nearest_node()
        if nearest:
            # Check if actually at this node (within tolerance)
            node_data = NODES.get(nearest)
            if node_data:
                dist = math.sqrt(
                    (self.pose["x"] - node_data["x"]) ** 2 +
                    (self.pose["y"] - node_data["y"]) ** 2
                )
                if dist < ARRIVAL_TOLERANCE_MM:
                    self.last_node_id = nearest
                    self.log(f"📍 Initial position: at node '{nearest}'", "info")
                else:
                    self.last_node_id = nearest
                    self.log(f"📍 Initial position: near node '{nearest}' (dist={dist:.0f}mm)", "info")
        self._initial_position_synced = True

    def _find_nearest_node(self) -> Optional[str]:
        """Find the nearest defined node to robot's current position"""
        if not self.pose:
            return None

        robot_x = self.pose.get("x", 0)
        robot_y = self.pose.get("y", 0)

        min_dist = float('inf')
        nearest_node = None

        for node_id, node_data in NODES.items():
            node_x = node_data.get("x", 0)
            node_y = node_data.get("y", 0)
            dist = math.sqrt((robot_x - node_x) ** 2 + (robot_y - node_y) ** 2)

            if dist < min_dist:
                min_dist = dist
                nearest_node = node_id

        return nearest_node

    def _find_path(self, start_node: str, end_node: str) -> List[str]:
        """
        Find a valid path from start_node to end_node using BFS.
        Uses the EDGES configuration to determine valid connections.
        Returns list of node IDs representing the path, or empty list if no path exists.
        """
        if start_node == end_node:
            return [start_node]

        if start_node not in NODES or end_node not in NODES:
            self.log(f"✗ Invalid nodes: {start_node} or {end_node}", "error")
            return []

        # Build adjacency list from EDGES (bidirectional)
        adjacency = {node: [] for node in NODES}
        for edge in EDGES:
            start = edge.get("start")
            end = edge.get("end")
            if start and end:
                adjacency[start].append(end)
                adjacency[end].append(start)  # Bidirectional

        # BFS to find shortest path
        queue = deque([(start_node, [start_node])])
        visited = {start_node}

        while queue:
            current, path = queue.popleft()

            for neighbor in adjacency.get(current, []):
                if neighbor == end_node:
                    return path + [neighbor]

                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))

        self.log(f"✗ No path found from {start_node} to {end_node}", "error")
        return []

    def send_goto_node(self, target_node: str):
        """
        Navigate to a target node using valid edges.
        Called when user clicks on a node in the GUI.
        """
        with self.lock:
            # Determine starting node
            start_node = self.last_node_id

            # If we don't have a last_node_id, find nearest node
            if not start_node:
                start_node = self._find_nearest_node()
                if start_node:
                    self.log(f"📍 Starting from nearest node: {start_node}", "info")
                    self.last_node_id = start_node
                else:
                    self.log("✗ Cannot determine starting position", "error")
                    return

            # If already at target
            if start_node == target_node:
                self.log(f"Already at {target_node}", "info")
                return

            # Find path using edges
            path = self._find_path(start_node, target_node)

            if not path:
                self.log(f"✗ No valid path from {start_node} to {target_node}", "error")
                return

            self.log(f"📍 Path: {' → '.join(path)}", "info")

            # Use the existing mission sequence logic
            self.set_mission_sequence(path)

    def send_mission(self, path: List[str]):
        """
        Start a mission with the given path.
        Called by GUI for predefined missions.
        """
        if not path:
            self.log("✗ Empty path provided", "error")
            return

        # If we don't know current position, find it first
        if not self.last_node_id:
            nearest = self._find_nearest_node()
            if nearest:
                self.last_node_id = nearest
                self.log(f"📍 Current position: {nearest}", "info")
            else:
                self.log("✗ Cannot determine starting position", "error")
                return

        # Check if robot is at the start of the mission
        if self.last_node_id != path[0]:
            # Robot is not at the start of the mission
            # Find path from current location to mission start
            prefix_path = self._find_path(self.last_node_id, path[0])
            if prefix_path and len(prefix_path) > 1:
                # Combine: prefix (without last element, since it's path[0]) + mission path
                full_path = prefix_path[:-1] + path
                self.log(f"📍 Extended path: {' → '.join(full_path)}", "info")
                self.set_mission_sequence(full_path)
            else:
                # No valid path or already at start, use mission as-is
                self.set_mission_sequence(path)
        else:
            self.set_mission_sequence(path)

    # ========================================================================
    # VDA5050 ORDER HANDLING
    # ========================================================================

    def _handle_order(self, order: Dict):
        """Handle incoming VDA5050 order"""
        incoming_order_id = order.get("orderId", "")
        incoming_update_id = order.get("orderUpdateId", 0)

        self.log(f"📥 ORDER: {incoming_order_id} (update: {incoming_update_id})", "info")

        with self.lock:
            if incoming_order_id == self.order_id:
                if incoming_update_id <= self.order_update_id:
                    self.log(f"   ⚠ Old update ignored", "warning")
                    return
                self.log(f"   📝 ORDER UPDATE", "info")
                self._process_order_update(order)
            else:
                if self.mission_active:
                    self.log(f"   ⏹ Cancelling previous order", "warning")
                self._process_new_order(order)

    def _process_new_order(self, order: Dict):
        """Process a new VDA5050 order"""
        self.order_id = order.get("orderId", "")
        self.order_update_id = order.get("orderUpdateId", 0)
        self.current_order = order

        self._reset_gui_colors()
        self.action_states = []  # Clear action states

        # Parse nodes with their actions
        self.node_states = []
        self.mission_queue = []
        self.traversed_node_indices = 0

        nodes = order.get("nodes", [])
        for node_data in nodes:
            node_actions = node_data.get("actions", [])

            ns = NodeState(
                nodeId=node_data.get("nodeId", ""),
                sequenceId=node_data.get("sequenceId", 0),
                released=node_data.get("released", True),
                nodePosition=node_data.get("nodePosition"),
                actions=node_actions
            )
            self.node_states.append(ns)

            # Register actions in action_states
            for action in node_actions:
                action_state = ActionState(
                    actionId=action.get("actionId", f"action_{len(self.action_states)}"),
                    actionType=action.get("actionType", "unknown"),
                    actionStatus="WAITING",
                    actionDescription=action.get("actionDescription", "")
                )
                self.action_states.append(action_state)

            status = "✓ BASE" if ns.released else "○ HORIZON"
            actions_str = f" [{len(node_actions)} actions]" if node_actions else ""
            self.log(f"   [{ns.sequenceId}] {ns.nodeId}: {status}{actions_str}", "info")

            if ns.released:
                self.mission_queue.append(ns.nodeId)
                if ns.nodeId in self.gui_node_states:
                    self.gui_node_states[ns.nodeId] = "planned"

        # Parse edges with their actions
        self.edge_states = []
        for edge_data in order.get("edges", []):
            edge_actions = edge_data.get("actions", [])

            es = EdgeState(
                edgeId=edge_data.get("edgeId", ""),
                sequenceId=edge_data.get("sequenceId", 0),
                released=edge_data.get("released", True),
                startNodeId=edge_data.get("startNodeId", ""),
                endNodeId=edge_data.get("endNodeId", ""),
                actions=edge_actions
            )
            self.edge_states.append(es)

            # Register edge actions
            for action in edge_actions:
                action_state = ActionState(
                    actionId=action.get("actionId", f"action_{len(self.action_states)}"),
                    actionType=action.get("actionType", "unknown"),
                    actionStatus="WAITING",
                    actionDescription=action.get("actionDescription", "")
                )
                self.action_states.append(action_state)

        # Decision point
        decision_point = self._find_decision_point()
        has_horizon = any(not ns.released for ns in self.node_states)
        if decision_point and has_horizon:
            self.log(f"   🎯 Decision point: {decision_point}", "info")

        # Start mission
        self.mission_active = True
        self.paused = False
        self.waiting_at_decision_point = False
        self.current_target_node = None
        self.pending_node_actions = []

        # Skip first node if already there
        if self.mission_queue and self._is_at_node(self.mission_queue[0]):
            first = self.mission_queue.pop(0)
            self.log(f"   Already at {first}, skipping", "info")
            self.gui_node_states[first] = "done"
            self.last_node_id = first
            self.traversed_node_indices = 1

            # Execute actions for this node
            for ns in self.node_states:
                if ns.nodeId == first and ns.actions:
                    self._execute_node_actions(ns.actions)

        self._advance_mission()

    def _process_order_update(self, order: Dict):
        """Process order update"""
        self.order_update_id = order.get("orderUpdateId", 0)
        self.current_order = order

        # Update node_states
        new_node_states = []
        for node_data in order.get("nodes", []):
            node_actions = node_data.get("actions", [])
            ns = NodeState(
                nodeId=node_data.get("nodeId", ""),
                sequenceId=node_data.get("sequenceId", 0),
                released=node_data.get("released", True),
                nodePosition=node_data.get("nodePosition"),
                actions=node_actions
            )
            new_node_states.append(ns)

            if ns.released and ns.nodeId not in self.mission_queue:
                if not self._is_at_node(ns.nodeId):
                    self.mission_queue.append(ns.nodeId)
                    if ns.nodeId in self.gui_node_states:
                        self.gui_node_states[ns.nodeId] = "planned"
                    self.log(f"   Released: {ns.nodeId}", "info")

        self.node_states = new_node_states

        # Update edge_states
        self.edge_states = []
        for edge_data in order.get("edges", []):
            edge_actions = edge_data.get("actions", [])
            es = EdgeState(
                edgeId=edge_data.get("edgeId", ""),
                sequenceId=edge_data.get("sequenceId", 0),
                released=edge_data.get("released", True),
                startNodeId=edge_data.get("startNodeId", ""),
                endNodeId=edge_data.get("endNodeId", ""),
                actions=edge_actions
            )
            self.edge_states.append(es)

        if self.waiting_at_decision_point:
            self.log(f"   ▶ Resuming from decision point", "success")
            self.waiting_at_decision_point = False
            self.new_base_request = False
            self._advance_mission()
        elif self.new_base_request:
            # Order update received while approaching decision point - good!
            self.log(f"   ✓ Base extended before reaching decision point", "success")
            self.new_base_request = False

    def _find_decision_point(self) -> Optional[str]:
        """Find last released node"""
        decision_point = None
        for ns in self.node_states:
            if ns.released:
                decision_point = ns.nodeId
            else:
                break
        return decision_point

    # ========================================================================
    # ACTION EXECUTION
    # ========================================================================

    def _execute_node_actions(self, actions: List[Dict]):
        """
        Execute actions attached to a node.

        Blocking Types:
        - NONE: Execute in parallel (don't wait)
        - SOFT: Execute, robot stopped but other actions can run in parallel
        - HARD: Execute alone, wait for completion before next action
        """
        # Separate actions by blocking type
        none_actions = []
        soft_actions = []
        hard_actions = []

        for action in actions:
            blocking_type = action.get("blockingType", "HARD")
            if blocking_type == "NONE":
                none_actions.append(action)
            elif blocking_type == "SOFT":
                soft_actions.append(action)
            else:  # HARD
                hard_actions.append(action)

        # Execute NONE actions first (fire and forget, parallel with driving)
        for action in none_actions:
            self._execute_single_action(action, wait=False)

        # Execute SOFT actions (can run in parallel with each other)
        for action in soft_actions:
            self._execute_single_action(action, wait=False)

        # Execute HARD actions sequentially (wait for each)
        for action in hard_actions:
            self._execute_single_action(action, wait=True)

    def _execute_single_action(self, action: Dict, wait: bool = True):
        """Execute a single action"""
        action_type = action.get("actionType", "")
        action_id = action.get("actionId", "")
        blocking_type = action.get("blockingType", "HARD")
        action_params = action.get("actionParameters", [])

        self.log(f"🎬 Action: {action_type} [{blocking_type}]", "info")

        # Update action state to RUNNING
        self._update_action_state(action_id, "RUNNING")

        # Execute based on action type
        success = False

        if action_type == "beep":
            self._send_action_to_bridge(action)
            success = True

        elif action_type == "locate":
            self._send_action_to_bridge(action)
            success = True

        elif action_type == "setFanSpeed":
            # Extract speed parameter
            speed = "medium"  # default
            for param in action_params:
                if param.get("key") == "speed":
                    speed = param.get("value", "medium")

            # Send setFanSpeed action
            fan_action = {
                "actionType": "setFanSpeed",
                "actionId": action_id,
                "blockingType": blocking_type,
                "actionParameters": [{"key": "speed", "value": speed}]
            }
            self._send_action_to_bridge(fan_action)
            self.log(f"   Fan speed → {speed}", "info")
            success = True

        elif action_type == "setVolume":
            # Extract volume parameter
            volume = 50  # default
            for param in action_params:
                if param.get("key") == "volume":
                    volume = param.get("value", 50)

            volume_action = {
                "actionType": "setVolume",
                "actionId": action_id,
                "blockingType": blocking_type,
                "actionParameters": [{"key": "volume", "value": volume}]
            }
            self._send_action_to_bridge(volume_action)
            self.log(f"   Volume → {volume}", "info")
            success = True

        elif action_type in ["startCleaning", "stopCleaning", "pauseCleaning"]:
            self._send_action_to_bridge(action)
            success = True

        elif action_type in ["dock", "quickCharge"]:
            self._send_action_to_bridge(action)
            success = True

        else:
            # Unknown action - forward to bridge anyway
            self.log(f"   ⚠ Unknown action type: {action_type}", "warning")
            self._send_action_to_bridge(action)
            success = True

        # Wait for action to complete if blocking
        if wait and success:
            # Simple delay for actions (in real system, wait for confirmation)
            action_delay = self._get_action_delay(action_type)
            time.sleep(action_delay)

        # Update action state
        if success:
            self._update_action_state(action_id, "FINISHED")
            self.log(f"   ✓ {action_type} finished", "success")
        else:
            self._update_action_state(action_id, "FAILED", "Action failed")
            self.log(f"   ✗ {action_type} failed", "error")

    def _get_action_delay(self, action_type: str) -> float:
        """Get delay time for action completion"""
        delays = {
            "beep": 1.0,
            "locate": 2.0,
            "setFanSpeed": 0.5,
            "setVolume": 0.3,
            "startCleaning": 0.5,
            "stopCleaning": 0.5,
            "pauseCleaning": 0.5,
            "dock": 0.5,
            "quickCharge": 0.5,
        }
        return delays.get(action_type, 0.5)

    def _send_action_to_bridge(self, action: Dict):
        """Send action to bridge for execution"""
        action_msg = {
            "headerId": self._next_header_id(),
            "timestamp": self._get_timestamp(),
            "version": VDA_VERSION,
            "manufacturer": MANUFACTURER,
            "serialNumber": SERIAL_NUMBER,
            "actions": [action]
        }
        self.client.publish(TOPIC_BRIDGE_ACTION, json.dumps(action_msg))

    def _update_action_state(self, action_id: str, status: str, result: str = ""):
        """Update action state"""
        for action_state in self.action_states:
            if action_state.actionId == action_id:
                action_state.actionStatus = status
                if result:
                    action_state.resultDescription = result
                break

    # ========================================================================
    # NAVIGATION
    # ========================================================================

    def _advance_mission(self):
        """Advance to next node"""
        with self.lock:
            if self.paused:
                return

            if not self.mission_queue:
                has_horizon = any(not ns.released for ns in self.node_states)
                if has_horizon:
                    self.log(f"⏸ Waiting at decision point", "warning")
                    self.waiting_at_decision_point = True
                    self.new_base_request = False  # Already at decision point, too late
                    self.current_target_node = None
                else:
                    self._complete_mission()
                return

            next_node = self.mission_queue.pop(0)
            self.current_target_node = next_node

            # Check if approaching decision point (next node is last in base, horizon exists)
            # newBaseRequest should be TRUE when we're heading to the decision point
            has_horizon = any(not ns.released for ns in self.node_states)
            is_heading_to_decision_point = (len(self.mission_queue) == 0) and has_horizon

            if is_heading_to_decision_point:
                self.new_base_request = True
                self.log(f"📡 newBaseRequest: true (approaching decision point)", "warning")
            else:
                self.new_base_request = False

            # Get pending actions for this node
            self.pending_node_actions = []
            for ns in self.node_states:
                if ns.nodeId == next_node:
                    self.pending_node_actions = ns.actions.copy()
                    break

            if next_node in self.gui_node_states:
                self.gui_node_states[next_node] = "planned"

            self.log(f"→ Moving to: {next_node}", "info")
            self._send_navigation_command(next_node)

    def _send_navigation_command(self, node_id: str):
        """Send goTo or dock command"""
        self.last_command_ts = time.time()

        if node_id == "home":
            self.log(f"📤 DOCK command (home)", "info")
            self._send_dock_command()
        else:
            self._send_goto(node_id)

    def _send_goto(self, node_id: str):
        """Send goTo command"""
        target = None

        # First try: Get from order's nodePosition
        for ns in self.node_states:
            if ns.nodeId == node_id and ns.nodePosition:
                pos = ns.nodePosition
                if pos.get("x") is not None and pos.get("y") is not None:
                    target = {"x": pos["x"], "y": pos["y"]}
                    break

        # Second try: Get from config NODES
        if not target and node_id in NODES:
            node_config = NODES[node_id]
            target = {"x": node_config["x"], "y": node_config["y"]}

        if not target:
            self.log(f"✗ Unknown node: {node_id} (not in order or config)", "error")
            return

        action_msg = {
            "headerId": self._next_header_id(),
            "timestamp": self._get_timestamp(),
            "version": VDA_VERSION,
            "manufacturer": MANUFACTURER,
            "serialNumber": SERIAL_NUMBER,
            "actions": [{
                "actionType": "goTo",
                "actionId": f"goTo_{node_id}_{int(time.time() * 1000)}",
                "blockingType": "HARD",
                "actionParameters": [
                    {"key": "x", "value": target["x"]},
                    {"key": "y", "value": target["y"]},
                    {"key": "targetNodeId", "value": node_id}
                ]
            }]
        }

        self.log(f"📤 goTo: {node_id} ({int(target['x'])}, {int(target['y'])})", "info")
        self.client.publish(TOPIC_BRIDGE_ACTION, json.dumps(action_msg))

    def _send_dock_command(self):
        """Send dock command"""
        action_msg = {
            "headerId": self._next_header_id(),
            "timestamp": self._get_timestamp(),
            "version": VDA_VERSION,
            "manufacturer": MANUFACTURER,
            "serialNumber": SERIAL_NUMBER,
            "actions": [{
                "actionType": "dock",
                "actionId": f"dock_{int(time.time() * 1000)}",
                "blockingType": "HARD",
                "actionParameters": []
            }]
        }
        self.client.publish(TOPIC_BRIDGE_ACTION, json.dumps(action_msg))

    def _complete_mission(self):
        """Complete mission"""
        self.log(f"✅ ORDER COMPLETE: {self.order_id}", "success")
        self.mission_active = False
        self.current_target_node = None
        self.waiting_at_decision_point = False
        self.new_base_request = False

        # Clear VDA 5050 state
        self.node_states = []
        self.edge_states = []
        self.action_states = []

        # Clear order info
        self.current_order = None
        self.order_id = ""
        self.order_update_id = 0

        # Reset all node colors to idle
        self._reset_gui_colors()

    def _reset_gui_colors(self):
        """Reset all nodes to idle"""
        for node_id in NODES:
            self.gui_node_states[node_id] = "idle"

    # ========================================================================
    # MAIN LOOP
    # ========================================================================

    def _main_loop(self):
        """Main loop"""
        while self.running:
            time.sleep(0.1)  # Reduced from 0.2 for faster arrival detection

            self._update_animation()

            # Publish state periodically
            if time.time() - self.last_state_publish_ts > STATE_PUBLISH_INTERVAL:
                self._publish_state()
                self.last_state_publish_ts = time.time()

            # Publish connection state every second
            if time.time() - self.last_connection_publish_ts > CONNECTION_PUBLISH_INTERVAL:
                self._publish_connection_state("ONLINE")
                self.last_connection_publish_ts = time.time()

            if not self.mission_active or not self.current_target_node:
                continue

            if self.paused or self.waiting_at_decision_point:
                continue

            # Check arrival - don't let command cooldown block arrival detection!
            if self._is_at_node(self.current_target_node) and not self.is_driving:
                self._on_node_reached(self.current_target_node)
            # Only apply cooldown for stalled retry commands
            elif time.time() - self.last_command_ts > STALLED_TIMEOUT_SEC and not self.is_driving:
                self.log(f"⚠ Stalled, retrying...", "warning")
                self._send_navigation_command(self.current_target_node)

    def _on_node_reached(self, node_id: str):
        """Node reached - execute pending actions then advance"""
        self.log(f"✓ ARRIVED: {node_id}", "success")

        self.last_node_id = node_id
        self.traversed_node_indices += 1

        for ns in self.node_states:
            if ns.nodeId == node_id:
                self.last_node_sequence_id = ns.sequenceId

                # ===== FIX: Remove the incoming edge =====
                # VDA 5050: When node is traversed, remove edge leading TO it
                # Edge sequenceId = Node sequenceId - 1
                incoming_edge_seq_id = ns.sequenceId - 1
                self.edge_states = [
                    es for es in self.edge_states
                    if es.sequenceId != incoming_edge_seq_id
                ]
                self.log(f"   Removed edge with seqId {incoming_edge_seq_id}")

                # Log node removal as well
                self.log(f"   Removed node {node_id} with seqId {ns.sequenceId}")
                # ===== END FIX =====

                break

        if node_id in self.gui_node_states:
            self.gui_node_states[node_id] = "done"

        # Execute pending actions for this node
        if self.pending_node_actions:
            self.log(f"   Executing {len(self.pending_node_actions)} actions...", "info")
            self._execute_node_actions(self.pending_node_actions)
            self.pending_node_actions = []

        self._advance_mission()

    def _is_at_node(self, node_id: str) -> bool:
        """Check if at node"""
        target = None

        # First try: Get from order's nodePosition
        for ns in self.node_states:
            if ns.nodeId == node_id and ns.nodePosition:
                pos = ns.nodePosition
                if pos.get("x") is not None and pos.get("y") is not None:
                    target = {"x": pos["x"], "y": pos["y"]}
                    break

        # Second try: Get from config NODES
        if not target and node_id in NODES:
            node_config = NODES[node_id]
            target = {"x": node_config["x"], "y": node_config["y"]}

        if not target:
            return False

        dist = math.sqrt(
            (self.pose["x"] - target["x"]) ** 2 +
            (self.pose["y"] - target["y"]) ** 2
        )
        return dist < ARRIVAL_TOLERANCE_MM

    # ========================================================================
    # INSTANT ACTIONS (pause, resume, stop from master control)
    # ========================================================================

    def _handle_instant_actions(self, payload: Dict):
        """Handle instant actions from master control"""
        actions = payload.get("actions", [])
        for action in actions:
            action_type = action.get("actionType", "")
            self.log(f"📥 Instant Action: {action_type}", "info")

            if action_type == "stopPause" or action_type == "resumeOrder":
                self.resume_mission()
            elif action_type == "startPause" or action_type == "pauseOrder":
                self.pause_mission()
            elif action_type == "cancelOrder":
                self.emergency_stop()
            else:
                # Forward other actions to bridge
                self.client.publish(TOPIC_BRIDGE_ACTION, json.dumps(payload))

    # ========================================================================
    # STATE PUBLISHING
    # ========================================================================

    def _publish_state(self):
        """Publish VDA5050 state"""
        if not self.client:
            return

        # Build remaining nodeStates
        remaining_node_states = []
        for i, ns in enumerate(self.node_states):
            if i >= self.traversed_node_indices:
                remaining_node_states.append(ns.to_dict())

        # Build remaining edgeStates
        remaining_edge_states = []
        for es in self.edge_states:
            remaining_edge_states.append(es.to_dict())

        # Build actionStates
        action_states_list = [a.to_dict() for a in self.action_states]

        state = {
            "headerId": self._next_header_id(),
            "timestamp": self._get_timestamp(),
            "version": VDA_VERSION,
            "manufacturer": MANUFACTURER,
            "serialNumber": SERIAL_NUMBER,
            "orderId": self.order_id,
            "orderUpdateId": self.order_update_id,
            "lastNodeId": self.last_node_id or "",  # VDA 5050: empty string if no node traversed
            "lastNodeSequenceId": self.last_node_sequence_id,
            "driving": self.is_driving,
            "paused": self.paused,
            "newBaseRequest": self.new_base_request,
            "distanceSinceLastNode": 0.0,
            "operatingMode": "AUTOMATIC",
            "nodeStates": remaining_node_states,
            "edgeStates": remaining_edge_states,
            "actionStates": action_states_list,
            "agvPosition": {
                "x": self.pose["x"],
                "y": self.pose["y"],
                "theta": self.pose["theta"],
                "mapId": MAP_ID,
                "positionInitialized": True
            },
            "velocity": {"vx": 0.0, "vy": 0.0, "omega": 0.0},
            "batteryState": {
                "batteryCharge": self.battery * 100,
                "charging": self.charging
            },
            "safetyState": {
                "eStop": "NONE",
                "fieldViolation": False
            },
            "errors": self.errors,
            "informations": []
        }

        self.client.publish(TOPIC_STATE, json.dumps(state))

    # ========================================================================
    # GUI CONTROLS - PAUSE/RESUME/STOP
    # ========================================================================

    def pause_mission(self):
        """Pause mission - sends pause command to robot"""
        self.paused = True
        self.log("⏸ Mission PAUSED", "warning")

        # Send pause command to bridge
        action_msg = {
            "headerId": self._next_header_id(),
            "timestamp": self._get_timestamp(),
            "version": VDA_VERSION,
            "manufacturer": MANUFACTURER,
            "serialNumber": SERIAL_NUMBER,
            "actions": [{
                "actionType": "pauseMovement",
                "actionId": f"pause_{int(time.time() * 1000)}",
                "blockingType": "HARD",
                "actionParameters": []
            }]
        }
        self.client.publish(TOPIC_BRIDGE_ACTION, json.dumps(action_msg))

    def resume_mission(self):
        """Resume mission - sends resume command to robot"""
        if self.mission_queue or self.current_target_node or self.waiting_at_decision_point:
            self.paused = False
            self.log("▶ Mission RESUMED", "success")

            # Send resume command to bridge
            action_msg = {
                "headerId": self._next_header_id(),
                "timestamp": self._get_timestamp(),
                "version": VDA_VERSION,
                "manufacturer": MANUFACTURER,
                "serialNumber": SERIAL_NUMBER,
                "actions": [{
                    "actionType": "resumeMovement",
                    "actionId": f"resume_{int(time.time() * 1000)}",
                    "blockingType": "HARD",
                    "actionParameters": []
                }]
            }
            self.client.publish(TOPIC_BRIDGE_ACTION, json.dumps(action_msg))

            # If we have a current target, resend navigation command
            if self.current_target_node and not self.waiting_at_decision_point:
                self._send_navigation_command(self.current_target_node)

    def emergency_stop(self):
        """Emergency stop - cancels order and stops robot"""
        self.log("🛑 ORDER CANCELLED", "error")

        # Send stop command to bridge
        action_msg = {
            "headerId": self._next_header_id(),
            "timestamp": self._get_timestamp(),
            "version": VDA_VERSION,
            "manufacturer": MANUFACTURER,
            "serialNumber": SERIAL_NUMBER,
            "actions": [{
                "actionType": "stopMovement",
                "actionId": f"stop_{int(time.time() * 1000)}",
                "blockingType": "HARD",
                "actionParameters": []
            }]
        }
        self.client.publish(TOPIC_BRIDGE_ACTION, json.dumps(action_msg))

        # Clear order state
        self.mission_active = False
        self.paused = False
        self.mission_queue = []
        self.current_target_node = None
        self.waiting_at_decision_point = False
        self.current_order = None
        self.order_id = ""
        self.node_states = []
        self.edge_states = []
        self.action_states = []
        self._reset_gui_colors()

    def stop_mission(self):
        """Stop current mission - alias for emergency_stop"""
        self.emergency_stop()

    def send_instant_action(self, action_type: str):
        """
        Send an instant action command.
        Called by GUI for actions like pause, resume, beep, locate, dock.
        """
        # Map GUI action names to actual action types
        action_map = {
            "pauseMovement": "startPause",
            "resumeMovement": "stopPause",
            "pause": "startPause",
            "resume": "stopPause",
            "beep": "beep",
            "locate": "locate",
            "dock": "dock"
        }

        actual_action = action_map.get(action_type, action_type)

        if actual_action == "startPause":
            self.pause_mission()
        elif actual_action == "stopPause":
            self.resume_mission()
        elif actual_action == "dock":
            self._send_dock_command()
        else:
            # Send action to bridge
            action_msg = {
                "headerId": self._next_header_id(),
                "timestamp": self._get_timestamp(),
                "version": VDA_VERSION,
                "manufacturer": MANUFACTURER,
                "serialNumber": SERIAL_NUMBER,
                "actions": [{
                    "actionType": actual_action,
                    "actionId": f"{actual_action}_{int(time.time() * 1000)}",
                    "blockingType": "NONE",
                    "actionParameters": []
                }]
            }
            self.client.publish(TOPIC_BRIDGE_ACTION, json.dumps(action_msg))
            self.log(f"📤 Instant action: {actual_action}", "info")

    # ========================================================================
    # GUI MISSION SUPPORT
    # ========================================================================

    def set_mission_sequence(self, path: List[str]):
        """Set mission from GUI"""
        with self.lock:
            if not path or len(path) < 2:
                self.log("✗ Invalid path", "error")
                return

            if self.mission_active:
                self.log("⏹ Cancelling current mission", "warning")
                self.mission_active = False
                self.mission_queue = []
                self.current_target_node = None

            self.log(f"🎯 GUI Mission: {' → '.join(path)}", "info")

            self._reset_gui_colors()

            # Use simple readable order ID for GUI missions
            self.order_id = "GUI_Order"
            self.order_update_id = 0
            self.current_order = {"orderId": self.order_id, "orderUpdateId": 0}

            self.node_states = []
            self.edge_states = []
            self.action_states = []
            self.traversed_node_indices = 0

            for i, node_id in enumerate(path):
                if node_id in NODES:
                    ns = NodeState(
                        nodeId=node_id,
                        sequenceId=i * 2,
                        released=True,
                        nodePosition={"x": NODES[node_id]["x"], "y": NODES[node_id]["y"], "mapId": MAP_ID}
                    )
                    self.node_states.append(ns)

            for i in range(len(path) - 1):
                es = EdgeState(
                    edgeId=f"e_{path[i]}_{path[i + 1]}",
                    sequenceId=i * 2 + 1,
                    released=True,
                    startNodeId=path[i],
                    endNodeId=path[i + 1]
                )
                self.edge_states.append(es)

            if path[0] == self.last_node_id:
                path = path[1:]
                self.traversed_node_indices = 1

            if not path:
                self.log("Already at destination", "info")
                return

            self.mission_queue = path.copy()
            for node_id in path:
                if node_id in self.gui_node_states:
                    self.gui_node_states[node_id] = "planned"

            self.mission_active = True
            self.paused = False
            self.waiting_at_decision_point = False

            self._advance_mission()

    def set_mission_with_horizon(self, path: List[str], base_count: int):
        """
        Set mission with base/horizon split.

        Args:
            path: Full path of node IDs
            base_count: Number of nodes to release (base), rest stay in horizon
        """
        with self.lock:
            if not path or len(path) < 2:
                self.log("✗ Invalid path", "error")
                return

            if self.mission_active:
                self.log("⏹ Cancelling current mission", "warning")
                self.mission_active = False
                self.mission_queue = []
                self.current_target_node = None

            # Clamp base_count
            base_count = max(1, min(base_count, len(path)))
            horizon_count = len(path) - base_count

            if horizon_count > 0:
                self.log(f"🎯 Mission with Horizon:", "info")
                self.log(f"   Base ({base_count}): {' → '.join(path[:base_count])}", "info")
                self.log(f"   Horizon ({horizon_count}): {' → '.join(path[base_count:])}", "warning")
                order_name = "GUI_Horizon_Order"
            else:
                self.log(f"🎯 GUI Mission: {' → '.join(path)}", "info")
                order_name = "GUI_Order"

            self._reset_gui_colors()

            # Use simple readable order ID for GUI missions
            self.order_id = order_name
            self.order_update_id = 0
            self.current_order = {"orderId": self.order_id, "orderUpdateId": 0}

            self.node_states = []
            self.edge_states = []
            self.action_states = []
            self.traversed_node_indices = 0

            # Create node states with released flag based on base_count
            for i, node_id in enumerate(path):
                if node_id in NODES:
                    is_released = (i < base_count)
                    ns = NodeState(
                        nodeId=node_id,
                        sequenceId=i * 2,
                        released=is_released,
                        nodePosition={"x": NODES[node_id]["x"], "y": NODES[node_id]["y"], "mapId": MAP_ID}
                    )
                    self.node_states.append(ns)

            # Create edge states - edge is released if BOTH its nodes are released
            for i in range(len(path) - 1):
                is_released = (i < base_count - 1)  # Edge i connects node i to node i+1
                es = EdgeState(
                    edgeId=f"e_{path[i]}_{path[i + 1]}",
                    sequenceId=i * 2 + 1,
                    released=is_released,
                    startNodeId=path[i],
                    endNodeId=path[i + 1]
                )
                self.edge_states.append(es)

            # Only queue the BASE nodes for navigation (not horizon)
            base_path = path[:base_count]

            if base_path[0] == self.last_node_id:
                base_path = base_path[1:]
                self.traversed_node_indices = 1

            if not base_path:
                if horizon_count > 0:
                    self.log("⏸ At decision point - waiting for horizon release", "warning")
                    self.waiting_at_decision_point = True
                    self.new_base_request = True
                else:
                    self.log("Already at destination", "info")
                return

            self.mission_queue = base_path.copy()

            # Color nodes appropriately
            for node_id in path[:base_count]:
                if node_id in self.gui_node_states:
                    self.gui_node_states[node_id] = "planned"

            self.mission_active = True
            self.paused = False
            self.waiting_at_decision_point = False

            self._advance_mission()

    def release_horizon_node(self, node_id: str = None):
        """
        Release the next horizon node(s) to base.

        If node_id is specified, releases up to and including that node.
        If node_id is None, releases just the next horizon node.
        """
        with self.lock:
            if not self.node_states:
                self.log("No active order", "warning")
                return

            # Find horizon nodes
            horizon_nodes = [(i, ns) for i, ns in enumerate(self.node_states) if not ns.released]

            if not horizon_nodes:
                self.log("No horizon nodes to release", "info")
                return

            # Release the first horizon node
            idx, first_horizon = horizon_nodes[0]
            first_horizon.released = True
            self.log(f"✓ Released: {first_horizon.nodeId}", "success")

            # Also release the edge leading to this node
            for es in self.edge_states:
                if es.endNodeId == first_horizon.nodeId and not es.released:
                    es.released = True
                    self.log(f"   Released edge: {es.edgeId}", "info")
                    break

            # Increment order update ID (VDA 5050 requirement)
            self.order_update_id += 1
            if self.current_order:
                self.current_order["orderUpdateId"] = self.order_update_id

            # Add released node to mission queue
            self.mission_queue.append(first_horizon.nodeId)
            if first_horizon.nodeId in self.gui_node_states:
                self.gui_node_states[first_horizon.nodeId] = "planned"

            # If we were waiting at decision point, continue
            if self.waiting_at_decision_point:
                self.waiting_at_decision_point = False
                self.new_base_request = False
                self.log("▶ Continuing mission after horizon release", "success")
                self._advance_mission()

    # ========================================================================
    # UTILITY
    # ========================================================================

    def _next_header_id(self) -> int:
        self._header_id += 1
        return self._header_id

    def _get_timestamp(self) -> str:
        return time.strftime('%Y-%m-%dT%H:%M:%S.000Z', time.gmtime())

    # ========================================================================
    # ERROR MANAGEMENT
    # ========================================================================

    def add_error(self, error_type: str, error_level: str = "WARNING",
                  error_description: str = "", error_references: List[Dict] = None):
        """
        Add an error to the error list.

        Args:
            error_type: Type of error (e.g., "orderError", "navigationError")
            error_level: "WARNING" (self-resolving) or "FATAL" (needs intervention)
            error_description: Human-readable description
            error_references: List of references to help find cause
        """
        error = {
            "errorType": error_type,
            "errorLevel": error_level,
            "errorDescription": error_description,
            "errorReferences": error_references or []
        }
        self.errors.append(error)
        self.log(f"⚠ Error added: {error_type} ({error_level})", "error")

    def clear_error(self, error_type: str):
        """Remove an error by type"""
        self.errors = [e for e in self.errors if e.get("errorType") != error_type]

    def clear_all_errors(self):
        """Clear all errors"""
        self.errors = []

    def get_gui_state(self) -> Dict:
        return {
            "pose": self.display_pose,
            "battery": self.battery,
            "charging": self.charging,
            "mission_active": self.mission_active,
            "current_target": self.current_target_node,
            "gui_node_states": self.gui_node_states,
            "order_id": self.order_id,
            "waiting_at_decision_point": self.waiting_at_decision_point,
        }

    @property
    def current_order_id(self):
        """Alias for order_id for GUI compatibility"""
        return self.order_id

    def stop(self):
        """Stop backend gracefully"""
        self.running = False
        self.mission_active = False

        # Publish OFFLINE before disconnecting
        if self.client and self._connected:
            self._publish_connection_state("OFFLINE", log=True)
            time.sleep(0.2)  # Allow message to be sent

        if self.client:
            self.client.loop_stop()
            self.client.disconnect()
        self.log("✓ Backend stopped", "info")


# Alias for backward compatibility with GUI
VDA5050Backend = RobotBackend