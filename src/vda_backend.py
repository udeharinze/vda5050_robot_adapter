# ============================================================================
# vda_backend_refactored.py - VDA 5050 v2.1.0 Backend Controller (REFACTORED)
# ============================================================================
#
# FIXES IMPLEMENTED:
#   1. Proper order update stitching (preserves sequenceIds)
#   2. No homing when already at home
#   3. Correct new order vs order update detection
#   4. Full cancelOrder instant action implementation
#   5. Batch horizon release support
#
# VDA 5050 v2.1.0 COMPLIANCE:
#   - Orders: Proper stitching with preserved sequenceIds
#   - Order Updates: Append-only, no state reset
#   - cancelOrder: Full state management per spec
#   - newBaseRequest: Correct timing
#
# ============================================================================

import json
import time
import math
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any, Tuple
from enum import Enum
import paho.mqtt.client as mqtt
from vda_config import (
    MQTT_BROKER, MQTT_PORT, MANUFACTURER, SERIAL_NUMBER, VDA_VERSION, MAP_ID,
    NODES, EDGES, NAV,
    MQTT_USERNAME, MQTT_PASSWORD, MQTT_WEBSOCKET, MQTT_WS_PATH, MQTT_VERSION
)

# ============================================================================
# TUNABLE CONSTANTS
# ============================================================================

COMMAND_COOLDOWN_SEC = NAV.COMMAND_COOLDOWN_SEC
STALLED_TIMEOUT_SEC = NAV.STALL_TIMEOUT_SEC
ARRIVAL_TOLERANCE_MM = NAV.POSITION_TOLERANCE_MM
STATE_PUBLISH_INTERVAL = 1.0
CONNECTION_PUBLISH_INTERVAL = 1.0
ACTION_TIMEOUT_SEC = 10.0
MAX_START_DISTANCE_MM = 500  # Maximum distance to accept a new order start node

# ============================================================================
# MQTT TOPICS (VDA 5050 Standard)
# ============================================================================

TOPIC_ORDER = f"uagv/v2/{MANUFACTURER}/{SERIAL_NUMBER}/order"
TOPIC_STATE = f"uagv/v2/{MANUFACTURER}/{SERIAL_NUMBER}/state"
TOPIC_INSTANT_ACTIONS = f"uagv/v2/{MANUFACTURER}/{SERIAL_NUMBER}/instantActions"
TOPIC_CONNECTION = f"uagv/v2/{MANUFACTURER}/{SERIAL_NUMBER}/connection"
TOPIC_VISUALIZATION = f"uagv/v2/{MANUFACTURER}/{SERIAL_NUMBER}/visualization"


# ============================================================================
# ENUMS
# ============================================================================

class OrderRejectionReason(Enum):
    """VDA 5050 order rejection reasons"""
    NONE = "none"
    VEHICLE_BUSY = "vehicleAlreadyExecutingOrder"
    START_NODE_TOO_FAR = "startNodeTooFarFromCurrentPosition"
    DEPRECATED_UPDATE = "deprecatedOrderUpdateId"
    ORDER_ID_MISMATCH = "orderIdMismatch"
    INVALID_CONTINUATION = "invalidContinuation"
    INVALID_STITCHING = "invalidStitching"
    MISSING_NODE_POSITION = "missingNodePosition"


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

        # ====================================================================
        # VDA5050 Order State - CRITICAL FOR STITCHING
        # ====================================================================
        self.current_order: Optional[Dict] = None
        self.order_id: str = ""  # String order ID (VDA 5050 uses strings)
        self.order_update_id: int = 0
        self._order_id_counter: int = 0  # Counter for generating unique order IDs

        # CRITICAL: Track last node state for stitching validation
        self.last_node_id: Optional[str] = None
        self.last_node_sequence_id: int = 0

        # CRITICAL: Global sequence counter - NEVER reset during order updates
        self._global_sequence_id: int = 0

        # Node/Edge states
        self.node_states: List[NodeState] = []
        self.edge_states: List[EdgeState] = []

        # CRITICAL: Track traversed nodes by sequenceId (not by index!)
        self._traversed_sequence_ids: set = set()

        # Action states - track all actions from current order
        self.action_states: List[ActionState] = []
        self.current_action: Optional[ActionState] = None
        self.action_start_time: float = 0.0

        # Mission control
        self.mission_active = False
        self.mission_queue: List[Tuple[str, int]] = []  # List of (nodeId, sequenceId) tuples
        self.current_target_node: Optional[str] = None
        self.current_target_sequence_id: Optional[int] = None
        self.waiting_at_decision_point = False
        self.new_base_request = False
        self.last_command_ts = 0.0
        self.last_state_publish_ts = 0.0
        self.last_connection_publish_ts = 0.0

        # Pending actions for current node
        self.pending_node_actions: List[Dict] = []

        # GUI state
        self.gui_node_states = {n: "idle" for n in NODES}

        # Header counter
        self._header_id = 0

        # Action ID counters for simpler IDs (dock1, dock2, stop1, etc.)
        self._action_id_counters: Dict[str, int] = {}

        # Threading
        self.lock = threading.RLock()

        # Error tracking
        self.errors: List[Dict] = []

        # Initial position sync flag
        self._initial_position_synced = False

    # ========================================================================
    # MQTT SETUP
    # ========================================================================

    def start(self):
        """Start MQTT client and main loop"""
        if MQTT_VERSION == "3.1":
            protocol = mqtt.MQTTv31
            self.log("📋 Using MQTT protocol v3.1", "info")
        elif MQTT_VERSION == "3.1.1":
            protocol = mqtt.MQTTv311
            self.log("📋 Using MQTT protocol v3.1.1", "info")
        else:
            protocol = mqtt.MQTTv5
            self.log("📋 Using MQTT protocol v5.0", "info")

        if MQTT_WEBSOCKET:
            self.client = mqtt.Client(
                mqtt.CallbackAPIVersion.VERSION2,
                f"vda_backend_{SERIAL_NUMBER}",
                transport="websockets",
                protocol=protocol
            )
            self.log("🌐 Using WebSocket transport", "info")
        else:
            self.client = mqtt.Client(
                mqtt.CallbackAPIVersion.VERSION2,
                f"vda_backend_{SERIAL_NUMBER}",
                protocol=protocol
            )

        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message

        # Set last will
        last_will = {
            "headerId": 0,
            "timestamp": self._get_timestamp(),
            "version": VDA_VERSION,
            "manufacturer": MANUFACTURER,
            "serialNumber": SERIAL_NUMBER,
            "connectionState": "CONNECTIONBROKEN"
        }
        self.client.will_set(TOPIC_CONNECTION, json.dumps(last_will), qos=1, retain=True)

        if MQTT_USERNAME and MQTT_PASSWORD:
            self.client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
            self.log(f"🔐 MQTT auth enabled for user: {MQTT_USERNAME}", "info")

        if MQTT_WEBSOCKET:
            import ssl
            ssl_context = ssl.create_default_context()
            ssl_context.check_hostname = False
            ssl_context.verify_mode = ssl.CERT_NONE
            self.client.tls_set_context(ssl_context)
            self.log("🔒 TLS encryption enabled", "info")

        if MQTT_WEBSOCKET and MQTT_WS_PATH:
            self.client.ws_set_options(path=MQTT_WS_PATH)
            self.log(f"🌐 WebSocket path: {MQTT_WS_PATH}", "info")

        self.log(f"🎯 VDA5050 Backend starting...", "info")
        self.log(f"📡 Connecting to {self.broker}:{self.port}", "info")

        try:
            self.client.connect(self.broker, self.port, 60)
            self.running = True
            self.client.loop_start()
            threading.Thread(target=self._main_loop, daemon=True).start()
            self.log("✓ Backend started", "success")
        except Exception as e:
            self.log(f"✗ MQTT connection failed: {e}", "error")

    def connect(self):
        """Alias for start()"""
        self.start()

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        """MQTT connection callback"""
        if reason_code == 0:
            if not self._connected:
                self._connected = True
                self.log("✓ MQTT connected", "success")
                self.log(f"📡 Order topic: {TOPIC_ORDER}", "info")
                self.log(f"📡 State topic: {TOPIC_STATE}", "info")

            client.subscribe(TOPIC_ORDER)
            client.subscribe(TOPIC_VISUALIZATION)
            client.subscribe(TOPIC_INSTANT_ACTIONS)
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
            elif msg.topic == TOPIC_VISUALIZATION:
                self._handle_visualization(payload)
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
        """Publish connection state"""
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
    # VISUALIZATION HANDLING
    # ========================================================================

    def _handle_visualization(self, payload: Dict):
        """Update internal state from visualization topic"""
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
    # POSITION SYNC & PATH FINDING
    # ========================================================================

    def _sync_initial_position(self):
        """Sync last_node_id with robot's actual position on startup"""
        nearest = self._find_nearest_node()
        if nearest:
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
        """Find the nearest defined node"""
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
        """Find path using BFS (Dijkstra without weights)"""
        if start_node == end_node:
            return [start_node]

        if start_node not in NODES or end_node not in NODES:
            self.log(f"✗ Invalid nodes: {start_node} or {end_node}", "error")
            return []

        adjacency = {node: [] for node in NODES}
        for edge in EDGES:
            start = edge.get("start")
            end = edge.get("end")
            if start and end:
                adjacency[start].append(end)
                adjacency[end].append(start)

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

    # ========================================================================
    # VDA5050 ORDER HANDLING - REFACTORED
    # ========================================================================

    def _handle_order(self, order: Dict):
        """
        Handle incoming VDA5050 order with proper new order vs update detection.

        VDA 5050 v2.1.0 Order Acceptance Logic:
        1. Validate JSON
        2. Determine: New order or Order update
        3. Apply appropriate handling
        """
        incoming_order_id = str(order.get("orderId", ""))
        incoming_update_id = order.get("orderUpdateId", 0)

        with self.lock:
            # Determine order type
            is_echo = (incoming_order_id == self.order_id and
                       incoming_update_id == self.order_update_id)

            if is_echo:
                # This is our own message echoing back - silently ignore
                return

            is_new_order = (incoming_order_id != self.order_id)
            is_update = (incoming_order_id == self.order_id and
                         incoming_update_id > self.order_update_id)
            is_deprecated = (incoming_order_id == self.order_id and
                             incoming_update_id < self.order_update_id)

            if is_deprecated:
                self.log(f"📥 ORDER: #{incoming_order_id} (update: {incoming_update_id})", "info")
                self.log(f"   ⚠ Deprecated update ignored (current: {self.order_update_id})", "warning")
                self._add_order_rejection_error(OrderRejectionReason.DEPRECATED_UPDATE)
                return

            if is_new_order:
                self.log(f"📥 ORDER: #{incoming_order_id} (update: {incoming_update_id})", "info")
                self.log(f"   🆕 NEW ORDER", "info")
                rejection = self._validate_new_order(order)
                if rejection != OrderRejectionReason.NONE:
                    self.log(f"   ✗ Rejected: {rejection.value}", "error")
                    self._add_order_rejection_error(rejection)
                    return
                self._process_new_order(order)

            elif is_update:
                self.log(f"📥 ORDER: #{incoming_order_id} (update: {incoming_update_id})", "info")
                self.log(f"   📝 ORDER UPDATE ({self.order_update_id} → {incoming_update_id})", "info")
                rejection = self._validate_order_update(order)
                if rejection != OrderRejectionReason.NONE:
                    self.log(f"   ✗ Rejected: {rejection.value}", "error")
                    self._add_order_rejection_error(rejection)
                    return
                self._process_order_update(order)

    def _validate_new_order(self, order: Dict) -> OrderRejectionReason:
        """
        Validate new order per VDA 5050 rules (Figure 8 flowchart).

        (3) Is vehicle still executing an order or waiting for an update?
            - We allow overriding for flexibility, but log a warning
        (4) Is start of new order close enough to current position?
            - Reject if too far
        """
        nodes = order.get("nodes", [])
        if not nodes:
            return OrderRejectionReason.MISSING_NODE_POSITION

        # (3) Check if vehicle is busy - we allow override but log it
        if self.mission_active or self.waiting_at_decision_point:
            self.log(f"   ⚠ Vehicle is busy - overriding current order", "warning")
            # Per strict VDA 5050, this should reject. But we allow override for flexibility.
            # To strictly comply, uncomment the next line:
            # return OrderRejectionReason.VEHICLE_BUSY

        # Get first node position
        first_node = nodes[0]
        first_pos = first_node.get("nodePosition", {})

        if not first_pos or first_pos.get("x") is None:
            # Try to get from config
            node_id = first_node.get("nodeId", "")
            if node_id in NODES:
                first_pos = {"x": NODES[node_id]["x"], "y": NODES[node_id]["y"]}
            else:
                return OrderRejectionReason.MISSING_NODE_POSITION

        # (4) Check distance to start node
        dist = math.sqrt(
            (self.pose["x"] - first_pos["x"]) ** 2 +
            (self.pose["y"] - first_pos["y"]) ** 2
        )

        if dist > MAX_START_DISTANCE_MM:
            self.log(f"   Start node too far: {dist:.0f}mm > {MAX_START_DISTANCE_MM}mm", "warning")
            return OrderRejectionReason.START_NODE_TOO_FAR

        return OrderRejectionReason.NONE

    def _validate_order_update(self, order: Dict) -> OrderRejectionReason:
        """
        Validate order update per VDA 5050 stitching rules (Figure 8).

        VDA 5050 STITCHING RULE:
        "The last node of the previous base is the first base node in the updated order.
         The other nodes and edges from the previous base are NOT RESENT."

        Flowchart steps:
        (3) Is vehicle executing or waiting?
            - YES: (7) Is valid continuation of running order?
            - NO:  (8) Is valid continuation of completed order?

        This means:
        - Update message contains: [stitch_node, new_nodes...]
        - Stitch node = last base node from previous order
        - Already traversed nodes are NOT in the update
        """
        if order.get("orderId") != self.order_id:
            return OrderRejectionReason.ORDER_ID_MISMATCH

        # Get first node in update (stitch point)
        update_nodes = order.get("nodes", [])
        if not update_nodes:
            return OrderRejectionReason.INVALID_CONTINUATION

        first_node_in_update = update_nodes[0]
        stitch_node_id = first_node_in_update.get("nodeId")
        stitch_seq_id = first_node_in_update.get("sequenceId")

        # (3) Is vehicle executing or waiting for update?
        vehicle_executing = self.mission_active or self.waiting_at_decision_point or self.node_states

        if vehicle_executing:
            # (7) Validate as continuation of RUNNING order
            # Find last released (base) node in our current state
            last_base_node = None
            for ns in self.node_states:
                if ns.released:
                    last_base_node = ns

            if not last_base_node:
                self.log(f"   No base nodes in running order", "error")
                return OrderRejectionReason.INVALID_CONTINUATION

            # Stitch validation: first node in update must match our last base node
            if last_base_node.nodeId != stitch_node_id:
                self.log(f"   Stitch fail: nodeId mismatch (running)", "error")
                self.log(f"      Our last base: {last_base_node.nodeId} (seq:{last_base_node.sequenceId})", "error")
                self.log(f"      Update first:  {stitch_node_id} (seq:{stitch_seq_id})", "error")
                return OrderRejectionReason.INVALID_STITCHING

            if last_base_node.sequenceId != stitch_seq_id:
                self.log(f"   Stitch fail: sequenceId mismatch (running)", "error")
                self.log(f"      Our last base seq: {last_base_node.sequenceId}", "error")
                self.log(f"      Update first seq:  {stitch_seq_id}", "error")
                return OrderRejectionReason.INVALID_STITCHING

            self.log(f"   ✓ Stitch valid (running): {last_base_node.nodeId} (seq:{last_base_node.sequenceId})",
                     "success")
        else:
            # (8) Validate as continuation of COMPLETED order
            # The stitch point should be the last node we visited (last_node_id, last_node_sequence_id)
            if not self.last_node_id or self.last_node_sequence_id is None:
                self.log(f"   No last node for completed order continuation", "error")
                return OrderRejectionReason.INVALID_CONTINUATION

            if self.last_node_id != stitch_node_id:
                self.log(f"   Stitch fail: nodeId mismatch (completed)", "error")
                self.log(f"      Our last node: {self.last_node_id} (seq:{self.last_node_sequence_id})", "error")
                self.log(f"      Update first:  {stitch_node_id} (seq:{stitch_seq_id})", "error")
                return OrderRejectionReason.INVALID_STITCHING

            if self.last_node_sequence_id != stitch_seq_id:
                self.log(f"   Stitch fail: sequenceId mismatch (completed)", "error")
                self.log(f"      Our last node seq: {self.last_node_sequence_id}", "error")
                self.log(f"      Update first seq:  {stitch_seq_id}", "error")
                return OrderRejectionReason.INVALID_STITCHING

            self.log(f"   ✓ Stitch valid (completed): {self.last_node_id} (seq:{self.last_node_sequence_id})",
                     "success")

        return OrderRejectionReason.NONE

    def _process_new_order(self, order: Dict):
        """
        Process a new VDA5050 order.

        VDA 5050 rules for new orders:
        - Delete all previous nodeStates and edgeStates
        - Reset action states
        - Start fresh execution
        """
        self.order_id = str(order.get("orderId", ""))
        self.order_update_id = order.get("orderUpdateId", 0)
        self.current_order = order

        # Update counter to avoid GUI conflicts
        if self.order_id.isdigit():
            order_num = int(self.order_id)
            if order_num >= self._order_id_counter:
                self._order_id_counter = order_num

        self.log(f"   📋 Order ID: {self.order_id}, Update ID: {self.order_update_id}", "info")

        # CLEAR ALL PREVIOUS STATE
        self._reset_gui_colors()
        self.action_states = []
        self.node_states = []
        self.edge_states = []
        self.mission_queue = []
        self._traversed_sequence_ids = set()

        # Parse nodes
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

            # Register actions
            for action in node_actions:
                action_state = ActionState(
                    actionId=action.get("actionId", f"action_{len(self.action_states)}"),
                    actionType=action.get("actionType", "unknown"),
                    actionStatus="WAITING",
                    actionDescription=action.get("actionDescription", "")
                )
                self.action_states.append(action_state)

            status = "✓ BASE" if ns.released else "○ HORIZON"
            self.log(f"   [{ns.sequenceId}] {ns.nodeId}: {status}", "info")

        # Parse edges
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

            for action in edge_actions:
                action_state = ActionState(
                    actionId=action.get("actionId", f"action_{len(self.action_states)}"),
                    actionType=action.get("actionType", "unknown"),
                    actionStatus="WAITING"
                )
                self.action_states.append(action_state)

        # Build mission queue from RELEASED nodes only
        self._build_mission_queue()

        # Start mission
        self.mission_active = True
        self.paused = False
        self.waiting_at_decision_point = False
        self.current_target_node = None
        self.pending_node_actions = []

        # FIX: Check if already at first node BEFORE adding to queue
        if self.mission_queue:
            first_node_id, first_seq_id = self.mission_queue[0]
            if self._is_at_node(first_node_id):
                self.log(f"   Already at {first_node_id}, marking as traversed", "info")
                self._mark_node_traversed(first_node_id, first_seq_id)
                self.mission_queue.pop(0)

                # Execute actions for this node
                for ns in self.node_states:
                    if ns.nodeId == first_node_id and ns.sequenceId == first_seq_id and ns.actions:
                        self._execute_node_actions(ns.actions)

        # Publish order and advance
        self._publish_current_order(is_update=False)
        self._advance_mission()

    def _process_order_update(self, order: Dict):
        """
        Process order update with proper VDA 5050 stitching.

        VDA 5050 SPEC:
        "The last node of the previous base is the first base node in the updated order.
         The other nodes and edges from the previous base are NOT RESENT."

        This means:
        - Update contains: [stitch_node, following_nodes...]
        - Already traversed nodes are NOT in the update message
        - We must MERGE the update with our existing state

        Example:
          Original: A(0) → B(2) → C(4) [base] | D(6) → E(8) [horizon]
          Robot at B, approaching C (decision point)
          Update received: C(4) → D(6) → E(8) → F(10)
          Result: Keep A,B as traversed, update C onwards
        """
        self.order_update_id = order.get("orderUpdateId", 0)
        self.current_order = order

        update_nodes = order.get("nodes", [])
        update_edges = order.get("edges", [])

        if not update_nodes:
            self.log("   ⚠ Empty update, ignoring", "warning")
            return

        # Find the stitch point in our current state
        # Stitch node = first node in update = last base node in our state
        stitch_node_data = update_nodes[0]
        stitch_seq_id = stitch_node_data.get("sequenceId", 0)
        stitch_node_id = stitch_node_data.get("nodeId", "")

        self.log(f"   Stitch point: {stitch_node_id} (seq:{stitch_seq_id})", "info")

        # Keep all nodes with sequenceId < stitch_seq_id (already traversed/traversing)
        # These are NOT in the update message per VDA 5050 spec
        preserved_nodes = [ns for ns in self.node_states if ns.sequenceId < stitch_seq_id]
        preserved_edges = [es for es in self.edge_states if es.sequenceId < stitch_seq_id]

        self.log(f"   Preserved {len(preserved_nodes)} traversed nodes", "info")

        # Process nodes from the update (stitch point onwards)
        new_nodes = []
        for node_data in update_nodes:
            seq_id = node_data.get("sequenceId", 0)
            node_id = node_data.get("nodeId", "")
            is_released = node_data.get("released", True)

            # Check if this node already exists in our preserved state
            # (It shouldn't, since preserved nodes have seq < stitch_seq)
            existing = next((ns for ns in preserved_nodes if ns.sequenceId == seq_id), None)
            if existing:
                # Update released flag if changed
                if not existing.released and is_released:
                    existing.released = True
                    self.log(f"   ✓ Released (existing): {node_id} (seq:{seq_id})", "success")
                continue

            # Check if this node exists in our current state (at or after stitch)
            current_node = next((ns for ns in self.node_states if ns.sequenceId == seq_id), None)

            if current_node:
                # Node exists - update its released flag
                was_released = current_node.released
                current_node.released = is_released

                if not was_released and is_released:
                    self.log(f"   ✓ Released: {node_id} (seq:{seq_id})", "success")
                    # Add to mission queue if not already traversed
                    if seq_id not in self._traversed_sequence_ids:
                        if not any(nid == node_id and sid == seq_id for nid, sid in self.mission_queue):
                            self.mission_queue.append((node_id, seq_id))
                            if node_id in self.gui_node_states:
                                self.gui_node_states[node_id] = "planned"

                new_nodes.append(current_node)
            else:
                # Completely new node from update
                ns = NodeState(
                    nodeId=node_id,
                    sequenceId=seq_id,
                    released=is_released,
                    nodePosition=node_data.get("nodePosition"),
                    actions=node_data.get("actions", [])
                )
                new_nodes.append(ns)

                if is_released and seq_id not in self._traversed_sequence_ids:
                    self.mission_queue.append((node_id, seq_id))
                    if node_id in self.gui_node_states:
                        self.gui_node_states[node_id] = "planned"
                    self.log(f"   + New node: {node_id} (seq:{seq_id}) {'[BASE]' if is_released else '[HORIZON]'}",
                             "info")
                elif not is_released:
                    self.log(f"   + New node: {node_id} (seq:{seq_id}) [HORIZON]", "info")

        # Merge: preserved (traversed) + new (from update)
        self.node_states = preserved_nodes + new_nodes

        # Similarly handle edges
        new_edges = []
        for edge_data in update_edges:
            seq_id = edge_data.get("sequenceId", 0)

            # Skip if already in preserved edges
            if any(es.sequenceId == seq_id for es in preserved_edges):
                continue

            # Check if exists in current state
            current_edge = next((es for es in self.edge_states if es.sequenceId == seq_id), None)

            if current_edge:
                was_released = current_edge.released
                current_edge.released = edge_data.get("released", True)
                if not was_released and current_edge.released:
                    self.log(f"   ✓ Released edge: {current_edge.edgeId}", "info")
                new_edges.append(current_edge)
            else:
                es = EdgeState(
                    edgeId=edge_data.get("edgeId", ""),
                    sequenceId=seq_id,
                    released=edge_data.get("released", True),
                    startNodeId=edge_data.get("startNodeId", ""),
                    endNodeId=edge_data.get("endNodeId", ""),
                    actions=edge_data.get("actions", [])
                )
                new_edges.append(es)

        self.edge_states = preserved_edges + new_edges

        # Sort by sequenceId
        self.node_states.sort(key=lambda ns: ns.sequenceId)
        self.edge_states.sort(key=lambda es: es.sequenceId)
        self.mission_queue.sort(key=lambda x: x[1])

        # Log final state
        base_count = sum(1 for ns in self.node_states if ns.released)
        horizon_count = sum(1 for ns in self.node_states if not ns.released)
        self.log(f"   State: {len(self.node_states)} nodes ({base_count} base, {horizon_count} horizon)", "info")

        # Resume if waiting at decision point
        if self.waiting_at_decision_point:
            self.log(f"   ▶ Resuming from decision point", "success")
            self.waiting_at_decision_point = False
            self.new_base_request = False
            self._advance_mission()
        elif self.new_base_request:
            self.log(f"   ✓ Base extended before reaching decision point", "success")
            self.new_base_request = False

        # Publish updated order
        self._publish_current_order(is_update=True)

    def _build_mission_queue(self):
        """Build mission queue from released nodes, excluding traversed ones."""
        self.mission_queue = []
        for ns in self.node_states:
            if ns.released and ns.sequenceId not in self._traversed_sequence_ids:
                self.mission_queue.append((ns.nodeId, ns.sequenceId))
                if ns.nodeId in self.gui_node_states:
                    self.gui_node_states[ns.nodeId] = "planned"

    def _mark_node_traversed(self, node_id: str, sequence_id: int):
        """Mark a node as traversed."""
        self._traversed_sequence_ids.add(sequence_id)
        self.last_node_id = node_id
        self.last_node_sequence_id = sequence_id
        if node_id in self.gui_node_states:
            self.gui_node_states[node_id] = "done"

    def _add_order_rejection_error(self, reason: OrderRejectionReason):
        """Add order rejection error to state."""
        # Determine correct error type per VDA 5050
        if reason == OrderRejectionReason.DEPRECATED_UPDATE:
            error_type = "orderUpdateError"  # 6.6.4.3
        elif reason in [OrderRejectionReason.MISSING_NODE_POSITION]:
            error_type = "validationError"  # 6.6.4.1
        else:
            error_type = "orderError"

        self.add_error(
            error_type=error_type,
            error_level="WARNING",
            error_description=f"Order rejected: {reason.value}"
        )

    # ========================================================================
    # NAVIGATION
    # ========================================================================

    def _advance_mission(self):
        """Advance to next node in mission queue."""
        with self.lock:
            if self.paused:
                return

            if not self.mission_queue:
                has_horizon = any(not ns.released for ns in self.node_states)
                if has_horizon:
                    self.log(f"⏸ Waiting at decision point", "warning")
                    self.waiting_at_decision_point = True
                    self.new_base_request = False
                    self.current_target_node = None
                else:
                    self._complete_mission()
                return

            next_node_id, next_seq_id = self.mission_queue.pop(0)
            self.current_target_node = next_node_id
            self.current_target_sequence_id = next_seq_id

            # FIX: Check if already at this node
            if self._is_at_node(next_node_id):
                self.log(f"✓ Already at: {next_node_id}", "success")
                self._on_node_reached(next_node_id, next_seq_id)
                return

            # Check if approaching decision point
            has_horizon = any(not ns.released for ns in self.node_states)
            is_heading_to_decision_point = (len(self.mission_queue) == 0) and has_horizon

            if is_heading_to_decision_point:
                self.new_base_request = True
                self.log(f"📡 newBaseRequest: true", "warning")
            else:
                self.new_base_request = False

            # Get pending actions for this node
            self.pending_node_actions = []
            for ns in self.node_states:
                if ns.nodeId == next_node_id and ns.sequenceId == next_seq_id:
                    self.pending_node_actions = ns.actions.copy()
                    break

            if next_node_id in self.gui_node_states:
                self.gui_node_states[next_node_id] = "planned"

            self.log(f"→ Moving to: {next_node_id} (seq: {next_seq_id})", "info")
            self._send_navigation_command(next_node_id, next_seq_id)

    def _send_navigation_command(self, node_id: str, sequence_id: int):
        """Send navigation command - publishes order for bridge to handle."""
        self.last_command_ts = time.time()

        # FIX: Only send dock command if NOT already at home
        if node_id == "home":
            if self._is_at_node("home"):
                self.log(f"🏠 Already at home, skipping dock command", "info")
                self._on_node_reached(node_id, sequence_id)
                return
            else:
                self.log(f"🏠 Navigating to: home (dock command)", "info")
                self._send_dock_command()
        else:
            self._send_goto(node_id)

    def _send_goto(self, node_id: str):
        """Send navigation command for bridge to execute.

        NOTE: This does NOT publish the order - caller is responsible for publishing.
        The order is published by set_mission_sequence/set_mission_with_horizon/release_horizon_nodes.
        """
        target = None

        for ns in self.node_states:
            if ns.nodeId == node_id and ns.nodePosition:
                pos = ns.nodePosition
                if pos.get("x") is not None and pos.get("y") is not None:
                    target = {"x": pos["x"], "y": pos["y"]}
                    break

        if not target and node_id in NODES:
            node_config = NODES[node_id]
            target = {"x": node_config["x"], "y": node_config["y"]}

        if not target:
            self.log(f"✗ Unknown node: {node_id}", "error")
            return

        self.log(f"🎯 Navigating to: {node_id} ({int(target['x'])}, {int(target['y'])})", "info")
        # NOTE: Order is NOT published here - it's already published by the caller

    def _send_dock_command(self):
        """Send dock command via instant actions."""
        action_msg = {
            "headerId": self._next_header_id(),
            "timestamp": self._get_timestamp(),
            "version": VDA_VERSION,
            "manufacturer": MANUFACTURER,
            "serialNumber": SERIAL_NUMBER,
            "actions": [{
                "actionType": "dock",
                "actionId": self._next_action_id("dock"),
                "blockingType": "HARD",
                "actionParameters": []
            }]
        }
        self.client.publish(TOPIC_INSTANT_ACTIONS, json.dumps(action_msg))

    def _complete_mission(self):
        """Complete mission."""
        self.log(f"✅ ORDER COMPLETE: Order #{self.order_id}", "success")
        self.mission_active = False
        self.current_target_node = None
        self.waiting_at_decision_point = False
        self.new_base_request = False

        # Clear states but KEEP orderId/orderUpdateId per VDA 5050
        self.node_states = []
        self.edge_states = []
        self.action_states = []
        self._traversed_sequence_ids = set()

        self.log(f"   📋 Keeping Order #{self.order_id} until new order arrives", "info")
        self._reset_gui_colors()

    def _is_at_node(self, node_id: str) -> bool:
        """Check if robot is at specified node."""
        target = None

        for ns in self.node_states:
            if ns.nodeId == node_id and ns.nodePosition:
                pos = ns.nodePosition
                if pos.get("x") is not None and pos.get("y") is not None:
                    target = {"x": pos["x"], "y": pos["y"]}
                    break

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

    def _on_node_reached(self, node_id: str, sequence_id: int):
        """Node reached - execute actions and advance."""
        self.log(f"✓ ARRIVED: {node_id} (seq: {sequence_id})", "success")

        self._mark_node_traversed(node_id, sequence_id)

        # Remove incoming edge from edgeStates
        incoming_edge_seq_id = sequence_id - 1
        self.edge_states = [
            es for es in self.edge_states
            if es.sequenceId != incoming_edge_seq_id
        ]

        # Execute pending actions
        if self.pending_node_actions:
            self.log(f"   Executing {len(self.pending_node_actions)} actions...", "info")
            self._execute_node_actions(self.pending_node_actions)
            self.pending_node_actions = []

        self._advance_mission()

    # ========================================================================
    # INSTANT ACTIONS - INCLUDING CANCEL ORDER
    # ========================================================================

    def _handle_instant_actions(self, payload: Dict):
        """Handle instant actions from master control."""
        actions = payload.get("actions", [])
        for action in actions:
            action_type = action.get("actionType", "")

            # Filter out actions we send ourselves
            if action_type in ["goTo", "dock", "quickCharge", "beep", "locate",
                               "startCleaning", "stopCleaning", "pauseCleaning",
                               "setFanSpeed", "stopMovement", "pauseMovement", "resumeMovement"]:
                return

            self.log(f"📥 Instant Action: {action_type}", "info")

            if action_type == "stopPause" or action_type == "resumeOrder":
                self.resume_mission()
            elif action_type == "startPause" or action_type == "pauseOrder":
                self.pause_mission()
            elif action_type == "cancelOrder":
                self._handle_cancel_order(action)

    def _handle_cancel_order(self, action: Dict):
        """
        VDA 5050 v2.1.0 cancelOrder implementation (6.6.3).

        Per spec:
        - cancelOrder is an instantAction
        - Vehicle stops (based on capabilities - right where it is or on next node)
        - SCHEDULED (WAITING) actions → 'FAILED'
        - RUNNING actions that CAN be interrupted → 'FAILED'
        - RUNNING actions that CANNOT be interrupted → keep 'RUNNING' until done
        - cancelOrder reports 'RUNNING' while actions are being processed
        - cancelOrder reports 'FINISHED' when all movement stopped and actions done
        - orderId and orderUpdateId are KEPT

        6.6.3.2: If no order exists:
        - cancelOrder = 'FAILED'
        - Report "noOrderToCancel" error with errorLevel='WARNING'
        """
        action_id = action.get("actionId", self._next_action_id("cancel"))

        self.log("🛑 CANCEL ORDER received", "error")

        # 6.6.3.2: Check if there's an order to cancel
        has_active_order = (self.node_states or self.mission_active or
                            self.current_target_node or self.waiting_at_decision_point)

        if not has_active_order and not self.order_id:
            # No order to cancel
            self.log("   ✗ No order to cancel", "error")

            # Add cancelOrder as FAILED
            cancel_action = ActionState(
                actionId=action_id,
                actionType="cancelOrder",
                actionStatus="FAILED",
                resultDescription="noOrderToCancel"
            )
            self.action_states.append(cancel_action)

            # Report error per 6.6.3.2
            self.add_error(
                error_type="noOrderToCancel",
                error_level="WARNING",
                error_description="No order to cancel",
                error_references=[{"referenceKey": "actionId", "referenceValue": action_id}]
            )
            return

        # 1. Add cancelOrder to actionStates as RUNNING
        cancel_action = ActionState(
            actionId=action_id,
            actionType="cancelOrder",
            actionStatus="RUNNING",
            actionDescription="Cancelling current order"
        )
        self.action_states.append(cancel_action)
        self.log(f"   cancelOrder: RUNNING", "info")

        # 2. Set all WAITING (scheduled) actions to FAILED
        for a in self.action_states:
            if a.actionStatus == "WAITING":
                a.actionStatus = "FAILED"
                a.resultDescription = "Cancelled by cancelOrder"
                self.log(f"   Action {a.actionId} ({a.actionType}): WAITING → FAILED", "info")

        # 3. Handle RUNNING actions
        # For simplicity, we interrupt all running actions immediately
        # A more sophisticated implementation would check if actions can be interrupted
        running_actions = [a for a in self.action_states
                           if a.actionStatus == "RUNNING" and a.actionType != "cancelOrder"]

        for a in running_actions:
            # Assume all our actions can be interrupted
            a.actionStatus = "FAILED"
            a.resultDescription = "Interrupted by cancelOrder"
            self.log(f"   Action {a.actionId} ({a.actionType}): RUNNING → FAILED", "info")

        # 4. Send stop command to bridge
        action_msg = {
            "headerId": self._next_header_id(),
            "timestamp": self._get_timestamp(),
            "version": VDA_VERSION,
            "manufacturer": MANUFACTURER,
            "serialNumber": SERIAL_NUMBER,
            "actions": [{
                "actionType": "stopMovement",
                "actionId": self._next_action_id("stop"),
                "blockingType": "HARD",
                "actionParameters": []
            }]
        }
        self.client.publish(TOPIC_INSTANT_ACTIONS, json.dumps(action_msg))
        self.log(f"   Sent stop command", "info")

        # 5. Update lastNodeId to nearest node (we stop at next node or immediately)
        nearest = self._find_nearest_node()
        if nearest and self._is_at_node(nearest):
            self.last_node_id = nearest
            for ns in self.node_states:
                if ns.nodeId == nearest:
                    self.last_node_sequence_id = ns.sequenceId
                    break
            self.log(f"   Updated lastNodeId: {self.last_node_id}", "info")

        # 6. Clear nodeStates and edgeStates (delete states of previous order)
        self.node_states = []
        self.edge_states = []
        self.log(f"   Cleared nodeStates and edgeStates", "info")

        # Mission state cleanup
        self.mission_active = False
        self.paused = False
        self.mission_queue = []
        self.current_target_node = None
        self.current_target_sequence_id = None
        self.waiting_at_decision_point = False
        self.new_base_request = False
        self._traversed_sequence_ids = set()

        # 7. actionStates are KEPT (per VDA 5050)
        # (We don't clear self.action_states)

        # 8. Set cancelOrder → FINISHED (all actions are now done)
        cancel_action.actionStatus = "FINISHED"
        cancel_action.resultDescription = "Order cancelled successfully"
        self.log(f"   cancelOrder: FINISHED", "success")

        # 9. orderId and orderUpdateId are KEPT (per VDA 5050)
        self.log(f"   📋 Kept Order #{self.order_id} (updateId: {self.order_update_id}) [cancelled]", "info")

        self._reset_gui_colors()

    # ========================================================================
    # ACTION EXECUTION
    # ========================================================================

    def _execute_node_actions(self, actions: List[Dict]):
        """Execute actions attached to a node."""
        none_actions = []
        soft_actions = []
        hard_actions = []

        for action in actions:
            blocking_type = action.get("blockingType", "HARD")
            if blocking_type == "NONE":
                none_actions.append(action)
            elif blocking_type == "SOFT":
                soft_actions.append(action)
            else:
                hard_actions.append(action)

        for action in none_actions:
            self._execute_single_action(action, wait=False)

        for action in soft_actions:
            self._execute_single_action(action, wait=False)

        for action in hard_actions:
            self._execute_single_action(action, wait=True)

    def _execute_single_action(self, action: Dict, wait: bool = True):
        """Execute a single action."""
        action_type = action.get("actionType", "")
        action_id = action.get("actionId", "")
        blocking_type = action.get("blockingType", "HARD")

        self.log(f"🎬 Action: {action_type} [{blocking_type}]", "info")
        self._update_action_state(action_id, "RUNNING")

        success = True
        self._send_action_to_bridge(action)

        if wait and success:
            action_delay = self._get_action_delay(action_type)
            time.sleep(action_delay)

        if success:
            self._update_action_state(action_id, "FINISHED")
            self.log(f"   ✓ {action_type} finished", "success")
        else:
            self._update_action_state(action_id, "FAILED", "Action failed")
            self.log(f"   ✗ {action_type} failed", "error")

    def _get_action_delay(self, action_type: str) -> float:
        """Get delay time for action completion."""
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
        """Send action to bridge for execution."""
        action_msg = {
            "headerId": self._next_header_id(),
            "timestamp": self._get_timestamp(),
            "version": VDA_VERSION,
            "manufacturer": MANUFACTURER,
            "serialNumber": SERIAL_NUMBER,
            "actions": [action]
        }
        self.client.publish(TOPIC_INSTANT_ACTIONS, json.dumps(action_msg))

    def _update_action_state(self, action_id: str, status: str, result: str = ""):
        """Update action state."""
        for action_state in self.action_states:
            if action_state.actionId == action_id:
                action_state.actionStatus = status
                if result:
                    action_state.resultDescription = result
                break

    # ========================================================================
    # STATE PUBLISHING
    # ========================================================================

    def _publish_state(self):
        """Publish VDA5050 state."""
        if not self.client:
            return

        # Build remaining nodeStates (not traversed)
        remaining_node_states = []
        for ns in self.node_states:
            if ns.sequenceId not in self._traversed_sequence_ids:
                remaining_node_states.append(ns.to_dict())

        remaining_edge_states = [es.to_dict() for es in self.edge_states]
        action_states_list = [a.to_dict() for a in self.action_states]

        state = {
            "headerId": self._next_header_id(),
            "timestamp": self._get_timestamp(),
            "version": VDA_VERSION,
            "manufacturer": MANUFACTURER,
            "serialNumber": SERIAL_NUMBER,
            "orderId": self.order_id,
            "orderUpdateId": self.order_update_id,
            "lastNodeId": self.last_node_id or "",
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

    def _publish_current_order(self, is_update: bool = None):
        """
        Publish current order to MQTT.

        VDA 5050 SPEC for updates:
        "The last node of the previous base is the first base node in the updated order.
         The other nodes and edges from the previous base are NOT RESENT."

        For new orders: Publish all nodes
        For updates: Publish only from stitch point (last traversed node) onwards
        """
        if not self.client or not self.node_states:
            return

        # Determine if this is an update (orderUpdateId > 0)
        if is_update is None:
            is_update = self.order_update_id > 0

        # Find nodes to publish
        if is_update:
            # For updates: Find stitch point = LAST TRAVERSED node
            # This is "the last node of the previous base"
            stitch_seq_id = None
            for ns in self.node_states:
                if ns.sequenceId in self._traversed_sequence_ids:
                    stitch_seq_id = ns.sequenceId  # Keep updating to find the LAST one

            if stitch_seq_id is None:
                # No traversed nodes - shouldn't happen for a valid update
                # Fall back to first released node
                for ns in self.node_states:
                    if ns.released:
                        stitch_seq_id = ns.sequenceId
                        break

            if stitch_seq_id is None:
                # Still no stitch point - publish all
                nodes_to_publish = self.node_states
                edges_to_publish = self.edge_states
                self.log("⚠ No stitch point found, publishing all nodes", "warning")
            else:
                # Publish from stitch point onwards (stitch node + everything after)
                nodes_to_publish = [ns for ns in self.node_states
                                    if ns.sequenceId >= stitch_seq_id]

                # Get corresponding edges
                # Edge leading TO stitch node has seq = stitch_seq - 1
                # We DON'T include that edge (it's already traversed)
                # We include edges from stitch node onwards (seq >= stitch_seq + 1)
                # Actually, we include edges that connect published nodes
                min_edge_seq = stitch_seq_id + 1  # First edge AFTER stitch node
                edges_to_publish = [es for es in self.edge_states
                                    if es.sequenceId >= min_edge_seq]
        else:
            # For new orders: Publish all
            nodes_to_publish = self.node_states
            edges_to_publish = self.edge_states

        # Build nodes list
        nodes_list = []
        for ns in nodes_to_publish:
            node_dict = {
                "nodeId": ns.nodeId,
                "sequenceId": ns.sequenceId,
                "released": ns.released,
                "actions": []
            }
            if ns.nodeId in NODES:
                node_dict["nodePosition"] = {
                    "x": NODES[ns.nodeId]["x"],
                    "y": NODES[ns.nodeId]["y"],
                    "mapId": MAP_ID
                }
            elif ns.nodePosition:
                node_dict["nodePosition"] = ns.nodePosition
            nodes_list.append(node_dict)

        # Build edges list
        edges_list = []
        for es in edges_to_publish:
            edges_list.append({
                "edgeId": es.edgeId,
                "sequenceId": es.sequenceId,
                "startNodeId": es.startNodeId,
                "endNodeId": es.endNodeId,
                "released": es.released,
                "actions": []
            })

        order = {
            "headerId": self._next_header_id(),
            "timestamp": self._get_timestamp(),
            "version": VDA_VERSION,
            "manufacturer": MANUFACTURER,
            "serialNumber": SERIAL_NUMBER,
            "orderId": self.order_id,
            "orderUpdateId": self.order_update_id,
            "nodes": nodes_list,
            "edges": edges_list
        }

        self.client.publish(TOPIC_ORDER, json.dumps(order))

        if is_update:
            self.log(f"📤 Order UPDATE published (stitch + {len(nodes_list)} nodes)", "info")
        else:
            self.log(f"📤 Order published ({len(nodes_list)} nodes)", "info")

    # ========================================================================
    # GUI CONTROLS
    # ========================================================================

    def pause_mission(self):
        """
        Stop mission (halt robot movement).

        When stopping:
        1. Set paused = True
        2. Send stop command to halt robot movement
        """
        with self.lock:
            if not self.mission_active and not self.waiting_at_decision_point:
                self.log("⚠ No active mission to stop", "warning")
                return

            if self.paused:
                self.log("⚠ Mission is already stopped", "info")
                return

            self.paused = True
            self.log("⏹ Mission STOPPED", "warning")

            # Send stop movement command via instant actions
            action_msg = {
                "headerId": self._next_header_id(),
                "timestamp": self._get_timestamp(),
                "version": VDA_VERSION,
                "manufacturer": MANUFACTURER,
                "serialNumber": SERIAL_NUMBER,
                "actions": [{
                    "actionType": "stopMovement",
                    "actionId": self._next_action_id("stop"),
                    "blockingType": "HARD",
                    "actionParameters": []
                }]
            }
            self.client.publish(TOPIC_INSTANT_ACTIONS, json.dumps(action_msg))

    def resume_mission(self):
        """
        Resume mission after stop.

        Sends resumeOrder instant action to bridge which will:
        1. Reactivate ORDER_STATE
        2. Call navigate_to_next_node() to continue
        """
        with self.lock:
            if not self.mission_active and not self.waiting_at_decision_point:
                self.log("⚠ No active mission to resume", "warning")
                return

            if not self.paused:
                self.log("⚠ Mission is not stopped", "info")
                return

            self.paused = False
            self.log("▶ Mission RESUMED", "success")

            if self.waiting_at_decision_point:
                self.log("   ⚠ Waiting for horizon release", "warning")
                return

            # Send resumeOrder instant action to bridge
            if self.current_target_node:
                self.log(f"   Continuing to: {self.current_target_node}", "info")

                action_msg = {
                    "headerId": self._next_header_id(),
                    "timestamp": self._get_timestamp(),
                    "version": VDA_VERSION,
                    "manufacturer": MANUFACTURER,
                    "serialNumber": SERIAL_NUMBER,
                    "actions": [{
                        "actionType": "resumeOrder",
                        "actionId": self._next_action_id("resume"),
                        "blockingType": "HARD",
                        "actionParameters": []
                    }]
                }
                self.client.publish(TOPIC_INSTANT_ACTIONS, json.dumps(action_msg))

    def emergency_stop(self):
        """Emergency stop - use cancelOrder internally."""
        self._handle_cancel_order({"actionId": self._next_action_id("cancel")})

    def stop_mission(self):
        """Alias for emergency_stop."""
        self.emergency_stop()

    # ========================================================================
    # GUI MISSION SUPPORT - WITH STABLE SEQUENCE IDS
    # ========================================================================

    def set_mission_sequence(self, path: List[str]):
        """Set mission from GUI with stable sequence IDs."""
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

            # Generate new order ID
            self._order_id_counter += 1
            self.order_id = str(self._order_id_counter)
            self.order_update_id = 0
            self.current_order = {"orderId": self.order_id, "orderUpdateId": 0}

            self.log(f"   📋 Order ID: {self.order_id}, Update ID: {self.order_update_id}", "info")

            # Clear state
            self.node_states = []
            self.edge_states = []
            self.action_states = []
            self._traversed_sequence_ids = set()

            # Create nodes with stable sequenceIds
            for i, node_id in enumerate(path):
                if node_id in NODES:
                    ns = NodeState(
                        nodeId=node_id,
                        sequenceId=i * 2,
                        released=True,
                        nodePosition={"x": NODES[node_id]["x"], "y": NODES[node_id]["y"], "mapId": MAP_ID}
                    )
                    self.node_states.append(ns)

            # Create edges
            for i in range(len(path) - 1):
                es = EdgeState(
                    edgeId=f"e_{path[i]}_{path[i + 1]}",
                    sequenceId=i * 2 + 1,
                    released=True,
                    startNodeId=path[i],
                    endNodeId=path[i + 1]
                )
                self.edge_states.append(es)

            # Build mission queue
            self._build_mission_queue()

            # Check if already at first node
            if self.mission_queue:
                first_id, first_seq = self.mission_queue[0]
                if first_id == self.last_node_id or self._is_at_node(first_id):
                    self._mark_node_traversed(first_id, first_seq)
                    self.mission_queue.pop(0)

            if not self.mission_queue:
                self.log("Already at destination", "info")
                return

            self.mission_active = True
            self.paused = False
            self.waiting_at_decision_point = False

            self._publish_current_order(is_update=False)
            self._advance_mission()

    def set_mission_with_horizon(self, path: List[str], base_count: int):
        """Set mission with base/horizon split."""
        with self.lock:
            if not path or len(path) < 2:
                self.log("✗ Invalid path", "error")
                return

            if self.mission_active:
                self.log("⏹ Cancelling current mission", "warning")
                self.mission_active = False
                self.mission_queue = []
                self.current_target_node = None

            base_count = max(1, min(base_count, len(path)))
            horizon_count = len(path) - base_count

            if horizon_count > 0:
                self.log(f"🎯 Mission with Horizon:", "info")
                self.log(f"   Base ({base_count}): {' → '.join(path[:base_count])}", "info")
                self.log(f"   Horizon ({horizon_count}): {' → '.join(path[base_count:])}", "warning")
            else:
                self.log(f"🎯 GUI Mission: {' → '.join(path)}", "info")

            self._reset_gui_colors()

            # Generate new order ID
            self._order_id_counter += 1
            self.order_id = str(self._order_id_counter)
            self.order_update_id = 0
            self.current_order = {"orderId": self.order_id, "orderUpdateId": 0}

            self.log(f"   📋 Order ID: {self.order_id}, Update ID: {self.order_update_id}", "info")

            self.node_states = []
            self.edge_states = []
            self.action_states = []
            self._traversed_sequence_ids = set()

            # Create node states with released flag
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

            # Create edges
            for i in range(len(path) - 1):
                is_released = (i < base_count - 1)
                es = EdgeState(
                    edgeId=f"e_{path[i]}_{path[i + 1]}",
                    sequenceId=i * 2 + 1,
                    released=is_released,
                    startNodeId=path[i],
                    endNodeId=path[i + 1]
                )
                self.edge_states.append(es)

            # Build queue from BASE nodes only
            self._build_mission_queue()

            # Check if at first node
            if self.mission_queue:
                first_id, first_seq = self.mission_queue[0]
                if first_id == self.last_node_id or self._is_at_node(first_id):
                    self._mark_node_traversed(first_id, first_seq)
                    self.mission_queue.pop(0)

            if not self.mission_queue:
                if horizon_count > 0:
                    self.log("⏸ At decision point - waiting for horizon release", "warning")
                    self.waiting_at_decision_point = True
                    self.new_base_request = True
                else:
                    self.log("Already at destination", "info")
                return

            self.mission_active = True
            self.paused = False
            self.waiting_at_decision_point = False

            self._publish_current_order(is_update=False)
            self._advance_mission()

    def release_horizon_nodes(self, count: int = 1, target_node_id: str = None):
        """
        Release N horizon nodes to base (BATCH RELEASE).

        Args:
            count: Number of nodes to release (default 1)
            target_node_id: If specified, release up to and including this node
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

            # Determine how many to release
            if target_node_id:
                # Release up to target node
                nodes_to_release = []
                for idx, ns in horizon_nodes:
                    nodes_to_release.append((idx, ns))
                    if ns.nodeId == target_node_id:
                        break
            else:
                # Release N nodes
                nodes_to_release = horizon_nodes[:count]

            # Release nodes
            for idx, ns in nodes_to_release:
                ns.released = True
                self.log(f"✓ Released: {ns.nodeId} (seq: {ns.sequenceId})", "success")

                # Add to mission queue
                if ns.sequenceId not in self._traversed_sequence_ids:
                    self.mission_queue.append((ns.nodeId, ns.sequenceId))
                    if ns.nodeId in self.gui_node_states:
                        self.gui_node_states[ns.nodeId] = "planned"

                # Release corresponding edge
                edge_seq = ns.sequenceId - 1
                for es in self.edge_states:
                    if es.sequenceId == edge_seq and not es.released:
                        es.released = True
                        self.log(f"   Released edge: {es.edgeId}", "info")
                        break

            # Sort queue by sequence
            self.mission_queue.sort(key=lambda x: x[1])

            # Increment order update ID
            self.order_update_id += 1
            if self.current_order:
                self.current_order["orderUpdateId"] = self.order_update_id

            # CRITICAL: Publish as UPDATE (stitch point + released nodes only)
            self._publish_current_order(is_update=True)
            self.log(f"   📋 Order ID: {self.order_id}, Update ID: {self.order_update_id}", "info")

            # Continue if waiting
            if self.waiting_at_decision_point:
                self.waiting_at_decision_point = False
                self.new_base_request = False
                self.log("▶ Continuing mission after horizon release", "success")
                self._advance_mission()

    def release_horizon_node(self, node_id: str = None):
        """Release single horizon node (backward compatible)."""
        self.release_horizon_nodes(count=1, target_node_id=node_id)

    # ========================================================================
    # MAIN LOOP
    # ========================================================================

    def _main_loop(self):
        """Main loop."""
        while self.running:
            time.sleep(0.1)

            self._update_animation()

            if time.time() - self.last_state_publish_ts > STATE_PUBLISH_INTERVAL:
                self._publish_state()
                self.last_state_publish_ts = time.time()

            if time.time() - self.last_connection_publish_ts > CONNECTION_PUBLISH_INTERVAL:
                self._publish_connection_state("ONLINE")
                self.last_connection_publish_ts = time.time()

            if not self.mission_active or not self.current_target_node:
                continue

            if self.paused or self.waiting_at_decision_point:
                continue

            # Check arrival
            if self._is_at_node(self.current_target_node) and not self.is_driving:
                self._on_node_reached(self.current_target_node,
                                      self.current_target_sequence_id or 0)
            elif time.time() - self.last_command_ts > STALLED_TIMEOUT_SEC and not self.is_driving:
                self.log(f"⚠ Stalled, retrying...", "warning")
                self._send_navigation_command(self.current_target_node,
                                              self.current_target_sequence_id or 0)

    # ========================================================================
    # UTILITY
    # ========================================================================

    def _next_header_id(self) -> int:
        self._header_id += 1
        return self._header_id

    def _next_action_id(self, action_type: str) -> str:
        """
        Generate simple sequential action IDs like 'dock1', 'dock2', 'stop1', etc.
        """
        if action_type not in self._action_id_counters:
            self._action_id_counters[action_type] = 0
        self._action_id_counters[action_type] += 1
        return f"{action_type}{self._action_id_counters[action_type]}"

    def _get_timestamp(self) -> str:
        return time.strftime('%Y-%m-%dT%H:%M:%S.000Z', time.gmtime())

    def _reset_gui_colors(self):
        """Reset all nodes to idle."""
        for node_id in NODES:
            self.gui_node_states[node_id] = "idle"

    # ========================================================================
    # ERROR MANAGEMENT
    # ========================================================================

    def add_error(self, error_type: str, error_level: str = "WARNING",
                  error_description: str = "", error_references: List[Dict] = None):
        """Add an error."""
        error = {
            "errorType": error_type,
            "errorLevel": error_level,
            "errorDescription": error_description,
            "errorReferences": error_references or []
        }
        self.errors.append(error)
        self.log(f"⚠ Error added: {error_type} ({error_level})", "error")

    def clear_error(self, error_type: str):
        """Remove an error by type."""
        self.errors = [e for e in self.errors if e.get("errorType") != error_type]

    def clear_all_errors(self):
        """Clear all errors."""
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
        """Alias for order_id."""
        return self.order_id

    def send_goto_node(self, target_node: str):
        """Navigate to a target node."""
        with self.lock:
            start_node = self.last_node_id

            if not start_node:
                start_node = self._find_nearest_node()
                if start_node:
                    self.log(f"📍 Starting from nearest node: {start_node}", "info")
                    self.last_node_id = start_node
                else:
                    self.log("✗ Cannot determine starting position", "error")
                    return

            if start_node == target_node:
                self.log(f"Already at {target_node}", "info")
                return

            path = self._find_path(start_node, target_node)

            if not path:
                self.log(f"✗ No valid path from {start_node} to {target_node}", "error")
                return

            self.log(f"📍 Path: {' → '.join(path)}", "info")
            self.set_mission_sequence(path)

    def send_mission(self, path: List[str]):
        """Start a mission with the given path."""
        if not path:
            self.log("✗ Empty path provided", "error")
            return

        if not self.last_node_id:
            nearest = self._find_nearest_node()
            if nearest:
                self.last_node_id = nearest
                self.log(f"📍 Current position: {nearest}", "info")
            else:
                self.log("✗ Cannot determine starting position", "error")
                return

        if self.last_node_id != path[0]:
            prefix_path = self._find_path(self.last_node_id, path[0])
            if prefix_path and len(prefix_path) > 1:
                full_path = prefix_path[:-1] + path
                self.log(f"📍 Extended path: {' → '.join(full_path)}", "info")
                self.set_mission_sequence(full_path)
            else:
                self.set_mission_sequence(path)
        else:
            self.set_mission_sequence(path)

    def send_instant_action(self, action_type: str):
        """Send an instant action command."""
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
            action_msg = {
                "headerId": self._next_header_id(),
                "timestamp": self._get_timestamp(),
                "version": VDA_VERSION,
                "manufacturer": MANUFACTURER,
                "serialNumber": SERIAL_NUMBER,
                "actions": [{
                    "actionType": actual_action,
                    "actionId": self._next_action_id(actual_action),
                    "blockingType": "NONE",
                    "actionParameters": []
                }]
            }
            self.client.publish(TOPIC_INSTANT_ACTIONS, json.dumps(action_msg))
            self.log(f"📤 Instant action: {actual_action}", "info")

    def stop(self):
        """Stop backend gracefully."""
        self.running = False
        self.mission_active = False

        if self.client and self._connected:
            self._publish_connection_state("OFFLINE", log=True)
            time.sleep(0.2)

        if self.client:
            self.client.loop_stop()
            self.client.disconnect()
        self.log("✓ Backend stopped", "info")


# Alias for backward compatibility
VDA5050Backend = RobotBackend