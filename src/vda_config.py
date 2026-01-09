# ============================================================================
# vda_config.py - Configuration for VDA 5050 Robot System
# ============================================================================
#
# Loads configuration from SQLite database (vda5050.db) if available,
# otherwise falls back to hardcoded values.
#
# ============================================================================

import os
import sqlite3
from dataclasses import dataclass

# ============================================================================
# DATABASE PATH
# ============================================================================

DB_PATH = "vda5050.db"

# ============================================================================
# MQTT CONFIGURATION
# ============================================================================


# ============================================================================
#
MQTT_BROKER = "#"          #change to your mqtt broker
MQTT_PORT = "#"            #chnage to your mqtt port for e.g 1883
MQTT_USERNAME = "#"        #set to None if no websocket
MQTT_PASSWORD = "#"        #set to None if no websocket/password needed
MQTT_WEBSOCKET = True      #change to false if not using a websocket
MQTT_WS_PATH = '#'         #set to None if no websocket
MQTT_VERSION = "3.1"  # ← MQTT protocol version: "3.1", "3.1.1", or "5.0"




# ============================================================================


# ============================================================================
# NAVIGATION CONSTANTS
# ============================================================================

@dataclass(frozen=True)
class NavigationConstants:
    """Navigation tuning constants"""
    POSITION_TOLERANCE_MM: int = 75  # When to detect arrival
    AT_NODE_THRESHOLD_MM: int = 50  # When robot is precisely at node
    PRECHECK_DISTANCE_MM: int = 150  # When to send next goTo (seamless navigation)
    CLICK_TOLERANCE_PX: int = 15
    COMMAND_COOLDOWN_SEC: float = 2.0
    STALL_TIMEOUT_SEC: float = 15.0
    ANIMATION_DURATION_SEC: float = 0.5


NAV = NavigationConstants()

# ============================================================================
# MAP VISUALIZATION
# ============================================================================

FLIP_X = False
FLIP_Y = True
SWAP_XY = False


# ============================================================================
# COLORS
# ============================================================================

class Colors:
    """Color scheme for GUI"""
    BG = "#2b2b2b"
    PANEL = "#3c3f41"
    TEXT = "#e0e0e0"
    ACCENT = "#2196F3"

    NODE_IDLE = "#808080"
    NODE_CURRENT = "#ff4444"
    NODE_PLANNED = "#FF9800"
    NODE_DONE = "#00ff88"
    NODE_HORIZON = "#4444AA"

    EDGE_IDLE = "#606060"
    EDGE_ACTIVE = "#FF9800"
    EDGE_PLANNED = "#FF9800"
    EDGE_DONE = "#00ff88"
    EDGE_HORIZON = "#4444AA"

    ROBOT_BODY = "#00BCD4"
    ROBOT_DOT = "#FFFFFF"


# Aliases for compatibility
COLOR_BG = Colors.BG
COLOR_PANEL = Colors.PANEL
COLOR_TEXT = Colors.TEXT
COLOR_ACCENT = Colors.ACCENT
COLOR_NODE_IDLE = Colors.NODE_IDLE
COLOR_NODE_CURRENT = Colors.NODE_CURRENT
COLOR_NODE_PLANNED = Colors.NODE_PLANNED
COLOR_NODE_DONE = Colors.NODE_DONE
COLOR_EDGE_IDLE = Colors.EDGE_IDLE
COLOR_EDGE_ACTIVE = Colors.EDGE_ACTIVE
COLOR_EDGE_PLANNED = Colors.EDGE_PLANNED
COLOR_EDGE_DONE = Colors.EDGE_DONE
COLOR_ROBOT_BODY = Colors.ROBOT_BODY
COLOR_ROBOT_DOT = Colors.ROBOT_DOT


# ============================================================================
# DATABASE LOADING FUNCTIONS
# ============================================================================

def load_from_database():
    """Load configuration from SQLite database"""
    if not os.path.exists(DB_PATH):
        print(f"⚠ Database not found: {DB_PATH}, using hardcoded config")
        return None, None, None, None, None

    try:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()

        # Load map info
        cursor.execute("SELECT map_id, name, description FROM maps LIMIT 1")
        map_row = cursor.fetchone()
        map_id = map_row[0] if map_row else "lab"
        map_name = map_row[1] if map_row else "Laboratory"

        # Load robot info
        cursor.execute("SELECT serial_number, manufacturer, name FROM robots LIMIT 1")
        robot_row = cursor.fetchone()
        serial_number = robot_row[0] if robot_row else "robot001"
        manufacturer = robot_row[1] if robot_row else "Dreame"
        robot_name = robot_row[2] if robot_row else "Robot"

        # Load nodes
        nodes = {}
        cursor.execute("SELECT node_id, x, y, name, node_type FROM nodes WHERE map_id = ?", (map_id,))
        for row in cursor.fetchall():
            nodes[row[0]] = {
                "x": row[1],
                "y": row[2],
                "id": row[0],
                "name": row[3],
                "type": row[4]
            }

        # Load edges
        edges = []
        cursor.execute("SELECT edge_id, start_node_id, end_node_id FROM edges WHERE map_id = ?", (map_id,))
        for row in cursor.fetchall():
            edges.append({
                "id": row[0],
                "start": row[1],
                "end": row[2]
            })

        conn.close()

        print(f"✓ Loaded from database: {map_name} ({len(nodes)} nodes, {len(edges)} edges)")
        return map_id, manufacturer, serial_number, nodes, edges

    except Exception as e:
        print(f"✗ Database error: {e}, using hardcoded config")
        return None, None, None, None, None


# ============================================================================
# LOAD CONFIGURATION
# ============================================================================

# Try to load from database first
_db_map_id, _db_manufacturer, _db_serial, _db_nodes, _db_edges = load_from_database()

# ============================================================================
# VDA 5050 IDENTITY
# ============================================================================

if _db_manufacturer and _db_serial and _db_map_id:
    MANUFACTURER = _db_manufacturer
    SERIAL_NUMBER = _db_serial
    MAP_ID = _db_map_id
else:
    MANUFACTURER = "Dreame"
    SERIAL_NUMBER = "robot001"
    MAP_ID = "lab"  # Change this to "ERP_lab" if needed

VDA_VERSION = "2.1.0"

# ============================================================================
# NODES - Positions of all waypoints
# ============================================================================

if _db_nodes:
    NODES = _db_nodes
else:
    NODES = {
        "home": {"x": 3259, "y": 3353, "id": "home"},
        "node1": {"x": 3228, "y": 3143, "id": "n1"},
        "node2": {"x": 3252, "y": 2918, "id": "n2"},
        "node3": {"x": 3093, "y": 2932, "id": "n3"},
        "node4": {"x": 3086, "y": 3141, "id": "n4"},
        "node5": {"x": 3096, "y": 3303, "id": "n5"}
    }

# ============================================================================
# EDGES - Connections between nodes
# ============================================================================

if _db_edges:
    EDGES = _db_edges
else:
    EDGES = [
        {"start": "home", "end": "node1", "id": "e_home_node1"},
        {"start": "node1", "end": "home", "id": "e_node1_home"},
        {"start": "home", "end": "node5", "id": "e_home_node5"},
        {"start": "node5", "end": "home", "id": "e_node5_home"},
        {"start": "node1", "end": "node2", "id": "e_node1_node2"},
        {"start": "node2", "end": "node1", "id": "e_node2_node1"},
        {"start": "node1", "end": "node4", "id": "e_node1_node4"},
        {"start": "node4", "end": "node1", "id": "e_node4_node1"},
        {"start": "node2", "end": "node3", "id": "e_node2_node3"},
        {"start": "node3", "end": "node2", "id": "e_node3_node2"},
        {"start": "node3", "end": "node4", "id": "e_node3_node4"},
        {"start": "node4", "end": "node3", "id": "e_node4_node3"},
        {"start": "node4", "end": "node5", "id": "e_node4_node5"},
        {"start": "node5", "end": "node4", "id": "e_node5_node4"},
    ]

# ============================================================================
# MISSIONS - Predefined routes
# ============================================================================

MISSIONS = {
    "Full Loop": ["home", "node1", "node2", "node3", "node4", "node5", "home"],
    "Quick Patrol": ["home", "node1", "node4", "node5", "home"],
    "Far Corner": ["home", "node1", "node2", "node3"],
    "Test 2 Nodes": ["home", "node1", "home"],
}


# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

def update_map_id(new_map_id: str):
    """Update map ID in database"""
    if not os.path.exists(DB_PATH):
        print(f"Database not found: {DB_PATH}")
        return False

    try:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()

        # Get current map_id
        cursor.execute("SELECT map_id FROM maps LIMIT 1")
        old_map_id = cursor.fetchone()[0]

        # Update all tables
        cursor.execute("UPDATE maps SET map_id = ? WHERE map_id = ?", (new_map_id, old_map_id))
        cursor.execute("UPDATE nodes SET map_id = ? WHERE map_id = ?", (new_map_id, old_map_id))
        cursor.execute("UPDATE edges SET map_id = ? WHERE map_id = ?", (new_map_id, old_map_id))

        conn.commit()
        conn.close()

        print(f"✓ Updated map_id: {old_map_id} → {new_map_id}")
        return True

    except Exception as e:
        print(f"✗ Error updating map_id: {e}")
        return False


def update_map_name(new_name: str):
    """Update map name in database"""
    if not os.path.exists(DB_PATH):
        print(f"Database not found: {DB_PATH}")
        return False

    try:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        cursor.execute("UPDATE maps SET name = ?", (new_name,))
        conn.commit()
        conn.close()
        print(f"✓ Updated map name to: {new_name}")
        return True
    except Exception as e:
        print(f"✗ Error updating map name: {e}")
        return False


# ============================================================================
# PRINT LOADED CONFIGURATION
# ============================================================================

if __name__ == "__main__":
    print("\n" + "=" * 50)
    print("VDA 5050 Configuration")
    print("=" * 50)

    print(f"\n📡 MQTT Settings:")
    print(f"   Broker: {MQTT_BROKER}")
    print(f"   Port: {MQTT_PORT}")
    print(f"   Username: {MQTT_USERNAME or '(none)'}")
    print(f"   WebSocket: {MQTT_WEBSOCKET}")

    print(f"\n🤖 Robot Identity:")
    print(f"   Manufacturer: {MANUFACTURER}")
    print(f"   Serial Number: {SERIAL_NUMBER}")
    print(f"   Map ID: {MAP_ID}")
    print(f"   VDA Version: {VDA_VERSION}")

    print(f"\n🗺️ Nodes ({len(NODES)}):")
    for node_id, data in NODES.items():
        print(f"   {node_id}: ({data['x']}, {data['y']})")

    print(f"\n🔗 Edges ({len(EDGES)}):")
    for edge in EDGES[:5]:
        print(f"   {edge['start']} → {edge['end']}")
    if len(EDGES) > 5:
        print(f"   ... and {len(EDGES) - 5} more")

    print("=" * 50)
