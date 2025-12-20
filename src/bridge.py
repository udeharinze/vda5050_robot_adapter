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

# --- CONFIGURATION ---
ROBOT_IP = "192.168.178.69"  # UPDATE with your robot's IP
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
STATE_INTERVAL = 5

print(f"Bridge is configured for Robot IP: {ROBOT_IP}")


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

        # Parse attributes - handle both __class and __class__ formats
        for attribute in state_data.get('attributes', []):
            # Get the class name - try both formats
            attr_class = attribute.get('__class') or attribute.get('__class__') or ""

            if 'BatteryStateAttribute' in attr_class:
                battery_level = attribute.get('level', 0)
                battery_flag = attribute.get('flag', 'unknown')
                # print(f"DEBUG: Found battery: {battery_level}%, flag: {battery_flag}")
            elif 'StatusStateAttribute' in attr_class:
                status_value = attribute.get('value', 'unknown')
                # print(f"DEBUG: Found status: {status_value}")

        # --- Derive VDA 5050 Fields ---
        is_driving = status_value in ['cleaning', 'returning', 'moving', 'manual_control', 'going_to_target']
        is_charging = (battery_flag == 'charging') or (status_value == 'docked')
        is_paused = (status_value == 'paused')

        return {
            "x": x,
            "y": y,
            "theta_deg": angle_deg,
            "battery": battery_level,
            "charging": is_charging,
            "driving": is_driving,
            "paused": is_paused,
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


def loop_state_and_visualization(mqtt_client):
    """Publishes the robot's full state AND visualization data."""
    header_id_counter = 0
    debug_printed = False

    while True:
        data = get_robot_data()

        if data:
            header_id_counter += 1
            timestamp = time.strftime('%Y-%m-%dT%H:%M:%S.000Z', time.gmtime())
            theta_vda = round(math.radians(data["theta_deg"]), 5)

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

            # Send visualization to VDA topic
            vda_vis = {
                "headerId": header_id_counter,
                "timestamp": timestamp,
                "version": "2.1.0",
                "manufacturer": "Dreame",
                "serialNumber": "robot001",
                "agvPosition": agv_position,
                "velocity": velocity
            }
            mqtt_client.publish("uagv/v2/Dreame/robot001/visualization", json.dumps(vda_vis))

            # Send state to bridge topic (for backend to consume)
            bridge_state = {
                "agvPosition": {
                    "x": data["x"],
                    "y": data["y"],
                    "theta": theta_vda
                },
                "driving": data["driving"],
                "batteryState": {
                    "batteryCharge": float(data["battery"]),
                    "charging": data["charging"]
                }
            }
            mqtt_client.publish("vda5050/robot/state", json.dumps(bridge_state))

            # Print state (only show debug once)
            if not debug_printed:
                debug_printed = True
                print("--- Debug messages above will stop after first successful parse ---")

            print(f"STATE: Pos({data['x']}, {data['y']}) | Theta {data['theta_deg']}° |  {theta_vda} | "
                  f"Batt {data['battery']}% | Charging: {data['charging']} | Status: {data['status']}")

        time.sleep(STATE_INTERVAL)


def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code.is_failure:
        print(f"Failed to connect: {reason_code}. Retrying...")
    else:
        print(f"Connected to MQTT broker at {MQTT_BROKER}!")
        client.subscribe("vda5050/robot/instantAction")
        print("Subscribed to vda5050/robot/instantAction")


def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode()
        vda_message = json.loads(payload)

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
                # Stop the robot using Valetudo API
                handle_stop(ROBOT_IP)
            elif action_type == 'pauseMovement':
                # Pause the robot using Valetudo API
                handle_pause(ROBOT_IP)
            elif action_type == 'resumeMovement':
                # Resume the robot using Valetudo API
                handle_resume(ROBOT_IP)
            else:
                print(f"Unknown VDA action type: {action_type}")

            time.sleep(0.1)

    except Exception as e:
        print(f"Error processing action message: {e}")


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


# --- MAIN ---
print("=== VDA 5050 Bridge Starting ===")
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, "Bridge_V1")
client.on_connect = on_connect
client.on_message = on_message

try:
    print(f"Connecting to MQTT broker at {MQTT_BROKER}...")
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