# actions_basic.py
import requests


# Helper function to send PUT requests. We only need PUT for commands.
def _put(robot_ip: str, path: str, json_data=None):
    """Sends a PUT request to the robot's API."""
    url = f"http://{robot_ip}{path}"
    try:
        # Print what we are about to do
        print(f"-> Sending command to: {url}")
        print(f"   With JSON body: {json_data or '{}'}")

        resp = requests.put(url, json=json_data, timeout=5)

        # Check if the robot liked the command
        if resp.status_code == 200:
            print("   Robot replied: 200 OK")
            return True
        else:
            print(f"   Error: Robot replied with code {resp.status_code}")
            print(f"   Response: {resp.text}")
            return False

    except requests.exceptions.RequestException as e:
        print(f"   Error: Connection failed for {url}: {e}")
        return False


# --- BASIC ACTIONS ---

def handle_beep(robot_ip: str):
    """
    Tells the robot to play its test sound.
    Maps to VDA 'beep'.
    """
    path = "/api/v2/robot/capabilities/SpeakerTestCapability"
    body = {"action": "play_test_sound"}
    return _put(robot_ip, path, body)


def handle_locate(robot_ip: str):
    """
    Tells the robot to play its "find me" sound.
    Maps to VDA 'locate'.
    """
    path = "/api/v2/robot/capabilities/LocateCapability"
    body = {"action": "locate"}
    return _put(robot_ip, path, body)


def handle_start_cleaning(robot_ip: str):
    """
    Tells the robot to start cleaning.
    Maps to VDA 'startCleaning'.
    """
    path = "/api/v2/robot/capabilities/BasicControlCapability"
    body = {"action": "start"}
    return _put(robot_ip, path, body)


def handle_stop_cleaning(robot_ip: str):
    """
    Tells the robot to stop all activity.
    Maps to VDA 'stopCleaning'.
    """
    path = "/api/v2/robot/capabilities/BasicControlCapability"
    body = {"action": "stop"}
    return _put(robot_ip, path, body)


def handle_pause_cleaning(robot_ip: str):
    """
    Tells the robot to pause cleaning.
    Maps to VDA 'pauseCleaning'.
    """
    path = "/api/v2/robot/capabilities/BasicControlCapability"
    body = {"action": "pause"}
    return _put(robot_ip, path, body)


def handle_dock(robot_ip: str):
    """
    Tells the robot to return to its charging dock.
    Maps to VDA 'dock' and 'quickCharge'.
    """
    path = "/api/v2/robot/capabilities/BasicControlCapability"
    body = {"action": "home"}
    return _put(robot_ip, path, body)


def handle_set_fan_speed(robot_ip: str, speed: str):
    """
    Sets the fan speed.
    Maps to VDA 'setFanSpeed'.
    """
    path = "/api/v2/robot/capabilities/FanSpeedControlCapability/preset"
    # FIX: The API expects {"name": "..."} not {"action": "..."}
    body = {"name": speed}
    return _put(robot_ip, path, body)


def handle_set_volume(robot_ip: str, volume: int):
    """
    Sets the speaker volume.
    Maps to VDA 'setVolume'.
    The volume should be an integer between 0 and 100.
    """
    path = "/api/v2/robot/capabilities/SpeakerVolumeControlCapability"
    body = {
        "action": "set_volume",
        "value": volume
    }
    return _put(robot_ip, path, body)