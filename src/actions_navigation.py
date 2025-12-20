import requests
import json


def handle_goto(robot_ip, action_parameters):
    try:
        url = f"http://{robot_ip}/api/v2/robot/capabilities/GoToLocationCapability"

        # 1. Extract parameters from VDA message (they are in meters)
        x_mm = 0
        y_mm = 0
        for param in action_parameters:
            if param['key'] == 'x':
                # Convert meters to millimeters for the robot
                x_mm = int(float(param['value']) )
            if param['key'] == 'y':
                # Convert meters to millimeters for the robot
                y_mm = int(float(param['value']))

        # 2. Create the robot's specific JSON command
        command = {
            "action": "goto",
            "coordinates": {
                "x": x_mm,
                "y": y_mm
            }
        }

        # 3. Send the command
        requests.put(url, json=command, timeout=5)
        print(f"-> Executed action: GOTO (x={x_mm}mm, y={y_mm}mm)")
        return True

    except Exception as e:
        print(f"Error executing GOTO: {e}")
        return False
