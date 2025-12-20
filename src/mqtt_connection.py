# ============================================================================
# mqtt_connection.py - MQTT Connection Helper with WebSocket Support
# ============================================================================
#
# This module provides a helper function to create MQTT clients that can
# connect via either TCP or WebSocket (including WSS for Flexus cloud broker).
#
# Usage:
#   from mqtt_connection import create_mqtt_client
#   client = create_mqtt_client(client_id="my_client")
#   client.connect()  # Uses settings from vda_config.py
#
# ============================================================================

import ssl
import paho.mqtt.client as mqtt

# Import configuration
from vda_config import (
    MQTT_BROKER, 
    MQTT_PORT, 
    MQTT_TRANSPORT, 
    MQTT_WS_PATH, 
    MQTT_USE_TLS,
    MQTT_USERNAME,
    MQTT_PASSWORD
)


def create_mqtt_client(client_id: str, callback_api_version=mqtt.CallbackAPIVersion.VERSION2):
    """
    Create an MQTT client configured for either TCP or WebSocket connection.
    
    Args:
        client_id: Unique identifier for this MQTT client
        callback_api_version: Paho MQTT callback API version
        
    Returns:
        Configured mqtt.Client instance (not yet connected)
        
    Example:
        client = create_mqtt_client("vda_backend_robot001")
        client.on_connect = my_on_connect
        client.on_message = my_on_message
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()
    """
    
    # Create client with appropriate transport
    if MQTT_TRANSPORT == "websockets":
        client = mqtt.Client(
            callback_api_version,
            client_id,
            transport="websockets"
        )
        
        # Set WebSocket path (required for Flexus broker)
        if MQTT_WS_PATH:
            client.ws_set_options(path=MQTT_WS_PATH)
            
        print(f"📡 MQTT Client configured for WebSocket")
        print(f"   Broker: {MQTT_BROKER}:{MQTT_PORT}")
        print(f"   Path: {MQTT_WS_PATH}")
        
    else:
        # Standard TCP connection
        client = mqtt.Client(
            callback_api_version,
            client_id
        )
        print(f"📡 MQTT Client configured for TCP")
        print(f"   Broker: {MQTT_BROKER}:{MQTT_PORT}")
    
    # Configure TLS/SSL if needed (required for WSS)
    if MQTT_USE_TLS:
        client.tls_set(
            ca_certs=None,  # Use system CA certificates
            certfile=None,
            keyfile=None,
            cert_reqs=ssl.CERT_REQUIRED,
            tls_version=ssl.PROTOCOL_TLS,
            ciphers=None
        )
        # Don't verify hostname for cloud brokers (optional, remove if issues)
        # client.tls_insecure_set(True)
        print(f"   TLS: Enabled")
    
    # Set authentication if provided
    if MQTT_USERNAME and MQTT_PASSWORD:
        client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        print(f"   Auth: Username/Password configured")
    
    return client


def get_connection_info() -> dict:
    """
    Get current MQTT connection configuration info.
    
    Returns:
        Dictionary with connection details
    """
    return {
        "broker": MQTT_BROKER,
        "port": MQTT_PORT,
        "transport": MQTT_TRANSPORT,
        "ws_path": MQTT_WS_PATH,
        "tls": MQTT_USE_TLS,
        "auth": bool(MQTT_USERNAME and MQTT_PASSWORD)
    }


# ============================================================================
# QUICK TEST
# ============================================================================

if __name__ == "__main__":
    print("\n=== MQTT Connection Test ===\n")
    
    info = get_connection_info()
    print(f"Broker: {info['broker']}")
    print(f"Port: {info['port']}")
    print(f"Transport: {info['transport']}")
    print(f"WS Path: {info['ws_path']}")
    print(f"TLS: {info['tls']}")
    print(f"Auth: {info['auth']}")
    
    print("\n--- Creating client ---")
    client = create_mqtt_client("test_client")
    
    def on_connect(client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            print("✅ Connected successfully!")
        else:
            print(f"❌ Connection failed: {reason_code}")
        client.disconnect()
    
    def on_disconnect(client, userdata, flags, reason_code, properties):
        print("Disconnected")
    
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    
    print("\n--- Connecting ---")
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_forever()
    except Exception as e:
        print(f"❌ Connection error: {e}")
