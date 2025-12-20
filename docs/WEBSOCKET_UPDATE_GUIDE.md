# ============================================================================
# HOW TO UPDATE vda_backend.py FOR WEBSOCKET SUPPORT
# ============================================================================
#
# This file explains the changes needed to connect to Flexus's cloud broker.
#
# ============================================================================

## STEP 1: Update vda_config.py
## ----------------------------
## Replace your current MQTT settings with:

```python
# OPTION 2: Flexus Cloud Broker (for production/integration)
MQTT_BROKER = "flexusag-cf-ft-demo-flexus-flexguide-mqtt-broker.cfapps.eu20-001.hana.ondemand.com"
MQTT_PORT = 443
MQTT_TRANSPORT = "websockets"
MQTT_WS_PATH = "/api_broker/mqtt/connect"
MQTT_USE_TLS = True
MQTT_USERNAME = None  # Set if Flexus requires authentication
MQTT_PASSWORD = None
```


## STEP 2: Update imports in vda_backend.py
## ----------------------------------------
## Add these imports at the top:

```python
import ssl
from vda_config import (
    MQTT_BROKER, MQTT_PORT, MANUFACTURER, SERIAL_NUMBER, VDA_VERSION, MAP_ID,
    NODES, EDGES, NAV,
    MQTT_TRANSPORT, MQTT_WS_PATH, MQTT_USE_TLS, MQTT_USERNAME, MQTT_PASSWORD  # ADD THESE
)
```


## STEP 3: Update the start() method in vda_backend.py
## ---------------------------------------------------
## Replace the current start() method with this:

```python
def start(self):
    """Start MQTT client and main loop"""
    
    # Create client with appropriate transport (TCP or WebSocket)
    if MQTT_TRANSPORT == "websockets":
        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2, 
            f"vda_backend_{SERIAL_NUMBER}",
            transport="websockets"
        )
        # Set WebSocket path for Flexus broker
        if MQTT_WS_PATH:
            self.client.ws_set_options(path=MQTT_WS_PATH)
        self.log(f"📡 Using WebSocket transport", "info")
    else:
        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2, 
            f"vda_backend_{SERIAL_NUMBER}"
        )
        self.log(f"📡 Using TCP transport", "info")
    
    # Configure TLS/SSL if needed (required for WSS)
    if MQTT_USE_TLS:
        self.client.tls_set(
            ca_certs=None,  # Use system CA certificates
            certfile=None,
            keyfile=None,
            cert_reqs=ssl.CERT_REQUIRED,
            tls_version=ssl.PROTOCOL_TLS,
            ciphers=None
        )
        self.log(f"🔒 TLS enabled", "info")
    
    # Set authentication if provided
    if MQTT_USERNAME and MQTT_PASSWORD:
        self.client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.log(f"🔑 Authentication configured", "info")
    
    # Set callbacks
    self.client.on_connect = self._on_connect
    self.client.on_disconnect = self._on_disconnect
    self.client.on_message = self._on_message
    
    # Set last will for connection topic
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
    self.log(f"📡 Broker: {self.broker}:{self.port}", "info")

    try:
        self.client.connect(self.broker, self.port, 60)
        self.running = True
        self.client.loop_start()
        threading.Thread(target=self._main_loop, daemon=True).start()
        self.log("✓ Backend started", "success")
    except Exception as e:
        self.log(f"✗ MQTT connection failed: {e}", "error")
```


## STEP 4: Also update bridge_new.py the same way
## -----------------------------------------------
## The bridge also connects to MQTT, so apply similar changes.


## ALTERNATIVE: Use mqtt_connection.py helper
## ------------------------------------------
## I've created mqtt_connection.py which handles all this for you.
## You can use it like this:

```python
from mqtt_connection import create_mqtt_client
from vda_config import MQTT_BROKER, MQTT_PORT

# In your start() method:
self.client = create_mqtt_client(f"vda_backend_{SERIAL_NUMBER}")
self.client.on_connect = self._on_connect
self.client.on_disconnect = self._on_disconnect
self.client.on_message = self._on_message
# ... rest of setup ...
self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
```


## TESTING THE CONNECTION
## ----------------------
## Run this to test if you can connect to Flexus:

```bash
python mqtt_connection.py
```

## If successful, you'll see:
## ✅ Connected successfully!


## SWITCHING BETWEEN LOCAL AND FLEXUS
## ----------------------------------
## In vda_config.py, just comment/uncomment the appropriate section:

```python
# For local testing:
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TRANSPORT = "tcp"
MQTT_WS_PATH = None
MQTT_USE_TLS = False

# For Flexus:
# MQTT_BROKER = "flexusag-cf-ft-demo-..."
# MQTT_PORT = 443
# MQTT_TRANSPORT = "websockets"
# MQTT_WS_PATH = "/api_broker/mqtt/connect"
# MQTT_USE_TLS = True
```
