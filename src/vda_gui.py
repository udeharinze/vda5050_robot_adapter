# ============================================================================
# vda_gui.py - Tkinter Dashboard for VDA 5050 Fleet Management
# ============================================================================
#
# Clean GUI implementation compatible with vda_backend.py
#
# Features:
#   - Real-time map visualization with robot position
#   - Node/edge state display (idle, planned, current, done, horizon)
#   - Mission controls (start, pause, resume, stop)
#   - Click-to-navigate (click any node)
#   - Pan and zoom support
#   - Log panel for debugging
#
# ============================================================================

import tkinter as tk
from tkinter import ttk
import json
import math
import os
from typing import Dict, Optional

from vda_config import (
    NODES, EDGES, MISSIONS,
    Colors, FLIP_Y, FLIP_X, SWAP_XY
)
from vda_backend import VDA5050Backend

# ============================================================================
# COLOR CONSTANTS (from config, with fallbacks)
# ============================================================================

COLOR_BG = Colors.BG
COLOR_PANEL = Colors.PANEL
COLOR_TEXT = Colors.TEXT
COLOR_ACCENT = Colors.ACCENT

COLOR_NODE_IDLE = Colors.NODE_IDLE
COLOR_NODE_CURRENT = Colors.NODE_CURRENT
COLOR_NODE_PLANNED = Colors.NODE_PLANNED
COLOR_NODE_DONE = Colors.NODE_DONE

COLOR_EDGE_IDLE = Colors.EDGE_IDLE
COLOR_EDGE_PLANNED = Colors.EDGE_PLANNED
COLOR_EDGE_DONE = Colors.EDGE_DONE

COLOR_ROBOT_BODY = Colors.ROBOT_BODY
COLOR_ROBOT_DOT = Colors.ROBOT_DOT

# Horizon color (for unreleased nodes)
COLOR_NODE_HORIZON = "#4444AA"  # Blue-ish for horizon nodes

# ============================================================================
# DASHBOARD APPLICATION
# ============================================================================

class DashboardApp:
    """Main GUI application for VDA 5050 fleet management dashboard."""
    
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("VDA 5050 Robot Dashboard - goTo Navigation")
        self.root.geometry("1200x800")
        self.root.configure(bg=COLOR_BG)
        
        # Backend
        self.backend = VDA5050Backend()
        
        # Register callbacks
        self.backend.log_callback = self.log_message
        self.backend.state_callback = self._on_state_update
        self.backend.node_state_callback = self._on_node_state_change
        self.backend.edge_state_callback = self._on_edge_state_change
        
        # Map data (optional Valetudo map)
        self.map_data = self._load_map_file()
        
        # View state
        self.zoom = 1.0
        self.pan_x = 0
        self.pan_y = 0
        self._pan_start_x = 0
        self._pan_start_y = 0
        
        # Node/edge visual states
        self.node_visual_states: Dict[str, str] = {}  # node_id -> state
        self.edge_visual_states: Dict[str, str] = {}  # "start_end" -> state
        
        # Robot state for display
        self.robot_pose = {"x": 0, "y": 0, "theta": 0}
        self.robot_driving = False
        self.robot_battery = 0
        self.robot_charging = False
        
        # Setup UI
        self._setup_ui()
        
        # Connect backend
        self.backend.connect()
        
        # Fit view to nodes
        self._fit_map_to_view()
        
        # Start UI loop
        self._ui_loop()
    
    # ========================================================================
    # MAP LOADING
    # ========================================================================
    
    def _load_map_file(self) -> Dict:
        """Load Valetudo map JSON file (optional)"""
        map_path = "valetudo_map.json"
        result = {
            "walls": [],
            "charger": None,
            "pixel_size": 5,
            "bounds": (0, 0, 4000, 4000)
        }
        
        if not os.path.exists(map_path):
            return result
        
        try:
            with open(map_path, 'r') as f:
                data = json.load(f)
            
            pixel_size = data.get("pixelSize", 5)
            result["pixel_size"] = pixel_size
            
            # Extract charger position
            for entity in data.get("entities", []):
                if entity.get("type") == "charger_location":
                    points = entity.get("points", [])
                    if len(points) >= 2:
                        result["charger"] = (points[0], points[1])
            
            # Extract floor pixels as line segments (optimization)
            layers = data.get("layers", [])
            for layer in layers:
                if layer.get("type") == "floor":
                    compressed = layer.get("compressedPixels", [])
                    # Format: [x, y, count, x, y, count, ...]
                    i = 0
                    min_x, min_y = float('inf'), float('inf')
                    max_x, max_y = float('-inf'), float('-inf')
                    
                    while i + 2 < len(compressed):
                        x = compressed[i] * pixel_size
                        y = compressed[i + 1] * pixel_size
                        count = compressed[i + 2]
                        
                        x2 = x + (count * pixel_size)
                        result["walls"].append((x, y, x2, y))
                        
                        min_x = min(min_x, x)
                        min_y = min(min_y, y)
                        max_x = max(max_x, x2)
                        max_y = max(max_y, y + pixel_size)
                        
                        i += 3
                    
                    if min_x != float('inf'):
                        result["bounds"] = (min_x, min_y, max_x, max_y)
            
            self.log_message(f"✓ Loaded map: {len(result['walls'])} segments", "success")
            
        except Exception as e:
            self.log_message(f"Map load error: {e}", "error")
        
        return result
    
    # ========================================================================
    # UI SETUP
    # ========================================================================
    
    def _setup_ui(self):
        """Create the main UI layout"""
        # Main container
        main_frame = tk.Frame(self.root, bg=COLOR_BG)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Canvas (left side - map view)
        self.canvas = tk.Canvas(
            main_frame,
            bg=COLOR_BG,
            highlightthickness=0
        )
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Bind mouse events
        self.canvas.bind("<Button-1>", self._start_pan)
        self.canvas.bind("<B1-Motion>", self._do_pan)
        self.canvas.bind("<ButtonRelease-1>", self._on_click_release)
        self.canvas.bind("<MouseWheel>", self._do_zoom)  # Windows/Mac
        self.canvas.bind("<Button-4>", self._do_zoom_in)  # Linux scroll up
        self.canvas.bind("<Button-5>", self._do_zoom_out)  # Linux scroll down
        
        # Sidebar (right side - controls)
        sidebar = tk.Frame(main_frame, bg=COLOR_PANEL, width=280)
        sidebar.pack(side=tk.RIGHT, fill=tk.Y, padx=(5, 0))
        sidebar.pack_propagate(False)
        
        self._create_sidebar(sidebar)
    
    def _create_sidebar(self, sidebar: tk.Frame):
        """Create the control panel"""
        # Title
        tk.Label(
            sidebar, text="VDA 5050 Dashboard",
            font=("Arial", 14, "bold"),
            bg=COLOR_PANEL, fg=COLOR_TEXT
        ).pack(pady=(10, 5))
        
        # Separator
        ttk.Separator(sidebar, orient=tk.HORIZONTAL).pack(fill=tk.X, padx=10, pady=5)
        
        # === ROBOT STATUS ===
        status_frame = tk.LabelFrame(
            sidebar, text="Robot Status",
            bg=COLOR_PANEL, fg=COLOR_TEXT,
            font=("Arial", 10, "bold")
        )
        status_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Status variables
        self.vars = {
            "pos": tk.StringVar(value="--"),
            "theta": tk.StringVar(value="--"),
            "batt": tk.StringVar(value="--"),
            "status": tk.StringVar(value="Disconnected"),
            "node": tk.StringVar(value="--"),
            "target": tk.StringVar(value="--"),
            "order": tk.StringVar(value="--")
        }
        
        labels = [
            ("Position:", "pos"),
            ("Heading:", "theta"),
            ("Battery:", "batt"),
            ("Status:", "status"),
            ("Last Node:", "node"),
            ("Target:", "target"),
            ("Order:", "order")
        ]
        
        for label_text, var_key in labels:
            row = tk.Frame(status_frame, bg=COLOR_PANEL)
            row.pack(fill=tk.X, padx=5, pady=2)
            tk.Label(row, text=label_text, bg=COLOR_PANEL, fg=COLOR_TEXT, width=10, anchor=tk.W).pack(side=tk.LEFT)
            tk.Label(row, textvariable=self.vars[var_key], bg=COLOR_PANEL, fg=COLOR_ACCENT).pack(side=tk.LEFT)
        
        # === MISSIONS ===
        mission_frame = tk.LabelFrame(
            sidebar, text="Missions",
            bg=COLOR_PANEL, fg=COLOR_TEXT,
            font=("Arial", 10, "bold")
        )
        mission_frame.pack(fill=tk.X, padx=10, pady=5)
        
        for mission_name in MISSIONS:
            btn = tk.Button(
                mission_frame, text=mission_name,
                bg=COLOR_ACCENT, fg="white",
                command=lambda m=mission_name: self._start_mission(m)
            )
            btn.pack(fill=tk.X, padx=5, pady=2)
        
        # === CONTROLS ===
        control_frame = tk.LabelFrame(
            sidebar, text="Controls",
            bg=COLOR_PANEL, fg=COLOR_TEXT,
            font=("Arial", 10, "bold")
        )
        control_frame.pack(fill=tk.X, padx=10, pady=5)
        
        controls = [
            ("⏸ Pause", self._pause_mission, "#FFA500"),
            ("▶ Resume", self._resume_mission, "#4CAF50"),
            ("⏹ Stop", self._stop_mission, "#F44336"),
            ("🏠 Dock", self._dock_robot, "#9C27B0"),
            ("📍 Locate", self._locate_robot, "#00BCD4"),
            ("🔊 Beep", self._beep_robot, "#607D8B")
        ]
        
        for text, command, color in controls:
            btn = tk.Button(
                control_frame, text=text,
                bg=color, fg="white",
                command=command
            )
            btn.pack(fill=tk.X, padx=5, pady=2)
        
        # === LOG ===
        log_frame = tk.LabelFrame(
            sidebar, text="Log",
            bg=COLOR_PANEL, fg=COLOR_TEXT,
            font=("Arial", 10, "bold")
        )
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        self.log_text = tk.Text(
            log_frame,
            bg="#1e1e1e", fg=COLOR_TEXT,
            font=("Consolas", 9),
            wrap=tk.WORD,
            height=15
        )
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Log tag colors
        self.log_text.tag_configure("info", foreground="#e0e0e0")
        self.log_text.tag_configure("success", foreground="#4CAF50")
        self.log_text.tag_configure("warning", foreground="#FFC107")
        self.log_text.tag_configure("error", foreground="#F44336")
        self.log_text.tag_configure("debug", foreground="#9E9E9E")
    
    # ========================================================================
    # CALLBACKS
    # ========================================================================
    
    def _on_state_update(self, state: Dict):
        """Called when robot state updates"""
        self.robot_pose = state.get("pose", self.robot_pose)
        self.robot_driving = state.get("driving", False)
        self.robot_battery = state.get("battery", 0)
        self.robot_charging = state.get("charging", False)
    
    def _on_node_state_change(self, node_id: str, state: str):
        """Called when a node's visual state changes"""
        self.node_visual_states[node_id] = state
    
    def _on_edge_state_change(self, start_node: str, end_node: str, state: str):
        """Called when an edge's visual state changes"""
        key = f"{start_node}_{end_node}"
        self.edge_visual_states[key] = state
    
    # ========================================================================
    # CONTROL HANDLERS
    # ========================================================================
    
    def _start_mission(self, mission_name: str):
        """Start a predefined mission"""
        path = MISSIONS.get(mission_name, [])
        if path:
            self.log_message(f"Starting mission: {mission_name}", "info")
            self.backend.send_mission(path)
        else:
            self.log_message(f"Unknown mission: {mission_name}", "error")
    
    def _pause_mission(self):
        """Pause current mission"""
        self.backend.send_instant_action("pauseMovement")
    
    def _resume_mission(self):
        """Resume paused mission"""
        self.backend.send_instant_action("resumeMovement")
    
    def _stop_mission(self):
        """Stop current mission"""
        self.backend.stop_mission()
    
    def _dock_robot(self):
        """Send robot to dock"""
        self.backend.send_instant_action("dock")
    
    def _locate_robot(self):
        """Flash robot LED"""
        self.backend.send_instant_action("locate")
    
    def _beep_robot(self):
        """Make robot beep"""
        self.backend.send_instant_action("beep")
    
    def _on_click_release(self, event):
        """Handle click release - check if node was clicked"""
        # Check if this was a drag or a click
        dx = abs(event.x - self._pan_start_x)
        dy = abs(event.y - self._pan_start_y)
        
        if dx > 10 or dy > 10:
            return  # Was a drag, not a click
        
        # Check if clicked on a node
        click_tolerance = 20
        for node_id, node_data in NODES.items():
            sx, sy = self._world_to_screen(node_data["x"], node_data["y"])
            if abs(event.x - sx) < click_tolerance and abs(event.y - sy) < click_tolerance:
                self.log_message(f"Clicked node: {node_id}", "info")
                self.backend.send_goto_node(node_id)
                return
    
    # ========================================================================
    # LOGGING
    # ========================================================================
    
    def log_message(self, message: str, level: str = "info"):
        """Add message to log panel"""
        self.log_text.insert(tk.END, f"{message}\n", level)
        self.log_text.see(tk.END)
        
        # Limit log size
        lines = int(self.log_text.index('end-1c').split('.')[0])
        if lines > 200:
            self.log_text.delete('1.0', '50.0')
    
    # ========================================================================
    # COORDINATE CONVERSION
    # ========================================================================
    
    def _world_to_screen(self, x: float, y: float) -> tuple:
        """Convert world coordinates to screen coordinates"""
        if SWAP_XY:
            x, y = y, x
        
        if FLIP_X:
            x = -x
        if FLIP_Y:
            y = -y
        
        canvas_w = self.canvas.winfo_width()
        canvas_h = self.canvas.winfo_height()
        
        # Center of canvas
        cx = canvas_w / 2
        cy = canvas_h / 2
        
        # Apply zoom and pan
        sx = cx + (x - 3200) * self.zoom * 0.5 + self.pan_x
        sy = cy + (y - 3200) * self.zoom * 0.5 + self.pan_y
        
        return sx, sy
    
    # ========================================================================
    # RENDERING
    # ========================================================================
    
    def _ui_loop(self):
        """Main rendering loop (50ms interval)"""
        # Update status display
        self.vars["pos"].set(f"{int(self.robot_pose['x'])}, {int(self.robot_pose['y'])}")
        
        theta_rad = self.robot_pose['theta']
        theta_deg = math.degrees(theta_rad) % 360
        self.vars["theta"].set(f"{theta_deg:.0f}° ({theta_rad:.2f} rad)")
        
        self.vars["batt"].set(f"{int(self.robot_battery)}%")
        
        if self.robot_charging:
            status = "⚡ Charging"
        elif self.robot_driving:
            status = "🚗 Driving"
        elif self.backend.mission_active:
            status = "📋 Mission"
        else:
            status = "⏸ Idle"
        self.vars["status"].set(status)
        
        self.vars["node"].set(self.backend.last_node_id or "--")
        self.vars["target"].set(self.backend.current_target_node or "--")
        self.vars["order"].set(self.backend.current_order_id or "--")
        
        # Render canvas
        self.canvas.delete("all")
        self._draw_floor()
        self._draw_charger()
        self._draw_edges()
        self._draw_nodes()
        self._draw_robot()
        
        # Schedule next frame
        self.root.after(50, self._ui_loop)
    
    def _draw_floor(self):
        """Draw floor from Valetudo map"""
        if not self.map_data["walls"]:
            return
        
        pixel_size = self.map_data["pixel_size"]
        
        for x1, y1, x2, y2 in self.map_data["walls"]:
            sx1, sy1 = self._world_to_screen(x1, y1)
            sx2, sy2 = self._world_to_screen(x2, y2 + pixel_size)
            self.canvas.create_rectangle(sx1, sy1, sx2, sy2, fill="#3a3a3a", outline="")
    
    def _draw_charger(self):
        """Draw charger location"""
        charger = self.map_data.get("charger")
        if not charger:
            return
        
        cx, cy = self._world_to_screen(charger[0], charger[1])
        size = 15
        
        # Draw charger icon
        self.canvas.create_rectangle(
            cx - size, cy - size, cx + size, cy + size,
            fill="#4CAF50", outline="white", width=2
        )
        self.canvas.create_text(cx, cy, text="⚡", fill="white", font=("Arial", 12))
    
    def _draw_edges(self):
        """Draw edges between nodes"""
        for edge in EDGES:
            start_node = NODES.get(edge["start"])
            end_node = NODES.get(edge["end"])
            
            if not start_node or not end_node:
                continue
            
            # Get edge visual state
            key = f"{edge['start']}_{edge['end']}"
            state = self.edge_visual_states.get(key, "idle")
            
            # Determine color
            if state == "done":
                color = COLOR_EDGE_DONE
                width = 3
            elif state == "planned":
                color = COLOR_EDGE_PLANNED
                width = 2
            elif state == "horizon":
                color = COLOR_NODE_HORIZON
                width = 2
            else:
                color = COLOR_EDGE_IDLE
                width = 1
            
            # Draw edge
            sx1, sy1 = self._world_to_screen(start_node["x"], start_node["y"])
            sx2, sy2 = self._world_to_screen(end_node["x"], end_node["y"])
            
            self.canvas.create_line(sx1, sy1, sx2, sy2, fill=color, width=width)
    
    def _draw_nodes(self):
        """Draw nodes"""
        for node_id, node_data in NODES.items():
            sx, sy = self._world_to_screen(node_data["x"], node_data["y"])
            
            # Get node visual state
            state = self.node_visual_states.get(node_id, "idle")
            
            # Determine color and size
            if state == "current":
                color = COLOR_NODE_CURRENT
                size = 14
            elif state == "planned":
                color = COLOR_NODE_PLANNED
                size = 12
            elif state == "done":
                color = COLOR_NODE_DONE
                size = 10
            elif state == "horizon":
                color = COLOR_NODE_HORIZON
                size = 10
            else:
                color = COLOR_NODE_IDLE
                size = 8
            
            # Draw node
            self.canvas.create_oval(
                sx - size, sy - size, sx + size, sy + size,
                fill=color, outline="white", width=2
            )
            
            # Draw label
            label = node_data.get("id", node_id)
            self.canvas.create_text(
                sx, sy - size - 10,
                text=label,
                fill="white",
                font=("Arial", 9, "bold")
            )
    
    def _draw_robot(self):
        """Draw robot with heading indicator"""
        rx, ry = self._world_to_screen(self.robot_pose["x"], self.robot_pose["y"])
        theta = self.robot_pose["theta"]
        
        if FLIP_Y:
            theta = -theta
        
        # Robot body (circle)
        r_size = 15
        self.canvas.create_oval(
            rx - r_size, ry - r_size, rx + r_size, ry + r_size,
            fill=COLOR_ROBOT_BODY, outline="white", width=2
        )
        
        # Heading indicator (dot at front)
        dot_dist = 10
        dx = rx + dot_dist * math.cos(theta)
        dy = ry - dot_dist * math.sin(theta)
        self.canvas.create_oval(
            dx - 4, dy - 4, dx + 4, dy + 4,
            fill=COLOR_ROBOT_DOT, outline=""
        )
    
    # ========================================================================
    # PAN & ZOOM
    # ========================================================================
    
    def _start_pan(self, event):
        """Start panning"""
        self._pan_start_x = event.x
        self._pan_start_y = event.y
    
    def _do_pan(self, event):
        """Handle pan drag"""
        dx = event.x - self._pan_start_x
        dy = event.y - self._pan_start_y
        
        self.pan_x += dx
        self.pan_y += dy
        
        self._pan_start_x = event.x
        self._pan_start_y = event.y
    
    def _do_zoom(self, event):
        """Handle mouse wheel zoom (Windows/Mac)"""
        factor = 1.1 if event.delta > 0 else 0.9
        self.zoom *= factor
    
    def _do_zoom_in(self, event):
        """Zoom in (Linux)"""
        self.zoom *= 1.1
    
    def _do_zoom_out(self, event):
        """Zoom out (Linux)"""
        self.zoom *= 0.9
    
    def _fit_map_to_view(self):
        """Center and fit map to view"""
        # Get bounds from nodes
        if not NODES:
            return
        
        xs = [n["x"] for n in NODES.values()]
        ys = [n["y"] for n in NODES.values()]
        
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        
        # Calculate center
        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2
        
        # Reset pan to center on nodes
        self.pan_x = 0
        self.pan_y = 0
        
        # Calculate appropriate zoom
        span_x = max_x - min_x
        span_y = max_y - min_y
        span = max(span_x, span_y, 500)
        
        canvas_size = min(
            self.canvas.winfo_width() or 800,
            self.canvas.winfo_height() or 600
        )
        
        self.zoom = (canvas_size * 0.7) / span


# ============================================================================
# MAIN
# ============================================================================

if __name__ == "__main__":
    root = tk.Tk()
    app = DashboardApp(root)
    root.mainloop()
