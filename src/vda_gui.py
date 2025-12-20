# ============================================================================
# vda_gui.py - VDA 5050 Robot Dashboard GUI
# ============================================================================
#
# Features:
#   - Path Builder Mode: Click nodes to build path, Enter to execute
#   - Horizon Mode: Visualize base/horizon, manual release
#   - Current position detection on startup
#   - Pan/Zoom with mouse
#
# ============================================================================

import tkinter as tk
from tkinter import ttk
import json
import math
import os
from vda_config import *
from vda_backend import RobotBackend


class DashboardApp:
    def __init__(self, root):
        self.root = root
        self.root.title("VDA 5050 Robot Dashboard")
        self.root.geometry("1400x900")
        self.root.configure(bg=COLOR_BG)

        # Initialize log_text as None first (for early log_message calls)
        self.log_text = None

        # Create backend (may call log_message)
        self.backend = RobotBackend(MQTT_BROKER, MQTT_PORT, self.log_message)

        # Load map
        self.map_data = self.load_map_file()

        # Initialize pan/zoom variables BEFORE setup_ui
        self.pan_x = 0
        self.pan_y = 0
        self.scale = 1.0
        self.zoom = 1.0
        self._pan_start_x = 0
        self._pan_start_y = 0
        self._is_dragging = False

        # ============================================================
        # PATH BUILDER STATE
        # ============================================================
        self.path_builder_mode = True  # Start in path builder mode
        self.selected_path = []  # List of node IDs user has clicked
        self.hover_node = None  # Node currently being hovered

        # ============================================================
        # HORIZON MODE STATE
        # ============================================================
        self.horizon_mode = False
        self.base_nodes = []  # Nodes in base (released)
        self.horizon_nodes = []  # Nodes in horizon (not released)

        # Now setup UI (creates canvas)
        self.setup_ui()

        # Bind keyboard events
        self.root.bind("<Return>", self.execute_path)
        self.root.bind("<Escape>", self.clear_path)
        self.root.bind("<BackSpace>", self.undo_last_node)

        # Fit map AFTER canvas is created
        self.root.update_idletasks()
        self.fit_map_to_view()

        self.backend.start()
        self.root.after(50, self.ui_loop)

    def load_map_file(self):
        """Load and parse Valetudo map JSON file."""
        data = {"lines": [], "charger": None, "pixel_size": 5, "bounds": None}

        if os.path.exists("valetudo_map.json"):
            try:
                with open("valetudo_map.json", "r") as f:
                    raw = json.load(f)

                pixel_size = raw.get("pixelSize", 5)
                data["pixel_size"] = pixel_size

                for entity in raw.get("entities", []):
                    if entity.get("type") == "charger_location":
                        points = entity.get("points", [])
                        if len(points) >= 2:
                            data["charger"] = (points[0], points[1])

                min_x, max_x, min_y, max_y = float('inf'), 0, float('inf'), 0

                for layer in raw.get("layers", []):
                    if layer.get("type") == "segment":
                        compressed = layer.get("compressedPixels", [])
                        i = 0
                        while i + 2 < len(compressed):
                            x = compressed[i]
                            y = compressed[i + 1]
                            count = compressed[i + 2]
                            x_start = x * pixel_size
                            y_pos = y * pixel_size
                            x_end = (x + count) * pixel_size
                            data["lines"].append((x_start, y_pos, x_end, y_pos))
                            min_x = min(min_x, x_start)
                            max_x = max(max_x, x_end)
                            min_y = min(min_y, y_pos)
                            max_y = max(max_y, y_pos)
                            i += 3

                data["bounds"] = (min_x, min_y, max_x, max_y)
                print(f"✓ Loaded map: {len(data['lines'])} line segments")

            except Exception as e:
                print(f"✗ Error loading map: {e}")

        return data

    def setup_ui(self):
        main = tk.Frame(self.root, bg=COLOR_BG)
        main.pack(fill=tk.BOTH, expand=True)

        # Canvas for map
        self.canvas = tk.Canvas(main, bg=COLOR_BG, highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Mouse bindings
        self.canvas.bind("<ButtonPress-1>", self.on_click_press)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_click_release)
        self.canvas.bind("<Motion>", self.on_mouse_move)

        # Middle mouse for pan
        self.canvas.bind("<ButtonPress-2>", self.start_pan)
        self.canvas.bind("<B2-Motion>", self.do_pan)

        # Zoom
        self.canvas.bind("<MouseWheel>", self.do_zoom)
        self.canvas.bind("<Button-4>", self.do_zoom_in)
        self.canvas.bind("<Button-5>", self.do_zoom_out)

        # Sidebar
        sidebar = tk.Frame(main, bg=COLOR_BG, width=420)
        sidebar.pack(side=tk.RIGHT, fill=tk.Y, padx=10, pady=10)
        sidebar.pack_propagate(False)

        self.create_sidebar(sidebar)

    def create_sidebar(self, sidebar):
        # ============================================================
        # ROBOT STATUS - Compact
        # ============================================================
        status_frame = tk.LabelFrame(sidebar, text="ROBOT STATUS", bg=COLOR_BG, fg=COLOR_ACCENT,
                                     font=("Arial", 12, "bold"))
        status_frame.pack(fill=tk.X, pady=3)

        self.vars = {
            "pos": tk.StringVar(value="--"),
            "theta": tk.StringVar(value="--"),
            "batt": tk.StringVar(value="--"),
            "status": tk.StringVar(value="Connecting..."),
            "current_node": tk.StringVar(value="--"),
            "order": tk.StringVar(value="--")
        }

        labels = [
            ("Position:", "pos"),
            ("Theta:", "theta"),
            ("Battery:", "batt"),
            ("Status:", "status"),
            ("Current Node:", "current_node"),
            ("Order:", "order")
        ]
        for label, var in labels:
            row = tk.Frame(status_frame, bg=COLOR_BG)
            row.pack(fill=tk.X, padx=10, pady=1)
            tk.Label(row, text=label, bg=COLOR_BG, fg="white", font=("Arial", 9), width=12, anchor="w").pack(
                side=tk.LEFT)
            tk.Label(row, textvariable=self.vars[var], bg=COLOR_BG, fg=COLOR_ACCENT,
                     font=("Arial", 9, "bold")).pack(side=tk.LEFT)

        # ============================================================
        # PATH BUILDER - Compact
        # ============================================================
        path_frame = tk.LabelFrame(sidebar, text="PATH BUILDER", bg=COLOR_BG, fg="#FF9800",
                                   font=("Arial", 12, "bold"))
        path_frame.pack(fill=tk.X, pady=3)

        # Instructions
        tk.Label(path_frame, text="Click nodes to build path, Enter to execute",
                 bg=COLOR_BG, fg="#888", font=("Arial", 8)).pack(pady=1)

        # Path display
        self.path_display = tk.Label(path_frame, text="No path selected",
                                     bg="#1a1a1a", fg="#FF9800", font=("Consolas", 9),
                                     wraplength=380, justify=tk.LEFT, padx=8, pady=5)
        self.path_display.pack(fill=tk.X, padx=10, pady=3)

        # Horizon split slider
        horizon_row = tk.Frame(path_frame, bg=COLOR_BG)
        horizon_row.pack(fill=tk.X, padx=10, pady=1)

        tk.Label(horizon_row, text="Base nodes:", bg=COLOR_BG, fg="#888",
                 font=("Arial", 8)).pack(side=tk.LEFT)

        self.horizon_split = tk.IntVar(value=99)  # Default: all nodes in base
        self.horizon_slider = tk.Scale(horizon_row, from_=1, to=10, orient=tk.HORIZONTAL,
                                       variable=self.horizon_split, bg=COLOR_BG, fg="white",
                                       highlightthickness=0, length=120, showvalue=True,
                                       command=self.on_horizon_slider_change)
        self.horizon_slider.pack(side=tk.LEFT, padx=3)

        self.horizon_label = tk.Label(horizon_row, text="(all released)", bg=COLOR_BG,
                                      fg="#4CAF50", font=("Arial", 8))
        self.horizon_label.pack(side=tk.LEFT)

        # Path builder buttons
        btn_frame = tk.Frame(path_frame, bg=COLOR_BG)
        btn_frame.pack(fill=tk.X, padx=10, pady=3)

        self.execute_btn = tk.Button(btn_frame, text="▶ Execute (Enter)", command=self.execute_path_click,
                                     bg="#4CAF50", fg="white", font=("Arial", 9, "bold"), width=14)
        self.execute_btn.pack(side=tk.LEFT, padx=2)

        tk.Button(btn_frame, text="✕ Clear", command=self.clear_path_click,
                  bg="#666", fg="white", font=("Arial", 9), width=8).pack(side=tk.LEFT, padx=2)

        tk.Button(btn_frame, text="↩ Undo", command=self.undo_last_node_click,
                  bg="#666", fg="white", font=("Arial", 9), width=7).pack(side=tk.LEFT, padx=2)

        # ============================================================
        # MISSIONS (Predefined) - All on one row, compact
        # ============================================================
        mission_frame = tk.LabelFrame(sidebar, text="QUICK MISSIONS", bg=COLOR_BG, fg=COLOR_ACCENT,
                                      font=("Arial", 12, "bold"))
        mission_frame.pack(fill=tk.X, pady=3)

        mission_btn_frame = tk.Frame(mission_frame, bg=COLOR_BG)
        mission_btn_frame.pack(fill=tk.X, padx=5, pady=3)

        col = 0
        for name in MISSIONS.keys():
            btn = tk.Button(mission_btn_frame, text=name, command=lambda n=name: self.start_mission(n),
                            bg="#444", fg="white", activebackground=COLOR_ACCENT, width=9, font=("Arial", 8))
            btn.grid(row=0, column=col, padx=1, pady=1)
            col += 1

        # ============================================================
        # CONTROLS - More compact
        # ============================================================
        ctrl_frame = tk.LabelFrame(sidebar, text="CONTROLS", bg=COLOR_BG, fg=COLOR_ACCENT,
                                   font=("Arial", 12, "bold"))
        ctrl_frame.pack(fill=tk.X, pady=3)

        ctrl_btn_frame = tk.Frame(ctrl_frame, bg=COLOR_BG)
        ctrl_btn_frame.pack(fill=tk.X, padx=10, pady=3)

        tk.Button(ctrl_btn_frame, text="⏸ Pause", command=self.backend.pause_mission,
                  bg="#FFA000", fg="black", width=9, font=("Arial", 9)).pack(side=tk.LEFT, padx=2)
        tk.Button(ctrl_btn_frame, text="▶ Resume", command=self.backend.resume_mission,
                  bg="#4CAF50", fg="white", width=9, font=("Arial", 9)).pack(side=tk.LEFT, padx=2)
        tk.Button(ctrl_btn_frame, text="⏹ Stop", command=self.stop_and_clear,
                  bg="#F44336", fg="white", width=9, font=("Arial", 9)).pack(side=tk.LEFT, padx=2)

        # ============================================================
        # HORIZON MODE (VDA 5050 Base/Horizon) - Compact
        # ============================================================
        horizon_frame = tk.LabelFrame(sidebar, text="HORIZON MODE", bg=COLOR_BG, fg="#4444AA",
                                      font=("Arial", 12, "bold"))
        horizon_frame.pack(fill=tk.X, pady=3)

        horizon_inner = tk.Frame(horizon_frame, bg=COLOR_BG)
        horizon_inner.pack(fill=tk.X, padx=10, pady=3)

        self.horizon_status = tk.Label(horizon_inner, text="No active order",
                                       bg=COLOR_BG, fg="#888", font=("Arial", 9))
        self.horizon_status.pack(side=tk.LEFT)

        tk.Button(horizon_inner, text="Release Next", command=self.release_next_horizon,
                  bg="#4444AA", fg="white", width=12, font=("Arial", 9)).pack(side=tk.RIGHT, padx=2)

        # ============================================================
        # LOG - Increased height
        # ============================================================
        log_frame = tk.LabelFrame(sidebar, text="LOG", bg=COLOR_BG, fg=COLOR_ACCENT,
                                  font=("Arial", 12, "bold"))
        log_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self.log_text = tk.Text(log_frame, bg="#1a1a1a", fg="white", font=("Consolas", 9),
                                height=18, state='disabled', wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.log_text.tag_config("info", foreground="#888")
        self.log_text.tag_config("success", foreground="#4CAF50")
        self.log_text.tag_config("warning", foreground="#FFA000")
        self.log_text.tag_config("error", foreground="#F44336")

    # ========================================================================
    # LOGGING
    # ========================================================================

    def log_message(self, msg, level="info"):
        if self.log_text is None:
            print(f"[{level}] {msg}")
            return
        self.log_text.configure(state='normal')
        self.log_text.insert(tk.END, f"> {msg}\n", level)
        self.log_text.see(tk.END)
        self.log_text.configure(state='disabled')

    # ========================================================================
    # PATH BUILDER
    # ========================================================================

    def add_node_to_path(self, node_id):
        """Add a node to the path builder"""
        # If path is empty or adding a valid next node
        if not self.selected_path:
            # First node - can be any node
            self.selected_path.append(node_id)
            self.log_message(f"Path start: {node_id}", "info")
        else:
            # Check if this node connects to the last node via an edge
            last_node = self.selected_path[-1]
            if self._nodes_connected(last_node, node_id):
                if node_id not in self.selected_path:  # Avoid loops for now
                    self.selected_path.append(node_id)
                    self.log_message(f"Added: {node_id}", "info")
                else:
                    self.log_message(f"Node {node_id} already in path", "warning")
            else:
                self.log_message(f"No edge from {last_node} to {node_id}", "warning")

        self.update_path_display()

    def _nodes_connected(self, node1, node2):
        """Check if two nodes are connected by an edge"""
        for edge in EDGES:
            if (edge["start"] == node1 and edge["end"] == node2) or \
                    (edge["start"] == node2 and edge["end"] == node1):
                return True
        return False

    def update_path_display(self):
        """Update the path display label"""
        if not self.selected_path:
            self.path_display.config(text="No path selected")
            self.execute_btn.config(state=tk.DISABLED)
            self.horizon_slider.config(to=10)
        else:
            # Update slider max to path length
            path_len = len(self.selected_path)
            self.horizon_slider.config(to=path_len)
            if self.horizon_split.get() > path_len:
                self.horizon_split.set(path_len)

            # Build display with base/horizon coloring
            split = min(self.horizon_split.get(), path_len)
            base_nodes = self.selected_path[:split]
            horizon_nodes = self.selected_path[split:]

            if horizon_nodes:
                path_str = " → ".join(base_nodes) + " │ " + " → ".join(horizon_nodes)
            else:
                path_str = " → ".join(base_nodes)

            self.path_display.config(text=path_str)
            self.execute_btn.config(state=tk.NORMAL)

            # Update horizon label
            self.on_horizon_slider_change(None)

    def on_horizon_slider_change(self, value):
        """Update horizon label when slider changes"""
        if not self.selected_path:
            self.horizon_label.config(text="(all released)", fg="#4CAF50")
            return

        path_len = len(self.selected_path)
        split = min(self.horizon_split.get(), path_len)
        horizon_count = path_len - split

        if horizon_count == 0:
            self.horizon_label.config(text="(all released)", fg="#4CAF50")
        else:
            self.horizon_label.config(text=f"({horizon_count} in horizon)", fg="#4444AA")

        # Update display
        if self.selected_path:
            base_nodes = self.selected_path[:split]
            horizon_nodes = self.selected_path[split:]

            if horizon_nodes:
                path_str = " → ".join(base_nodes) + " │ " + " → ".join(horizon_nodes)
            else:
                path_str = " → ".join(base_nodes)

            self.path_display.config(text=path_str)

    def execute_path(self, event=None):
        """Execute the built path with base/horizon split"""
        if len(self.selected_path) < 1:
            self.log_message("No path to execute", "warning")
            return

        # Get current position
        current_node = self._get_current_node()

        # Get horizon split
        path_len = len(self.selected_path)
        split = min(self.horizon_split.get(), path_len)

        if len(self.selected_path) == 1:
            # Single node selected - go to it (no horizon concept)
            target = self.selected_path[0]
            self.log_message(f"Navigate to: {target}", "success")
            self.backend.send_goto_node(target)
        else:
            # Multi-node path with base/horizon
            base_nodes = self.selected_path[:split]
            horizon_nodes = self.selected_path[split:]

            if horizon_nodes:
                self.log_message(f"Base: {' → '.join(base_nodes)}", "success")
                self.log_message(f"Horizon: {' → '.join(horizon_nodes)}", "warning")
            else:
                self.log_message(f"Execute: {' → '.join(self.selected_path)}", "success")

            # Send mission with horizon split
            self.backend.set_mission_with_horizon(self.selected_path, split)

        # Clear path after execution
        self.selected_path = []
        self.update_path_display()

    def execute_path_click(self):
        self.execute_path()

    def clear_path(self, event=None):
        """Clear the selected path"""
        self.selected_path = []
        self.update_path_display()
        self.log_message("Path cleared", "info")

    def clear_path_click(self):
        self.clear_path()

    def undo_last_node(self, event=None):
        """Remove the last node from the path"""
        if self.selected_path:
            removed = self.selected_path.pop()
            self.log_message(f"Removed: {removed}", "info")
            self.update_path_display()

    def undo_last_node_click(self):
        self.undo_last_node()

    def _get_current_node(self):
        """Get the robot's current node (from backend or by position)"""
        if self.backend.last_node_id:
            return self.backend.last_node_id

        # Find nearest node based on position
        return self.backend._find_nearest_node()

    # ========================================================================
    # MISSIONS
    # ========================================================================

    def start_mission(self, name):
        """Start a predefined mission"""
        waypoints = MISSIONS[name]
        self.log_message(f"Mission '{name}': {' → '.join(waypoints)}", "info")
        self.backend.send_mission(waypoints)

    def stop_and_clear(self):
        """Stop mission and clear path builder"""
        self.backend.emergency_stop()
        self.selected_path = []
        self.update_path_display()

    # ========================================================================
    # HORIZON MODE
    # ========================================================================

    def release_next_horizon(self):
        """Release the next horizon node"""
        if not self.backend.node_states:
            self.log_message("No active order", "info")
            return

        # Find horizon nodes (not released)
        horizon_nodes = [ns for ns in self.backend.node_states if not ns.released]

        if not horizon_nodes:
            self.log_message("No horizon nodes to release", "info")
            return

        # Release the first horizon node
        next_horizon = horizon_nodes[0]
        self.log_message(f"Releasing: {next_horizon.nodeId}", "success")

        # Call backend to release
        self.backend.release_horizon_node(next_horizon.nodeId)

    # ========================================================================
    # MOUSE EVENTS
    # ========================================================================

    def on_click_press(self, event):
        self._pan_start_x = event.x
        self._pan_start_y = event.y
        self._is_dragging = False

    def on_drag(self, event):
        dx = event.x - self._pan_start_x
        dy = event.y - self._pan_start_y
        if abs(dx) > 3 or abs(dy) > 3:
            self._is_dragging = True
            self.pan_x += dx
            self.pan_y += dy if FLIP_Y else -dy
            self._pan_start_x = event.x
            self._pan_start_y = event.y

    def on_click_release(self, event):
        """Handle click release - either pan ended or node click"""
        if self._is_dragging:
            self._is_dragging = False
            return

        # Check if clicked on a node
        for name, data in NODES.items():
            sx, sy = self.world_to_screen(data["x"], data["y"])
            if abs(event.x - sx) < 18 and abs(event.y - sy) < 18:
                self.add_node_to_path(name)
                return

    def on_mouse_move(self, event):
        """Track hover for visual feedback"""
        self.hover_node = None
        for name, data in NODES.items():
            sx, sy = self.world_to_screen(data["x"], data["y"])
            if abs(event.x - sx) < 18 and abs(event.y - sy) < 18:
                self.hover_node = name
                break

    def start_pan(self, event):
        self._pan_start_x = event.x
        self._pan_start_y = event.y

    def do_pan(self, event):
        dx = event.x - self._pan_start_x
        dy = event.y - self._pan_start_y
        self.pan_x += dx
        self.pan_y += dy if FLIP_Y else -dy
        self._pan_start_x = event.x
        self._pan_start_y = event.y

    def do_zoom(self, event):
        if event.delta > 0:
            factor = 1.15
        else:
            factor = 1 / 1.15
        self._zoom_at(event.x, event.y, factor)

    def do_zoom_in(self, event):
        self._zoom_at(event.x, event.y, 1.15)

    def do_zoom_out(self, event):
        self._zoom_at(event.x, event.y, 1 / 1.15)

    def _zoom_at(self, mouse_x, mouse_y, factor):
        new_zoom = self.zoom * factor
        if new_zoom < 0.1 or new_zoom > 10:
            return
        self.pan_x = mouse_x - (mouse_x - self.pan_x) * factor
        self.pan_y = mouse_y - (mouse_y - self.pan_y) * factor
        self.zoom = new_zoom

    # ========================================================================
    # COORDINATE TRANSFORMS
    # ========================================================================

    def world_to_screen(self, wx, wy):
        sx = (wx * self.scale * self.zoom) + self.pan_x
        if FLIP_Y:
            sy = (wy * self.scale * self.zoom) + self.pan_y
        else:
            canvas_height = self.canvas.winfo_height() if hasattr(self, 'canvas') else 600
            sy = canvas_height - ((wy * self.scale * self.zoom) + self.pan_y)
        return sx, sy

    def fit_map_to_view(self):
        xs = [n["x"] for n in NODES.values()]
        ys = [n["y"] for n in NODES.values()]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        vw, vh = 900, 700
        self.scale = min(vw / (max_x - min_x + 500), vh / (max_y - min_y + 500))
        self.zoom = 1.0
        self.pan_x = (vw - (min_x + max_x) * self.scale) / 2 + 50
        self.pan_y = (vh - (min_y + max_y) * self.scale) / 2

    # ========================================================================
    # MAIN UI LOOP
    # ========================================================================

    def ui_loop(self):
        p = self.backend.display_pose

        # Update status variables
        self.vars["pos"].set(f"{int(p['x'])}, {int(p['y'])}")
        self.vars["theta"].set(f"{p['theta']:.5f} rad")
        self.vars["batt"].set(f"{int(self.backend.battery * 100)}%")

        # Status
        if self.backend.charging:
            status = "Charging"
        elif self.backend.mission_active:
            status = "Mission Active"
        elif self.backend.is_driving:
            status = "Driving"
        else:
            status = "Idle"
        self.vars["status"].set(status)

        # Current node - show detected position
        current = self._get_current_node()
        self.vars["current_node"].set(current or "--")

        # Order
        if self.backend.current_order:
            order_id = self.backend.current_order.get("orderId", "--")
            self.vars["order"].set(order_id[:20] if len(order_id) > 20 else order_id)
        else:
            self.vars["order"].set("--")

        # Update horizon status
        if self.backend.node_states:
            base_count = sum(1 for ns in self.backend.node_states if ns.released)
            horizon_count = sum(1 for ns in self.backend.node_states if not ns.released)
            self.horizon_status.config(text=f"Base: {base_count} nodes | Horizon: {horizon_count} nodes")
        else:
            self.horizon_status.config(text="No active order")

        # Redraw canvas
        self.draw_canvas()

        self.root.after(50, self.ui_loop)

    def draw_canvas(self):
        """Draw the entire canvas"""
        self.canvas.delete("all")
        p = self.backend.display_pose

        # Draw map floor
        pixel_size = self.map_data.get("pixel_size", 5)
        for x1, y1, x2, y2 in self.map_data.get("lines", []):
            sx1, sy1 = self.world_to_screen(x1, y1)
            sx2, sy2 = self.world_to_screen(x2, y2 + pixel_size)
            self.canvas.create_rectangle(sx1, sy1, sx2, sy2, fill="#3a3a3a", outline="")

        # Draw charger
        if self.map_data.get("charger"):
            cx, cy = self.world_to_screen(*self.map_data["charger"])
            self.canvas.create_rectangle(cx - 8, cy - 8, cx + 8, cy + 8, fill="#4CAF50", outline="white")

        # Draw edges
        drawn_edges = set()
        for e in EDGES:
            if e["start"] in NODES and e["end"] in NODES:
                edge_pair = tuple(sorted([e["start"], e["end"]]))
                if edge_pair in drawn_edges:
                    continue
                drawn_edges.add(edge_pair)

                x1, y1 = self.world_to_screen(NODES[e["start"]]["x"], NODES[e["start"]]["y"])
                x2, y2 = self.world_to_screen(NODES[e["end"]]["x"], NODES[e["end"]]["y"])

                # Check if edge is part of selected path
                edge_in_path = False
                for i in range(len(self.selected_path) - 1):
                    if (self.selected_path[i] == e["start"] and self.selected_path[i + 1] == e["end"]) or \
                            (self.selected_path[i] == e["end"] and self.selected_path[i + 1] == e["start"]):
                        edge_in_path = True
                        break

                if edge_in_path:
                    edge_color = "#FF9800"  # Orange for selected path
                    edge_width = 4
                else:
                    edge_color = COLOR_EDGE_IDLE
                    edge_width = 2

                self.canvas.create_line(x1, y1, x2, y2, fill=edge_color, width=edge_width)

                # Edge label
                mid_x = (x1 + x2) / 2
                mid_y = (y1 + y2) / 2
                dx = x2 - x1
                dy = y2 - y1
                length = math.sqrt(dx * dx + dy * dy)
                if length > 0:
                    perp_x = -dy / length
                    perp_y = dx / length
                    offset = 12
                    label_x = mid_x + perp_x * offset
                    label_y = mid_y + perp_y * offset
                else:
                    label_x = mid_x
                    label_y = mid_y - 12

                label = f"{e['start']}↔{e['end']}"
                self.canvas.create_text(label_x, label_y, text=label, fill="#666", font=("Arial", 8))

        # Draw nodes
        current_node = self._get_current_node()

        # Get horizon split for path builder visualization
        path_len = len(self.selected_path)
        split = min(self.horizon_split.get(), path_len) if path_len > 0 else 0

        # Build sets for quick lookup of mission node states
        base_mission_nodes = set()
        horizon_mission_nodes = set()
        for ns in self.backend.node_states:
            if ns.released:
                base_mission_nodes.add(ns.nodeId)
            else:
                horizon_mission_nodes.add(ns.nodeId)

        for name, data in NODES.items():
            sx, sy = self.world_to_screen(data["x"], data["y"])

            # Determine node color based on various states
            # Order matters! More specific states first.
            if name in self.selected_path:
                # Node is in selected path (path builder)
                idx = self.selected_path.index(name)
                if idx < split:
                    # Base node (will be released)
                    fill = "#FF9800"  # Orange
                    outline_color = "#FF9800"
                else:
                    # Horizon node (will NOT be released yet)
                    fill = "#4444AA"  # Blue/purple for horizon
                    outline_color = "#6666CC"
                outline_width = 3
            elif name in horizon_mission_nodes:
                # Active mission - horizon node (not released)
                fill = "#4444AA"  # Blue/purple
                outline_color = "#6666CC"
                outline_width = 3
            elif name == self.hover_node:
                fill = "#555"  # Hover highlight
                outline_color = "#888"
                outline_width = 2
            elif self.backend.gui_node_states.get(name) == "done":
                # Traversed during active mission - GREEN
                fill = COLOR_NODE_DONE  # Green
                outline_color = "white"
                outline_width = 2
            elif self.backend.gui_node_states.get(name) == "planned" or name in base_mission_nodes:
                fill = COLOR_NODE_PLANNED  # Orange - mission base/planned
                outline_color = "white"
                outline_width = 2
            else:
                fill = COLOR_NODE_IDLE  # Gray - idle
                outline_color = "white"
                outline_width = 2

            # Draw node circle
            self.canvas.create_oval(sx - 12, sy - 12, sx + 12, sy + 12,
                                    fill=fill, outline=outline_color, width=outline_width)

            # Draw sequence number if in selected path
            if name in self.selected_path:
                idx = self.selected_path.index(name) + 1
                text_color = "white" if idx >= split else "black"
                self.canvas.create_text(sx, sy, text=str(idx), fill=text_color, font=("Arial", 10, "bold"))

            # Node label
            self.canvas.create_text(sx, sy - 22, text=name, fill="white", font=("Arial", 9, "bold"))

        # Draw robot
        rx, ry = self.world_to_screen(p["x"], p["y"])
        th = p["theta"]
        if not FLIP_Y:
            th = -th

        # Robot body
        self.canvas.create_oval(rx - 14, ry - 14, rx + 14, ry + 14,
                                fill=COLOR_ROBOT_BODY, outline="white", width=2)

        # Direction indicator
        th_normalized = th
        if th_normalized > math.pi:
            th_normalized = th_normalized - 2 * math.pi
        dx = rx - 10 * math.sin(th_normalized)
        dy = ry - 10 * math.cos(th_normalized)
        self.canvas.create_oval(dx - 4, dy - 4, dx + 4, dy + 4, fill=COLOR_ROBOT_DOT, outline="")

        # Draw path preview line from robot to first selected node
        if self.selected_path:
            first_node = self.selected_path[0]
            if first_node in NODES:
                nx, ny = self.world_to_screen(NODES[first_node]["x"], NODES[first_node]["y"])
                self.canvas.create_line(rx, ry, nx, ny, fill="#FF9800", width=2, dash=(5, 3))


# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    root = tk.Tk()
    app = DashboardApp(root)
    root.mainloop()