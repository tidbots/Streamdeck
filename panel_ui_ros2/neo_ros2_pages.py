#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
StreamDeck Neo panel controller (ROS2 Humble) — Page-enabled COMPLETE VERSION

Features
- Dynamic panel update via ROS2 topic:   /panel/cmd   (std_msgs/msg/String JSON)
- Key press events via ROS2 topic:       /panel/event (std_msgs/msg/String JSON)
- State query service:                  /panel/get_state (std_srvs/srv/Trigger) -> JSON in response.message
- Optional long-press event (default 1.0s) published as event="long"
- Page (key layout set) support:
    - PAGES[name][key_index] = {"icon": "...png", "label": "...", "font": "...ttf"}
    - Priority: key_override > current_page > default
    - Commands: page, page_define, page_patch, page_delete

Command JSON examples (publish to /panel/cmd):
  {"type":"page_define","name":"home","keys":{"0":{"icon":"gpsr.png","label":"GPSR"}}}
  {"type":"page","name":"home"}
  {"type":"page_patch","name":"home","keys":{"0":{"label":"GPSR*"}}}
  {"type":"key","index":0,"icon":"ok.png","label":"GO"}           # key override
  {"type":"reset_key","index":0}                                  # remove key override
  {"type":"reset_all"}                                            # remove all key overrides
  {"type":"screen","text":"LISTENING..."}                         # Neo screen text

Assets
- Put icon PNGs under ./Assets/
- Font TTF under ./Assets/ (default: Roboto-Regular.ttf)

Run (example):
  ros2 run <your_pkg> hsr_neo_ros2_pages.py
"""

import os
import json
import time
import queue
import threading
from typing import Optional, Dict, Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from PIL import Image, ImageDraw, ImageFont

from StreamDeck.DeviceManager import DeviceManager
from StreamDeck.ImageHelpers import PILHelper
from StreamDeck.Transport.Transport import TransportError


# ---- Paths / Defaults ----

ASSETS_PATH = os.path.join(os.path.dirname(__file__), "Assets")

DEFAULT_TOPIC_CMD = "/panel/cmd"
DEFAULT_TOPIC_EVENT = "/panel/event"
DEFAULT_SRV_GET_STATE = "/panel/get_state"

DEFAULT_FONT = "Roboto-Regular.ttf"
DEFAULT_ICON_RELEASED = "Released.png"
DEFAULT_ICON_PRESSED = "Pressed.png"


# ---- Shared state (updated by UI thread only; read by UI rendering) ----

# Thread-safe command queue (ROS callbacks push here; UI thread applies)
CMD_Q: "queue.Queue[dict]" = queue.Queue()

# Key override styles (highest priority)
# {idx: {"icon": "...", "label": "...", "font": "..."}}
KEY_OVERRIDE: Dict[int, Dict[str, Any]] = {}

# Page definitions and current page
# PAGES[name] = { idx: {"icon": "...", "label": "...", "font": "..."} }
PAGES: Dict[str, Dict[int, Dict[str, Any]]] = {}
CURRENT_PAGE: str = "home"

# Screen state
SCREEN_TEXT: Optional[str] = None

# Key press timing (for held_ms / long press)
KEY_DOWN_T: Dict[int, float] = {}


# --------------------------
# Rendering helpers
# --------------------------

def _asset(path_or_name: str) -> str:
    """Return absolute path under Assets if a relative name is given."""
    if not path_or_name:
        return ""
    if os.path.isabs(path_or_name):
        return path_or_name
    return os.path.join(ASSETS_PATH, path_or_name)


def render_key_image(deck, icon_filename: str, font_filename: str, label_text: str):
    """Generate an image for a key with icon + label text."""
    image = PILHelper.create_image(deck)

    # Background black
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, image.width, image.height), fill="black")

    # Icon
    try:
        icon = Image.open(icon_filename).convert("RGBA")
        icon.thumbnail((image.width, image.height - 20), Image.LANCZOS)
        icon_x = (image.width - icon.width) // 2
        icon_y = 0
        image.paste(icon, (icon_x, icon_y), icon)
    except Exception:
        pass

    # Label
    if label_text is None:
        label_text = ""
    try:
        font = ImageFont.truetype(font_filename, 14)
    except Exception:
        font = ImageFont.load_default()

    draw = ImageDraw.Draw(image)
    draw.text(
        (image.width / 2, image.height - 5),
        text=label_text,
        font=font,
        anchor="ms",
        fill="white",
    )

    return PILHelper.to_native_key_format(deck, image)


def render_screen_image(deck, font_filename: str, text: str):
    """Generate an image for the Neo screen."""
    image = PILHelper.create_screen_image(deck)

    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, image.width, image.height), fill="black")

    if text is None:
        text = ""

    try:
        font = ImageFont.truetype(font_filename, 20)
    except Exception:
        font = ImageFont.load_default()

    draw.text(
        (image.width / 2, image.height - 25),
        text=text,
        font=font,
        anchor="ms",
        fill="white",
    )

    return PILHelper.to_native_screen_format(deck, image)


# --------------------------
# Style resolution + UI update
# --------------------------

def get_key_style(deck, key: int, state: bool):
    """
    Returns style dict:
      {"name":..., "icon":..., "font":..., "label":...}
    Priority: KEY_OVERRIDE > CURRENT_PAGE (PAGES) > DEFAULT
    """
    # 1) Key override (highest priority)
    if key in KEY_OVERRIDE:
        o = KEY_OVERRIDE[key]
        icon = o.get("icon") or DEFAULT_ICON_RELEASED
        font = o.get("font") or DEFAULT_FONT
        label = o.get("label") or ""
        return {"name": "override", "icon": _asset(icon), "font": _asset(font), "label": label}

    # 2) Page style (second priority)
    page = PAGES.get(CURRENT_PAGE, {})
    if key in page:
        o = page[key]
        icon = o.get("icon") or DEFAULT_ICON_RELEASED
        font = o.get("font") or DEFAULT_FONT
        label = o.get("label") or ""
        return {"name": f"page:{CURRENT_PAGE}", "icon": _asset(icon), "font": _asset(font), "label": label}

    # 3) Default style
    return {
        "name": "pressed" if state else "released",
        "icon": _asset(DEFAULT_ICON_PRESSED if state else DEFAULT_ICON_RELEASED),
        "font": _asset(DEFAULT_FONT),
        "label": f"K{key}" if not state else f"K{key}!",
    }


def update_key_image(deck, key: int, state: bool):
    """Refresh a single key image."""
    style = get_key_style(deck, key, state)
    image = render_key_image(deck, style["icon"], style["font"], style["label"])
    with deck:
        deck.set_key_image(key, image)


def update_all_keys(deck):
    """Refresh all key images as released state."""
    for k in range(deck.key_count()):
        update_key_image(deck, k, False)


def set_screen(deck, text: str):
    """Set Neo screen image (text)."""
    global SCREEN_TEXT
    SCREEN_TEXT = text
    font = _asset(DEFAULT_FONT)
    img = render_screen_image(deck, font, SCREEN_TEXT)
    with deck:
        deck.set_screen_image(img)


def switch_page(deck, name: str):
    """Switch CURRENT_PAGE and redraw all keys."""
    global CURRENT_PAGE
    CURRENT_PAGE = name
    update_all_keys(deck)


# --------------------------
# Node
# --------------------------

class PanelControllerNode(Node):
    def __init__(self):
        super().__init__("panel_controller")

        # Parameters (ROS2-style)
        self.declare_parameter("topic_cmd", DEFAULT_TOPIC_CMD)
        self.declare_parameter("topic_event", DEFAULT_TOPIC_EVENT)
        self.declare_parameter("srv_get_state", DEFAULT_SRV_GET_STATE)
        self.declare_parameter("brightness", 40)
        self.declare_parameter("long_press_sec", 1.0)
        self.declare_parameter("startup_screen_text", "<- YES    NO ->")
        self.declare_parameter("startup_page", "home")

        self.topic_cmd = self.get_parameter("topic_cmd").get_parameter_value().string_value
        self.topic_event = self.get_parameter("topic_event").get_parameter_value().string_value
        self.srv_get_state = self.get_parameter("srv_get_state").get_parameter_value().string_value

        self.brightness = int(self.get_parameter("brightness").value)
        self.long_press_sec = float(self.get_parameter("long_press_sec").value)
        self.startup_screen_text = str(self.get_parameter("startup_screen_text").value)
        self.startup_page = str(self.get_parameter("startup_page").value)

        # ROS pub/sub/srv
        self.pub_event = self.create_publisher(String, self.topic_event, 50)
        self.sub_cmd = self.create_subscription(String, self.topic_cmd, self._ros_cmd_cb, 50)
        self.srv_state = self.create_service(Trigger, self.srv_get_state, self._ros_get_state)

        # StreamDeck
        self.deck = None
        self._ui_thread = None
        self._stopping = threading.Event()

        # Ensure there is at least a "home" page (can be empty)
        if "home" not in PAGES:
            PAGES["home"] = {}

        self._open_deck_or_raise()

        # Startup page + screen
        if self.startup_page not in PAGES:
            PAGES[self.startup_page] = {}
        switch_page(self.deck, self.startup_page)
        set_screen(self.deck, self.startup_screen_text)

        self._start_ui_pump()
        self.get_logger().info("panel_controller (ROS2) with pages is running.")

    # ---- ROS callbacks ----

    def _ros_cmd_cb(self, msg: String):
        """Subscriber callback: push JSON command into queue (do NOT touch deck here)."""
        try:
            cmd = json.loads(msg.data)
            if isinstance(cmd, dict):
                CMD_Q.put(cmd)
            else:
                self.get_logger().warn("cmd must be JSON object")
        except Exception as e:
            self.get_logger().warn(f"bad cmd json: {e} data={msg.data[:200]}")

    def _ros_get_state(self, _req: Trigger.Request, res: Trigger.Response):
        """Service: returns JSON string in response.message"""
        state = {
            "current_page": CURRENT_PAGE,
            "pages": PAGES,
            "key_override": KEY_OVERRIDE,
            "screen_text": SCREEN_TEXT,
            "long_press_sec": self.long_press_sec,
        }
        res.success = True
        res.message = json.dumps(state)
        return res

    # ---- Event publish ----

    def _publish_panel_event(self, key: int, event: str, is_pressed=None, held_ms=None):
        deck_id = None
        try:
            deck_id = self.deck.id()
        except Exception:
            deck_id = None

        payload = {
            "schema": "panel_event_v1",
            "ts": time.time(),
            "deck_id": deck_id,
            "key": int(key),
            "event": event,  # "down" / "up" / "long"
            "page": CURRENT_PAGE,
        }
        if is_pressed is not None:
            payload["is_pressed"] = bool(is_pressed)
        if held_ms is not None:
            payload["held_ms"] = int(held_ms)

        self.pub_event.publish(String(data=json.dumps(payload)))

    # ---- StreamDeck callback ----

    def _key_change_callback(self, deck, key, state):
        """
        Called by StreamDeck library when key changes state.
        state: True (down) / False (up)
        """
        # Visual feedback
        update_key_image(deck, key, state)

        now = time.time()
        if state:  # down
            KEY_DOWN_T[key] = now
            self._publish_panel_event(key, event="down", is_pressed=True)
        else:      # up
            t0 = KEY_DOWN_T.pop(key, None)
            held_ms = int((now - t0) * 1000) if t0 else None
            self._publish_panel_event(key, event="up", is_pressed=False, held_ms=held_ms)

            # Long press (decide on release)
            if t0 and (now - t0) >= self.long_press_sec:
                self._publish_panel_event(key, event="long", held_ms=held_ms)

    # ---- UI command pump ----

    def _apply_command(self, cmd: dict):
        """Apply a single command dict. Must be called from the UI thread."""
        t = cmd.get("type")

        # ---------- Basic ----------
        if t == "key":
            idx = int(cmd["index"])
            KEY_OVERRIDE[idx] = {
                "icon": cmd.get("icon"),
                "label": cmd.get("label", ""),
                "font": cmd.get("font", DEFAULT_FONT),
            }
            update_key_image(self.deck, idx, False)

        elif t == "screen":
            text = cmd.get("text", "")
            set_screen(self.deck, text)

        elif t == "reset_key":
            idx = int(cmd["index"])
            KEY_OVERRIDE.pop(idx, None)
            update_key_image(self.deck, idx, False)

        elif t == "reset_all":
            KEY_OVERRIDE.clear()
            update_all_keys(self.deck)

        # ---------- Pages ----------
        elif t == "page":
            name = str(cmd.get("name", "home"))
            if name not in PAGES:
                PAGES[name] = {}
            switch_page(self.deck, name)

        elif t == "page_define":
            name = str(cmd["name"])
            keys = cmd.get("keys", {})
            page_map: Dict[int, Dict[str, Any]] = {}
            for k_str, style in keys.items():
                page_map[int(k_str)] = dict(style)
            PAGES[name] = page_map
            if name == CURRENT_PAGE:
                update_all_keys(self.deck)

        elif t == "page_patch":
            name = str(cmd["name"])
            keys = cmd.get("keys", {})
            if name not in PAGES:
                PAGES[name] = {}
            for k_str, style in keys.items():
                k = int(k_str)
                if k not in PAGES[name]:
                    PAGES[name][k] = {}
                PAGES[name][k].update(dict(style))
            if name == CURRENT_PAGE:
                update_all_keys(self.deck)

        elif t == "page_delete":
            name = str(cmd["name"])
            PAGES.pop(name, None)
            if name == CURRENT_PAGE:
                switch_page(self.deck, "home")

        else:
            self.get_logger().warn(f"unknown cmd type: {t}")

    def _ui_pump_loop(self):
        """UI thread loop: drains CMD_Q and updates deck."""
        while not self._stopping.is_set():
            try:
                cmd = CMD_Q.get(timeout=0.05)
            except queue.Empty:
                continue
            try:
                self._apply_command(cmd)
            except Exception as e:
                self.get_logger().warn(f"apply error: {e} cmd={cmd}")

    def _start_ui_pump(self):
        self._ui_thread = threading.Thread(target=self._ui_pump_loop, daemon=True)
        self._ui_thread.start()

    # ---- StreamDeck open/close ----

    def _open_deck_or_raise(self):
        streamdecks = DeviceManager().enumerate()
        if not streamdecks:
            raise RuntimeError("No StreamDeck found.")

        self.deck = streamdecks[0]
        self.deck.open()
        self.deck.reset()

        self.get_logger().info(f"Connected to {self.deck.deck_type()} (keys={self.deck.key_count()})")

        try:
            self.deck.set_brightness(self.brightness)
        except Exception:
            pass

        # Initialize keys (will be redrawn again when switching startup page)
        update_all_keys(self.deck)

        # Register callback for key changes
        self.deck.set_key_callback(self._key_change_callback)

    def destroy_node(self):
        """Ensure StreamDeck is closed on shutdown."""
        self._stopping.set()
        try:
            if self.deck is not None:
                try:
                    self.deck.reset()
                except Exception:
                    pass
                try:
                    self.deck.close()
                except Exception:
                    pass
        finally:
            super().destroy_node()


# --------------------------
# Main
# --------------------------

def main():
    rclpy.init()
    node = None
    try:
        node = PanelControllerNode()
        rclpy.spin(node)
    except (TransportError, RuntimeError) as e:
        if node is not None:
            node.get_logger().error(str(e))
        else:
            print(e)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

