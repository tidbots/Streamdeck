#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
StreamDeck Neo panel controller (ROS1)

Features:
- Dynamic panel update via ROS topic:  /panel/cmd (std_msgs/String JSON)
- Key press events published via ROS topic: /panel/event (std_msgs/String JSON)
- State query service: /panel/get_state (std_srvs/Trigger) -> JSON in message
- Optional long-press event (default 1.0s) published as event="long"

Command JSON examples (publish to /panel/cmd):
  {"type":"key","index":0,"icon":"init-pos.png","label":"GO"}
  {"type":"screen","text":"LISTENING..."}
  {"type":"reset_key","index":0}
  {"type":"reset_all"}

Assets:
- Put icon PNGs under ./Assets/
- Font TTF under ./Assets/ (default: Roboto-Regular.ttf)

Run:
  rosrun <your_pkg> hsr_neo.py
"""

import os
import json
import time
import queue
import threading

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

from PIL import Image, ImageDraw, ImageFont

from StreamDeck.DeviceManager import DeviceManager
from StreamDeck.ImageHelpers import PILHelper
from StreamDeck.Transport.Transport import TransportError


# Folder location of image assets used by this example.
ASSETS_PATH = os.path.join(os.path.dirname(__file__), "Assets")

# ROS topics/services
TOPIC_CMD = "/panel/cmd"
TOPIC_EVENT = "/panel/event"
SRV_GET_STATE = "/panel/get_state"

# Thread-safe command queue (ROS callbacks push here; UI thread applies)
CMD_Q: "queue.Queue[dict]" = queue.Queue()

# Override style storage (key index -> style dict)
KEY_OVERRIDE = {}  # {idx: {"icon": "xxx.png", "label": "TEXT", "font": "xxx.ttf"}}
SCREEN_TEXT = None

# Key press timing (for held_ms / long press)
KEY_DOWN_T = {}  # {idx: time.time()}
LONG_PRESS_SEC = 1.0

# ROS publisher (initialized later)
panel_event_pub = None


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
    """
    Generate an image for a key with icon + label text.
    """
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
        # If icon missing, keep blank background
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
    """
    Generate an image for the Neo screen.
    """
    image = PILHelper.create_screen_image(deck)

    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, image.width, image.height), fill="black")

    if text is None:
        text = ""

    try:
        font = ImageFont.truetype(font_filename, 20)
    except Exception:
        font = ImageFont.load_default()

    # Center-ish near the bottom (Neo screen is short)
    draw.text(
        (image.width / 2, image.height - 25),
        text=text,
        font=font,
        anchor="ms",
        fill="white",
    )

    return PILHelper.to_native_screen_format(deck, image)


# --------------------------
# Style + UI update
# --------------------------

def get_key_style(deck, key: int, state: bool):
    """
    Returns style dict:
      {"name":..., "icon":..., "font":..., "label":...}
    state: True if pressed
    """

    # 1) External override has priority (dynamic update)
    if key in KEY_OVERRIDE:
        o = KEY_OVERRIDE[key]
        icon = o.get("icon") or "Released.png"
        font = o.get("font") or "Roboto-Regular.ttf"
        label = o.get("label") or ""
        return {
            "name": "override",
            "icon": _asset(icon),
            "font": _asset(font),
            "label": label,
        }

    # 2) Default style (customize as you like)
    #    Example: random icons on released; "Pressed.png" on pressed
    return {
        "name": "pressed" if state else "released",
        "icon": _asset("Pressed.png" if state else "Released.png"),
        "font": _asset("Roboto-Regular.ttf"),
        "label": f"K{key}" if not state else f"K{key}!",
    }


def update_key_image(deck, key: int, state: bool):
    """
    Refresh a single key image.
    """
    style = get_key_style(deck, key, state)
    image = render_key_image(deck, style["icon"], style["font"], style["label"])

    # Avoid concurrent access to the deck transport
    with deck:
        deck.set_key_image(key, image)


def update_all_keys(deck):
    """
    Refresh all key images as released state.
    """
    for k in range(deck.key_count()):
        update_key_image(deck, k, False)


def set_screen(deck, text: str):
    """
    Set Neo screen image (text).
    """
    global SCREEN_TEXT
    SCREEN_TEXT = text
    font = _asset("Roboto-Regular.ttf")
    img = render_screen_image(deck, font, SCREEN_TEXT)
    with deck:
        deck.set_screen_image(img)


# --------------------------
# ROS I/O
# --------------------------

def publish_panel_event(deck, key: int, event: str, is_pressed=None, held_ms=None):
    """
    Publish /panel/event as std_msgs/String JSON
    """
    global panel_event_pub
    if panel_event_pub is None:
        return

    deck_id = None
    try:
        # Some StreamDeck implementations provide id()
        deck_id = deck.id()
    except Exception:
        deck_id = None

    msg = {
        "schema": "panel_event_v1",
        "ts": time.time(),
        "deck_id": deck_id,
        "key": int(key),
        "event": event,  # "down" / "up" / "long"
    }
    if is_pressed is not None:
        msg["is_pressed"] = bool(is_pressed)
    if held_ms is not None:
        msg["held_ms"] = int(held_ms)

    panel_event_pub.publish(String(data=json.dumps(msg)))


def ros_cmd_cb(msg: String):
    """
    Subscriber callback: push JSON command into queue (do NOT touch deck here).
    """
    try:
        cmd = json.loads(msg.data)
        if isinstance(cmd, dict):
            CMD_Q.put(cmd)
        else:
            rospy.logwarn("[panel] cmd must be JSON object")
    except Exception as e:
        rospy.logwarn(f"[panel] bad cmd json: {e} data={msg.data[:200]}")


def ros_get_state(_req):
    """
    Service: returns JSON string in TriggerResponse.message
    """
    state = {
        "key_override": KEY_OVERRIDE,
        "screen_text": SCREEN_TEXT,
        "long_press_sec": LONG_PRESS_SEC,
    }
    return TriggerResponse(success=True, message=json.dumps(state))


# --------------------------
# Command application (UI thread)
# --------------------------

def apply_command(deck, cmd: dict):
    """
    Apply a single command dict. Must be called from the UI thread (the one touching deck).
    """
    global SCREEN_TEXT

    t = cmd.get("type")

    if t == "key":
        idx = int(cmd["index"])
        KEY_OVERRIDE[idx] = {
            "icon": cmd.get("icon"),
            "label": cmd.get("label", ""),
            "font": cmd.get("font", "Roboto-Regular.ttf"),
        }
        update_key_image(deck, idx, False)

    elif t == "screen":
        text = cmd.get("text", "")
        set_screen(deck, text)

    elif t == "reset_key":
        idx = int(cmd["index"])
        KEY_OVERRIDE.pop(idx, None)
        update_key_image(deck, idx, False)

    elif t == "reset_all":
        KEY_OVERRIDE.clear()
        update_all_keys(deck)

    else:
        rospy.logwarn(f"[panel] unknown cmd type: {t}")


def command_pump_loop(deck):
    """
    UI thread loop: drains CMD_Q and updates deck.
    """
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        try:
            cmd = CMD_Q.get_nowait()
        except queue.Empty:
            rate.sleep()
            continue

        try:
            apply_command(deck, cmd)
        except Exception as e:
            rospy.logwarn(f"[panel] apply error: {e} cmd={cmd}")


# --------------------------
# StreamDeck callback
# --------------------------

def key_change_callback(deck, key, state):
    """
    Called by StreamDeck library when key changes state.
    state: True (down) / False (up)
    """
    # Keep the visual feedback consistent with your own scheme:
    # If you want "pressed look" even when override exists, you can modify get_key_style().
    update_key_image(deck, key, state)

    now = time.time()
    if state:  # down
        KEY_DOWN_T[key] = now
        publish_panel_event(deck, key, event="down", is_pressed=True)
    else:      # up
        t0 = KEY_DOWN_T.pop(key, None)
        held_ms = int((now - t0) * 1000) if t0 else None
        publish_panel_event(deck, key, event="up", is_pressed=False, held_ms=held_ms)

        # Long press (decide on release)
        if t0 and (now - t0) >= LONG_PRESS_SEC:
            publish_panel_event(deck, key, event="long", held_ms=held_ms)


# --------------------------
# Main
# --------------------------

def main():
    global panel_event_pub

    rospy.init_node("panel_controller", anonymous=True)
    panel_event_pub = rospy.Publisher(TOPIC_EVENT, String, queue_size=50)
    rospy.Subscriber(TOPIC_CMD, String, ros_cmd_cb, queue_size=50)
    rospy.Service(SRV_GET_STATE, Trigger, ros_get_state)

    streamdecks = DeviceManager().enumerate()

    if not streamdecks:
        rospy.logerr("No StreamDeck found.")
        return

    deck = streamdecks[0]
    deck.open()
    deck.reset()

    rospy.loginfo(f"Connected to {deck.deck_type()} (keys={deck.key_count()})")

    # Brightness (0-100)
    try:
        deck.set_brightness(40)
    except Exception:
        pass

    # Initialize keys
    update_all_keys(deck)

    # Register callback function for when a key state changes.
    deck.set_key_callback(key_change_callback)

    # Initialize screen text
    set_screen(deck, "<- YES    NO ->")

    # Start UI command pump thread
    threading.Thread(target=command_pump_loop, args=(deck,), daemon=True).start()

    # Keep alive until shutdown
    rospy.loginfo("panel_controller running. Ctrl-C to stop.")
    try:
        rospy.spin()
    finally:
        try:
            deck.reset()
            deck.close()
        except Exception:
            pass


if __name__ == "__main__":
    try:
        main()
    except (TransportError, RuntimeError):
        # StreamDeck transport may throw on disconnect
        pass

