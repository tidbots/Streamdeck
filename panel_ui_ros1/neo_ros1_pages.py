#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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

ASSETS_PATH = os.path.join(os.path.dirname(__file__), "Assets")
DEFAULT_FONT = "Roboto-Regular.ttf"

CMD_Q = queue.Queue()

KEY_OVERRIDE = {}
PAGES = {"home": {}}
CURRENT_PAGE = "home"
SCREEN_TEXT = None

KEY_DOWN_T = {}
LONG_PRESS_SEC = 1.0


def asset(p):
    return p if os.path.isabs(p) else os.path.join(ASSETS_PATH, p)


def render_key(deck, icon, font, label):
    img = PILHelper.create_image(deck)
    d = ImageDraw.Draw(img)
    d.rectangle((0, 0, img.width, img.height), fill="black")

    try:
        icon_img = Image.open(icon).convert("RGBA")
        icon_img.thumbnail((img.width, img.height - 20))
        img.paste(icon_img,
                  ((img.width - icon_img.width)//2, 0),
                  icon_img)
    except Exception:
        pass

    try:
        f = ImageFont.truetype(font, 14)
    except Exception:
        f = ImageFont.load_default()

    d.text((img.width/2, img.height-5), label,
           font=f, anchor="ms", fill="white")
    return PILHelper.to_native_key_format(deck, img)


def render_screen(deck, text):
    img = PILHelper.create_screen_image(deck)
    d = ImageDraw.Draw(img)
    d.rectangle((0, 0, img.width, img.height), fill="black")
    try:
        f = ImageFont.truetype(asset(DEFAULT_FONT), 20)
    except Exception:
        f = ImageFont.load_default()
    d.text((img.width/2, img.height-25), text,
           font=f, anchor="ms", fill="white")
    return PILHelper.to_native_screen_format(deck, img)


def get_style(key, state):
    if key in KEY_OVERRIDE:
        o = KEY_OVERRIDE[key]
        return o.get("icon"), o.get("font"), o.get("label")

    page = PAGES.get(CURRENT_PAGE, {})
    if key in page:
        o = page[key]
        return o.get("icon"), o.get("font"), o.get("label")

    return (
        asset("Pressed.png" if state else "Released.png"),
        asset(DEFAULT_FONT),
        f"K{key}"
    )


def update_key(deck, key, state=False):
    icon, font, label = get_style(key, state)
    img = render_key(deck, asset(icon), asset(font), label)
    with deck:
        deck.set_key_image(key, img)


def update_all(deck):
    for k in range(deck.key_count()):
        update_key(deck, k, False)


def set_screen(deck, text):
    global SCREEN_TEXT
    SCREEN_TEXT = text
    with deck:
        deck.set_screen_image(render_screen(deck, text))


def switch_page(deck, name):
    global CURRENT_PAGE
    CURRENT_PAGE = name
    update_all(deck)


def apply_cmd(deck, cmd):
    t = cmd.get("type")

    if t == "screen":
        set_screen(deck, cmd.get("text", ""))

    elif t == "key":
        KEY_OVERRIDE[int(cmd["index"])] = {
            "icon": cmd.get("icon"),
            "font": cmd.get("font", DEFAULT_FONT),
            "label": cmd.get("label", "")
        }
        update_key(deck, int(cmd["index"]))

    elif t == "reset_key":
        KEY_OVERRIDE.pop(int(cmd["index"]), None)
        update_key(deck, int(cmd["index"]))

    elif t == "reset_all":
        KEY_OVERRIDE.clear()
        update_all(deck)

    elif t == "page":
        name = cmd.get("name", "home")
        PAGES.setdefault(name, {})
        switch_page(deck, name)

    elif t == "page_define":
        name = cmd["name"]
        PAGES[name] = {int(k): v for k, v in cmd.get("keys", {}).items()}
        if name == CURRENT_PAGE:
            update_all(deck)

    elif t == "page_patch":
        name = cmd["name"]
        PAGES.setdefault(name, {})
        for k, v in cmd.get("keys", {}).items():
            PAGES[name].setdefault(int(k), {}).update(v)
        if name == CURRENT_PAGE:
            update_all(deck)


def cmd_loop(deck):
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        try:
            cmd = CMD_Q.get_nowait()
            apply_cmd(deck, cmd)
        except queue.Empty:
            rate.sleep()


def cmd_cb(msg):
    try:
        CMD_Q.put(json.loads(msg.data))
    except Exception:
        pass


def key_cb(deck, key, state):
    update_key(deck, key, state)
    evt = {
        "key": key,
        "event": "down" if state else "up",
        "page": CURRENT_PAGE,
        "ts": time.time()
    }
    pub_event.publish(String(json.dumps(evt)))


if __name__ == "__main__":
    rospy.init_node("panel_controller")

    pub_event = rospy.Publisher("/panel/event", String, queue_size=50)
    rospy.Subscriber("/panel/cmd", String, cmd_cb)

    decks = DeviceManager().enumerate()
    deck = decks[0]
    deck.open()
    deck.reset()
    deck.set_key_callback(key_cb)

    update_all(deck)
    set_screen(deck, "READY")

    threading.Thread(target=cmd_loop, args=(deck,), daemon=True).start()
    rospy.spin()

