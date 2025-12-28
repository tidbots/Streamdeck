#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
panel_ui_yaml_ros1.py

- Read common UI YAML (panel_ui.yaml)
- Publish page definitions to /panel/cmd on startup
- Subscribe /panel/event and route to /panel/cmd based on YAML routes

Topics (default):
  cmd:   /panel/cmd   (std_msgs/String JSON)
  event: /panel/event (std_msgs/String JSON)

YAML path:
  ~yaml_path (required)

Run:
  rosrun <your_pkg> panel_ui_yaml_ros1.py _yaml_path:=/path/to/panel_ui.yaml
"""

import json
import yaml
import rospy
from std_msgs.msg import String


def to_json_str(obj) -> str:
    return json.dumps(obj, ensure_ascii=False)


class PanelUIYamlROS1:
    def __init__(self):
        self.yaml_path = rospy.get_param("~yaml_path", "")
        if not self.yaml_path:
            raise RuntimeError("~yaml_path is required")

        self.topic_cmd = rospy.get_param("~topic_cmd", "/panel/cmd")
        self.topic_event = rospy.get_param("~topic_event", "/panel/event")

        self.ui = self._load_yaml(self.yaml_path)

        self.pub_cmd = rospy.Publisher(self.topic_cmd, String, queue_size=50)
        self.sub_evt = rospy.Subscriber(self.topic_event, String, self._on_event, queue_size=50)

        # startup send (page_define for all pages, then startup page + screen)
        rospy.sleep(0.5)  # allow connections
        self._send_all_pages()
        self._send_startup()

        rospy.loginfo(f"[panel_ui_yaml_ros1] ready. yaml={self.yaml_path}")

    def _load_yaml(self, path):
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)

    def _publish_cmd(self, cmd_dict):
        self.pub_cmd.publish(String(data=to_json_str(cmd_dict)))

    def _send_all_pages(self):
        pages = self.ui.get("pages", {}) or {}
        for name, page in pages.items():
            keys = page.get("keys", {}) or {}
            # keys in YAML are typically int; JSON keys should be str for our controller
            keys_json = {str(int(k)): dict(v) for k, v in keys.items()}
            self._publish_cmd({
                "type": "page_define",
                "name": str(name),
                "keys": keys_json
            })

    def _send_startup(self):
        defaults = self.ui.get("defaults", {}) or {}
        startup = (defaults.get("startup", {}) or {})
        startup_page = str(startup.get("page", "home"))
        startup_screen = startup.get("screen_text", None)

        # switch page
        self._publish_cmd({"type": "page", "name": startup_page})

        # screen text: if YAML has per-page screen_text, prefer that; else startup default
        pages = self.ui.get("pages", {}) or {}
        page_screen = None
        if startup_page in pages:
            page_screen = (pages[startup_page] or {}).get("screen_text", None)

        text = page_screen if page_screen is not None else startup_screen
        if text is not None:
            self._publish_cmd({"type": "screen", "text": str(text)})

    def _on_event(self, msg: String):
        try:
            e = json.loads(msg.data)
        except Exception:
            return

        # route only defined events; common default: react on "down"
        page = str(e.get("page", "home"))
        key = e.get("key", None)
        ev = str(e.get("event", "down"))
        if key is None:
            return
        try:
            key = int(key)
        except Exception:
            return

        routes = self.ui.get("routes", {}) or {}
        page_rules = routes.get(page, {}) or {}
        key_rules = page_rules.get(key, {}) or page_rules.get(str(key), {}) or {}
        actions = key_rules.get(ev, None)

        # Optional: global routes (any page)
        if actions is None:
            global_rules = routes.get("global", {}) or {}
            g_key_rules = global_rules.get(key, {}) or global_rules.get(str(key), {}) or {}
            actions = g_key_rules.get(ev, None)

        if not actions:
            return

        # actions: list[dict] -> publish in order
        for cmd in actions:
            if isinstance(cmd, dict):
                self._publish_cmd(cmd)


def main():
    rospy.init_node("panel_ui_yaml")
    PanelUIYamlROS1()
    rospy.spin()


if __name__ == "__main__":
    main()

