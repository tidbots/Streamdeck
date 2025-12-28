#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
panel_page_router.py (ROS2 Humble)

Purpose
- Subscribe:  /panel/event  (std_msgs/msg/String JSON)
- Publish:    /panel/cmd    (std_msgs/msg/String JSON)
- Route key events to "page switch" and/or "commands" (screen update, key override, etc.)

Typical usage:
- You run the StreamDeck panel controller (hsr_neo_ros2_pages.py) in one terminal.
- You run this router node in another terminal.
- When you press a key on the StreamDeck, this node sends /panel/cmd commands:
    - Switch pages (home -> gpsr, gpsr -> home, etc.)
    - Update screen text or patch a page to reflect state

Event JSON coming from /panel/event (from the controller):
  {"schema":"panel_event_v1","ts":...,"deck_id":"...","key":0,"event":"down","page":"home",...}

Command JSON to /panel/cmd (to the controller):
  {"type":"page","name":"gpsr"}
  {"type":"screen","text":"LISTENING..."}
  {"type":"page_patch","name":"home","keys":{"0":{"label":"GPSR*"}}}

This node keeps things simple:
- React on event == "down" only (to avoid double-trigger).
- Use mapping rules (page + key -> actions).

Requirements
- rclpy
- std_msgs
- (No extra msg types; everything is JSON in std_msgs/String)

Run:
  ros2 run <your_pkg> panel_page_router.py
or:
  python3 panel_page_router.py

Parameters (optional):
  - panel_event_topic (default: /panel/event)
  - panel_cmd_topic   (default: /panel/cmd)

Example:
  ros2 run <your_pkg> panel_page_router.py --ros-args \
    -p panel_event_topic:=/panel/event -p panel_cmd_topic:=/panel/cmd
"""

import json
from typing import Any, Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def j(cmd: Dict[str, Any]) -> String:
    """Dict -> std_msgs/String(JSON)"""
    return String(data=json.dumps(cmd))


class PanelPageRouter(Node):
    def __init__(self):
        super().__init__("panel_page_router")

        self.declare_parameter("panel_event_topic", "/panel/event")
        self.declare_parameter("panel_cmd_topic", "/panel/cmd")

        self.event_topic = self.get_parameter("panel_event_topic").value
        self.cmd_topic = self.get_parameter("panel_cmd_topic").value

        self.pub_cmd = self.create_publisher(String, self.cmd_topic, 50)
        self.sub_evt = self.create_subscription(String, self.event_topic, self._on_event, 50)

        # ------------------------------------------------------------
        # Routing rules: (page_name, key_index) -> list of command dicts
        # Edit this table to fit your UI.
        # ------------------------------------------------------------
        self.routes: Dict[str, Dict[int, List[Dict[str, Any]]]] = {
            "home": {
                0: [
                    {"type": "page", "name": "gpsr"},
                    {"type": "screen", "text": "GPSR"},
                ],
                1: [
                    {"type": "page", "name": "nav"},
                    {"type": "screen", "text": "NAV"},
                ],
                2: [
                    {"type": "page", "name": "debug"},
                    {"type": "screen", "text": "DEBUG"},
                ],
            },
            "gpsr": {
                0: [{"type": "screen", "text": "LISTENING..."}],
                1: [{"type": "screen", "text": "STOP"}],
                2: [{"type": "page", "name": "home"}, {"type": "screen", "text": "HOME"}],  # BACK
            },
            "nav": {
                0: [{"type": "screen", "text": "NAV: START"}],
                1: [{"type": "screen", "text": "NAV: STOP"}],
                2: [{"type": "page", "name": "home"}, {"type": "screen", "text": "HOME"}],  # BACK
            },
            "debug": {
                0: [{"type": "screen", "text": "DBG: INFO"}],
                1: [{"type": "screen", "text": "DBG: RESET"}],
                2: [{"type": "page", "name": "home"}, {"type": "screen", "text": "HOME"}],  # BACK
            },
        }

        # Optional: press feedback (patch label with * on press)
        self.enable_press_mark = True

        self.get_logger().info(
            f"panel_page_router started. event={self.event_topic} cmd={self.cmd_topic}"
        )

    def _publish_cmds(self, cmds: List[Dict[str, Any]]):
        for c in cmds:
            self.pub_cmd.publish(j(c))

    def _on_event(self, msg: String):
        try:
            e = json.loads(msg.data)
        except Exception as ex:
            self.get_logger().warn(f"bad event json: {ex} data={msg.data[:200]}")
            return

        # We react only on "down" to avoid double-triggering.
        if e.get("event") != "down":
            return

        page = str(e.get("page", "home"))
        key = int(e.get("key", -1))
        if key < 0:
            return

        # Optional UI feedback: mark the pressed key label by patching current page
        if self.enable_press_mark:
            self._publish_cmds(
                [
                    {
                        "type": "page_patch",
                        "name": page,
                        "keys": {
                            str(key): {"label": f"{key}*"}  # simple; customize if you want
                        },
                    }
                ]
            )

        cmds = self.routes.get(page, {}).get(key)
        if not cmds:
            self.get_logger().info(f"no route for page={page} key={key}")
            return

        self._publish_cmds(cmds)


# --------------------------
# Main
# --------------------------

def main():
    rclpy.init()
    node = PanelPageRouter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

