#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import yaml
from typing import Any, Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def to_json_str(obj) -> str:
    return json.dumps(obj, ensure_ascii=False)


class PanelUIYamlROS2(Node):
    def __init__(self):
        super().__init__("panel_ui_yaml")

        self.declare_parameter("yaml_path", "")
        self.declare_parameter("topic_cmd", "/panel/cmd")
        self.declare_parameter("topic_event", "/panel/event")
        self.declare_parameter("auto_screen_on_page", True)

        self.yaml_path = str(self.get_parameter("yaml_path").value)
        if not self.yaml_path:
            raise RuntimeError("parameter yaml_path is required")

        self.topic_cmd = str(self.get_parameter("topic_cmd").value)
        self.topic_event = str(self.get_parameter("topic_event").value)
        self.auto_screen_on_page = bool(self.get_parameter("auto_screen_on_page").value)

        self.ui = self._load_yaml(self.yaml_path)

        self.pub_cmd = self.create_publisher(String, self.topic_cmd, 50)
        self.sub_evt = self.create_subscription(String, self.topic_event, self._on_event, 50)

        self._startup_done = False
        self.timer = self.create_timer(0.5, self._startup_once)

        self.get_logger().info(
            f"[panel_ui_yaml_ros2] loaded yaml={self.yaml_path} auto_screen_on_page={self.auto_screen_on_page}"
        )

    def _load_yaml(self, path):
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)

    def _publish_cmd(self, cmd_dict: Dict[str, Any]):
        self.pub_cmd.publish(String(data=to_json_str(cmd_dict)))

    def _defaults_screen(self):
        defaults = self.ui.get("defaults", {}) or {}
        startup = (defaults.get("startup", {}) or {})
        return startup.get("screen_text", None)

    def _page_screen(self, page_name: str):
        pages = self.ui.get("pages", {}) or {}
        if page_name in pages:
            return (pages[page_name] or {}).get("screen_text", None)
        return None

    def _screen_for_page(self, page_name: str):
        ps = self._page_screen(page_name)
        return ps if ps is not None else self._defaults_screen()

    def _send_all_pages(self):
        pages = self.ui.get("pages", {}) or {}
        for name, page in pages.items():
            keys = page.get("keys", {}) or {}
            keys_json = {str(int(k)): dict(v) for k, v in keys.items()}
            self._publish_cmd({"type": "page_define", "name": str(name), "keys": keys_json})

    def _send_startup(self):
        defaults = self.ui.get("defaults", {}) or {}
        startup = (defaults.get("startup", {}) or {})
        startup_page = str(startup.get("page", "home"))

        self._publish_cmd({"type": "page", "name": startup_page})

        text = self._screen_for_page(startup_page)
        if text is not None:
            self._publish_cmd({"type": "screen", "text": str(text)})

    def _startup_once(self):
        if self._startup_done:
            return
        self._startup_done = True
        self._send_all_pages()
        self._send_startup()
        self.get_logger().info("[panel_ui_yaml_ros2] published pages + startup")

    def _expand_actions(self, actions: List[Dict[str, Any]]):
        if not self.auto_screen_on_page:
            return actions

        expanded: List[Dict[str, Any]] = []
        for i, a in enumerate(actions):
            expanded.append(a)
            if not isinstance(a, dict):
                continue

            if a.get("type") == "page":
                page_name = str(a.get("name", "home"))
                text = self._screen_for_page(page_name)

                next_action = actions[i + 1] if i + 1 < len(actions) else None
                if isinstance(next_action, dict) and next_action.get("type") == "screen":
                    continue

                if text is not None:
                    expanded.append({"type": "screen", "text": str(text)})

        return expanded

    def _resolve_actions(self, page: str, key: int, ev: str):
        routes = self.ui.get("routes", {}) or {}

        page_rules = routes.get(page, {}) or {}
        key_rules = page_rules.get(key, {}) or page_rules.get(str(key), {}) or {}
        actions = key_rules.get(ev, None)

        if actions is None:
            global_rules = routes.get("global", {}) or {}
            g_key_rules = global_rules.get(key, {}) or global_rules.get(str(key), {}) or {}
            actions = g_key_rules.get(ev, None)

        if not actions or not isinstance(actions, list):
            return []

        # Only keep dict actions
        cleaned = [a for a in actions if isinstance(a, dict)]
        return self._expand_actions(cleaned)

    def _on_event(self, msg: String):
        try:
            e = json.loads(msg.data)
        except Exception:
            return

        page = str(e.get("page", "home"))
        key = e.get("key", None)
        ev = str(e.get("event", "down"))
        if key is None:
            return
        try:
            key = int(key)
        except Exception:
            return

        actions = self._resolve_actions(page, key, ev)
        for cmd in actions:
            self._publish_cmd(cmd)


def main():
    rclpy.init()
    node = None
    try:
        node = PanelUIYamlROS2()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

