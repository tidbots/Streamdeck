#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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

        # option
        self.auto_screen_on_page = bool(rospy.get_param("~auto_screen_on_page", True))

        self.pub_cmd = rospy.Publisher(self.topic_cmd, String, queue_size=50)
        self.sub_evt = rospy.Subscriber(self.topic_event, String, self._on_event, queue_size=50)

        rospy.sleep(0.5)
        self._send_all_pages()
        self._send_startup()

        rospy.loginfo(
            f"[panel_ui_yaml_ros1] ready. yaml={self.yaml_path} auto_screen_on_page={self.auto_screen_on_page}"
        )

    def _load_yaml(self, path):
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)

    def _publish_cmd(self, cmd_dict):
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
        return self._page_screen(page_name) if self._page_screen(page_name) is not None else self._defaults_screen()

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

    def _expand_actions(self, actions):
        """
        Add automatic screen update right after a page switch command.
        Avoid duplicating screen if the next action is already screen.
        """
        if not self.auto_screen_on_page:
            return actions

        expanded = []
        for i, a in enumerate(actions):
            expanded.append(a)
            if not isinstance(a, dict):
                continue

            if a.get("type") == "page":
                page_name = str(a.get("name", "home"))
                text = self._screen_for_page(page_name)

                # if next action already sets screen, do nothing
                next_action = actions[i + 1] if i + 1 < len(actions) else None
                if isinstance(next_action, dict) and next_action.get("type") == "screen":
                    continue

                if text is not None:
                    expanded.append({"type": "screen", "text": str(text)})

        return expanded

    def _resolve_actions(self, page: str, key: int, ev: str):
        routes = self.ui.get("routes", {}) or {}

        # page-specific
        page_rules = routes.get(page, {}) or {}
        key_rules = page_rules.get(key, {}) or page_rules.get(str(key), {}) or {}
        actions = key_rules.get(ev, None)

        # global fallback
        if actions is None:
            global_rules = routes.get("global", {}) or {}
            g_key_rules = global_rules.get(key, {}) or global_rules.get(str(key), {}) or {}
            actions = g_key_rules.get(ev, None)

        if not actions:
            return []

        if not isinstance(actions, list):
            return []

        return self._expand_actions(actions)

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
            if isinstance(cmd, dict):
                self._publish_cmd(cmd)


def main():
    rospy.init_node("panel_ui_yaml")
    PanelUIYamlROS1()
    rospy.spin()


if __name__ == "__main__":
    main()

