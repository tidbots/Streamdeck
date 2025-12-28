#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
panel_screen_listening_ros2.py (ROS2 Humble)

Equivalent to:
  ros2 topic pub -1 /panel/cmd std_msgs/msg/String \
    "{data: '{\"type\":\"screen\",\"text\":\"LISTENING...\"}'}"

- Publishes once, then exits.
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PanelScreenListeningOnce(Node):
    def __init__(self):
        super().__init__("panel_screen_listening_once")

        self.pub = self.create_publisher(String, "/panel/cmd", 10)
        self.sent = False

        # Wait a bit so the publisher is ready / discovery completes.
        self.timer = self.create_timer(0.5, self._publish_once)

    def _publish_once(self):
        if self.sent:
            return

        cmd = {"type": "screen", "text": "LISTENING..."}
        msg = String()
        msg.data = json.dumps(cmd)

        self.pub.publish(msg)
        self.get_logger().info('Published: {"type":"screen","text":"LISTENING..."}')

        self.sent = True
        # stop after one publish
        rclpy.shutdown()


def main():
    rclpy.init()
    node = PanelScreenListeningOnce()
    rclpy.spin(node)


if __name__ == "__main__":
    main()

