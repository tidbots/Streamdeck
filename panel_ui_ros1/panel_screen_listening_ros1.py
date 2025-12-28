#!/usr/bin/env python3
import rospy, json
from std_msgs.msg import String

if __name__ == "__main__":
    rospy.init_node("panel_screen_once")
    pub = rospy.Publisher("/panel/cmd", String, queue_size=1)
    rospy.sleep(0.5)

    cmd = {"type": "screen", "text": "LISTENING..."}
    pub.publish(String(json.dumps(cmd)))

