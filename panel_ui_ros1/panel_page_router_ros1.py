#!/usr/bin/env python3
import rospy, json
from std_msgs.msg import String

ROUTES = {
    "home": {
        0: [{"type": "page", "name": "gpsr"}],
        1: [{"type": "page", "name": "nav"}],
        2: [{"type": "page", "name": "debug"}],
    },
    "gpsr": {
        2: [{"type": "page", "name": "home"}],
    }
}

def cb(msg):
    e = json.loads(msg.data)
    if e.get("event") != "down":
        return
    page = e.get("page", "home")
    key = e.get("key")

    for c in ROUTES.get(page, {}).get(key, []):
        pub.publish(String(json.dumps(c)))

if __name__ == "__main__":
    rospy.init_node("panel_page_router")
    pub = rospy.Publisher("/panel/cmd", String, queue_size=10)
    rospy.Subscriber("/panel/event", String, cb)
    rospy.spin()

