# Elgato STREAMDECK をPythonから使う

[Python Elgato Stream Deck Library](https://github.com/abcminiuser/python-elgato-streamdeck)を使用

[マニュアル](https://python-elgato-streamdeck.readthedocs.io/en/stable/)

## ToDo
デバイスを握ったまま異常終了したばあい、以下のコマンドを実行する。
```
$ sudo udevadm control --reload-rules
```

## 準備
### インストール
```
sudo apt update && sudo apt dist-upgrade -y

# Install the pip Python package manager
sudo apt install -y python3-pip python3-setuptools

# Install system packages needed for the default LibUSB HIDAPI backend
sudo apt install -y libudev-dev libusb-1.0-0-dev libhidapi-libusb0

# Install system packages needed for the Python Pillow package installation
sudo apt install -y libjpeg-dev zlib1g-dev libopenjp2-7 libtiff5

# Install python library dependencies
pip3 install wheel
pip3 install pillow
```
### udev（userspace device management）の設定
```
$ sudo vi /etc/udev/rules.d/50-elgato.rules
```
```
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0fd9", GROUP="users", TAG+="uaccess"
```
再起動する
```
$ sudo udevadm control --reload-rules
```

## サンプルの実行
```
$ git clone https://github.com/tidbots/Streamdeck.git
$ cd Streamdeck/src
```
STREAM DECK NEOの場合
```
$ python3 example_neo.py
```
```
cd scripts
python3 ros_hsr_neo.py
```

## ROS1 Noeticで使う

## ROS2 Humbleで使う
### 仕様
#### ボタンがおされたら
```
ros2 topic echo /panel/event
```

受け取る側（例）

他ノード側は /panel/event を subscribe して JSON を parse するだけです。
```
def cb(msg):
    e = json.loads(msg.data)
    if e["event"] == "down" and e["key"] == 0:
        # 0番キーが押されたら…
        pass
```

#### パネルの表示を変更
- Publish topic: /panel/cmd
- msg型: std_msgs/String（中身はJSON）
- 送る内容例：
```
{"schema":"panel_event_v1","ts":1766.123,"deck_id":"SERIAL","key":3,"event":"down","is_pressed":true}
{"schema":"panel_event_v1","ts":1766.456,"deck_id":"SERIAL","key":3,"event":"up","is_pressed":false,"held_ms":333}
{"schema":"panel_event_v1","ts":1766.900,"deck_id":"SERIAL","key":3,"event":"long","held_ms":1200}様
```

例1：キー0を「GO」＋アイコンに変更
```
ros2 topic pub -1 /panel/cmd std_msgs/msg/String \
"{data: '{\"type\":\"key\",\"index\":0,\"icon\":\"init-pos.png\",\"label\":\"GO\"}'}"
```

例2：開始状態
```
ros2 topic pub -1 /panel/cmd std_msgs/msg/String \
"{data: '{\"type\":\"key\",\"index\":0,\"icon\":\"listen.png\",\"label\":\"LISTEN\"}'}"

ros2 topic pub -1 /panel/cmd std_msgs/msg/String \
"{data: '{\"type\":\"key\",\"index\":1,\"icon\":\"stop.png\",\"label\":\"STOP\"}'}"
```

例3：パネル（Neoスクリーンの場合）のテキスト変更
```
ros2 topic pub -1 /panel/cmd std_msgs/msg/String \
"{data: '{\"type\":\"screen\",\"text\":\"LISTENING...\"}'}"
```

例4：キー1だけリセット
```
ros2 topic pub -1 /panel/cmd std_msgs/msg/String \
"{data: '{\"type\":\"reset_key\",\"index\":1}'}"
```

例5：全キーをリセット
```
ros2 topic pub -1 /panel/cmd std_msgs/msg/String \
"{data: '{\"type\":\"reset_all\"}'}"
```

送る側のプログラム例：
```
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PanelScreenOnce(Node):
    def __init__(self):
        super().__init__("panel_screen_once")

        self.pub = self.create_publisher(String, "/panel/cmd", 10)

        # 少し待ってから publish（publisher 初期化待ち）
        self.timer = self.create_timer(0.5, self.publish_once)
        self.sent = False

    def publish_once(self):
        if self.sent:
            return

        cmd = {
            "type": "screen",
            "text": "LISTENING..."
        }

        msg = String()
        msg.data = json.dumps(cmd)

        self.pub.publish(msg)
        self.get_logger().info("Sent /panel/cmd: screen=LISTENING...")

        self.sent = True
        rclpy.shutdown()


def main():
    rclpy.init()
    node = PanelScreenOnce()
    rclpy.spin(node)


if __name__ == "__main__":
    main()

```



#### ページ遷移ノードの例
/panel/event を受けて「homeの0番を押したらgpsrページへ」みたいな ページ遷移ノードpanel_page_router.pyの例

0. 前提

先にあなたの パネルコントローラ（hsr_neo_ros2_pages.py：完成版）を起動しておきます。

Assets/ に使うアイコン（gpsr.png, nav.png, debug.png, back.png など）を置きます。

1. ターミナルA：パネルコントローラ起動
```
ros2 run <your_pkg> hsr_neo_ros2_pages.py
```
（または python3 hsr_neo_ros2_pages.py）

2. 別のターミナルから：ページ定義を流す（例）
- homeページ
```
ros2 topic pub -1 /panel/cmd std_msgs/msg/String \
"{data: '{\"type\":\"page_define\",\"name\":\"home\",\"keys\":{\"0\":{\"icon\":\"gpsr.png\",\"label\":\"GPSR\"},\"1\":{\"icon\":\"nav.png\",\"label\":\"NAV\"},\"2\":{\"icon\":\"debug.png\",\"label\":\"DBG\"}}}'}"
```
- gpsrページ
```
ros2 topic pub -1 /panel/cmd std_msgs/msg/String \
"{data: '{\"type\":\"page_define\",\"name\":\"gpsr\",\"keys\":{\"0\":{\"icon\":\"listen.png\",\"label\":\"LISTEN\"},\"1\":{\"icon\":\"stop.png\",\"label\":\"STOP\"},\"2\":{\"icon\":\"back.png\",\"label\":\"BACK\"}}}'}"
```
- navページ
```
ros2 topic pub -1 /panel/cmd std_msgs/msg/String \
"{data: '{\"type\":\"page_define\",\"name\":\"nav\",\"keys\":{\"0\":{\"icon\":\"nav.png\",\"label\":\"START\"},\"1\":{\"icon\":\"stop.png\",\"label\":\"STOP\"},\"2\":{\"icon\":\"back.png\",\"label\":\"BACK\"}}}'}"
```
- debugページ
```
ros2 topic pub -1 /panel/cmd std_msgs/msg/String \
"{data: '{\"type\":\"page_define\",\"name\":\"debug\",\"keys\":{\"0\":{\"icon\":\"debug.png\",\"label\":\"INFO\"},\"1\":{\"icon\":\"reset.png\",\"label\":\"RESET\"},\"2\":{\"icon\":\"back.png\",\"label\":\"BACK\"}}}'}"
```
- homeに切替
```
ros2 topic pub -1 /panel/cmd std_msgs/msg/String \
"{data: '{\"type\":\"page\",\"name\":\"home\"}'}"
```

3. ターミナルC：この router ノード起動
```
ros2 run <your_pkg> panel_page_router.py
```
（または python3 panel_page_router.py）

4. 動作確認
- StreamDeck の homeページ で
   - key0 → gpsrページへ
   - key1 → navページへ
   - key2 → debugページへ
- 各ページの key2（BACK） → homeへ戻る

##### カスタマイズ方法（最重要）
panel_page_router.py の self.routes を編集してください。

例：gpsrページの key0 を押したら「LISTENING」表示に加えて、別ノードへ合図したい場合は
- /gpsr/start に publish する等を追加（このノード側にpubを追加）
または
- /panel/event を受けた側（SMACH側）で処理する（おすすめ）


## Docker
### ROS1 Noetic
### ROS2 Humble
