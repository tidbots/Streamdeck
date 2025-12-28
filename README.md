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

## ROS1 Noeticで使う
```
cd scripts
python3 ros_hsr_neo.py
```
### 仕様
#### ボタンがおされたら



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
- Publish topic: /panel/event
- msg型: std_msgs/String（中身はJSON）
- 送る内容例：
```
{"schema":"panel_event_v1","ts":1766.123,"deck_id":"SERIAL","key":3,"event":"down","is_pressed":true}
{"schema":"panel_event_v1","ts":1766.456,"deck_id":"SERIAL","key":3,"event":"up","is_pressed":false,"held_ms":333}
{"schema":"panel_event_v1","ts":1766.900,"deck_id":"SERIAL","key":3,"event":"long","held_ms":1200}様
```

例1：キー0を「GO」＋アイコンに変更
```
rostopic pub -1 /panel/cmd std_msgs/msg/String \
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

## ROS2

## Docker
### ROS1 Noetic
### ROS2 Humble
