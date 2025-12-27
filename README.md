# Elgato STREAMDECK をPythonから使う

[Python Elgato Stream Deck Library](https://github.com/abcminiuser/python-elgato-streamdeck)を使用

[マニュアル](https://python-elgato-streamdeck.readthedocs.io/en/stable/)

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
$ git clone https://gitlab.com/okadalaboratory/my-streamdeck.git
$ cd Streamdeck/src
```
STREAM DECK NEO
```
$ python3 example_neo.py
```

## ROS1 Noetic

## ROS2

## Docker
### ROS1 Noetic
### ROS2 Humble
