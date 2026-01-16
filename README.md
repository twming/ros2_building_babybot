# Setup

software installation
```
sudo chmod 777 ros2-humble-base-main.sh
./ros2-humble-base-main.sh
```
udev rule setup
```
sudo nano /etc/udev/rules.d/90-babybot.rules
SUBSYSTEMS=="usb", ATTRS{idVendor}=="2341", GROUP="plugdev", MODE="0666"
```

# ROS2 Building BabyBot

### URDF of BabyBot

### Control in Gazebo

### Design Hardwawre Interface with Arduino Nano

### Bring Up BabyBot
