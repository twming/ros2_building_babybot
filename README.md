# Setup

Install ROS and dependency packages
```
sudo chmod 777 ros2-humble-base-main.sh
./ros2-humble-base-main.sh
```

Customize Setting to setup the wifi
- Connect up Raspberry Pi to TV monitor, keyboard, mouse.
- Boot up the into Ubuntu, to obtain the IP address
```
ip addr
```

Enable password login through ssh, edit the /etc/ssh/sshd_config.d/50-cloud-init.conf
- Change the PasswordAuthentication from 'no' to 'yes'
```
sudo nano /etc/ssh/sshd_config.d/50-cloud-init.conf
```

Edit the /etc/apt/apt.conf.d/20auto-upgrades
- Disable Auto Update (prevent long wait for application update), to disable it, set the value '1' to '0'
```
sudo nano /etc/apt/apt.conf.d/20auto-upgrades
```

Disable Network Sleep, Suspend, Hibernate
```
sudo systemctl mask systemd-networkd-wait-online.service
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
```

(Optional) Edit /etc/netplan/50-cloud-init.yaml
```
sudo nano /etc/netplan/50-cloud-init.yaml
```
Configure Wifi, get the password "wpa_passphrase ros_public 0123456789"
```
network:
    ethernets:
        eth0:
            dhcp4: true
            optional: true
    version: 2
    wifis:
        renderer: networkd
        wlan0:
            access-points:
                ros_public:
                    password: 9fea575b6a0d4668c666a4b111d7784ca062496a4570715e0d5120714b9b3f90
                SSID:
                    password: xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
            dhcp4: true
            optional: true
```

udev rule setup for Arduino nano and YDLidar
```
sudo nano /etc/udev/rules.d/90-babybot.rules
SUBSYSTEMS=="usb", ATTRS{idVendor}=="2341", GROUP="plugdev", MODE="0666"
```

# ROS2 Building BabyBot

### URDF of BabyBot

### Control in Gazebo

### Design Hardwawre Interface with Arduino Nano

### Bring Up BabyBot
