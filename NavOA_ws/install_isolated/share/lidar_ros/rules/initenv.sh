###
 # @Version      : V1.0
 # @Date         : 2024-10-18 09:53:56
 # @Description  : 
### 

#!/bin/bash
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="2e3c", ATTRS{idProduct}=="5740", MODE:="0777", GROUP:="dialout",  SYMLINK+="lidar"' >/etc/udev/rules.d/lidar_usb.rules

echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", GROUP:="dialout",  SYMLINK+="lidar"' >/etc/udev/rules.d/lidar_cp2102.rules

service udev reload
sleep 2
service udev restart

