#! /bin/bash

echo "123" | sudo -S chmod 777 /dev/ttyACM0  （串口赋权）
./build/uav_debug
