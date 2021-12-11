#!/bin/bash

#shutdown.sh
pkill -f socket
sudo ./socket_server.py &
sudo ./bluetooth_server.py Ed &
sleep 2
sudo ./bluetooth_server.py Zach &
sleep 3
sudo ./bluetooth_server.py Machi &
