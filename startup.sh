#!/bin/bash

#shutdown.sh
pkill -f socket
sudo ./socket_server.py &
sudo ./bluetooth_test10.py Ed &
sleep 2
sudo ./bluetooth_test10.py Zach &
sleep 3
sudo ./bluetooth_test10.py Machi &
