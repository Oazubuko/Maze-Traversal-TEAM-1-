#!/bin/bash

#shutdown.sh
pkill -f socket
socket_server.py &
sudo ./bluetooth_test9.py Ed &
sleep 2
sudo ./bluetooth_test9.py Zach &
sleep 3
sudo ./bluetooth_test9.py Loaner &
