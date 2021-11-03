#!/bin/bash

#ps -ef | grep bluetooth_test7
#sudo kill -9 #

sudo ./bluetooth_test7.py Ed &
sleep 2
sudo ./bluetooth_test7.py Zach &
sleep 3
sudo ./bluetooth_test7.py Machi &
