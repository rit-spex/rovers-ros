#!/bin/bash

sudo chmod +x /home/rovers/ros/rovers-ros/src/externalScript/autoBoot.sh

sudo cp autoBoot.service /etc/systemd/system
sudo chmod 644 /etc/systemd/system/autoBoot.service