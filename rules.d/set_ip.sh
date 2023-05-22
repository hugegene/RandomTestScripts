#!/bin/bash
echo "Set IP at $(date)" >>/tmp/scripts.log
sudo ifconfig ueth0 10.10.10.10 netmask 255.255.255.0 up
exit 0
