#!/bin/bash

wifidev="wlan0"

KillHotspot()
{
    echo "Shutting Down Hotspot"
    ip link set dev "$wifidev" down
    systemctl stop hostapd
    systemctl stop dnsmasq
    ip addr flush dev "$wifidev"
    ip link set dev "$wifidev" up
    dhcpcd  -n "$wifidev" >/dev/null 2>&1
}

sleep 5
sudo systemctl stop scopeserver_webapp.service
sudo systemctl stop scopeserver_control.service
sudo systemctl stop autohotspot.service

KillHotspot
sleep 10
sudo cp /etc/wpa_supplicant/wpa_supplicant.conf.on /etc/wpa_supplicant/wpa_supplicant.conf
sudo systemctl start autohotspot.service
sleep 60

sudo systemctl start scopeserver_control.service
sudo systemctl start scopeserver_webapp.service

# Experimental method to force hotspot on next start of autohotspot.service
sudo cp /etc/wpa_supplicant/wpa_supplicant.conf.off /etc/wpa_supplicant/wpa_supplicant.conf

