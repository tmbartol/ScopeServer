#!/bin/bash

wifidev="wlan0"

KillHotspot()
{
    echo "Shutting Down Hotspot"
    ip link set dev "$wifidev" down
    systemctl stop hostapd
    systemctl stop dnsmasq
    iptables -D FORWARD -i eth0 -o "$wifidev" -m state --state RELATED,ESTABLISHED -j ACCEPT
    iptables -D FORWARD -i "$wifidev" -o eth0 -j ACCEPT
    echo 0 > /proc/sys/net/ipv4/ip_forward
    ip addr flush dev "$wifidev"
    ip link set dev "$wifidev" up
}


systemctl stop scopeserver_control.service
systemctl stop scopeserver_webapp.service
systemctl stop timeservice.service
sudo systemctl stop hotspot.service

KillHotspot
sleep 10
/usr/bin/autohotspotN
sleep 60

systemctl start timeservice.service
systemctl start scopeserver_webapp.service
systemctl start scopeserver_control.service
