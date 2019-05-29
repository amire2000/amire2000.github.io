---
layout: post
title: Raspberry Pi as Access Pint
categories: hw
tags: [raspberrypi, wifi, ap]
description: Setting Raspberry Pi as access point, 
public: false
image: wifi_pi_ap.png 
---
# Content 
- PI WIFI Enabled
- AP Setup
  - Install specific hostapd version
- Troubleshooting
  
# PI WIFI Enabled
- Set wifi country using `raspi-config` or  `wpa_supplicant.conf`

> ssh login return the lines
```
Wi-Fi is disabled because the country is not set.
Use raspi-config to set the country before use.
```
- Setup `/etc/wpa_supplicant/wpa_supplicant.conf` file
    - Add country to conf file
```
country=US
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
```

# AP Setup
- install daemons
- config static ip from wifi interface
- config dhcp server / dnsmasq service
- config hostapd

## install daemons
- dnsmasq
```
sudo apt install dnsmasq 
```
- install hostapd specific version
  - version 2:2.6 has some bugs
  
```bash
sudo apt update
sudo apt-cache madison hostapd
#
hostapd | 2:2.6-21~bpo9~rpt1 | http://archive.raspberrypi.org/debian stretch/main armhf Packages
hostapd | 2:2.4-1+deb9u4 | http://raspbian.raspberrypi.org/raspbian stretch/main armhf Packages
#
sudo apt instal hostapd=2:2.4-1+deb9u4
```


## Configuring a static IP
- edit file
```
sudo vim /etc/dhcpcd.conf
```
- add lines
```
interface wlan0
    static ip_address=192.168.4.1/24
    nohook wpa_supplicant
```
- restart service
```
sudo systemctl restart dhcpcd
```

## config dhcp server
- rm/mv original file
```
sudo mv /etc/dnsmasq.conf /etc/dnsmasq.conf.orig
```
- edit file
```
sudo vim /etc/dnsmasq.conf
```
- paste lines
```
interface=wlan0
dhcp-range=192.168.4.2,192.168.4.20,255.255.255.0,24h
```
- restart service
```
sudo systemctl reload dnsmasq
```

# config hostapd
- edit `hostapd.conf`
```
sudo vim /etc/hostapd/hostapd.conf
```
- paste lines (customize parameters)
  - ssid
  - wpa_passphrase (8 characters)
  - channel
```
interface=wlan0
driver=nl80211
ssid=pi_network
hw_mode=g
channel=7
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=12341234
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
```
- edit `hostapd`
```
sudo vim /etc/default/hostapd
```
- edit line
```
DAEMON_CONF="/etc/hostapd/hostapd.conf"
```

# Troubleshooting
- `sudo systemctl status hostapd`
  
```bash
 sudo systemctl status hostapd
● hostapd.service - LSB: Advanced IEEE 802.11 management daemon
   Loaded: loaded (/etc/init.d/hostapd; generated; vendor preset: enabled)
   Active: active (exited) since Tue 2019-05-28 17:57:23 BST; 1min 6s ago
     Docs: man:systemd-sysv-generator(8)
  Process: 536 ExecStart=/etc/init.d/hostapd start (code=exited, status=0/SUCCESS)
   CGroup: /system.slice/hostapd.service

May 28 17:57:22 raspberrypi systemd[1]: Starting LSB: Advanced IEEE 802.11 management daemon...
May 28 17:57:23 raspberrypi hostapd[536]: Starting advanced IEEE 802.11 management: hostapd failed!

```

- Run `hostapd -d /etc/hostapd/hostapd.conf` it will list errors
  - check and fix config file
```
sudo hostapd -d /etc/hostapd/hostapd.conf
random: Trying to read entropy from /dev/random
Configuration file: /etc/hostapd/hostapd.conf
Line 1: unknown configuration item 'nterface'
Line 11: invalid WPA passphrase length 4 (expected 8..63)
WPA-PSK enabled, but PSK or passphrase is not configured.
3 errors found in configuration file '/etc/hostapd/hostapd.conf'
```

> Tip: for config pi as a router check reference 1
# Reference
1. [Setting up a Raspberry Pi as an access point](https://www.raspberrypi.org/documentation/configuration/wireless/access-point.md)
2. [Updating hostapd fails…](https://www.raspberrypi.org/forums/viewtopic.php?t=235145)
3. [hostapd will not start via “service”](https://unix.stackexchange.com/questions/119209/hostapd-will-not-start-via-service-but-will-start-directly)