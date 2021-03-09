---
layout: post
title: Allowing SSH on a server with an active OpenVPN client
categories: ospython
tags: [route, openvpn, iptables, ]
image: network.png
description: 
public: true
---

- Redirect packet with another table route
- Mark outgoing packet with iptables
- Add Route rule to the table

# Add new ROUTE table
- View current tables

```bash
cat /etc/iproute2/rt_tables
#
255      local 
254      main
253      default   
0        unspec
#
# local
#
#1      inr.ruhep
```

- Add new table

```bash
# Add new table with number: 201 and name: novpn
echo "201 novpn" | sudo tee -a /etc/iproute2/rt_tables
```

# Route Rules
- Add new mark rule 
- Add Route rule

```bash
# 1 number to mark the packet
ip rule add fwmark 1 table novpn
#
ip route add default via <YOUR.GATEWAY> dev eth0 table novpn
```

# iptables (mark)
Add iptables rule to mark the outgoing packets
  
```bash
iptables -t mangle -A OUTPUT -p tcp --sport 22 -j MARK --set-mark 1
# block incoming ssh traffic from tun
iptables -A INPUT -i tun0 -p tcp -m tcp --dport 22 -j DROP

```
# Reference
- [Allowing SSH on a server with an active OpenVPN client](https://gist.github.com/Shourai/1088f78b5e3696190a8ce6a6045c477b)
- [Allowing SSH on a server with an active OpenVPN client](https://gist.github.com/gpolitis/512193dd88c18aeb6900229187198899)