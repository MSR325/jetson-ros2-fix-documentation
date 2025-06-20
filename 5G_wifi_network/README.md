# 🚀 Jetson 5GHz Hotspot with Realtek 8822 and hostapd

This guide sets up your NVIDIA Jetson Nano / Xavier NX to act as a **5 GHz Wi-Fi hotspot** using a **Realtek 8822 USB antenna**, `hostapd`, and `dhcpd`. It is designed to **auto-start on boot** and remain stable even without NetworkManager control.

> NOTE: If something fails, research and/or ask chat, i'm not certain that it is complete.

---

## ✅ Features

- Creates a 5 GHz access point on `wlan0`
- Static IP: `10.42.0.1/24`
- DHCP range: `10.42.0.10 – 10.42.0.50`
- Fully unmanaged by NetworkManager
- Autostart on boot with `systemd`
- Compatible with Realtek RTL8822 USB antennas

---

## 🧰 Requirements

Install the required packages:

```bash
sudo apt update
sudo apt install hostapd isc-dhcp-server rfkill
```

---

## ⚙️ Config Files

### `/etc/hostapd/hostapd.conf`

```ini
interface=wlan0
driver=nl80211
ssid=Jetson5GHz
hw_mode=a
channel=36
country_code=MX
ieee80211d=1
ieee80211n=1
ieee80211ac=1
wmm_enabled=1

auth_algs=1
wpa=2
wpa_passphrase=jetsonpass
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
```

---

### `/etc/dhcp/dhcpd.conf`

```conf
subnet 10.42.0.0 netmask 255.255.255.0 {
  range 10.42.0.10 10.42.0.50;
  option routers 10.42.0.1;
  option broadcast-address 10.42.0.255;
  default-lease-time 600;
  max-lease-time 7200;
}
```

---

### `/etc/NetworkManager/NetworkManager.conf`

```ini
[keyfile]
unmanaged-devices=interface-name:wlan0
```

Then restart NetworkManager:

```bash
sudo systemctl restart NetworkManager
```

---

### `/etc/systemd/network/10-wlan0-static.network`

```ini
[Match]
Name=wlan0

[Network]
Address=10.42.0.1/24
DHCP=no
```

Enable and start `systemd-networkd`:

```bash
sudo systemctl enable systemd-networkd
sudo systemctl start systemd-networkd
```

---

## 📜 Startup Script

### `/home/msr/ap.sh`

```bash
#!/bin/bash

FLAG=/tmp/wifi_ap_started.flag
LOG=/var/log/wifi-ap.log

if [ -f "$FLAG" ]; then
    echo "[INFO] Already started." >> "$LOG"
    exit 0
fi
touch "$FLAG"
echo "[BOOT] Running ap.sh at $(date)" >> "$LOG"

ip link set wlan0 down
ip addr flush dev wlan0
ip addr add 10.42.0.1/24 dev wlan0
ip link set wlan0 up

sleep 2
hostapd -B /etc/hostapd/hostapd.conf
sleep 3
dhcpd -cf /etc/dhcp/dhcpd.conf wlan0
```

Make executable:

```bash
chmod +x /home/msr/ap.sh
```

---

## 🧩 Systemd Units

### `/etc/systemd/system/wifi-ap.service`

```ini
[Unit]
Description=Jetson 5GHz Wi-Fi AP
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/home/msr/ap.sh

[Install]
WantedBy=multi-user.target
```

---

### `/etc/systemd/system/wifi-ap-cleanup.service`

```ini
[Unit]
Description=Clean AP flag file
DefaultDependencies=no
Before=shutdown.target

[Service]
Type=oneshot
ExecStart=/bin/rm -f /tmp/wifi_ap_started.flag

[Install]
WantedBy=shutdown.target
```

---

## 🔄 Activation

```bash
sudo systemctl daemon-reload
sudo systemctl enable wifi-ap.service
sudo systemctl enable wifi-ap-cleanup.service
sudo systemctl start wifi-ap.service
```

---

## 🧪 Verification

```bash
ip addr show wlan0
pgrep hostapd
arp -i wlan0 -n
journalctl -u wifi-ap.service
```

---

## ✍️ Author

Adapted and tested for Jetson with Realtek RTL8822 USB antenna.
