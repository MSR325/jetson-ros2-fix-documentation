# 📡 Jetson Hotspot Autoiniciable con hostapd + dhcpd

Este proyecto configura un punto de acceso Wi-Fi (AP) en tu Jetson Nano / Xavier NX usando `hostapd` y `dhcpd`. Se levanta automáticamente al arrancar, con validación robusta y prevención de errores comunes.

---

## ✅ ¿Qué hace?

- Configura `wlan0` con IP estática `10.42.0.1/24`
- Inicia `hostapd` para emitir la red `msr_jetson`
- Inicia `dhcpd` para entregar IPs
- Se ejecuta automáticamente al boot con `systemd`
- Evita conflictos con el DHCP interno de NVIDIA (`l4tbr0`)
- Guarda logs detallados en `/var/log/wifi-ap.log`

---

## 🧱 Estructura del sistema

```
/etc/systemd/system/
├── wifi-ap.service              # Servicio principal al boot
├── wifi-ap-cleanup.service      # Limpia flag al apagar
/home/msr/
├── ap.sh                        # Script de configuración robusta del AP
├── status_ap.sh                 # Script de diagnóstico
/etc/
├── hostapd/hostapd.conf         # Config de hostapd
├── dhcp/dhcpd.conf              # Config de dhcpd
/var/log/wifi-ap.log             # Log de ejecución
```

---

## ⚙️ Servicios Systemd

### `/etc/systemd/system/wifi-ap.service`

```ini
[Unit]
Description=Bring up Wi-Fi AP on wlan0
After=network-online.target
Wants=network-online.target
Requires=network.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/home/msr/ap.sh
ExecStartPost=/bin/bash -c 'echo "[wifi-ap.service] Ejecutado al boot" >> /var/log/wifi-ap.log'

[Install]
WantedBy=multi-user.target
```

### `/etc/systemd/system/wifi-ap-cleanup.service`

```ini
[Unit]
Description=Clean flag file for wifi-ap
DefaultDependencies=no
Before=shutdown.target

[Service]
Type=oneshot
ExecStart=/bin/rm -f /tmp/wifi_ap_started.flag

[Install]
WantedBy=shutdown.target
```

---

## 📄 Configuración `/home/msr/ap.sh`

- Detecta si `wlan0` está disponible y la pone en modo AP
- Asigna IP
- Inicia `hostapd` si no está activo
- Inicia `dhcpd` para `wlan0` si no está activo (ignora el de `l4tbr0`)
- Guarda log en `/var/log/wifi-ap.log`
- Usa `/tmp/wifi_ap_started.flag` para evitar múltiples ejecuciones

---

## 📡 `hostapd.conf`

Ruta: `/etc/hostapd/hostapd.conf`

```ini
interface=wlan0
driver=nl80211
ssid=*****
hw_mode=g
channel=6
country_code=MX
ieee80211d=1
wpa=2
wpa_passphrase=*****
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
auth_algs=1
```

---

## 📦 `dhcpd.conf`

Ruta: `/etc/dhcp/dhcpd.conf`

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

## 🔍 Diagnóstico: `/home/msr/status_ap.sh`

```bash
#!/bin/bash
echo "🔍 Estado del Wi-Fi AP"
echo "=========================="

echo -n "📡 wlan0: "
ip link show wlan0 &>/dev/null && echo "✅" || echo "❌"

STATE=$(cat /sys/class/net/wlan0/operstate 2>/dev/null)
echo "📶 Estado: $STATE"

IP=$(ip addr show wlan0 | grep 'inet ' | awk '{print $2}')
echo "🌐 IP: ${IP:-❌ Sin IP}"

echo -n "🛠️ hostapd: "
pgrep hostapd &>/dev/null && echo "✅" || echo "❌"

echo -n "📦 dhcpd (wlan0): "
pgrep -a dhcpd | grep -q 'wlan0' && echo "✅" || echo "❌"

echo "📋 Clientes conectados:"
arp -i wlan0 -n | grep -v incomplete | tail -n +2 || echo "❌ Ninguno"
```

---

## 🌀 Activación

```bash
sudo systemctl daemon-reload
sudo systemctl enable wifi-ap.service
sudo systemctl enable wifi-ap-cleanup.service
sudo systemctl start wifi-ap.service
```

---

## 🧪 Prueba

```bash
./status_ap.sh
cat /var/log/wifi-ap.log
pgrep -a dhcpd | grep wlan0
```

---

## 🧯 Desinstalar

```bash
sudo systemctl stop wifi-ap.service
sudo systemctl disable wifi-ap.service
sudo rm /etc/systemd/system/wifi-ap.service
sudo rm /etc/systemd/system/wifi-ap-cleanup.service
```

---

## ✍️ Autor

Configuración y documentación por **Genaro Acosta**.  
Optimizado para Jetson Nano / Xavier NX en Ubuntu 20.04+.
