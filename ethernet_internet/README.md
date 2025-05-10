# 📡 Jetson Hotspot con Salida a Internet vía Ethernet (eth0)

Este proyecto configura una Jetson (como una Jetson Nano, TX2 o Xavier) para funcionar como un **hotspot WiFi** en `wlan0`, mientras **comparte su conexión a Internet recibida por cable Ethernet (`eth0`)** hacia los clientes conectados al hotspot. Además, se corrige el DNS para garantizar el acceso a dominios como `google.com`.

---

## ✅ Requisitos

- Jetson con Ubuntu (L4T)
- Acceso a superusuario (`sudo`)
- Interfaces `eth0` (cable) y `wlan0` (WiFi)
- `hostapd`, `dnsmasq`, `iptables`, `nmcli`

---

## 🔎 Diagnóstico paso a paso

Durante la configuración se detectaron los siguientes problemas y se resolvieron en orden:

1. **`apt update` fallaba** con errores tipo `Could not resolve 'packages.ros.org'`, indicando **falla de DNS**.
2. Se ejecutó `ip a` para verificar que `eth0` tenía IP válida (`192.168.100.2`), lo que confirmó **conectividad física con el módem**.
3. Se revisó la tabla de rutas (`ip route`) y se encontró que **la salida por defecto (`default`) usaba `l4tbr0`**, una interfaz virtual del hotspot. Esto impedía que el tráfico saliera por `eth0`.
4. Se cambió la ruta por defecto a través del gateway del módem: `sudo ip route add default via 192.168.100.1 dev eth0`.
5. El `ping 8.8.8.8` comenzó a funcionar, **pero aún fallaba `ping google.com`**, lo cual confirmó **problema persistente de DNS**.
6. Se revisó `/etc/resolv.conf` y se vio que usaba `127.0.0.53`, el stub de `systemd-resolved`, pero no estaba resolviendo nombres.
7. Se inspeccionó `systemd-resolve --status` y no había DNS configurado en la interfaz activa.
8. Finalmente, se usó `nmcli` para forzar DNS manual a `eth0`, deshabilitando el uso de auto-DNS.

---

## 🔧 Pasos realizados

### 1. Activar IP Forwarding (reenviar tráfico entre interfaces)

```bash
sudo sysctl -w net.ipv4.ip_forward=1
echo "net.ipv4.ip_forward=1" | sudo tee -a /etc/sysctl.conf
```

---

### 2. Configurar NAT (traducción de direcciones)

```bash
sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
sudo iptables -A FORWARD -i eth0 -o wlan0 -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i wlan0 -o eth0 -j ACCEPT
```

### 2.1 Guardar reglas iptables (opcional)

```bash
sudo apt install iptables-persistent
sudo netfilter-persistent save
```

---

### 3. Cambiar la ruta por defecto a través de `eth0`

```bash
sudo ip route del default
sudo ip route add default via 192.168.100.1 dev eth0
```

*`192.168.100.1` es el gateway del módem conectado por Ethernet.*

---

### 4. Configurar DNS manual para `eth0` con `nmcli`

```bash
sudo nmcli con mod "Wired connection 1" ipv4.dns "8.8.8.8 1.1.1.1"
sudo nmcli con mod "Wired connection 1" ipv4.ignore-auto-dns yes
```

### 4.1 Reiniciar la conexión Ethernet

```bash
sudo nmcli con down "Wired connection 1"
sudo nmcli con up "Wired connection 1"
```

---

### 5. Verificación de DNS

```bash
systemd-resolve --status
```

Deberías ver:
```
DNS Servers: 8.8.8.8 1.1.1.1
DefaultRoute setting: yes
```

Y luego:

```bash
ping google.com
```

---

## 📦 Archivos recomendados

- `ap.sh`: script que activa el hotspot (hostapd, dnsmasq)
- `wifi-ap.service`: servicio systemd que ejecuta `ap.sh` al arranque
- `fix_routing_dns.sh`: script opcional para aplicar NAT, rutas y DNS automáticamente

---

## 🧠 Nota

Este enfoque **NO desactiva el hotspot WiFi**, simplemente rerutea el tráfico saliente para que la Jetson (y sus clientes WiFi) tengan acceso real a Internet **vía Ethernet**.

---

## 🛠️ Troubleshooting

| Problema | Solución |
|---------|----------|
| `ping google.com` no resuelve | Verifica `resolv.conf` y `systemd-resolve --status` |
| Clientes del hotspot no tienen Internet | Verifica `iptables` y que `eth0` tenga salida |
| DNS se borra al reiniciar | Usa `nmcli` para ignorar auto-DNS |

---

## ✍️ Autor

- ⚙️ Configurado por: `msr@jetson`
- 🧠 Asistido por: ChatGPT (OpenAI)
