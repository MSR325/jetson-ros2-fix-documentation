# 📷 Uso de Cámaras MIPI con NVIDIA Jetson y ROS 2

Este documento describe las buenas prácticas para trabajar con cámaras MIPI-CSI (como IMX219 o IMX477) en plataformas Jetson (Nano, Xavier NX, AGX Orin), especialmente al usarlas con ROS 2 y GStreamer.

## 🧠 Requisitos

- Jetson con JetPack instalado correctamente  
- Cámara MIPI-CSI conectada al puerto CSI  
- `nvarguscamerasrc` disponible (verificado con `gst-launch-1.0`)  
- Driver NVIDIA cargado correctamente (verificable con `glxinfo | grep NVIDIA`)

## ⚠️ Consideraciones sobre SSH

### ❌ No usar `ssh -X` ni `ssh -Y`

Cuando usas:

```bash
ssh -X usuario@jetson
```

Esto activa X11 Forwarding, lo cual:
- Asigna `DISPLAY=localhost:10.0`
- Deshabilita el acceso al EGL nativo de Jetson
- Usa renderizado por software (`llvmpipe`)
- Provoca errores como:

```
nvbuf_utils: Could not get EGL display connection  
NvEGLImageFromFd: No EGLDisplay to create EGLImage
```

### ✅ En su lugar, usar:

```bash
ssh usuario@jetson
```

Y si necesitas entorno gráfico:

```bash
export DISPLAY=:0
```

Este valor asume que estás ejecutando en el entorno gráfico local de Jetson (no por SSH con forwarding).

## 🚀 Ejemplo de prueba con GStreamer

```bash
export DISPLAY=:0
gst-launch-1.0 nvarguscamerasrc ! \
'video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1, format=NV12' ! \
nvvidconv ! nvoverlaysink
```

## 🛠 Alternativas para entornos sin GUI (headless)

Si estás trabajando en modo consola (sin GUI), usa `appsink`, `fakesink` o guarda a disco en lugar de `nvoverlaysink`:

```bash
gst-launch-1.0 nvarguscamerasrc ! \
'video/x-raw(memory:NVMM), width=640, height=480, format=NV12' ! \
nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! fakesink
```

Esto evita dependencias de EGL y visualización.

## 🐞 Diagnóstico rápido

```bash
echo $DISPLAY
# Si es :0 → OK
# Si es localhost:10.0 → ssh -X activo → ❌

glxinfo | grep OpenGL
# Debe decir "NVIDIA", NO "llvmpipe"
```

Si ves `llvmpipe` estás en renderizado por software → no funcionará EGL.

## 🧩 Tips ROS 2

Si usas un nodo ROS 2 que emplea `nvarguscamerasrc`, asegúrate de:

1. No usar `ssh -X`
2. Exportar `DISPLAY=:0` si tienes GUI activa
3. Reiniciar el daemon si es necesario:

```bash
sudo systemctl restart nvargus-daemon
```

## ✨ Recomendación

Para pruebas visuales, trabaja localmente en la Jetson (conectado por monitor y teclado o escritorio remoto con soporte EGL). Para entornos headless, usa `appsink` y evita sinks como `nvoverlaysink` o buffers NVMM que dependan de EGL.
