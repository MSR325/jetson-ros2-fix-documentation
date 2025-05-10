# 🟢 ROS 2 Color Detector Node (Jetson Optimized)

Este nodo en C++ para ROS 2 Humble está optimizado para detectar colores de semáforo (rojo, amarillo, verde) desde un stream de cámara comprimido (`compressed`). Fue diseñado específicamente para dispositivos NVIDIA Jetson, priorizando eficiencia y compatibilidad con `image_transport` y `nvjpeg`.

---

## 🎯 Objetivo

Desarrollar un nodo robusto, visualizable vía `rqt_image_view`, que:

- 📸 Reciba imágenes comprimidas a través de `image_transport` (`compressed`)
- 🧠 Detecte colores rojo, amarillo y verde usando OpenCV
- 🔄 Mantenga una detección estable, sin parpadeo
- 📤 Publique:
  - una bandera (`std_msgs/String`) con el color detectado (`red`, `yellow`, `green`, `none`)
  - la imagen anotada (`sensor_msgs/Image`) con el contorno y el color detectado en `/camera/image_processed`
- 🚀 Funcione en tiempo real (~25 FPS) en Jetson Nano/Orin

---

## ⚠️ Problemas encontrados y soluciones

### 🐢 Latencia alta (solo 12 FPS)

**Problema**: El nodo mostraba la imagen en OpenCV a solo ~12 FPS mientras `rqt_image_view` lo mostraba fluido (~30 FPS).

**Causa**: OpenCV GUI en Jetson no está optimizada, y abrir una ventana ralentiza el nodo.

**Solución**:
- ✅ Se eliminó el uso de `cv::imshow()` y `cv::waitKey()`
- ✅ Se sustituyó por publicación de imagen anotada (`camera/image_processed`) para visualización en `rqt_image_view`

---

### 🚨 Parpadeo en detección de color

**Problema**: La detección "desaparecía" entre frames, haciendo que el círculo de color parpadeara.

**Causa**: Al no detectar color en un frame, se borraba el último resultado inmediatamente.

**Solución**:
- ✅ Se mantiene el último contorno válido
- ✅ Si no se detecta nada, no se dibuja ningún contorno
- ✅ Esto evita parpadeos mientras mantiene un comportamiento consistente

---

### 📉 Caídas en FPS tras procesamiento

**Problema**: El procesamiento completo (HSV, contornos, anotación) ralentizaba los FPS.

**Solución**:
- ✅ Se agregó `detect_every_` (por default en 1 o 2) para procesar 1 de cada N frames
- ✅ El FPS se mantiene alrededor de 25 FPS en Jetson Orin

---

### ❌ Error: `Failed to create CaptureSession`

**Mensaje de error**:
```
Error generated. Failed to create CaptureSession
gstCamera::Capture() -- a timeout occurred waiting for the next image buffer
```

**Causa**: El demonio `nvargus-daemon` se bloqueó por un mal cierre de sesión CSI o conflicto con otro proceso.

**Solución**:
```bash
sudo systemctl restart nvargus-daemon
```

---

## 🧩 Requisitos

### ROS 2 Humble

Asegúrate de tener `ros-humble` y los siguientes plugins:

```bash
sudo apt install \
  ros-humble-image-transport-plugins \
  ros-humble-compressed-image-transport \
  gstreamer1.0-tools \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev
```

### Variable de entorno para NVJPEG

```bash
export COMPRESSED_IMAGE_TRANSPORT_USE_NVJPEGDEC=1
```

Esto permite a `compressed_image_transport` utilizar el plugin acelerado `nvjpeg` en Jetson.

---

## 📸 Verificar estado de la cámara

Antes de lanzar el nodo, asegúrate que tu cámara CSI funcione:

```bash
gst-launch-1.0 nvarguscamerasrc sensor-id=0 \
  ! 'video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1,format=NV12' \
  ! nvvidconv ! 'video/x-raw,format=I420' ! fakesink -e
```

Si esto imprime modos disponibles y `Starting repeat capture requests`, la cámara funciona.

---

## 🔧 Uso

### Compilación

```bash
cd ~/ros2_packages_ws
colcon build --packages-select ros_deep_learning --cmake-clean-cache
```

### Ejecución

```bash
source install/setup.bash
ros2 run ros_deep_learning color_detector_node
```

---

## 📤 Tópicos publicados

| Tópico                    | Tipo                  | Descripción                             |
|---------------------------|-----------------------|-----------------------------------------|
| `/color_flag`             | `std_msgs/msg/String` | Color detectado (`red`, `green`, etc.)  |
| `/camera/image_processed` | `sensor_msgs/msg/Image` | Imagen con anotaciones OpenCV           |

---

## 🧠 Lógica general

- Decimación opcional de detección (cada N frames)
- Procesamiento optimizado vía `cv::inRange`, `findContours`, `moments`
- Se mantiene último resultado válido hasta nuevo frame positivo
- Se evita uso de GUI en favor de publicación visual

---

## 📁 Estructura del paquete

```
ros_deep_learning/
├── src/
│   └── color_detector_node.cpp
├── CMakeLists.txt
├── package.xml
└── ...
```

---

## ✅ Conclusiones

- Este nodo aprovecha lo mejor de OpenCV, `compressed_image_transport` y Jetson.
- Minimiza el uso de recursos sin sacrificar calidad de detección.
- Compatible con flujos `compressed` estándar y acelerados (`nvjpeg`)
- Ideal para proyectos FSAE, robots de competencia y visión ligera en edge

---

¿Problemas persistentes con la cámara CSI? Reinicia:

```bash
sudo systemctl restart nvargus-daemon
```

---
