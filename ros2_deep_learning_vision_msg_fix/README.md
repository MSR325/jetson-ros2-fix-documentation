# 🧠 ros_deep_learning – Jetson ROS2 Integration Fix Log

### 📝 Resumen rápido para principiantes

Este proyecto está basado en **ROS 2 Humble** corriendo en una **Jetson Nano**, utilizando la librería `jetson-inference` de NVIDIA para tareas de visión por computadora. Al intentar compilar el paquete `ros_deep_learning` con `colcon build`, surgieron varios errores por dependencias faltantes, incompatibilidades entre versiones de ROS y problemas de vinculación con CUDA.

Este README documenta todos los problemas encontrados, cómo fueron solucionados, y los pasos exactos de verificación para que futuras personas puedan replicarlo.

---

## 🚨 Problemas detectados y soluciones aplicadas

### ❌ Error #1: `classification.hpp` no encontrado
```
fatal error: vision_msgs/msg/classification.hpp: No such file or directory
```

#### 🔧 Solución:
- Se confirmó que el archivo **sí existe** en el sistema:
```bash
find /opt/ros/humble/include -name classification.hpp
# /opt/ros/humble/include/vision_msgs/vision_msgs/msg/classification.hpp
```
- Se modificó `ros_compat.h` para ROS2 con la ruta correcta:
```cpp
#if ROS_DISTRO >= ROS_GALACTIC
  #include <vision_msgs/vision_msgs/msg/classification.hpp>
#else
  #include <vision_msgs/msg/classification2_d.hpp>
#endif
```
- En `CMakeLists.txt`, se agregó:

```cmake
include_directories(
  ${CUDA_INCLUDE_DIRS}
  ${vision_msgs_INCLUDE_DIRS}
)
```
y:
```cmake
ament_target_dependencies(color_detector_node rclcpp std_msgs sensor_msgs cv_bridge OpenCV vision_msgs)
```

---

### ❌ Error #2: `undefined reference to cuda*` al compilar `image_converter.cpp`
```
undefined reference to `cudaFreeHost`
undefined reference to `cudaConvertColor`
```

#### 🔧 Solución:
- Se estaban usando funciones de `jetson-utils` sin enlazar correctamente.
- Se agregó a `color_detector_node` el enlace explícito:

```cmake
target_link_libraries(color_detector_node
  ${catkin_LIBRARIES}
  jetson-utils
)
```

---

### ⚠️ Warnings comunes detectados
```
warning: control reaches end of non-void function [-Wreturn-type]
```

#### 🔧 Solución:
- Se revisaron los `node_segnet.cpp` y `node_detectnet.cpp` y se añadió un `return false;` al final de funciones tipo `bool`.

---

## ✅ Procesos de verificación

### 1. **Limpieza del cache de CMake**
Para evitar errores arrastrados:

```bash
colcon build --packages-select ros_deep_learning --cmake-clean-cache
```

### 2. **Verificar instalación de `vision_msgs`**
```bash
ros2 interface list | grep vision_msgs
# Deberías ver:
# vision_msgs/msg/Classification
# vision_msgs/msg/Detection2DArray
```

Si no aparece:
```bash
sudo apt install ros-humble-vision-msgs
```

### 3. **Confirmar que `classification.hpp` está donde debe:**
```bash
ls /opt/ros/humble/include/vision_msgs/vision_msgs/msg/classification.hpp
```

### 4. **Verificar que `jetson-utils` esté instalado**
```bash
dpkg -l | grep jetson
# O busca manualmente en `/usr/include/jetson-utils`
```

---

## 🧪 Validación de build

```bash
cd ~/ros2_packages_ws
colcon build --packages-select ros_deep_learning --cmake-clean-cache
```

> Tiempo de compilación esperado en Jetson Nano: 6–12 minutos

---

## 📁 Estructura recomendada del proyecto

```bash
ros2_packages_ws/
└── src/
    └── ros_deep_learning/
        ├── CMakeLists.txt
        ├── package.xml
        ├── src/
        │   ├── color_detector_node.cpp
        │   ├── image_converter.cpp
        │   └── ros_compat.cpp
        └── launch/
```

---

## 🗂️ Nombre sugerido para esta carpeta

```bash
ros_deep_learning_fix_humble_jetson
```

---

## 🧠 Recomendación final

> Si usas tanto ROS 1 como ROS 2 en el mismo proyecto, ten especial cuidado con `ros_compat.h` y `CMakeLists.txt`, y asegúrate de probar siempre con `--cmake-clean-cache` si modificas dependencias de bajo nivel.
