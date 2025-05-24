# 🤖 Conexión y Control de xArm con ROS 2 y Gazebo Classic

Este repositorio documenta el proceso de conexión, configuración y puesta en marcha del brazo robótico **xArm** usando **ROS 2**, **MoveIt**, y **Gazebo Classic**.

---

## 🔧 Requisitos previos

Asegúrate de tener instalados los siguientes componentes antes de comenzar:

- Ubuntu 22.04 (preferentemente)
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [Gazebo Classic](http://gazebosim.org/tutorials?tut=install_ubuntu&ver=classic)
- `ros2_control`, `ros2_controllers`
- Interfaces específicas de `ros2_control` para xArm
- Paquetes ROS 2 de MoveIt
- Paquetes ROS 2 de integración Gazebo (`gazebo_ros_pkgs`)

---

## 🛜 Configuración de red

1. Conecta el cable Ethernet del xArm directamente a tu PC.
2. Ve a **Configuración de red** → **Red cableada** → **IPv4**.
3. Establece el modo **Manual** y usa la siguiente configuración:

| Dirección IP   | Máscara de red     | Puerta de enlace   |
|----------------|--------------------|---------------------|
| `192.168.1.50` | `255.255.255.0`    | `192.168.1.1`       |

🔁 **Importante:** A veces puede tardar un poco en aparecer como *Conectado*. Ubuntu mostrará `Wired Connection - Connected`.

4. Verifica conectividad haciendo ping a la IP del xArm (ejemplo):

```bash
ping 192.168.1.206
```

📸 **Referencia visual de configuración de red:**

![Configuración IPv4 Manual](./config-ipv4-xarm.png)

---

## 🛠️ Instalación de xArm ROS 2

Sigue los pasos del repositorio oficial de [xarm_ros2](https://github.com/xArm-Developer/xarm_ros2):

```bash
cd ~/ros2_ws/src
git clone https://github.com/xArm-Developer/xarm_ros2.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

---

## 🚀 Ejecución con brazo real y MoveIt

En tu workspace de ROS 2 ejecuta el siguiente launch:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py robot_ip:=192.168.1.206
```

---

## 🧪 Simulación con Gazebo Classic

En caso de no tener el hardware disponible, puedes utilizar Gazebo Classic:

```bash
ros2 launch xarm_gazebo xarm6_beside_table.launch.py
```

---

## 📝 Notas adicionales

- El brazo puede tardar unos segundos en estar completamente disponible tras el lanzamiento.
- Se recomienda siempre usar `source install/setup.bash` antes de ejecutar cualquier comando ROS 2.
- Si el `ping` al xArm falla, verifica tu firewall y que el adaptador Ethernet esté en modo activo.

---

## 🧑‍💻 Autor

Este repositorio fue documentado por [Tu Nombre] como parte del proyecto de integración ROS 2 + xArm para control en tiempo real.
