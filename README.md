# 🛠️ Jetson ROS 2 Fix Documentation

Este repositorio tiene como objetivo registrar y documentar de forma clara y accesible todos los fixes, configuraciones especiales y soluciones aplicadas durante el desarrollo e integración de proyectos ROS 2 sobre plataformas Jetson (Nano, Xavier, Orin, etc).

## 📚 Propósito

Durante la implementación de sistemas embebidos y visión computacional en plataformas Jetson con ROS 2, es común enfrentar problemas como:

- Errores de compilación  
- Conflictos con dependencias  
- Fallos de red o periféricos  
- Configuraciones específicas del sistema operativo  

Este repositorio busca dejar un historial documentado de cada fix implementado, para:

- Facilitar la trazabilidad de cambios y soluciones aplicadas  
- Acelerar la resolución de errores recurrentes  
- Servir como guía para otros desarrolladores o futuras generaciones del equipo  

## 📂 Estructura del repositorio

Cada fix o problema se documentará en una carpeta independiente con su propio `README.md`, la cual contendrá:

- Descripción del problema  
- Sistema afectado y condiciones específicas  
- Pasos para replicar el error  
- Solución implementada  
- Comandos, archivos modificados o logs relevantes  
- Resultado esperado o validación  

Ejemplo de estructura:

```
jetson-ros2-fix-documentation/
├── README.md
├── wifi-hotspot-boot/
│   └── README.md
├── ethernet-nmcli-permission/
│   └── README.md
├── ros2-colcon-build-error/
│   └── README.md
└── ...
```

## ✍️ Cómo contribuir

1. Crea una carpeta con un nombre breve y descriptivo del problema  
2. Dentro, crea un `README.md` que documente claramente el fix  
3. Agrega capturas, logs o scripts si son relevantes para entender o replicar la solución  
4. Realiza un commit con mensaje claro: `fix: document wifi hotspot boot issue`  

> ✅ Este repositorio **no** almacena código funcional, únicamente documentación técnica de fixes y configuraciones.

---

Desarrollado y mantenido por [@MSR325](https://github.com/MSR325)
