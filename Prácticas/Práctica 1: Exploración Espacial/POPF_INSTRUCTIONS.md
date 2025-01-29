# Instrucciones para instalar y ejecutar el planificador POPF

**Nota importante: Se asume que se tiene ROS 2 (jazzy) instalado en el sistema y que se ha realizado el source correspondiente para cargar las variables de entorno.**

## Cómo instalar [POPF](https://github.com/fmrico/popf)
POPF está disponible como paquete Debian en los repositorios de ROS, por lo que se puede instalar directamente desde apt en Ubuntu:

```bash
sudo apt install ros-jazzy-popf
```

## Cómo ejecutar POPF para generar un plan:
Se necesitan los archivos PDDL con la definición del dominio y del problema. En este ejemplo asumimos que estos archivos se llaman `x_domain.pddl` y `x_problem.pddl`, respectivamente.

```bash
ros2 run popf popf x_domain.pddl x_problem.pddl
```

Puedes ver las opciones de ejecución de POPF con el siguiente comando:

```bash
ros2 run popf popf -h
```
