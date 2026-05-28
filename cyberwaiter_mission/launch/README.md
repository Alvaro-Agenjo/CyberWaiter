Bringup launch
================

Este launch inicia los nodos kin_silver, navigator, gripper y mission.

Uso
----

Desde el paquete `cyberwaiter_mission` ejecuta:

```bash
ros2 launch cyberwaiter_mission bringup_launch.py
```

Si los ejecutables o paquetes tienen nombres distintos, sobreescribe los argumentos:

```bash
ros2 launch cyberwaiter_mission bringup_launch.py \
  kin_pkg:=<pkg> kin_exec:=<exe> \
  nav_pkg:=<pkg> nav_exec:=<exe> \
  gripper_pkg:=<pkg> gripper_exec:=<exe> \
  mission_pkg:=<pkg> mission_exec:=<exe>
```

Por ejemplo, si el ejecutable de misión se llama `mision_control` en el paquete `cyberwaiter_mission`:

```bash
ros2 launch cyberwaiter_mission bringup_launch.py mission_exec:=mision_control
```
