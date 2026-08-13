# cbr_bringup

`robot.launch.py` é o perfil embarcado do robô. Ele executa hardware,
`ros2_control`, controllers e `move_group` na Banana Pi, sem iniciar RViz,
Gazebo ou teleop. Ele também inicia sempre o driver da câmera, a retificação
calibrada e o detector AprilTag. O notebook é usado apenas para visualização.

A futura lógica autônoma deve ser incluída neste perfil depois que os
controllers e o `/move_action` estiverem prontos; o notebook não é requisito
para essa inicialização.

O driver físico pode levar cerca de 20 segundos para conectar na Banana Pi.
O launch aguarda até 45 segundos por um estado completo das juntas antes de
iniciar os controllers. O limite pode ser alterado, se necessário:

```bash
ros2 launch cbr_bringup robot.launch.py \
  port:=/dev/ttyUSB0 \
  hardware_state_timeout:=60.0
```
