# cbr_bringup

`robot.launch.py` é o perfil embarcado do robô composto. Ele executa os
drivers do braço e da base, um único `controller_manager`, um único
`joint_state_broadcaster`, os controllers e o `move_group` na Banana Pi, sem
iniciar RViz, Gazebo ou teleop. Ele também inicia sempre o driver da câmera, a
retificação calibrada e o detector AprilTag. O notebook é usado apenas para
visualização.

A futura lógica autônoma deve ser incluída neste perfil depois que os
controllers e o `/move_action` estiverem prontos; o notebook não é requisito
para essa inicialização.

Os drivers físicos podem levar cerca de 20 segundos para conectar na Banana
Pi. O launch aguarda até 45 segundos pelos estados completos das seis juntas
comandadas do braço e das quatro rodas antes de iniciar os controllers. A saída
de qualquer driver encerra todo o stack.

```bash
ros2 launch cbr_bringup robot.launch.py \
  port:=/dev/ttyUSB0 \
  hardware_state_timeout:=60.0
```

Os argumentos acima pertencem ao braço e ao gate geral. A base não possui
configuração por CLI: hardware, geometria e controllers ficam nos YAMLs dos
pacotes `cbr_base_hardware`, `cbr_base_bringup` e `cbr_bringup`.
