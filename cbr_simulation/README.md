# Simulação CBR no Gazebo Sim

Este pacote monta a arena a partir do mapa de ocupação e das poses da missão,
carrega o robô com `gz_ros2_control`, sensores Gazebo e a mesma pilha ROS 2
usada no robô físico. O perfil simulado usa `/clock` e arquivos próprios de
controladores, localização e visão. Não abre as portas seriais nem a câmera USB.

## Início

Requer ROS 2 Jazzy, Gazebo Harmonic, `ros_gz_sim`, `ros_gz_bridge`,
`gz_ros2_control`, Navigation2, `robot_localization`, MoveIt e as dependências
dos pacotes do workspace. Na raiz do workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to cbr_simulation
source install/setup.bash
ros2 launch cbr_simulation simulation.launch.py gui:=true
```

Use `gui:=false` para execução sem janela. A pose inicial padrão é
`(2.5, 2.0, 0)` no mapa `arena`. `processing_components` aceita a lista dos
componentes do `processing.launch.py`, por exemplo
`processing_components:=ekf,vision,moveit,manipulation` para inspecionar os
sensores e o braço sem a navegação. Os argumentos `x`, `y` e `yaw` reposicionam
somente o modelo no Gazebo; a pose inicial da localização e o início da missão
continuam fixos em `(2.5, 2.0, 0)`. Use esses argumentos apenas nos testes sem
AMCL e missão até que as três poses sejam parametrizadas juntas.

A cena é gerada por:

```bash
/usr/bin/python3 src/work/cbr_simulation/scripts/generate_arena.py
```

O script lê `bringup/maps/arena.pgm` e
`mission_manager/config/arena.yaml`. As tags são quadrados pretos e brancos
modelados no SDF; os cubos e contêineres têm colisão física.

## Estado verificado

- Modelo URDF/SDF válido e controllers de base, braço e garra ativos.
- Deslocamento lateral mecanum, LiDAR, IMU, câmera e dois sensores de distância.
- EKF, AMCL, Nav2 e MoveIt ativos; navegação até WS 1 concluiu com sucesso.
- `FollowWall` chegou a 35 mm da borda da mesa com odometria válida.
- Visão identificou AprilTags 2 e 10, contêiner vermelho e 291 células livres
  na grade da mesa em um ensaio na WS 1.
- A action de coleta executou o movimento do braço e informou sucesso no ROS.

## Limite atual

A coleta ainda **não transfere o cubo fisicamente**: após a action de coleta
do cubo 2 informar sucesso, a pose de `cube_2` no Gazebo permaneceu em
`(4.30026, 1.515, 0.07)`; o chassi também inclinou durante a
manobra. Portanto, armazenamento, transporte, depósito,
empilhamento e missão completa não estão validados como operações físicas.
É necessário integrar pegada e liberação baseadas em contato e confirmar o
estado do objeto antes de considerar uma action concluída. O perfil atual serve
para testar navegação, percepção, aproximação e trajetórias do braço, mas
**não é uma simulação 100% funcional da missão**.
