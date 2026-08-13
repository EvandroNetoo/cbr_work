# SO-ARM-101 follower no ROS 2 Jazzy

Este projeto controla somente o braço follower do SO-ARM101. O braço líder e a
garra original não fazem parte da arquitetura. A ferramenta instalada é uma
garra paralela adaptada, com uma junta física comandada (`right_clamp`) e uma
junta passiva (`left_clamp`) que a acompanha por `mimic`.

## Arquitetura

```text
cbr_bringup              -> perfil embarcado do robô (Banana Pi)
so_arm_101_description  -> URDF/Xacro, ros2_control, limites e meshes
so_arm_101_bringup      -> launch headless, controllers, RViz e Gazebo
so_arm_101_hardware     -> nó Python LeRobot/Feetech do follower
so_arm_101_hardware_interface -> plugin C++ SystemInterface
so_arm_101_moveit_config -> MoveIt 2, SRDF, OMPL e RViz de planejamento
so_arm_101_teleop       -> teclado opcional para os controllers
so_arm_101              -> metapacote
```

O nó Python conhece a API do LeRobot. O plugin C++ implementa
`hardware_interface::SystemInterface`, exporta as interfaces para o
`controller_manager` e troca estados/comandos com o nó Python por tópicos
internos. Assim, o código dependente do vendor fica fora do ciclo de controle.

## Estrutura dos diretórios

```text
so_arm_101_description/       urdf/, config/, meshes/
so_arm_101_hardware/          adapter LeRobot, parâmetros e calibração
so_arm_101_hardware_interface/ plugin C++ e registro pluginlib
so_arm_101_bringup/            launch/, controllers.yaml e worlds/
so_arm_101_moveit_config/      SRDF, cinemática, OMPL e launch MoveIt
so_arm_101_teleop/             nó de teclado
```

## Requisitos e build

Requisitos principais: Ubuntu 24.04, ROS 2 Jazzy, `colcon`, `xacro`,
`ros2_control`, Gazebo Sim e MoveIt 2. O hardware real também requer LeRobot
com suporte Feetech em um ambiente Python compatível com o ROS instalado.

Na raiz do workspace:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install
source install/setup.bash
```

Se o `rosdep` reportar que não conhece o próprio `ament_python`, isso é uma
limitação da base rosdep local; o pacote continua usando o build type correto
e o build deve ser a verificação final.

## Verificar a descrição

```bash
xacro src/cbr_work/so_arm_101/so_arm_101_description/urdf/so_101.urdf.xacro \
  -o /tmp/so_arm_101.urdf
check_urdf /tmp/so_arm_101.urdf
```

Para o hardware, o Xacro recebe `use_real_ros2_control:=true` e o plugin
`so_arm_101_hardware_interface/SO101System`. Para Gazebo, recebe
`use_gz_ros2_control:=true`.

## RViz e simulação

Visualização do robô em execução (no notebook):

```bash
ros2 launch so_arm_101_bringup rviz.launch.py
```

Inspeção offline do modelo, usando sliders:

```bash
ros2 launch so_arm_101_bringup model_demo.launch.py
```

Gazebo com `gz_ros2_control`:

```bash
ros2 launch so_arm_101_bringup sim.launch.py
ros2 launch so_arm_101_bringup sim.launch.py headless:=true
```

O teclado (executado separadamente no notebook) publica `JointTrajectory` em
`/arm_controller/joint_trajectory` e
`/gripper_controller/joint_trajectory`. As teclas são `q/a`, `w/s`, `e/d`,
`r/f`, `t/g` para as cinco juntas do braço e `y/h` para fechar/abrir a garra.
Execute-o no notebook com `ros2 run so_arm_101_teleop keyboard_teleop`.

## Hardware real

Instale o LeRobot no ambiente Python usado para executar o nó. O arquivo
`so_arm_101_hardware/config/so101_follower.json` é um exemplo de calibração;
`so_arm_101_hardware/config/gripper_calibration.yaml` guarda os endpoints
angulares da garra adaptada. Confirme IDs, offsets e faixas antes de energizar
o braço.

Inicie o sistema completo, informando explicitamente a porta:

```bash
ros2 launch so_arm_101_bringup real.launch.py \
  port:=/dev/ttyUSB0 robot_id:=meu_so101
```

Esse launch inicia o driver LeRobot, `robot_state_publisher`,
`ros2_control_node`, `joint_state_broadcaster`, `arm_controller` e
`gripper_controller`. O launch `so_arm_101_hardware driver.launch.py` inicia
somente o driver para diagnóstico; `real.launch.py` nesse pacote é mantido
como wrapper de compatibilidade.

Para executar o perfil embarcado completo na Banana Pi, com planejamento
MoveIt e sem telas:

```bash
ros2 launch cbr_bringup robot.launch.py \
  port:=/dev/ttyUSB0 robot_id:=meu_so101
```

O perfil embarcado inicia também a câmera calibrada e o detector AprilTag:

```bash
ros2 launch cbr_bringup robot.launch.py \
  port:=/dev/ttyUSB0
```

Para calibrar pelo serviço do driver:

```bash
ros2 service call /so101_hardware/calibrate std_srvs/srv/Trigger '{}'
```

## MoveIt 2

Simulação completa com MoveIt e RViz:

```bash
ros2 launch so_arm_101_moveit_config demo.launch.py
```

Hardware real e MoveIt no computador do robô:

```bash
ros2 launch so_arm_101_moveit_config real_planning.launch.py \
  port:=/dev/ttyUSB0 robot_id:=meu_so101
```

No notebook, abra apenas a interface gráfica do MoveIt:

```bash
ros2 launch so_arm_101_moveit_config moveit_rviz.launch.py
```

O MoveIt usa o grupo `arm` para as cinco juntas revolutas e o grupo `gripper`
para os dois dedos. A execução usa as actions padrão dos
`JointTrajectoryController`:

```text
/arm_controller/follow_joint_trajectory
/gripper_controller/follow_joint_trajectory
```

### Pegar e colocar por AprilTag

Em `so_arm_101_moveit_config/so_arm_101_moveit_config/configuracao.py`, defina
`APRIL_TAG_ID` com o ID preso ao objeto:

```python
APRIL_TAG_ID: Final[int | None] = 7
```

A sequência move a câmera para o estado `detect_apriltags`, chama a ação
`/apriltags/analyze` por `TEMPO_DE_ANALISE_DA_APRIL_TAG`, procura o ID no
resultado transformado para `base_link` e usa a posição da tag como
`OBJETO_X`, `OBJETO_Y` e `OBJETO_Z`. Como a tag fica sobre o cubo e o TCP está
na ponta da garra, a sequência subtrai `TAMANHO_DO_CUBO` do `Z` detectado para
calcular a altura da pegada. O valor inicial é `0.05` m (5 cm). O yaw da
pegada vem da orientação detectada da tag. Como o objeto é um cubo e a garra é
paralela, orientações separadas por 90 graus são equivalentes; a sequência
escolhe automaticamente a equivalente no intervalo de -45 a +45 graus para
evitar uma rotação desnecessária do punho.

Para manter o comportamento cartesiano anterior, use:

```python
APRIL_TAG_ID: Final[int | None] = None
```

Nesse modo, o yaw continua vindo de `ANGULO_DO_OBJETO_EM_GRAUS` e recebe a
mesma normalização por simetria.

Se o detector, a transformação para `base_link` ou o ID solicitado não
estiver disponível, a sequência é interrompida antes da aproximação.

## Garra adaptada

`right_clamp` é uma junta `prismatic` de `0.0 m` fechada a `0.037 m` aberta.
`left_clamp` é passiva, tem limite `[-0.037, 0.0]` e usa
`mimic multiplier="-1"`; ela não exporta interface no `ros2_control` e não é
enviada ao servo.

Os limites físicos do modelo ficam em
`so_arm_101_description/config/joint_limits.yaml`. O Xacro, o teleop e o
adapter de hardware usam esse arquivo. A conversão restante é específica do
hardware: a posição linear ROS da garra é convertida para o ângulo do servo
LeRobot, usando o endpoint angular existente de `1.7428 rad`. Esse endpoint e
os valores de `so101_follower.json` ainda precisam ser confirmados por medição
no conjunto mecânico real; não foram alterados por suposição.

O `so_arm_101_moveit_config/config/joint_limits.yaml` contém somente a
tolerância numérica adicional do planejamento e limites de aceleração. Ele não
substitui os limites físicos do URDF.

O endpoint angular do servo fica em
`so_arm_101_hardware/config/gripper_calibration.yaml`; ele é separado dos
limites geométricos do URDF porque pertence à calibração física do atuador.

## Diagnóstico

```bash
ros2 node list
ros2 topic list -t
ros2 topic echo /joint_states --once
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 action info /arm_controller/follow_joint_trajectory
ros2 topic echo /tf --once
ros2 service list | sort
```

Estados devem chegar a `/joint_states` pelo `joint_state_broadcaster`. No
hardware, os tópicos internos são `/so101_hardware/raw_joint_states` e
`/so101_hardware/command_positions`; eles não substituem a API pública dos
controllers.

## Testes locais

```bash
/usr/bin/python3 -m pytest src/cbr_work/so_arm_101/so_arm_101_description/test -v
/usr/bin/python3 -m pytest src/cbr_work/so_arm_101/so_arm_101_bringup/test -v
/usr/bin/python3 -m pytest src/cbr_work/so_arm_101/so_arm_101_moveit_config/test -v
/usr/bin/python3 -m pytest src/cbr_work/so_arm_101/so_arm_101_teleop/test -v
```

O teste do hardware real não conecta automaticamente à porta serial. Confira
interfaces e limites com o braço desligado primeiro e faça depois uma
trajetória pequena e supervisionada.

## Como expandir

Adicione novos links/juntas e limites em `description`, atualize o SRDF e os
controllers apenas quando a interface for realmente controlável, e acrescente
um teste de consistência. Novas formas de execução devem compor os launchs de
`bringup`; não copie novamente a criação do `robot_description`, do
`controller_manager` ou dos spawners.
