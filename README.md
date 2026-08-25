# CBR ROS 2 Workspace

Pacotes ROS 2 Jazzy do robô CBR. A Banana Pi é autônoma: hardware, sensores,
localização, percepção, controllers e MoveIt executam nela. O notebook apenas
consome DDS para visualização e teleoperação opcional.

## Build

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install
source install/setup.bash
```

## Banana Pi

Perfil completo de competição:

```bash
ros2 launch bringup robot.launch.py
```

Desenvolvimento de braço, garra, percepção e MoveIt sem a base:

```bash
ros2 launch bringup robot.launch.py enable_base:=false
```

Desenvolvimento da base com LiDAR, IMU e EKF, sem braço ou visão:

```bash
ros2 launch bringup robot.launch.py \
  enable_arm:=false enable_moveit:=false enable_perception:=false
```

Os tópicos internos do hardware são `/so101_hardware/raw_joint_states` e
`/base_hardware/raw_joint_states`. A interface pública inclui `/joint_states`,
`/cmd_vel`, `/wheel/odom`, `/odom`, `/scan_front`, `/camera/image_rect`, TF e as
actions dos controllers/MoveIt.

## Workstation

Use o mesmo `ROS_DOMAIN_ID` do robô e execute:

```bash
ros2 launch bringup workstation.launch.py
```

Teleop do braço é opcional:

```bash
ros2 launch bringup workstation.launch.py enable_keyboard_teleop:=true
```

Teleop da base com controle Xbox USB/Bluetooth:

```bash
ros2 launch bringup workstation.launch.py enable_xbox_teleop:=true
```

Segure `RB` para habilitar o movimento. O stick esquerdo controla avanço e
deslocamento lateral; o eixo horizontal do stick direito controla o giro.
Segure também `LB` para turbo. Soltar `RB` ou perder a conexão publica parada;
o timeout do controller fornece uma segunda camada de segurança.

Esse perfil não inicia hardware, sensores, TF, controllers, percepção,
localização ou planejamento. O mapeamento e os limites do controle ficam na
seção `xbox_base_teleop` de `bringup/config/controllers.yaml`.

## Mapeamento e navegação

O perfil autônomo é opt-in e inicia AMCL, Nav2 e os dois nós de missão:

```bash
ros2 launch bringup robot.launch.py \
  enable_navigation:=true enable_mission:=true
```

Antes do primeiro uso, configure as poses reais e `enabled: true` em
`mission_manager/config/missions.yaml`. Cadastre também as poses articulares
da caixa direita no SRDF e seus nomes em `bringup/config/manipulation.yaml`.
Enquanto esses valores não forem fornecidos, a missão é rejeitada sem mover o
robô.

Para iniciar a missão de exemplo e acompanhar feedback/cancelamento:

```bash
ros2 action send_goal --feedback /mission/execute \
  interfaces/action/ExecuteMission "{mission_name: exemplo}"
```

O mapa estático, AMCL e Nav2 são instalados pelo pacote `bringup`. SLAM continua
sendo uma operação separada de manutenção do mapa e não roda junto com AMCL.

Launchs de simulação antigos foram removidos desta arquitetura; simulação não é
parte do escopo atual.
