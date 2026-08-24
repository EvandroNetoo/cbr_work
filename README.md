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

Esse perfil não inicia hardware, sensores, TF, controllers, percepção,
localização ou planejamento. Não há integração de joystick no repositório;
ela deve ser adicionada somente quando os pacotes e mapeamentos reais forem
definidos.

## Mapeamento e navegação

O repositório ainda não contém SLAM Toolbox, Nav2, mapa ou parâmetros de
localização global. Por isso não existe um `mapping.launch.py` fictício. Quando
essa infraestrutura for implementada, mapping e localização sobre mapa devem
executar na Banana Pi e ser integrados ao perfil embarcado.

Launchs de simulação antigos foram removidos desta arquitetura; simulação não é
parte do escopo atual.
