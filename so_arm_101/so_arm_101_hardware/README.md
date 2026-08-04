# so_arm_101_hardware

Ponte ROS 2 para o SO-ARM-101 físico usando `SO101Follower` e o barramento
Feetech do LeRobot. O nó publica `/joint_states` em radianos e recebe
`trajectory_msgs/JointTrajectory` nos mesmos tópicos usados pelo teleop e
pelos controladores do projeto.

## Instalação

O ROS 2 Jazzy deste workspace usa o Python do sistema (`/usr/bin/python3`).
Não instale LeRobot no Python global nem no Python selecionado pelo `pyenv`:
isso pode quebrar o PEP 668 e deixar o `rclpy` indisponível. Use o ambiente
virtual do workspace:

```bash
cd /home/evandro/ros2_ws
source /opt/ros/jazzy/setup.bash
uv venv --python /usr/bin/python3 .venv
uv pip install --python .venv/bin/python -r requirements/lerobot-feetech.txt
source .venv/bin/activate
python -c "import rclpy, lerobot, serial, scservo_sdk; print(lerobot.__version__)"
.venv/bin/colcon build --symlink-install --base-paths src/cbr_work --packages-select so_arm_101_hardware
source install/setup.bash
```

Em cada novo terminal, carregue ROS, o venv e o workspace nessa ordem. Use
`.venv/bin/colcon` explicitamente: o alias `build` do shell pode selecionar o
`colcon` do sistema e gerar o executável ROS com `/usr/bin/python3`.

```bash
cd /home/evandro/ros2_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
```

Para verificar dependências sem instalar novamente:

```bash
uv pip check --python .venv/bin/python
```

Faça a configuração dos IDs e a calibração pelo LeRobot antes de conectar o
nó. O `robot_id` deve ser o mesmo usado na calibração.

```bash
ros2 launch so_arm_101_hardware real.launch.py \
  port:=/dev/ttyUSB0 robot_id:=meu_so101
ros2 service call /so101_hardware/calibrate std_srvs/srv/Trigger '{}'
```

O pacote não inicia Gazebo nem `ros2_control`: ele é a camada inicial de
hardware real e pode ser substituído posteriormente por um plugin
`hardware_interface` se for necessário executar controladores ros2_control
diretamente sobre o barramento.
