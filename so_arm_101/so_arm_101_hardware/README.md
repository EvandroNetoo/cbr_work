# so_arm_101_hardware

Ponte física LeRobot/Feetech. Publica estados em
`/so101_hardware/raw_joint_states` e recebe posições em
`/so101_hardware/command_positions`. O `ros2_control` permanece entre o driver
e MoveIt/teleop.

O driver lê `config/real.yaml`. Para diagnóstico isolado:

```bash
ros2 run so_arm_101_hardware so101_hardware_node --ros-args \
  --params-file $(ros2 pkg prefix so_arm_101_hardware)/share/so_arm_101_hardware/config/real.yaml
```

Para o stack real do braço:

```bash
ros2 launch bringup robot.launch.py \
  enable_base:=false enable_perception:=false enable_moveit:=false \
  port:=/dev/ttyUSB0 robot_id:=meu_so101
```

O launch procura a virtualenv do workspace. Um interpretador alternativo pode
ser indicado por `SO_ARM_101_PYTHON=/caminho/para/python`.

No perfil padrão, um ciclo de 30 Hz mantém o alvo mais recente, deduplica
comandos estacionários e usa heartbeat de 5 Hz. Calibrações de referência ficam
em `config/`; confirme IDs, offsets e limites no hardware real antes de mover.
