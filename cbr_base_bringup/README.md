# cbr_base_bringup

Bringup físico isolado da base mecanum. Inicia o driver Python, o
`MariolaSystem`, o `controller_manager`, um `joint_state_broadcaster` e o
`mecanum_drive_controller`.

```bash
ros2 launch cbr_base_bringup real.launch.py
```

O launch não aceita argumentos. As configurações ficam em
`config/controllers.yaml` e no YAML do pacote `cbr_base_hardware`. A entrada
`/cmd_vel` usa `geometry_msgs/msg/TwistStamped`; as saídas são `/odom`, TF
`odom -> base_footprint` e as quatro rodas em `/joint_states`.
