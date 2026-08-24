# base_bringup

Bringup físico isolado da base mecanum. Inicia o driver Python, o LiDAR em
`/scan_front`, o `MariolaSystem`, o `controller_manager`, um
`joint_state_broadcaster` e o `mecanum_drive_controller`.

```bash
ros2 launch bringup robot.launch.py \
  enable_arm:=false enable_moveit:=false enable_perception:=false
```

O launch não aceita argumentos. As configurações ficam em
`config/controllers.yaml`, no YAML do pacote `base_hardware` e em
`lidar/config/lidar.yaml`. A entrada `/cmd_vel` usa
`geometry_msgs/msg/TwistStamped`; as saídas são `/odom`, TF
`odom -> base_footprint`, `/scan_front` e as quatro rodas em `/joint_states`.
