# imu

Driver ROS 2 da IMU integrada à Mariola. Publica `sensor_msgs/Imu` em
`/imu/data` após calibrar o bias angular com o robô parado.

```bash
ros2 run imu imu_node --ros-args \
  --params-file $(ros2 pkg prefix imu)/share/imu/config/imu.yaml
ros2 topic hz /imu/data
```

A frequência e os parâmetros de calibração ficam em `config/imu.yaml`.
O `bringup/localization.launch.py` inicia o EKF que combina
`/wheel/odom` com a velocidade angular Z e publica `/odom`.
