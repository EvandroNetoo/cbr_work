# cbr_imu

Driver ROS 2 da IMU integrada à Mariola. Publica `sensor_msgs/Imu` em
`/imu/data` após calibrar o bias angular com o robô parado.

```bash
ros2 launch cbr_imu imu.launch.py
ros2 topic hz /imu/data
```

A frequência e os parâmetros de calibração ficam em `config/imu.yaml`.
O launch `imu_localization.launch.py` também inicia o EKF que combina
`/wheel/odom` com a velocidade angular Z e publica `/odom`.
