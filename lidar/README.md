# lidar

Driver ROS 2 do LiDAR XV-11 montado na base CBR. O nó publica uma varredura
`sensor_msgs/msg/LaserScan` por rotação em `/scan_front`, usando o frame
`lidar_front_link` fornecido pelo `robot_state_publisher`.

Toda a configuração do robô real fica em `config/lidar.yaml`. O launch não
possui argumentos:

```bash
ros2 run lidar lidar_node --ros-args \
  --params-file $(ros2 pkg prefix lidar)/share/lidar/config/lidar.yaml
```

O setor 307°→67° contém sempre 121 posições. Graus perdidos ou leituras
inválidas permanecem como `NaN`, sem deslocar as demais medições. O relé do
motor é desligado quando o nó encerra. A serial é processada em blocos de 64
bytes para reduzir polling e chamadas ao kernel. O timestamp parte do fim da
aquisição e do RPM, garantindo que o último raio nunca seja datado no futuro.
O executor consulta a varredura concluída a 20 Hz; para um sensor próximo de
5 rotações/s, isso mantém a latência adicional abaixo de 50 ms sem callbacks
vazios a 100 Hz.

Diagnóstico:

```bash
ros2 topic hz /scan_front
ros2 topic echo /scan_front --once --qos-reliability best_effort
ros2 run tf2_ros tf2_echo base_link lidar_front_link
```
