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

O arco publicado 307°→217° contém sempre 271 posições. Os setores válidos são
307°→67° (frente) e 167°→196° (traseira); os trechos bloqueados
pelo robô permanecem como `NaN`. Graus perdidos ou leituras inválidas também
permanecem como `NaN`, sem deslocar as demais medições. O relé do
motor é desligado quando o nó encerra. A serial é acumulada em blocos de 128
bytes e decodificada diretamente em pacotes de 22 bytes, com ressincronização
após perda de dados. Máscaras angulares são pré-calculadas e o lock protege
somente a entrega de uma varredura completa. O timestamp parte do fim da
aquisição e do RPM, garantindo que o último raio nunca seja datado no futuro.
O executor consulta a varredura concluída a 10 Hz; para um sensor próximo de
5 rotações/s, isso limita a latência adicional de entrega a 100 ms.

Diagnóstico:

```bash
ros2 topic hz /scan_front
ros2 topic echo /scan_front --once --qos-reliability best_effort
ros2 run tf2_ros tf2_echo base_link lidar_front_link
```
