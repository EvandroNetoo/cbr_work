# vl53_distance

Servidor da action `/vl53/move_to_distance`. O nó usa diretamente dois
sensores VL53L0X atrás de um TCA9548A e envia `TwistStamped` para `/cmd_vel`;
não existe um nó ou tópico intermediário de distância.

```bash
ros2 launch vl53_distance vl53_distance.launch.py
ros2 action send_goal /vl53/move_to_distance interfaces/action/MoveToDistance \
  "{distance_mm: 300, tolerance_mm: 10, timeout: {sec: 10, nanosec: 0}}" \
  --feedback
```

Os canais, offsets e ganhos ficam em `config/vl53_distance.yaml`. Durante um
goal, nenhum outro nó deve publicar em `/cmd_vel`.
