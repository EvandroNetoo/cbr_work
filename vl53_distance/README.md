# vl53_distance

Servidor da action `/vl53/follow_wall`. O nó usa
diretamente dois sensores VL53L0X atrás de um TCA9548A e envia `TwistStamped`
para `/cmd_vel`; não existe um nó ou tópico intermediário de distância.

```bash
ros2 launch vl53_distance vl53_distance.launch.py
```

Para ficar a 50 mm da parede sem mudar de posição lateral:

```bash
ros2 action send_goal /vl53/follow_wall interfaces/action/FollowWall \
  "{wall_distance_mm: 50, travel_distance_mm: 0, wall_tolerance_mm: 5, \
  travel_tolerance_mm: 5, timeout: {sec: 10, nanosec: 0}}" --feedback
```

Para percorrer 500 mm para a direita mantendo 300 mm da parede frontal:

```bash
ros2 action send_goal /vl53/follow_wall interfaces/action/FollowWall \
  "{wall_distance_mm: 300, travel_distance_mm: 500, wall_tolerance_mm: 10, \
  travel_tolerance_mm: 10, timeout: {sec: 15, nanosec: 0}}" --feedback
```

Em `travel_distance_mm`, valores positivos movimentam para a direita e
negativos para a esquerda. O percurso é o deslocamento líquido de `/odom`
projetado sobre o eixo direito que o robô possuía no início do goal. A action
aborta e publica parada se a odometria deixar de chegar dentro do prazo
configurado.

Os canais, offsets, ganhos e limites ficam em `config/vl53_distance.yaml`.
Durante um goal, nenhum outro nó deve publicar em `/cmd_vel`.
