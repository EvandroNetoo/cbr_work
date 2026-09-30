# vl53_distance

Servidor da action `/vl53/follow_wall`. O nó usa
diretamente dois sensores VL53L0X atrás de um TCA9548A e envia `TwistStamped`
para `/cmd_vel`; não existe um nó ou tópico intermediário de distância.

Enquanto não existe um goal, o timer de comandos fica cancelado, o nó não
assina `/odom` e o barramento/sensores VL53L0X permanecem fechados. Esses
recursos são ativados ao iniciar cada goal e liberados em qualquer término
(sucesso, cancelamento, timeout ou erro). A inicialização e calibração dos
sensores, portanto, fazem parte da latência inicial de cada execução.
As subscriptions ROS são criadas e removidas pela thread do executor através
de um guard condition acionado somente nessas transições. Assim, o nó não
processa odometria ou LaserScan em standby e evita destruir handles enquanto o
executor monta seu wait set.

```bash
ros2 launch vl53_distance vl53_distance.launch.py
```

O resultado do `FollowWall` inclui as folgas laterais medidas pelo LiDAR,
separadas para esquerda e direita, mesmo quando a proteção lateral do goal
está desativada. `has_fresh_lateral_scan` sinaliza scan recente; cada
`has_valid_*_lateral_clearance` sinaliza obstáculo detectado no lado. Sem
cluster lateral dentro do alcance, a respectiva distância não tem valor válido.

Para ficar a 50 mm da parede sem mudar de posição lateral:

```bash
ros2 action send_goal /vl53/follow_wall interfaces/action/FollowWall \
  "{wall_distance_mm: 50, travel_distance_mm: 0, wall_tolerance_mm: 5, \
  travel_tolerance_mm: 5, max_alignment_error_mm: 0, \
  alignment_error_ignore_duration: {sec: 0, nanosec: 0}, \
  alignment_recovery_distance_mm: 0, \
  minimum_lateral_clearance_mm: 0, \
  timeout: {sec: 10, nanosec: 0}}" --feedback
```

Para percorrer 500 mm para a direita mantendo 300 mm da parede frontal:

```bash
ros2 action send_goal /vl53/follow_wall interfaces/action/FollowWall \
  "{wall_distance_mm: 300, travel_distance_mm: 500, wall_tolerance_mm: 10, \
  travel_tolerance_mm: 10, max_alignment_error_mm: 100, \
  alignment_error_ignore_duration: {sec: 2, nanosec: 0}, \
  alignment_recovery_distance_mm: 200, \
  minimum_lateral_clearance_mm: 100, \
  timeout: {sec: 15, nanosec: 0}}" --feedback
```

Em `travel_distance_mm`, valores positivos movimentam para a direita e
negativos para a esquerda. O percurso é o deslocamento líquido de `/odom`
projetado sobre o eixo direito que o robô possuía no início do goal. A action
aborta e publica parada se a odometria deixar de chegar dentro do prazo
configurado.

`minimum_lateral_clearance_mm` define a folga minima entre a borda do footprint
e um obstaculo no lado do movimento. O valor `0` desativa a protecao e preserva
o comportamento anterior. Um comando lateral positivo no frame ROS verifica o
lado esquerdo; um comando negativo verifica o direito. Rotacao sem movimento
lateral nao aciona a verificacao. A velocidade lateral e reduzida dentro da
margem configurada. Ao atingir a folga minima, `linear.y` e `angular.z` sao
bloqueados, mas uma correcao frontal pendente em `linear.x` continua ate
estabilizar na distancia solicitada. A action entao termina abortada, informando
que concluiu a aproximacao frontal e interrompeu o percurso lateral. Com a
protecao habilitada, um `/scan_front` ausente ou obsoleto ainda causa parada
total e aborto imediato.

`max_alignment_error_mm` limita a diferença absoluta entre as distâncias dos
dois sensores. Se uma leitura válida ultrapassar esse valor, a action publica
parada e aborta o goal imediatamente. O valor `0` desativa essa proteção e é o
padrão quando o campo não é preenchido. `alignment_error_ignore_duration`
adia o aborto ou o início da recuperação por um período contado desde a
primeira leitura VL53 válida. `0` aplica o limite desde a primeira leitura.
Durante a janela, a diferença continua sendo medida e publicada no feedback.
Se ela ultrapassar o limite, o robô continua o percurso lateral solicitado
pela odometria, usando temporariamente a distância frontal alvo no controlador
(como na recuperação). Se a diferença voltar ao limite, o controle dos VL53
retorna imediatamente. Ao terminar a janela, um desalinhamento persistente
ativa a recuperação ou o aborto normal. O goal não termina em sucesso
antes disso se a diferença ainda exceder o limite. Timeout, falha de sensores,
odometria e proteção lateral permanecem ativos durante todo o período.

Quando `alignment_recovery_distance_mm` é positivo, um desalinhamento inicia
um retorno lateral no sentido oposto ao percurso solicitado, em vez do aborto
imediato. Enquanto o desalinhamento persistir, somente a odometria controla o
retorno lateral; ao normalizar, o controle dos sensores volta a manter a
distância da parede e o alinhamento. Após percorrer a distância de recuperação,
a action para e termina abortada com a mensagem `Recuperação concluída`.

Os canais, offsets, ganhos e limites ficam em `config/vl53_distance.yaml`.
Durante um goal, nenhum outro nó deve publicar em `/cmd_vel`.
