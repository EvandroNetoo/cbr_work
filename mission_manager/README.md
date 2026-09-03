# mission_manager

Executor sequencial das missões da RoboCup@Work. O pacote não controla motores
diretamente: ele compõe Nav2, alinhamento VL53 e as actions semânticas do pacote
`manipulation`.

## Arquivos

- `config/arena.yaml`: poses fixas, alturas, tipos, alinhamento, recuo e
  recuperação de coleta;
- `config/plans/*.yaml`: passos sequenciais selecionados por `plan_id`;
- `config/mission_manager.yaml`: nomes das actions, serviço/tópico de estado,
  compartimentos disponíveis e timeouts ROS.

As poses vazias de `arena.yaml` devem ser medidas antes da execução. O nó inicia
normalmente, mas um goal retorna `CONFIGURATION_ERROR` sem movimentar o robô se
a arena ou o plano não forem válidos.

## Navegação

Para uma service area, `navigate` executa:

```text
PrepareManipulator(NAVIGATION) → NavigateToPose → FollowWall(travel=0)
```

Ao sair de uma service area para outro destino, o fluxo começa com:

```text
FollowWall(departure, travel=0) → PrepareManipulator(NAVIGATION) → NavigateToPose
```

Para `start` e `finish`, o alinhamento de chegada é omitido. Os blocos
`alignment` e `departure` de uma service area sobrescrevem parcialmente
`alignment_defaults` e `departure_defaults`, respectivamente.

## Recuperação de coleta fora do alcance

Quando o filtro de alcance bloqueia a AprilTag, ou o MoveIt devolve o código
`99999` antes de fechar a garra, o resultado de `PickObject` inclui a pose
detectada. Se `pickup_recovery.enabled` estiver ativo, o mission manager:

```text
PrepareManipulator(OBSERVATION) → FollowWall → PickObject (nova detecção)
```

O alvo do `FollowWall` é calculado a partir da última distância VL53 válida:

```text
travel_mm = 1000 * (preferred_tag_x_m - tag_x_m)
wall_mm = current_wall_mm + 1000 * (tag_y_m - preferred_tag_y_m)
```

`wall_mm` é limitado por `minimum_wall_distance_mm` e
`maximum_wall_distance_mm`. Deslocamento lateral positivo significa direita e
negativo significa esquerda.

## Execução

```bash
ros2 launch mission_manager mission_manager.launch.py
```

```bash
ros2 action send_goal /mission/execute interfaces/action/ExecuteMission \
  "{plan_id: example_transport}" --feedback
```

Somente uma missão é aceita por vez. Qualquer passo que falhar encerra a missão,
e o cancelamento é propagado para o goal filho ativo.

## Estado do mundo

O `mission_manager` é o dono do estado lógico da garra e dos compartimentos.
Depois de validar o plano e a arena, cada nova missão reinicia esse estado como
conhecido, com garra e slots vazios. O pacote `manipulation` solicita validações
e confirma transições somente nos pontos em que o movimento físico foi
efetivado; em falhas ambíguas, marca o estado como desconhecido.

O snapshot atual é publicado em `/mission/state` com QoS `transient_local`. A
API interna `/mission/manipulation_state` atende as transações usadas pelo
servidor de manipulação. `WorldState`, no pacote de missão, é o ponto de extensão
para incorporar futuramente estados de objetos, estações e outros elementos da
arena.
