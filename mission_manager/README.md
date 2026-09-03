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

Cada resultado de `PickObject`, bem-sucedido ou não, inclui todas as AprilTags
observadas enquanto a base permaneceu parada. O mission manager associa cada
detecção à distância atual da parede e a uma coordenada lateral, cuja origem é
o alinhamento de chegada à área. A memória é separada por área e permanece
válida durante toda a missão, inclusive depois de navegar para outra área.

Quando o filtro de alcance bloqueia a AprilTag, ou o MoveIt devolve o código
`99999` antes de fechar a garra, o resultado também inclui a pose específica
usada para recuperar a coleta. Se `pickup_recovery.enabled` estiver ativo, o
mission manager:

```text
PrepareManipulator(OBSERVATION) → FollowWall → PickObject (nova detecção)
```

O alvo do `FollowWall` é calculado a partir da última distância VL53 válida:

```text
travel_mm = 1000 * (preferred_tag_x_m - tag_x_m)
wall_mm = current_wall_mm + 1000 * (tag_y_m - preferred_tag_y_m)
```

`wall_mm` é limitado por `minimum_wall_distance_mm` e
`maximum_wall_distance_mm`. O destino lateral absoluto é limitado por
`minimum_lateral_position_mm` e `maximum_lateral_position_mm`, relativos à
posição `0`. Assim, se a centralização desejada ultrapassar uma extremidade, o
robô avança somente até o limite e repete a detecção nessa posição.
Deslocamento lateral positivo significa direita e negativo significa esquerda.

O deslocamento realmente medido pela action é acumulado na coordenada lateral,
em vez do valor comandado. Para uma tag vista anteriormente, o destino salvo é
convertido novamente em um deslocamento relativo à posição atual.

Se uma tag com posição salva não reaparecer no destino estimado de coleta, o
robô retorna uma vez ao ponto original onde ela foi observada e repete a
detecção. Se ainda não encontrá-la, ou se a tag nunca foi observada na área
atual, o robô visita a posição de busca ainda não observada mais próxima. As
posições são coordenadas absolutas em milímetros, configuradas em
`pickup_recovery.search_positions_mm`; o padrão da arena é `[0, 250, -250]`.
Todas as posições de busca precisam estar dentro dos limites laterais.
Em cada posição, todas as outras tags encontradas também atualizam a memória.
Uma tag coletada é removida, sem apagar as demais observações. Se a próxima tag
não apareceu na última análise e a base continua na mesma posição, essa análise
não é repetida: o robô segue diretamente para o ponto de busca não examinado
mais próximo. Um ponto fixo só é marcado como examinado quando a distância da
parede também corresponde à distância de observação da área.

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
conhecido, com garra e slots vazios. Antes de cada action física, o gerenciador
valida a precondição, preenche explicitamente o ID do objeto e somente depois
do resultado confirmado faz o commit da transição. Timeout, perda de comunicação,
cancelamento sem resultado ou efeito físico ambíguo tornam o estado desconhecido
e bloqueiam novas operações automáticas.

O snapshot atual é publicado em `/mission/state` com QoS `transient_local`.
Não existe uma API de estado usada pelo servidor de manipulação: `WorldState`
permanece interno ao gerenciador e é o ponto de extensão para incorporar
futuramente estados de objetos, estações e outros elementos da arena.
