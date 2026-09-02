# mission_manager

Executor sequencial das missões da RoboCup@Work. O pacote não controla motores
diretamente: ele compõe Nav2, alinhamento VL53 e as actions semânticas do pacote
`manipulation`.

## Arquivos

- `config/arena.yaml`: poses fixas, alturas, tipos e alinhamento;
- `config/plans/*.yaml`: passos sequenciais selecionados por `plan_id`;
- `config/mission_manager.yaml`: nomes das actions e timeouts ROS.

As poses vazias de `arena.yaml` devem ser medidas antes da execução. O nó inicia
normalmente, mas um goal retorna `CONFIGURATION_ERROR` sem movimentar o robô se
a arena ou o plano não forem válidos.

## Navegação

Para uma service area, `navigate` executa:

```text
PrepareManipulator(NAVIGATION) → NavigateToPose → MoveToDistance
```

Para `start` e `finish`, o alinhamento é omitido. O bloco `alignment` de uma
service area sobrescreve parcialmente `alignment_defaults`.

## Execução

```bash
ros2 launch mission_manager mission_manager.launch.py
```

```bash
ros2 action send_goal /mission/execute interfaces/action/ExecuteMission \
  "{plan_id: example_transport}" --feedback
```

Somente uma missão é aceita por vez. Qualquer passo que falhar encerra a missão,
e o cancelamento é propagado para o goal filho ativo. O mission manager mantém
apenas o passo atual e a localização lógica; a garra e os slots continuam sob
responsabilidade de `manipulation`.
