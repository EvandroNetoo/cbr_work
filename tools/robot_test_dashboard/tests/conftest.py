from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from copy import deepcopy
from domain import LEGACY_RULES, import_scenario, new_record


@pytest.fixture
def scenario():
    path = Path(__file__).parent / 'fixtures' / 'FINAL_seed8_equipe.yaml'
    return import_scenario(path.read_text(), path.name)


@pytest.fixture
def record(scenario):
    # Cobertura de compatibilidade: as fichas v1 devem conservar seus resultados.
    payload = new_record(scenario, '8', {'name': 'plano.yaml', 'text': 'plan_id: exemplo\n'})
    payload['rules'] = deepcopy(LEGACY_RULES)
    payload['attempts'][0].pop('automatic_percentages')
    payload['attempts'][0].pop('followed_line')
    return payload
