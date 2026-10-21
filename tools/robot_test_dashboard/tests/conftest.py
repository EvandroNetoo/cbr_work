from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from domain import import_scenario, new_record


@pytest.fixture
def scenario():
    path = Path(__file__).parent / 'fixtures' / 'FINAL_seed8_equipe.yaml'
    return import_scenario(path.read_text(), path.name)


@pytest.fixture
def record(scenario):
    return new_record(scenario, '8', {'name': 'plano.yaml', 'text': 'plan_id: exemplo\n'})
