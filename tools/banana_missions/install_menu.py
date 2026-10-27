"""Create one menuPrincipal entry for each source plan, without starting missions."""

import argparse
from pathlib import Path
import re
import shlex

import yaml


def install(menu_root: Path) -> list[Path]:
    source = Path(__file__).resolve().parents[2]
    runner = source / 'tools' / 'banana_missions' / 'run_mission.sh'
    created = []
    for plan_path in sorted((source / 'mission_manager' / 'config' / 'plans').glob('*.yaml')):
        plan = yaml.safe_load(plan_path.read_text())
        plan_id = plan['plan_id']
        if not isinstance(plan_id, str) or not re.fullmatch(r'[A-Za-z0-9_-]+', plan_id):
            raise ValueError(f'plan_id inválido em {plan_path}')
        if plan_id != plan_path.stem:
            raise ValueError(f'plan_id diferente do nome do arquivo: {plan_path}')
        folder = menu_root / f'Missao_{plan_id}'
        folder.mkdir(parents=True, exist_ok=True)
        # The menu lists folders with main.py, but prioritizes main.sh to run them.
        (folder / 'main.py').write_text('# Entrada do menu; execução em main.sh.\n')
        (folder / 'main.sh').write_text(
            '#!/bin/bash\nexec bash ' + shlex.quote(str(runner)) + ' '
            + shlex.quote(plan_id) + '\n')
        created.append(folder)
    return created


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--menu-root', type=Path, default=Path.home())
    args = parser.parse_args()
    for folder in install(args.menu_root):
        print(folder)
