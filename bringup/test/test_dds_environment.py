"""Exercise shell selection without starting ROS or changing network state."""

from pathlib import Path
import shutil
import subprocess

import pytest


PACKAGE = Path(__file__).parents[1]


@pytest.fixture
def selector(tmp_path):
    # Installed share directories and checkouts both use this layout.
    root = tmp_path / 'share with spaces' / 'bringup'
    shutil.copytree(PACKAGE / 'scripts', root / 'scripts')
    shutil.copytree(PACKAGE / 'config', root / 'config')
    return root / 'scripts' / 'dds_environment.bash'


def shell(selector, commands, *args):
    result = subprocess.run(
        ['bash', '--noprofile', '--norc', '-c', commands,
         'test', str(selector), *args], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    return result.stdout.strip().splitlines()


@pytest.mark.parametrize('role', ['banana', 'rasp', 'notebook', 'wsl'])
def test_selection_works_in_installed_layout_with_spaces(selector, role):
    lines = shell(selector, '''
        export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 ROS_STATIC_PEERS=old
        source "$1" "$2" || exit 1
        test -r "${CYCLONEDDS_URI#file://}" || exit 1
        test -z "${ROS_LOCALHOST_ONLY+x}${ROS_STATIC_PEERS+x}" || exit 1
        printf '%s\\n' "$RMW_IMPLEMENTATION" "$ROS_DOMAIN_ID" \
            "$ROS_AUTOMATIC_DISCOVERY_RANGE" "$CYCLONEDDS_URI"
    ''', role)
    assert lines[:3] == ['rmw_cyclonedds_cpp', '10', 'SUBNET']
    assert lines[3].endswith(f'/config/cyclonedds_{role}.xml')


def test_fastdds_can_be_selected_after_cyclone(selector):
    assert shell(selector, '''
        source "$1" banana || exit 1
        source "$1" fastdds || exit 1
        test -z "${CYCLONEDDS_URI+x}" || exit 1
        printf '%s\\n' "$RMW_IMPLEMENTATION"
    ''') == ['rmw_fastrtps_cpp']


@pytest.mark.parametrize('missing_profile', [False, True])
def test_invalid_selection_preserves_environment(selector, missing_profile):
    if missing_profile:
        (selector.parent.parent / 'config' / 'cyclonedds_banana.xml').unlink()
    assert shell(selector, '''
        export RMW_IMPLEMENTATION=original CYCLONEDDS_URI=original ROS_DOMAIN_ID=42
        if source "$1" "$2"; then exit 1; fi
        printf '%s\\n' "$RMW_IMPLEMENTATION" "$CYCLONEDDS_URI" "$ROS_DOMAIN_ID"
    ''', 'banana' if missing_profile else 'invalid') == ['original', 'original', '42']
