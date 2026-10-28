"""Guard cable continuity while retaining direct notebook DDS access."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


CONFIG = Path(__file__).parents[1] / 'config'
NS = {'c': 'https://cdds.io/config'}


@pytest.mark.parametrize('role,own,other,wifi', [
    ('banana', '10.50.0.1', '10.50.0.2', 'wlan1'),
    ('rasp', '10.50.0.2', '10.50.0.1', 'wlan0'),
])
def test_cable_peers_and_optional_wifi_coexist(role, own, other, wifi):
    domain = ET.parse(CONFIG / f'cyclonedds_{role}.xml').find('c:Domain', NS)
    general = domain.find('c:General', NS)
    interfaces = general.findall('c:Interfaces/c:NetworkInterface', NS)
    assert len(interfaces) == 2
    assert interfaces[0].get('name') == 'eth0'
    assert interfaces[0].get('address') == own
    assert interfaces[0].get('presence_required', 'true') == 'true'
    assert general.findtext('c:DontRoute', namespaces=NS) == 'true'
    assert interfaces[1].get('name') == wifi
    assert interfaces[1].get('presence_required') == 'false'
    assert int(interfaces[0].get('priority')) > int(interfaces[1].get('priority'))
    assert general.findtext('c:AllowMulticast', namespaces=NS) == 'spdp'
    # Stock 0.10.5 crashes in wras_cover_locatorset when redundancy is enabled.
    assert general.findtext('c:RedundantNetworking', namespaces=NS) == 'false'
    discovery = domain.find('c:Discovery', NS)
    assert discovery.findtext('c:ParticipantIndex', namespaces=NS) == 'auto'
    assert discovery.findtext('c:SPDPInterval', namespaces=NS) == '1 s'
    assert {peer.get('Address') for peer in discovery.findall('c:Peers/c:Peer', NS)} == {own, other}
