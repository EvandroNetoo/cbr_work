"""Watch launch log files in a separate process, independent of robot control."""

import argparse
from collections import deque
import os
from pathlib import Path
import sys
import time


def latest_run():
    ros_home = Path(os.environ.get('ROS_HOME', str(Path.home() / '.ros')))
    root = Path(os.environ.get('ROS_LOG_DIR', str(ros_home / 'log'))).expanduser()
    runs = [log.parent for log in root.glob('*/launch.log')]
    if not runs:
        raise ValueError(f'Nenhuma execução encontrada em {root}. Inicie o launch primeiro.')
    return max(runs, key=lambda run: (run / 'launch.log').stat().st_mtime_ns)


def follow(directory):
    streams = {}
    print(f'Acompanhando {directory}\nCtrl+C encerra somente esta visualização.', flush=True)
    try:
        while True:
            for path in sorted(directory.glob('*.log')):
                # own_log writes each message to a combined file and a separate
                # stdout/stderr file. Read only combined files to avoid duplicates.
                if path.name.endswith(('-stdout.log', '-stderr.log')):
                    continue
                try:
                    identity = (path.stat().st_dev, path.stat().st_ino)
                    previous = streams.get(path)
                    if previous is not None and previous[0] != identity:
                        previous[1].close()
                        del streams[path]
                    if path not in streams:
                        stream = path.open('r', encoding='utf-8', errors='replace')
                        streams[path] = (identity, stream)
                        # Bound the initial read even for large DDS error logs.
                        stream.seek(max(0, path.stat().st_size - 65536))
                        if stream.tell():
                            stream.readline()
                        for line in deque(stream, maxlen=20):
                            print(f'[{path.name}] {line}', end='', flush=True)
                        continue
                    stream = streams[path][1]
                    if path.stat().st_size < stream.tell():
                        stream.seek(0)
                    # A busy file must not prevent discovery of other nodes' logs.
                    chunk = stream.read(65536)
                    if chunk:
                        print(f'[{path.name}] {chunk}', end='', flush=True)
                except FileNotFoundError:
                    continue
            time.sleep(0.2)
    finally:
        for _identity, stream in streams.values():
            stream.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', nargs='?', type=Path,
                        help='Pasta informada pelo launch; padrão: execução mais recente.')
    args = parser.parse_args()
    try:
        directory = args.directory.expanduser() if args.directory else latest_run()
        if not directory.is_dir():
            raise ValueError(f'Diretório de logs não encontrado: {directory}')
        follow(directory)
    except KeyboardInterrupt:
        return 0
    except (OSError, ValueError) as error:
        print(str(error), file=sys.stderr)
        return 1
    return 0
