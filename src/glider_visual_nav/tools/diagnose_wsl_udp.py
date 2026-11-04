"""Independent MTU control: separate OS sender, no ROS imports, no root access."""
import argparse
import json
import socket
import subprocess
import sys

parser = argparse.ArgumentParser()
parser.add_argument('--send', nargs=2, type=int, metavar=('PORT', 'BYTES'))
parser.add_argument('--output')
args = parser.parse_args()
if args.send:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as peer:
        peer.sendto(b'g' * args.send[1], ('127.0.0.1', args.send[0]))
else:
    results = []
    for size in (1200, 1400, 1472, 1473, 1500, 2000, 6000, 60000):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as peer:
            peer.bind(('127.0.0.1', 0))
            peer.settimeout(.5)
            subprocess.run([sys.executable, __file__, '--send', str(peer.getsockname()[1]), str(size)], check=True)
            try:
                received, _ = peer.recvfrom(65536)
                passed = received == b'g' * size
            except TimeoutError:
                passed = False
            results.append(dict(payload_bytes=size, delivered=passed))
    text = json.dumps(results, indent=2) + '\n'
    print(text)
    if args.output:
        from pathlib import Path
        Path(args.output).write_text(text)
