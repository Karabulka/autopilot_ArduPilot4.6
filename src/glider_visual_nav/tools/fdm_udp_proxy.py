#!/usr/bin/env python3
"""Transparent diagnostic FDM relay: no state generation or modification."""
import argparse, json, socket, time
p = argparse.ArgumentParser()
p.add_argument('output')
a = p.parse_args()
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('127.0.0.1', 9002))
peer = None
with open(a.output, 'w') as f:
    while True:
        packet, addr = s.recvfrom(65535)
        if addr[1] == 9003:
            if peer:
                s.sendto(packet, peer)
            try:
                row = json.loads(packet.strip())
                row['relay_wall'] = time.monotonic()
                f.write(json.dumps(row) + '\n')
                f.flush()
            except (ValueError, TypeError):
                pass
        else:
            peer = addr
            s.sendto(packet, ('127.0.0.1', 9003))
