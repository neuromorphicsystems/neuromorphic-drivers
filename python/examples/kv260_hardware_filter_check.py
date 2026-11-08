#!/usr/bin/env python3
"""Short live camera check. KV260_FILTERS selects experimental hardware blocks."""
import os
import time

import neuromorphic_drivers as nd
from neuromorphic_drivers.generated.devices.prophesee_evk4 import Biases, Configuration

spec = os.environ.get('KV260_FILTERS', '0,0,50,520')
print('TEST_START', spec, flush=True)
config = Configuration(biases=Biases(diff_on=10, diff_off=10, fo=-20))
with nd.open(configuration=config, iterator_timeout=0.02) as camera:
    deadline = time.monotonic() + 4
    count = 0
    packets = 0
    for _status, packet in camera:
        if packet is not None and packet.polarity_events is not None:
            count += len(packet.polarity_events)
            packets += 1
        if time.monotonic() >= deadline:
            break
    print('TEST_RESULT', spec, 'events', count, 'packets', packets, flush=True)
    if packets == 0:
        print('NO_EVENTS_OBSERVED: check with a moving scene', flush=True)
