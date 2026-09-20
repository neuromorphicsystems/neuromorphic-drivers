import datetime
import pathlib
import time

import neuromorphic_drivers as nd
import numpy as np

nd.print_device_list()

PRIMARY_SERIAL = "00050421"
SECONDARY_SERIAL = "00050423"

secondary_device = nd.open(
    serial=SECONDARY_SERIAL,
    raw=True,
    iterator_timeout=0.1,
    configuration=nd.prophesee_evk4.Configuration(
        nd.prophesee_evk4.Biases(
            diff_on=120,
            diff_off=120,
        ),
        clock=nd.prophesee_evk4.Clock.EXTERNAL,
    ),
)
time.sleep(1.0)  # wait to make sure that secondary is ready
primary_device = nd.open(
    serial=PRIMARY_SERIAL,
    raw=True,
    iterator_timeout=0.1,
    configuration=nd.prophesee_evk4.Configuration(
        nd.prophesee_evk4.Biases(
            diff_on=120,
            diff_off=120,
        ),
        clock=nd.prophesee_evk4.Clock.INTERNAL_WITH_OUTPUT_ENABLED,
    ),
)
devices = [primary_device, secondary_device]

dirname = pathlib.Path(__file__).resolve().parent
name = (
    datetime.datetime.now(tz=datetime.timezone.utc)
    .isoformat()
    .replace("+00:00", "Z")
    .replace(":", "-")
)
with (
    open(dirname / f"{name}_{PRIMARY_SERIAL}.raw", "wb") as primary_output,
    open(dirname / f"{name}_{SECONDARY_SERIAL}.raw", "wb") as secondary_output,
):
    outputs = [primary_output, secondary_output]

    backlogs = np.array([0 for _ in devices])
    while True:
        index = np.argmax(backlogs)
        status, packet = devices[index].__next__()
        if status.ring is None:
            backlogs[:] += 1
            backlogs[index] = 0
        else:
            backlogs[:] += 1
            backlogs[index] = status.ring.backlog
            delay = status.delay()
            if delay is not None:
                if packet is not None:
                    print(
                        f"{index}: {round(delay * 1e6)} µs, backlog: {status.ring.backlog}, bytes: {len(packet)}"
                    )
                    outputs[index].write(packet)
                    outputs[index].flush()
                else:
                    print(
                        f"{index}: {round(delay * 1e6)} µs, backlog: {status.ring.backlog}"
                    )
