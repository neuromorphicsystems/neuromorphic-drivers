"""Stream events from a SENSING SE1-S4-USB (Sony IMX646) and print statistics.

Biases are offsets from the factory-programmed values. The driver clamps them
to the IMX646 ranges recommended by OpenEB (see device.biases_bounds()).
"""

import time

import neuromorphic_drivers as nd

nd.print_device_list()

configuration = nd.sensing_se1_s4.Configuration(
    biases=nd.sensing_se1_s4.Biases(
        diff_on=0,
        diff_off=0,
    ),
    enable_external_trigger=True,
)

with nd.open(configuration=configuration) as device:
    print(device.serial(), device.properties())
    print(device.biases_bounds())
    next = time.monotonic() + 1.0
    events = 0
    for status, packet in device:
        if packet.polarity_events is not None:
            events += len(packet.polarity_events)
        if packet.trigger_events is not None:
            for trigger_event in packet.trigger_events:
                print(f"trigger {trigger_event}")
        if time.monotonic() >= next:
            print(
                f"{events / 1e6:.3f} Mev/s, {device.temperature_celsius():.1f} ºC, "
                f"backlog {status.ring.backlog if status.ring else 0}"
            )
            events = 0
            next += 1.0
