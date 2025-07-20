import time

import neuromorphic_drivers as nd

configuration = nd.prophesee_evk4.Configuration(
    biases=nd.prophesee_evk4.Biases(
        diff_off=130,
        diff_on=150,
    )
)

with nd.open(configuration=configuration) as device:
    start = time.monotonic()
    previous_output = start
    count = 0
    updated = False
    for status, packet in device:
        now = time.monotonic()
        if now - previous_output >= 1.0:
            print(f"{count / (now - previous_output):.2f} events/s")
            previous_output = now
            count = 0
        if packet.polarity_events is not None:
            count += len(packet.polarity_events)
        if not updated and now - start > 5.0:
            print("update biases")
            configuration.biases.diff_on -= 50
            configuration.biases.diff_off -= 50
            device.update_configuration(configuration)
            updated = True
