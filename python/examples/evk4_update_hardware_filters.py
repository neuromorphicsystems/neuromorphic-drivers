"""Exercise live AFK/STC updates on one camera without reopening it."""

import time

import neuromorphic_drivers as nd

if __name__ == "__main__":
    afk = nd.prophesee_evk4.AntiFlicker(50, 520)
    stages = [
        (None, None),
        (afk, nd.prophesee_evk4.Stc(1)),
        (afk, nd.prophesee_evk4.Stc(100)),
        (None, nd.prophesee_evk4.Stc(20)),
        (afk, None),
        (None, None),
    ]
    with nd.open(
        configuration=nd.prophesee_evk4.Configuration(
            biases=nd.prophesee_evk4.Biases(diff_on=10, diff_off=10, fo=-20),
        ),
        iterator_timeout=0.02,
    ) as camera:
        stage = 0
        count = 0
        start = time.monotonic()
        for _status, packet in camera:
            if packet is not None and packet.polarity_events is not None:
                count += len(packet.polarity_events)
            if time.monotonic() - start < 2:
                continue
            print("Stage", stage, stages[stage], "events:", count, flush=True)
            stage += 1
            if stage == len(stages):
                break
            anti_flicker, stc = stages[stage]
            camera.update_configuration(
                nd.prophesee_evk4.Configuration(
                    biases=nd.prophesee_evk4.Biases(diff_on=10, diff_off=10, fo=-20),
                    anti_flicker=anti_flicker,
                    stc=stc,
                )
            )
            count = 0
            start = time.monotonic()
    print("Completed live updates on a single camera session.", flush=True)
