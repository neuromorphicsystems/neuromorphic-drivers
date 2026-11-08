"""Short live camera check with per-device hardware filter settings."""

import argparse
import time

import neuromorphic_drivers as nd

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--afk-off", action="store_true")
    parser.add_argument("--stc-off", action="store_true")
    parser.add_argument("--stc-ms", type=int, choices=range(1, 101), default=10)
    args = parser.parse_args()
    configuration = nd.prophesee_evk4.Configuration(
        biases=nd.prophesee_evk4.Biases(diff_on=10, diff_off=10, fo=-20),
        anti_flicker=None if args.afk_off else nd.prophesee_evk4.AntiFlicker(50, 520),
        stc=None if args.stc_off else nd.prophesee_evk4.Stc(args.stc_ms),
    )
    print("Requested configuration:", configuration, flush=True)
    with nd.open(configuration=configuration, iterator_timeout=0.02) as camera:
        deadline = time.monotonic() + 4
        count = 0
        packets = 0
        for _status, packet in camera:
            if packet is not None and packet.polarity_events is not None:
                count += len(packet.polarity_events)
                packets += 1
            if time.monotonic() >= deadline:
                break
        print("Received events:", count, "packets:", packets, flush=True)
        if packets == 0:
            print("No events observed; check with a moving scene.", flush=True)
