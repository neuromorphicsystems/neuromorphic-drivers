import pathlib

import neuromorphic_drivers as nd
import numpy as np
import plotly.express

nd.print_device_list()

output = pathlib.Path(__file__).resolve().parent / "hot_pixels"
output.mkdir(exist_ok=True)

diff_off_to_counts: dict[int, np.ndarray] = {}
diff_off_to_duration: dict[int, float] = {}

with nd.open(configuration=nd.prophesee_evk4.Configuration()) as device:
    diff_off = 60
    start_t = 0
    next_t = 1000000
    state = 0
    total = 0
    counts = np.zeros(
        (device.properties().width, device.properties().height), dtype=np.uint64
    )

    print("wait for 1 seconds")
    for status, packet in device:
        if diff_off == 45:
            break
        if packet.polarity_events is not None:
            if state == 1:
                events = packet.polarity_events[packet.polarity_events["t"] >= start_t]
                total += len(events)
                np.add.at(counts, (events["x"], events["y"]), 1)
            if packet.polarity_events["t"][-1] > next_t:
                match state:
                    case 0:
                        print("start counting")
                        start_t = packet.polarity_events["t"][-1] + 1
                        next_t += 1000000
                        total = 0
                        counts.fill(0)
                        state = 1
                    case 1:
                        duration = packet.polarity_events["t"][-1] - start_t
                        print(
                            f"{diff_off=}, {total=}, {duration=}, {total / duration * 1e6} events/s"
                        )
                        diff_off_to_counts[diff_off] = counts
                        diff_off_to_duration[diff_off] = float(duration)
                        counts = np.zeros(
                            (device.properties().width, device.properties().height),
                            dtype=np.uint64,
                        )
                        next_t += 1000000
                        state = 0
                        diff_off -= 1
                        device.update_configuration(
                            nd.prophesee_evk4.Configuration(
                                biases=nd.prophesee_evk4.Biases(
                                    diff_off=diff_off,
                                )
                            )
                        )
                        print("wait for 1 seconds")


for diff_off, counts in diff_off_to_counts.items():
    log_counts = np.flipud(np.log((counts + 1).astype(np.float64)).transpose())
    figure = plotly.express.imshow(log_counts, color_continuous_scale="thermal")
    figure.update_layout(
        width=1280,
        height=720,
        margin=dict(l=0, r=0, b=0, t=0),
        xaxis={"visible": False, "showticklabels": False},
        yaxis={"visible": False, "showticklabels": False},
        showlegend=False,
    )
    figure.update_coloraxes(showscale=False)
    figure.write_image(output / f"diff_off_{diff_off}_map.png")
    sorted_counts = np.sort(counts[counts > 0])[::-1].astype(np.float64) * (
        1e6 / diff_off_to_duration[diff_off],
    )
    print(f"{diff_off=}, maximum={sorted_counts[0]} events/s")
    figure = plotly.express.line(
        x=np.arange(1, len(sorted_counts) + 1),
        y=sorted_counts,
        log_y=True,
        log_x=True,
    )
    figure.update_layout(
        width=1280,
        height=720,
        showlegend=False,
    )
    figure.write_image(output / f"diff_off_{diff_off}_count.png")
