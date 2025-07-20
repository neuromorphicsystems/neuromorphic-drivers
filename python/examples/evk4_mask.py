import pathlib

import neuromorphic_drivers as nd
import numpy as np
import plotly.express

dirname = pathlib.Path(__file__).resolve().parent

pixel_mask = nd.PropheseeEvk4PixelMask()
pixel_mask.set(63, 1026, 664)
nd.write_mask_as_png(
    pixel_mask.pixels(),
    dirname / "evk4_mask.png",
)

print(pixel_mask.pixel_mask())

counts = np.zeros(
    (nd.prophesee_evk4.Properties().width, nd.prophesee_evk4.Properties().height),
    dtype=np.uint64,
)

with nd.open(
    configuration=nd.prophesee_evk4.Configuration(
        biases=nd.prophesee_evk4.Biases(
            diff_off=50,
        ),
        pixel_mask=pixel_mask.pixel_mask(),
    )
) as device:
    for status, packet in device:
        if packet.polarity_events is not None:
            np.add.at(
                counts, (packet.polarity_events["x"], packet.polarity_events["y"]), 1
            )
            if packet.polarity_events["t"][-1] > 2000000:
                break

log_counts = np.flipud(np.log((counts + 1).astype(np.float64)).transpose())
figure = plotly.express.imshow(log_counts, color_continuous_scale="thermal")
figure.update_layout(
    width=nd.prophesee_evk4.Properties().width,
    height=nd.prophesee_evk4.Properties().height,
    margin=dict(l=0, r=0, b=0, t=0),
    xaxis={"visible": False, "showticklabels": False},
    yaxis={"visible": False, "showticklabels": False},
    showlegend=False,
)
figure.update_coloraxes(showscale=False)
figure.write_image(dirname / f"evk4_mask_accumulated.png")
