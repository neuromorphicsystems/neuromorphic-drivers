import pathlib

import neuromorphic_drivers as nd

dirname = pathlib.Path(__file__).resolve().parent

row_column_mask = nd.RowColumnMask(
    width=nd.prophesee_evk4.Properties.width,
    height=nd.prophesee_evk4.Properties.height,
    set=True,
)

row_column_mask.clear_rectangle(x=576, y=296, width=100, height=100)

nd.write_mask_as_png(
    row_column_mask.pixels(),
    dirname / "evk4_row_column_mask.png",
)

configuration = nd.prophesee_evk4.Configuration(
    biases=nd.prophesee_evk4.Biases(
        diff_off=50,
    ),
    x_mask=row_column_mask.x_mask(),  # type: ignore
    y_mask=row_column_mask.y_mask(),  # type: ignore
    mask_intersection_only=False,
)

with nd.open(configuration=configuration) as device:
    for status, packet in device:
        ...
