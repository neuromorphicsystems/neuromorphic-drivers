import typing

import numpy
import numpy.typing


class Frame:
    start_t: int
    exposure_start_t: typing.Optional[int]
    exposure_end_t: typing.Optional[int]
    t: int
    pixels: numpy.typing.NDArray[numpy.uint16]


class Davis346Packet:
    polarity_events: typing.Optional[numpy.ndarray]
    polarity_events_overflow_indices: typing.Optional[numpy.ndarray]
    frames: list[Frame]


class Evt3Packet:
    polarity_events: typing.Optional[numpy.ndarray]
    polarity_events_overflow_indices: typing.Optional[numpy.ndarray]
    trigger_events: typing.Optional[numpy.ndarray]
    trigger_events_overflow_indices: typing.Optional[numpy.ndarray]
