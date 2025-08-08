import typing

import numpy
import numpy.typing


class Frame:
    start_t: int
    """
    Timestamp of the start of the frame acquisition process

    start_t < exposure_start_t < exposure_end_t < t
    """

    exposure_start_t: typing.Optional[int]
    """
    Timestamp of the start of the frame exposure

    start_t < exposure_start_t < exposure_end_t < t
    """

    exposure_end_t: typing.Optional[int]
    """
    Timestamp of the end of the frame exposure

    start_t < exposure_start_t < exposure_end_t < t
    """

    t: int
    """
    Timestamp of the end of the frame acquisition process

    start_t < exposure_start_t < exposure_end_t < t

    Frames
    """

    pixels: numpy.typing.NDArray[numpy.uint16]
    """
    Pixel values in the range [0, 65535]

    The Davis346 ADC has a precision of 10 bits, hence the sensor values are in the range [0, 1023]

    We multiply the raw values by 64, making the effective range [0, 65472], to avoid display issues
    (16 bits images look very dark if only the range [0, 1023] is used)
    """


class Davis346Packet:
    polarity_events: typing.Optional[numpy.ndarray]
    """
    Polarity events (also known as change detection events and ON/OFF events)

    Timestamps are in microseconds, the origin (x = 0 and y = 0) is at the top-left corner
    """

    imu_events: typing.Optional[numpy.ndarray]
    """
    Inertial Measurement Unit events (or samples)

    Each sample contains the acceleration (in m/s), the rotation (in rad/s), and the temperature (in ºC)

    The IMU's X axis is parallel to the camera's horizontal direction and oriented left-to-right when looking in the same direction as the camera

    The IMU's Y axis is parallel to the camera's vertical direction and oriented bottom-to-top

    The IMU's Z axis is aligned with the camera's optical axis and oriented in the same direction as the camera (sensor-to-lens)
    """

    trigger_events: typing.Optional[numpy.ndarray]
    """
    External trigger events (rising and falling edges), timestamped with the same clock as the polarity events

    Polarity 0 indicates a falling edge, 1 a rising edge, and 2 a pulse
    """

    frames: list[Frame]
    """
    Grey level frames (also known as Active Pixel Sensor)
    """

    polarity_events_overflow_indices: typing.Optional[numpy.ndarray]
    """
    Each entry in this list indicates an overflow (USB packets dropped by the computer because the queue was full)

    For each overflow, the array entry is the index (in polarity_events) of the first event after the overflow
    """

    imu_events_overflow_indices: typing.Optional[numpy.ndarray]
    """
    Each entry in this list indicates an overflow (USB packets dropped by the computer because the queue was full)

    For each overflow, the array entry is the index (in imu_events) of the first event after the overflow
    """

    trigger_events_overflow_indices: typing.Optional[numpy.ndarray]
    """
    Each entry in this list indicates an overflow (USB packets dropped by the computer because the queue was full)

    For each overflow, the array entry is the index (in trigger_events) of the first event after the overflow
    """

    frames_overflow_indices: typing.Optional[numpy.ndarray]
    """
    Each entry in this list indicates an overflow (USB packets dropped by the computer because the queue was full)

    For each overflow, the array entry is the index (in frames) of the first event after the overflow
    """

class DvxplorerPacket:
    polarity_events: typing.Optional[numpy.ndarray]
    """
    Polarity events (also known as change detection events and ON/OFF events)

    Timestamps are in microseconds, the origin (x = 0 and y = 0) is at the top-left corner
    """

    imu_events: typing.Optional[numpy.ndarray]
    """
    Inertial Measurement Unit events (or samples)

    Each sample contains the acceleration (in m/s), the rotation (in rad/s), and the temperature (in ºC)

    The IMU's X axis is parallel to the camera's horizontal direction and oriented left-to-right when looking in the same direction as the camera

    The IMU's Y axis is parallel to the camera's vertical direction and oriented bottom-to-top

    The IMU's Z axis is aligned with the camera's optical axis and oriented in the same direction as the camera (sensor-to-lens)
    """

    trigger_events: typing.Optional[numpy.ndarray]
    """
    External trigger events (rising and falling edges), timestamped with the same clock as the polarity events
    """

    polarity_events_overflow_indices: typing.Optional[numpy.ndarray]
    """
    Each entry in this list indicates an overflow (USB packets dropped by the computer because the queue was full)

    For each overflow, the array entry is the index (in polarity_events) of the first event after the overflow
    """

    imu_events_overflow_indices: typing.Optional[numpy.ndarray]
    """
    Each entry in this list indicates an overflow (USB packets dropped by the computer because the queue was full)

    For each overflow, the array entry is the index (in imu_events) of the first event after the overflow
    """

    trigger_events_overflow_indices: typing.Optional[numpy.ndarray]
    """
    Each entry in this list indicates an overflow (USB packets dropped by the computer because the queue was full)

    For each overflow, the array entry is the index (in trigger_events) of the first event after the overflow
    """


class Evt3Packet:
    polarity_events: typing.Optional[numpy.ndarray]
    """
    Polarity events (also known as change detection events and ON/OFF events)

    Timestamps are in microseconds, the origin (x = 0 and y = 0) is at the top-left corner
    """

    trigger_events: typing.Optional[numpy.ndarray]
    """
    External trigger events (rising and falling edges), timestamped with the same clock as the polarity events
    """

    polarity_events_overflow_indices: typing.Optional[numpy.ndarray]
    """
    Each entry in this list indicates an overflow (USB packets dropped by the computer because the queue was full)

    For each overflow, the array entry is the index (in polarity_events) of the first event after the overflow
    """

    trigger_events_overflow_indices: typing.Optional[numpy.ndarray]
    """
    Each entry in this list indicates an overflow (USB packets dropped by the computer because the queue was full)

    For each overflow, the array entry is the index (in trigger_events) of the first event after the overflow
    """
