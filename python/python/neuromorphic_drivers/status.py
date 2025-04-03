import dataclasses
import typing


@dataclasses.dataclass
class RawRingStatus:
    system_time: float
    """
    Seconds since the UNIX epoch (1970-01-01 00:00:00 UTC) measured when the USB packet was received.
    """

    backlog: int
    raw_packets: int
    clutch_engaged: bool
    overflow_indices: typing.Optional[list[int]]


@dataclasses.dataclass
class RingStatus:
    system_time: float
    """
    Seconds since the UNIX epoch (1970-01-01 00:00:00 UTC) measured when the USB packet was received.
    """

    backlog: int
    raw_packets: int
    clutch_engaged: bool
    current_t: int


@dataclasses.dataclass
class RawStatus:
    system_time: float
    """
    Seconds since the UNIX epoch (1970-01-01 00:00:00 UTC) measured when the user consumed the packet.

    Status.ring.system_time is always smaller than status.system_time.
    The difference between Status.ring.system_time and Status.system_time is a measurement
    of software latency (status.system_time - status.ring.system_time).
    """

    ring: typing.Optional[RawRingStatus]
    """
    ring is None if no data became available before iterator_timeout
    ring may only be None if iterator_timeout is not None
    """

    def delay(self) -> typing.Optional[float]:
        if self.ring is None:
            return None
        return self.system_time - self.ring.system_time


@dataclasses.dataclass
class RawStatusNonOptional:
    system_time: float
    """
    Seconds since the UNIX epoch (1970-01-01 00:00:00 UTC) measured when the user consumed the packet.

    Status.ring.system_time is always smaller than status.system_time.
    The difference between Status.ring.system_time and Status.system_time is a measurement
    of software latency (status.system_time - status.ring.system_time).
    """

    ring: RawRingStatus

    def delay(self) -> float: ...


@dataclasses.dataclass
class Status:
    system_time: float
    """
    Seconds since the UNIX epoch (1970-01-01 00:00:00 UTC) measured when the user consumed the packet.

    Status.ring.system_time is always smaller than status.system_time.
    The difference between Status.ring.system_time and Status.system_time is a measurement
    of software latency (status.system_time - status.ring.system_time).
    """

    ring: typing.Optional[RingStatus]
    """
    ring is None if no data became available before iterator_timeout
    ring may only be None if iterator_timeout is not None
    """

    def delay(self) -> typing.Optional[float]:
        if self.ring is None:
            return None
        return self.system_time - self.ring.system_time


@dataclasses.dataclass
class StatusNonOptional:
    system_time: float
    """
    Seconds since the UNIX epoch (1970-01-01 00:00:00 UTC) measured when the user consumed the packet.

    Status.ring.system_time is always smaller than status.system_time.
    The difference between Status.ring.system_time and Status.system_time is a measurement
    of software latency (status.system_time - status.ring.system_time).
    """

    ring: RingStatus

    def delay(self) -> float: ...
