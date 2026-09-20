import dataclasses
import enum
import types
import typing

import numpy

from .. import enums
from ... import mask
from ... import orientation
from ... import packet
from ... import serde
from ... import status


@dataclasses.dataclass
class Biases:
    pr: serde.type.int16 = 0
    fo: serde.type.int16 = 0
    hpf: serde.type.int16 = 0
    diff_on: serde.type.int16 = 0
    diff: serde.type.int16 = 0
    diff_off: serde.type.int16 = 0
    inv: serde.type.int16 = 0
    refr: serde.type.int16 = 0
    reqpuy: serde.type.int16 = 0
    reqpux: serde.type.int16 = 0
    sendreqpdy: serde.type.int16 = 0
    unknown_1: serde.type.int16 = 0
    unknown_2: serde.type.int16 = 0

    def serialize(self) -> bytes:
        return serde.bincode.serialize(self, Biases)


class Clock(enum.Enum):
    INTERNAL = 0
    INTERNAL_WITH_OUTPUT_ENABLED = 1
    EXTERNAL = 2

    def serialize(self) -> bytes:
        return serde.bincode.serialize(self, Clock)


@dataclasses.dataclass
class RateLimiter:
    reference_period_us: serde.type.uint16
    maximum_events_per_period: serde.type.uint32

    def serialize(self) -> bytes:
        return serde.bincode.serialize(self, RateLimiter)


@dataclasses.dataclass
class Configuration:
    biases: Biases = dataclasses.field(default_factory=Biases)
    x_mask: tuple[
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
    ] = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    y_mask: tuple[
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
    ] = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    pixel_mask: tuple[
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
    ] = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    mask_intersection_only: bool = False
    enable_external_trigger: bool = True
    clock: Clock = Clock.INTERNAL
    rate_limiter: typing.Optional[RateLimiter] = None
    enable_output: bool = True

    def serialize(self) -> bytes:
        return serde.bincode.serialize(self, Configuration)

    @staticmethod
    def type() -> str:
        return "prophesee_evk4"


@dataclasses.dataclass(frozen=True)
class Bounds:
    minimum: serde.type.int16 = 0
    maximum: serde.type.int16 = 0


@dataclasses.dataclass(frozen=True)
class BiasesBounds:
    pr: Bounds = dataclasses.field(default_factory=Bounds)
    fo: Bounds = dataclasses.field(default_factory=Bounds)
    hpf: Bounds = dataclasses.field(default_factory=Bounds)
    diff_on: Bounds = dataclasses.field(default_factory=Bounds)
    diff: Bounds = dataclasses.field(default_factory=Bounds)
    diff_off: Bounds = dataclasses.field(default_factory=Bounds)
    inv: Bounds = dataclasses.field(default_factory=Bounds)
    refr: Bounds = dataclasses.field(default_factory=Bounds)
    reqpuy: Bounds = dataclasses.field(default_factory=Bounds)
    reqpux: Bounds = dataclasses.field(default_factory=Bounds)
    sendreqpdy: Bounds = dataclasses.field(default_factory=Bounds)
    unknown_1: Bounds = dataclasses.field(default_factory=Bounds)
    unknown_2: Bounds = dataclasses.field(default_factory=Bounds)


@dataclasses.dataclass
class RingConfiguration:
    buffer_length: serde.type.uint64 = 131072
    ring_length: serde.type.uint64 = 4096
    parallel_submissions: serde.type.uint64 = 32

    def serialize(self) -> bytes:
        return serde.bincode.serialize(self, RingConfiguration)


@dataclasses.dataclass(frozen=True)
class Properties:
    width: serde.type.uint16 = 1280
    height: serde.type.uint16 = 720


XMask = tuple[
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
]

YMask = tuple[
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
    serde.type.uint64,
]


class RowColumnMask(mask.RowColumnMask[XMask, YMask]):
    def __init__(self, set: bool):
        super().__init__(width=Properties.width, height=Properties.height, set=set)


class PropheseeEvk4Device(typing.Protocol):
    def __enter__(self) -> "PropheseeEvk4Device": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool: ...

    def close(self) -> None: ...

    def __iter__(self) -> "PropheseeEvk4Device": ...

    def __next__(self) -> tuple[status.StatusNonOptional, packet.Evt3Packet]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.PROPHESEE_EVK4]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def biases_bounds(self) -> BiasesBounds: ...

    def connection(self) -> enums.Connection: ...

    def update_configuration(self, configuration: Configuration): ...

    def temperature_celsius(self) -> float: ...

    def illuminance(self) -> int: ...


class PropheseeEvk4DeviceOptional(typing.Protocol):
    def __enter__(self) -> "PropheseeEvk4DeviceOptional": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool: ...

    def close(self) -> None: ...

    def __iter__(self) -> "PropheseeEvk4DeviceOptional": ...

    def __next__(self) -> tuple[status.Status, typing.Optional[packet.Evt3Packet]]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.PROPHESEE_EVK4]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def biases_bounds(self) -> BiasesBounds: ...

    def connection(self) -> enums.Connection: ...

    def update_configuration(self, configuration: Configuration): ...

    def temperature_celsius(self) -> float: ...

    def illuminance(self) -> int: ...


class PropheseeEvk4DeviceRaw(typing.Protocol):
    def __enter__(self) -> "PropheseeEvk4DeviceRaw": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool: ...

    def close(self) -> None: ...

    def __iter__(self) -> "PropheseeEvk4DeviceRaw": ...

    def __next__(self) -> tuple[status.RawStatusNonOptional, bytes]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.PROPHESEE_EVK4]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def biases_bounds(self) -> BiasesBounds: ...

    def connection(self) -> enums.Connection: ...

    def update_configuration(self, configuration: Configuration): ...

    def temperature_celsius(self) -> float: ...

    def illuminance(self) -> int: ...


class PropheseeEvk4DeviceRawOptional(typing.Protocol):
    def __enter__(self) -> "PropheseeEvk4DeviceRawOptional": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool: ...

    def close(self) -> None: ...

    def __iter__(self) -> "PropheseeEvk4DeviceRawOptional": ...

    def __next__(self) -> tuple[status.RawStatus, typing.Optional[bytes]]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.PROPHESEE_EVK4]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def biases_bounds(self) -> BiasesBounds: ...

    def connection(self) -> enums.Connection: ...

    def update_configuration(self, configuration: Configuration): ...

    def temperature_celsius(self) -> float: ...

    def illuminance(self) -> int: ...
