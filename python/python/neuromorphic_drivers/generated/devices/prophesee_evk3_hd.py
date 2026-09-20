import dataclasses
import enum
import types
import typing

import numpy

from .. import enums
from ... import orientation as orientation_module
from ... import packet
from ... import serde
from ... import status


@dataclasses.dataclass
class Biases:
    pr: serde.type.uint8 = 0x69
    fo_p: serde.type.uint8 = 0x4A
    fo_n: serde.type.uint8 = 0x00
    hpf: serde.type.uint8 = 0x00
    diff_on: serde.type.uint8 = 0x73
    diff: serde.type.uint8 = 0x50
    diff_off: serde.type.uint8 = 0x34
    refr: serde.type.uint8 = 0x44
    reqpuy: serde.type.uint8 = 0x94
    blk: serde.type.uint8 = 0x78

    def serialize(self) -> bytes:
        return serde.bincode.serialize(self, Biases)


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
    mask_intersection_only: bool = False
    rate_limiter: typing.Optional[RateLimiter] = None

    def serialize(self) -> bytes:
        return serde.bincode.serialize(self, Configuration)

    @staticmethod
    def type() -> str:
        return "prophesee_evk3_hd"


@dataclasses.dataclass(frozen=True)
class Bounds:
    minimum: serde.type.uint8 = 0x00
    maximum: serde.type.uint8 = 0x00


@dataclasses.dataclass(frozen=True)
class BiasesBounds:
    pr: Bounds = dataclasses.field(default_factory=Bounds)
    fo_p: Bounds = dataclasses.field(default_factory=Bounds)
    fo_n: Bounds = dataclasses.field(default_factory=Bounds)
    hpf: Bounds = dataclasses.field(default_factory=Bounds)
    diff_on: Bounds = dataclasses.field(default_factory=Bounds)
    diff: Bounds = dataclasses.field(default_factory=Bounds)
    diff_off: Bounds = dataclasses.field(default_factory=Bounds)
    refr: Bounds = dataclasses.field(default_factory=Bounds)
    reqpuy: Bounds = dataclasses.field(default_factory=Bounds)
    blk: Bounds = dataclasses.field(default_factory=Bounds)


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


class PropheseeEvk3HdDevice(typing.Protocol):
    def __enter__(self) -> "PropheseeEvk3HdDevice": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool: ...

    def close(self) -> None: ...

    def __iter__(self) -> "PropheseeEvk3HdDevice": ...

    def __next__(self) -> tuple[status.StatusNonOptional, packet.Evt3Packet]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.PROPHESEE_EVK3_HD]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def biases_bounds(self) -> BiasesBounds: ...

    def connection(self) -> enums.Connection: ...

    def update_configuration(self, configuration: Configuration): ...


class PropheseeEvk3HdDeviceOptional(typing.Protocol):
    def __enter__(self) -> "PropheseeEvk3HdDeviceOptional": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool: ...

    def close(self) -> None: ...

    def __iter__(self) -> "PropheseeEvk3HdDeviceOptional": ...

    def __next__(self) -> tuple[status.Status, typing.Optional[packet.Evt3Packet]]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.PROPHESEE_EVK3_HD]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def biases_bounds(self) -> BiasesBounds: ...

    def connection(self) -> enums.Connection: ...

    def update_configuration(self, configuration: Configuration): ...


class PropheseeEvk3HdDeviceRaw(typing.Protocol):
    def __enter__(self) -> "PropheseeEvk3HdDeviceRaw": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool: ...

    def close(self) -> None: ...

    def __iter__(self) -> "PropheseeEvk3HdDeviceRaw": ...

    def __next__(self) -> tuple[status.RawStatusNonOptional, bytes]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.PROPHESEE_EVK3_HD]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def biases_bounds(self) -> BiasesBounds: ...

    def connection(self) -> enums.Connection: ...

    def update_configuration(self, configuration: Configuration): ...


class PropheseeEvk3HdDeviceRawOptional(typing.Protocol):
    def __enter__(self) -> "PropheseeEvk3HdDeviceRawOptional": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool: ...

    def close(self) -> None: ...

    def __iter__(self) -> "PropheseeEvk3HdDeviceRawOptional": ...

    def __next__(self) -> tuple[status.RawStatus, typing.Optional[bytes]]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.PROPHESEE_EVK3_HD]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def biases_bounds(self) -> BiasesBounds: ...

    def connection(self) -> enums.Connection: ...

    def update_configuration(self, configuration: Configuration): ...
