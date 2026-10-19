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
    pr: serde.type.uint8 = 0x83
    fo_n: serde.type.uint8 = 0x17
    hpf: serde.type.uint8 = 0x30
    diff_on: serde.type.uint8 = 0x70
    diff: serde.type.uint8 = 0x45
    diff_off: serde.type.uint8 = 0x34
    refr: serde.type.uint8 = 0x2D

    def serialize(self) -> bytes:
        return serde.bincode.serialize(self, Biases)


@dataclasses.dataclass
class Configuration:
    biases: Biases = dataclasses.field(default_factory=Biases)

    def serialize(self) -> bytes:
        return serde.bincode.serialize(self, Configuration)

    @staticmethod
    def type() -> str:
        return "prophesee_evk1_gen4"


@dataclasses.dataclass(frozen=True)
class Bounds:
    minimum: serde.type.uint8 = 0x00
    maximum: serde.type.uint8 = 0x00


@dataclasses.dataclass(frozen=True)
class BiasesBounds:
    pr: Bounds = dataclasses.field(default_factory=Bounds)
    fo_n: Bounds = dataclasses.field(default_factory=Bounds)
    hpf: Bounds = dataclasses.field(default_factory=Bounds)
    diff_on: Bounds = dataclasses.field(default_factory=Bounds)
    diff: Bounds = dataclasses.field(default_factory=Bounds)
    diff_off: Bounds = dataclasses.field(default_factory=Bounds)
    refr: Bounds = dataclasses.field(default_factory=Bounds)


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


class PropheseeEvk1Gen4Device(typing.Protocol):
    def __enter__(self) -> "PropheseeEvk1Gen4Device": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool: ...

    def close(self) -> None: ...

    def __iter__(self) -> "PropheseeEvk1Gen4Device": ...

    def __next__(self) -> tuple[status.StatusNonOptional, packet.Evt3Packet]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.PROPHESEE_EVK1_GEN4]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def biases_bounds(self) -> BiasesBounds: ...

    def connection(self) -> enums.Connection: ...

    def update_configuration(self, configuration: Configuration): ...


class PropheseeEvk1Gen4DeviceOptional(typing.Protocol):
    def __enter__(self) -> "PropheseeEvk1Gen4DeviceOptional": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool: ...

    def close(self) -> None: ...

    def __iter__(self) -> "PropheseeEvk1Gen4DeviceOptional": ...

    def __next__(self) -> tuple[status.Status, typing.Optional[packet.Evt3Packet]]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.PROPHESEE_EVK1_GEN4]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def biases_bounds(self) -> BiasesBounds: ...

    def connection(self) -> enums.Connection: ...

    def update_configuration(self, configuration: Configuration): ...


class PropheseeEvk1Gen4DeviceRaw(typing.Protocol):
    def __enter__(self) -> "PropheseeEvk1Gen4DeviceRaw": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool: ...

    def close(self) -> None: ...

    def __iter__(self) -> "PropheseeEvk1Gen4DeviceRaw": ...

    def __next__(self) -> tuple[status.RawStatusNonOptional, bytes]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.PROPHESEE_EVK1_GEN4]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def biases_bounds(self) -> BiasesBounds: ...

    def connection(self) -> enums.Connection: ...

    def update_configuration(self, configuration: Configuration): ...


class PropheseeEvk1Gen4DeviceRawOptional(typing.Protocol):
    def __enter__(self) -> "PropheseeEvk1Gen4DeviceRawOptional": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool: ...

    def close(self) -> None: ...

    def __iter__(self) -> "PropheseeEvk1Gen4DeviceRawOptional": ...

    def __next__(self) -> tuple[status.RawStatus, typing.Optional[bytes]]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.PROPHESEE_EVK1_GEN4]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def biases_bounds(self) -> BiasesBounds: ...

    def connection(self) -> enums.Connection: ...

    def update_configuration(self, configuration: Configuration): ...
