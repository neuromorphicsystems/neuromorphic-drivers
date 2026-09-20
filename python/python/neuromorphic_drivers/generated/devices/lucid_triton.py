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
    fo: serde.type.int16 = 0
    hpf: serde.type.int16 = 0
    diff_on: serde.type.int16 = 0
    diff: serde.type.int16 = 0
    diff_off: serde.type.int16 = 0
    refr: serde.type.int16 = 0

    def serialize(self) -> bytes:
        return serde.bincode.serialize(self, Biases)


@dataclasses.dataclass
class Configuration:
    biases: Biases = dataclasses.field(default_factory=Biases)
    enable_output: bool = True

    def serialize(self) -> bytes:
        return serde.bincode.serialize(self, Configuration)

    @staticmethod
    def type() -> str:
        return "lucid_triton"


@dataclasses.dataclass(frozen=True)
class Bounds:
    minimum: serde.type.int16 = 0
    maximum: serde.type.int16 = 0


@dataclasses.dataclass(frozen=True)
class BiasesBounds:
    fo: Bounds = dataclasses.field(default_factory=Bounds)
    hpf: Bounds = dataclasses.field(default_factory=Bounds)
    diff_on: Bounds = dataclasses.field(default_factory=Bounds)
    diff: Bounds = dataclasses.field(default_factory=Bounds)
    diff_off: Bounds = dataclasses.field(default_factory=Bounds)
    refr: Bounds = dataclasses.field(default_factory=Bounds)


@dataclasses.dataclass
class RingConfiguration:
    buffer_length: serde.type.uint64 = 1400
    ring_length: serde.type.uint64 = 131072
    parallel_submissions: serde.type.uint64 = 1

    def serialize(self) -> bytes:
        return serde.bincode.serialize(self, RingConfiguration)


@dataclasses.dataclass(frozen=True)
class Properties:
    width: serde.type.uint16 = 1280
    height: serde.type.uint16 = 720


class LucidTritonDevice(typing.Protocol):
    def __enter__(self) -> "LucidTritonDevice": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool: ...

    def close(self) -> None: ...

    def __iter__(self) -> "LucidTritonDevice": ...

    def __next__(self) -> tuple[status.StatusNonOptional, packet.Evt3Packet]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.LUCID_TRITON]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def biases_bounds(self) -> BiasesBounds: ...

    def connection(self) -> enums.Connection: ...

    def update_configuration(self, configuration: Configuration): ...

    def address(self) -> str: ...

    def temperature_celsius(self) -> float: ...


class LucidTritonDeviceOptional(typing.Protocol):
    def __enter__(self) -> "LucidTritonDeviceOptional": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool: ...

    def close(self) -> None: ...

    def __iter__(self) -> "LucidTritonDeviceOptional": ...

    def __next__(self) -> tuple[status.Status, typing.Optional[packet.Evt3Packet]]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.LUCID_TRITON]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def biases_bounds(self) -> BiasesBounds: ...

    def connection(self) -> enums.Connection: ...

    def update_configuration(self, configuration: Configuration): ...

    def address(self) -> str: ...

    def temperature_celsius(self) -> float: ...


class LucidTritonDeviceRaw(typing.Protocol):
    def __enter__(self) -> "LucidTritonDeviceRaw": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool: ...

    def close(self) -> None: ...

    def __iter__(self) -> "LucidTritonDeviceRaw": ...

    def __next__(self) -> tuple[status.RawStatusNonOptional, bytes]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.LUCID_TRITON]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def biases_bounds(self) -> BiasesBounds: ...

    def connection(self) -> enums.Connection: ...

    def update_configuration(self, configuration: Configuration): ...

    def address(self) -> str: ...

    def temperature_celsius(self) -> float: ...


class LucidTritonDeviceRawOptional(typing.Protocol):
    def __enter__(self) -> "LucidTritonDeviceRawOptional": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool: ...

    def close(self) -> None: ...

    def __iter__(self) -> "LucidTritonDeviceRawOptional": ...

    def __next__(self) -> tuple[status.RawStatus, typing.Optional[bytes]]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.LUCID_TRITON]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def biases_bounds(self) -> BiasesBounds: ...

    def connection(self) -> enums.Connection: ...

    def update_configuration(self, configuration: Configuration): ...

    def address(self) -> str: ...

    def temperature_celsius(self) -> float: ...
