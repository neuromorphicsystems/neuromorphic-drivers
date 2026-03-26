import dataclasses
import enum
import types
import typing

import numpy

from .. import enums
from ... import orientation
from ... import packet
from ... import serde
from ... import status


@dataclasses.dataclass
class Biases:
    diff_on: serde.type.uint16 = 384
    diff: serde.type.uint16 = 296
    diff_off: serde.type.uint16 = 222
    fo: serde.type.uint16 = 1477
    hpf: serde.type.uint16 = 1499
    pr: serde.type.uint16 = 1250
    refr: serde.type.uint16 = 1500

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
    ] = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    y_mask: tuple[
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
    ] = (0, 0, 0, 0, 0, 0, 0, 0)
    mask_intersection_only: bool = False
    rate_limiter: typing.Optional[RateLimiter] = None

    def serialize(self) -> bytes:
        return serde.bincode.serialize(self, Configuration)

    @staticmethod
    def type() -> str:
        return "centuryarks_vga"


@dataclasses.dataclass
class UsbConfiguration:
    buffer_length: serde.type.uint64 = 131072
    ring_length: serde.type.uint64 = 4096
    transfer_queue_length: serde.type.uint64 = 32
    allow_dma: bool = False

    def serialize(self) -> bytes:
        return serde.bincode.serialize(self, UsbConfiguration)


@dataclasses.dataclass(frozen=True)
class Properties:
    width: serde.type.uint16 = 640
    height: serde.type.uint16 = 480


class CenturyarksVgaDevice(typing.Protocol):
    def __enter__(self) -> "CenturyarksVgaDevice": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool:
        ...

    def __iter__(self) -> "CenturyarksVgaDevice": ...

    def __next__(self) -> tuple[status.StatusNonOptional, packet.Evt3Packet]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.CENTURYARKS_VGA]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def chip_firmware_configuration(self) -> Configuration: ...

    def speed(self) -> enums.Speed: ...

    def update_configuration(self, configuration: Configuration): ...


class CenturyarksVgaDeviceOptional(typing.Protocol):
    def __enter__(self) -> "CenturyarksVgaDeviceOptional": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool:
        ...

    def __iter__(self) -> "CenturyarksVgaDeviceOptional": ...

    def __next__(self) -> tuple[status.Status, typing.Optional[packet.Evt3Packet]]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.CENTURYARKS_VGA]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def chip_firmware_configuration(self) -> Configuration: ...

    def speed(self) -> enums.Speed: ...

    def update_configuration(self, configuration: Configuration): ...


class CenturyarksVgaDeviceRaw(typing.Protocol):
    def __enter__(self) -> "CenturyarksVgaDeviceRaw": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool:
        ...

    def __iter__(self) -> "CenturyarksVgaDeviceRaw": ...

    def __next__(self) -> tuple[status.RawStatusNonOptional, bytes]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.CENTURYARKS_VGA]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def chip_firmware_configuration(self) -> Configuration: ...

    def speed(self) -> enums.Speed: ...

    def update_configuration(self, configuration: Configuration): ...


class CenturyarksVgaDeviceRawOptional(typing.Protocol):
    def __enter__(self) -> "CenturyarksVgaDeviceRawOptional": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool:
        ...

    def __iter__(self) -> "CenturyarksVgaDeviceRawOptional": ...

    def __next__(self) -> tuple[status.RawStatus, typing.Optional[bytes]]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.CENTURYARKS_VGA]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def chip_firmware_configuration(self) -> Configuration: ...

    def speed(self) -> enums.Speed: ...

    def update_configuration(self, configuration: Configuration): ...
