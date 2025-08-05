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
    amp: serde.type.uint8 = 0x04
    on: serde.type.uint8 = 0x08
    off: serde.type.uint8 = 0x08
    sf: serde.type.uint8 = 0x00
    nrst: bool = False
    log: bool = False
    log_a: bool = True
    log_d: serde.type.uint8 = 0x01

    def serialize(self) -> bytes:
        return serde.bincode.serialize(self, Biases)


class ReadoutFramesPerSecond(enum.Enum):
    CONSTANT100 = 0
    CONSTANT200 = 1
    CONSTANT500 = 2
    CONSTANT1000 = 3
    CONSTANT_LOSSY2000 = 4
    CONSTANT_LOSSY5000 = 5
    CONSTANT_LOSSY10000 = 6
    VARIABLE2000 = 7
    VARIABLE5000 = 8
    VARIABLE10000 = 9
    VARIABLE15000 = 10

    def serialize(self) -> bytes:
        return serde.bincode.serialize(self, ReadoutFramesPerSecond)


@dataclasses.dataclass
class Configuration:
    biases: Biases = dataclasses.field(default_factory=Biases)
    readout_frames_per_second: ReadoutFramesPerSecond = ReadoutFramesPerSecond.CONSTANT100

    def serialize(self) -> bytes:
        return serde.bincode.serialize(self, Configuration)

    @staticmethod
    def type() -> str:
        return "inivation_dvxplorer"


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


class InivationDvxplorerDevice(typing.Protocol):
    def __enter__(self) -> "InivationDvxplorerDevice": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool:
        ...

    def __iter__(self) -> "InivationDvxplorerDevice": ...

    def __next__(self) -> tuple[status.StatusNonOptional, packet.DvxplorerPacket]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.INIVATION_DVXPLORER]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def chip_firmware_configuration(self) -> Configuration: ...

    def speed(self) -> enums.Speed: ...

    def update_configuration(self, configuration: Configuration): ...

    def orientation(self) -> orientation.DvxplorerOrientation: ...



class InivationDvxplorerDeviceOptional(typing.Protocol):
    def __enter__(self) -> "InivationDvxplorerDeviceOptional": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool:
        ...

    def __iter__(self) -> "InivationDvxplorerDeviceOptional": ...

    def __next__(self) -> tuple[status.Status, typing.Optional[packet.DvxplorerPacket]]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.INIVATION_DVXPLORER]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def chip_firmware_configuration(self) -> Configuration: ...

    def speed(self) -> enums.Speed: ...

    def update_configuration(self, configuration: Configuration): ...

    def orientation(self) -> orientation.DvxplorerOrientation: ...



class InivationDvxplorerDeviceRaw(typing.Protocol):
    def __enter__(self) -> "InivationDvxplorerDeviceRaw": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool:
        ...

    def __iter__(self) -> "InivationDvxplorerDeviceRaw": ...

    def __next__(self) -> tuple[status.RawStatusNonOptional, bytes]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.INIVATION_DVXPLORER]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def chip_firmware_configuration(self) -> Configuration: ...

    def speed(self) -> enums.Speed: ...

    def update_configuration(self, configuration: Configuration): ...

    def orientation(self) -> orientation.DvxplorerOrientation: ...



class InivationDvxplorerDeviceRawOptional(typing.Protocol):
    def __enter__(self) -> "InivationDvxplorerDeviceRawOptional": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool:
        ...

    def __iter__(self) -> "InivationDvxplorerDeviceRawOptional": ...

    def __next__(self) -> tuple[status.RawStatus, typing.Optional[bytes]]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.INIVATION_DVXPLORER]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def chip_firmware_configuration(self) -> Configuration: ...

    def speed(self) -> enums.Speed: ...

    def update_configuration(self, configuration: Configuration): ...

    def orientation(self) -> orientation.DvxplorerOrientation: ...

