import dataclasses
import enum
import types
import typing

import numpy

from ... import serde
from ... import status
from .. import enums


@dataclasses.dataclass
class ActivityFilter:
    mask_isolated_enable: bool = True
    mask_isolated_tau: serde.type.uint32 = 8
    refractory_period_enable: bool = False
    refractory_period_tau: serde.type.uint32 = 1

    def serialize(self) -> bytes:
        return serde.bincode.serialize(self, ActivityFilter)


@dataclasses.dataclass
class Biases:
    localbufbn: serde.type.uint16 = 1444
    padfollbn: serde.type.uint16 = 2007
    diffbn: serde.type.uint16 = 1063
    onbn: serde.type.uint16 = 1535
    offbn: serde.type.uint16 = 1025
    pixinvbn: serde.type.uint16 = 1680
    prbp: serde.type.uint16 = 570
    prsfbp: serde.type.uint16 = 272
    refrbp: serde.type.uint16 = 1049
    readoutbufbp: serde.type.uint16 = 1556
    apsrosfbn: serde.type.uint16 = 1755
    adccompbp: serde.type.uint16 = 1300
    colsellowbn: serde.type.uint16 = 1
    dacbufbp: serde.type.uint16 = 1596
    lcoltimeoutbn: serde.type.uint16 = 1329
    aepdbn: serde.type.uint16 = 1627
    aepuxbp: serde.type.uint16 = 1104
    aepuybp: serde.type.uint16 = 1944
    ifrefrbn: serde.type.uint16 = 1535
    ifthrbn: serde.type.uint16 = 1535
    biasbuffer: serde.type.uint16 = 1534

    def serialize(self) -> bytes:
        return serde.bincode.serialize(self, Biases)


class PolarityFilter(enum.Enum):
    DISABLED = 0
    FLATTEN = 1
    MASK_ON = 2
    MASK_OFF = 3
    MASK_OFF_FLATTEN = 4

    def serialize(self) -> bytes:
        return serde.bincode.serialize(self, PolarityFilter)


@dataclasses.dataclass
class RegionOfInterest:
    left: serde.type.uint16 = 0
    top: serde.type.uint16 = 0
    width: serde.type.uint16 = 346
    height: serde.type.uint16 = 260

    def serialize(self) -> bytes:
        return serde.bincode.serialize(self, RegionOfInterest)


@dataclasses.dataclass
class Configuration:
    biases: Biases = dataclasses.field(default_factory=Biases)
    region_of_interest: RegionOfInterest = dataclasses.field(default_factory=RegionOfInterest)
    pixel_mask: tuple[
        serde.type.uint32,
        serde.type.uint32,
        serde.type.uint32,
        serde.type.uint32,
        serde.type.uint32,
        serde.type.uint32,
        serde.type.uint32,
        serde.type.uint32,
    ] = (0, 0, 0, 0, 0, 0, 0, 0)
    activity_filter: ActivityFilter = dataclasses.field(default_factory=ActivityFilter)
    skip_events_every: serde.type.uint32 = 0
    polarity_filter: PolarityFilter = PolarityFilter.DISABLED
    exposure_us: serde.type.uint32 = 4000
    frame_interval_us: serde.type.uint32 = 40000

    def serialize(self) -> bytes:
        return serde.bincode.serialize(self, Configuration)

    @staticmethod
    def type() -> str:
        return "inivation_davis346"


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
    width: serde.type.uint16 = 346
    height: serde.type.uint16 = 260


class InivationDavis346Device(typing.Protocol):
    def __enter__(self) -> "InivationDavis346Device": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool:
        ...

    def __iter__(self) -> "InivationDavis346Device": ...

    def __next__(self) -> tuple[status.StatusNonOptional, dict[str, numpy.ndarray[typing.Any, numpy.dtype[numpy.void]]]]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.INIVATION_DAVIS346]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def chip_firmware_configuration(self) -> Configuration: ...

    def speed(self) -> enums.Speed: ...

    def temperature_celsius(self) -> float: ...

    def update_configuration(self, configuration: Configuration): ...


class InivationDavis346DeviceOptional(typing.Protocol):
    def __enter__(self) -> "InivationDavis346DeviceOptional": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool:
        ...

    def __iter__(self) -> "InivationDavis346DeviceOptional": ...

    def __next__(self) -> tuple[status.Status, typing.Optional[dict[str, numpy.ndarray[typing.Any, numpy.dtype[numpy.void]]]]]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.INIVATION_DAVIS346]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def chip_firmware_configuration(self) -> Configuration: ...

    def speed(self) -> enums.Speed: ...

    def temperature_celsius(self) -> float: ...

    def update_configuration(self, configuration: Configuration): ...


class InivationDavis346DeviceRaw(typing.Protocol):
    def __enter__(self) -> "InivationDavis346DeviceRaw": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool:
        ...

    def __iter__(self) -> "InivationDavis346DeviceRaw": ...

    def __next__(self) -> tuple[status.RawStatusNonOptional, bytes]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.INIVATION_DAVIS346]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def chip_firmware_configuration(self) -> Configuration: ...

    def speed(self) -> enums.Speed: ...

    def temperature_celsius(self) -> float: ...

    def update_configuration(self, configuration: Configuration): ...


class InivationDavis346DeviceRawOptional(typing.Protocol):
    def __enter__(self) -> "InivationDavis346DeviceRawOptional": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool:
        ...

    def __iter__(self) -> "InivationDavis346DeviceRawOptional": ...

    def __next__(self) -> tuple[status.RawStatus, typing.Optional[bytes]]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> typing.Literal[enums.Name.INIVATION_DAVIS346]: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def chip_firmware_configuration(self) -> Configuration: ...

    def speed(self) -> enums.Speed: ...

    def temperature_celsius(self) -> float: ...

    def update_configuration(self, configuration: Configuration): ...
