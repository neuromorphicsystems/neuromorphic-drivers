# pyright: reportOverlappingOverload=false

import types
import typing

import numpy

from .. import device
from .. import packet
from .. import status
from .devices import inivation_davis346 as inivation_davis346
from .devices import inivation_dvxplorer as inivation_dvxplorer
from .devices import prophesee_evk3_hd as prophesee_evk3_hd
from .devices import prophesee_evk4 as prophesee_evk4
from .enums import *
from .unions import *



class GenericDevice(typing.Protocol):
    def __enter__(self) -> "GenericDevice": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool:
        ...

    def __iter__(self) -> "GenericDevice": ...

    def __next__(self) -> tuple[status.StatusNonOptional, typing.Union[packet.Davis346Packet, packet.DvxplorerPacket, packet.Evt3Packet]]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> Name: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def chip_firmware_configuration(self) -> Configuration: ...

    def speed(self) -> Speed: ...

    def update_configuration(self, configuration: Configuration): ...



class GenericDeviceOptional(typing.Protocol):
    def __enter__(self) -> "GenericDeviceOptional": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool:
        ...

    def __iter__(self) -> "GenericDeviceOptional": ...

    def __next__(self) -> tuple[status.Status, typing.Optional[typing.Union[packet.Davis346Packet, packet.DvxplorerPacket, packet.Evt3Packet]]]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> Name: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def chip_firmware_configuration(self) -> Configuration: ...

    def speed(self) -> Speed: ...

    def update_configuration(self, configuration: Configuration): ...



class GenericDeviceRaw(typing.Protocol):
    def __enter__(self) -> "GenericDeviceRaw": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool:
        ...

    def __iter__(self) -> "GenericDeviceRaw": ...

    def __next__(self) -> tuple[status.RawStatusNonOptional, bytes]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> Name: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def chip_firmware_configuration(self) -> Configuration: ...

    def speed(self) -> Speed: ...

    def update_configuration(self, configuration: Configuration): ...



class GenericDeviceRawOptional(typing.Protocol):
    def __enter__(self) -> "GenericDeviceRawOptional": ...

    def __exit__(
        self,
        exception_type: typing.Optional[typing.Type[BaseException]],
        value: typing.Optional[BaseException],
        traceback: typing.Optional[types.TracebackType],
    ) -> bool:
        ...

    def __iter__(self) -> "GenericDeviceRawOptional": ...

    def __next__(self) -> tuple[status.RawStatus, typing.Optional[bytes]]: ...

    def backlog(self) -> int: ...

    def clear_backlog(self, until: int): ...

    def overflow(self) -> bool: ...

    def name(self) -> Name: ...

    def properties(self) -> Properties: ...

    def serial(self) -> str: ...

    def chip_firmware_configuration(self) -> Configuration: ...

    def speed(self) -> Speed: ...

    def update_configuration(self, configuration: Configuration): ...



@typing.overload
def open(
    configuration: inivation_davis346.Configuration,
    iterator_timeout: typing.Literal[None] = None,
    raw: typing.Literal[False] = False,
    serial: typing.Optional[str] = None,
    usb_configuration: typing.Optional[UsbConfiguration] = None,
    iterator_maximum_raw_packets: int = 64,
) -> inivation_davis346.InivationDavis346Device:
    ...


@typing.overload
def open(
    configuration: inivation_davis346.Configuration,
    iterator_timeout: float,
    raw: typing.Literal[False] = False,
    serial: typing.Optional[str] = None,
    usb_configuration: typing.Optional[UsbConfiguration] = None,
    iterator_maximum_raw_packets: int = 64,
) -> inivation_davis346.InivationDavis346DeviceOptional:
    ...


@typing.overload
def open(
    configuration: inivation_davis346.Configuration,
    iterator_timeout: typing.Literal[None] = None,
    raw: typing.Literal[True] = True,
    serial: typing.Optional[str] = None,
    usb_configuration: typing.Optional[UsbConfiguration] = None,
    iterator_maximum_raw_packets: int = 64,
) -> inivation_davis346.InivationDavis346DeviceRaw:
    ...


@typing.overload
def open(
    configuration: inivation_davis346.Configuration,
    iterator_timeout: float,
    raw: typing.Literal[True] = True,
    serial: typing.Optional[str] = None,
    usb_configuration: typing.Optional[UsbConfiguration] = None,
    iterator_maximum_raw_packets: int = 64,
) -> inivation_davis346.InivationDavis346DeviceRawOptional:
    ...


@typing.overload
def open(
    configuration: inivation_dvxplorer.Configuration,
    iterator_timeout: typing.Literal[None] = None,
    raw: typing.Literal[False] = False,
    serial: typing.Optional[str] = None,
    usb_configuration: typing.Optional[UsbConfiguration] = None,
    iterator_maximum_raw_packets: int = 64,
) -> inivation_dvxplorer.InivationDvxplorerDevice:
    ...


@typing.overload
def open(
    configuration: inivation_dvxplorer.Configuration,
    iterator_timeout: float,
    raw: typing.Literal[False] = False,
    serial: typing.Optional[str] = None,
    usb_configuration: typing.Optional[UsbConfiguration] = None,
    iterator_maximum_raw_packets: int = 64,
) -> inivation_dvxplorer.InivationDvxplorerDeviceOptional:
    ...


@typing.overload
def open(
    configuration: inivation_dvxplorer.Configuration,
    iterator_timeout: typing.Literal[None] = None,
    raw: typing.Literal[True] = True,
    serial: typing.Optional[str] = None,
    usb_configuration: typing.Optional[UsbConfiguration] = None,
    iterator_maximum_raw_packets: int = 64,
) -> inivation_dvxplorer.InivationDvxplorerDeviceRaw:
    ...


@typing.overload
def open(
    configuration: inivation_dvxplorer.Configuration,
    iterator_timeout: float,
    raw: typing.Literal[True] = True,
    serial: typing.Optional[str] = None,
    usb_configuration: typing.Optional[UsbConfiguration] = None,
    iterator_maximum_raw_packets: int = 64,
) -> inivation_dvxplorer.InivationDvxplorerDeviceRawOptional:
    ...


@typing.overload
def open(
    configuration: prophesee_evk3_hd.Configuration,
    iterator_timeout: typing.Literal[None] = None,
    raw: typing.Literal[False] = False,
    serial: typing.Optional[str] = None,
    usb_configuration: typing.Optional[UsbConfiguration] = None,
    iterator_maximum_raw_packets: int = 64,
) -> prophesee_evk3_hd.PropheseeEvk3HdDevice:
    ...


@typing.overload
def open(
    configuration: prophesee_evk3_hd.Configuration,
    iterator_timeout: float,
    raw: typing.Literal[False] = False,
    serial: typing.Optional[str] = None,
    usb_configuration: typing.Optional[UsbConfiguration] = None,
    iterator_maximum_raw_packets: int = 64,
) -> prophesee_evk3_hd.PropheseeEvk3HdDeviceOptional:
    ...


@typing.overload
def open(
    configuration: prophesee_evk3_hd.Configuration,
    iterator_timeout: typing.Literal[None] = None,
    raw: typing.Literal[True] = True,
    serial: typing.Optional[str] = None,
    usb_configuration: typing.Optional[UsbConfiguration] = None,
    iterator_maximum_raw_packets: int = 64,
) -> prophesee_evk3_hd.PropheseeEvk3HdDeviceRaw:
    ...


@typing.overload
def open(
    configuration: prophesee_evk3_hd.Configuration,
    iterator_timeout: float,
    raw: typing.Literal[True] = True,
    serial: typing.Optional[str] = None,
    usb_configuration: typing.Optional[UsbConfiguration] = None,
    iterator_maximum_raw_packets: int = 64,
) -> prophesee_evk3_hd.PropheseeEvk3HdDeviceRawOptional:
    ...


@typing.overload
def open(
    configuration: prophesee_evk4.Configuration,
    iterator_timeout: typing.Literal[None] = None,
    raw: typing.Literal[False] = False,
    serial: typing.Optional[str] = None,
    usb_configuration: typing.Optional[UsbConfiguration] = None,
    iterator_maximum_raw_packets: int = 64,
) -> prophesee_evk4.PropheseeEvk4Device:
    ...


@typing.overload
def open(
    configuration: prophesee_evk4.Configuration,
    iterator_timeout: float,
    raw: typing.Literal[False] = False,
    serial: typing.Optional[str] = None,
    usb_configuration: typing.Optional[UsbConfiguration] = None,
    iterator_maximum_raw_packets: int = 64,
) -> prophesee_evk4.PropheseeEvk4DeviceOptional:
    ...


@typing.overload
def open(
    configuration: prophesee_evk4.Configuration,
    iterator_timeout: typing.Literal[None] = None,
    raw: typing.Literal[True] = True,
    serial: typing.Optional[str] = None,
    usb_configuration: typing.Optional[UsbConfiguration] = None,
    iterator_maximum_raw_packets: int = 64,
) -> prophesee_evk4.PropheseeEvk4DeviceRaw:
    ...


@typing.overload
def open(
    configuration: prophesee_evk4.Configuration,
    iterator_timeout: float,
    raw: typing.Literal[True] = True,
    serial: typing.Optional[str] = None,
    usb_configuration: typing.Optional[UsbConfiguration] = None,
    iterator_maximum_raw_packets: int = 64,
) -> prophesee_evk4.PropheseeEvk4DeviceRawOptional:
    ...


@typing.overload
def open(
    configuration: typing.Optional[Configuration] = None,
    iterator_timeout: typing.Literal[None] = None,
    raw: typing.Literal[False] = False,
    serial: typing.Optional[str] = None,
    usb_configuration: typing.Optional[UsbConfiguration] = None,
    iterator_maximum_raw_packets: int = 64,
) -> GenericDevice:
    ...


@typing.overload
def open(
    configuration: typing.Optional[Configuration] = None,
    iterator_timeout: typing.Optional[float] = None,
    raw: typing.Literal[False] = False,
    serial: typing.Optional[str] = None,
    usb_configuration: typing.Optional[UsbConfiguration] = None,
    iterator_maximum_raw_packets: int = 64,
) -> GenericDeviceOptional:
    ...


@typing.overload
def open(
    configuration: typing.Optional[Configuration] = None,
    iterator_timeout: typing.Literal[None] = None,
    raw: typing.Literal[True] = True,
    serial: typing.Optional[str] = None,
    usb_configuration: typing.Optional[UsbConfiguration] = None,
    iterator_maximum_raw_packets: int = 64,
) -> GenericDeviceRaw:
    ...


@typing.overload
def open(
    configuration: typing.Optional[Configuration] = None,
    iterator_timeout: typing.Optional[float] = None,
    raw: typing.Literal[True] = True,
    serial: typing.Optional[str] = None,
    usb_configuration: typing.Optional[UsbConfiguration] = None,
    iterator_maximum_raw_packets: int = 64,
) -> GenericDeviceRawOptional:
    ...


def open(
    configuration: typing.Optional[Configuration] = None,
    iterator_timeout: typing.Optional[float] = None,
    raw: bool = False,
    serial: typing.Optional[str] = None,
    usb_configuration: typing.Optional[UsbConfiguration] = None,
    iterator_maximum_raw_packets: int = 64,
) -> typing.Any:
    return device.Device.__new__(
        device.Device,
        raw,
        iterator_maximum_raw_packets,
        None if configuration is None else configuration.type(),
        None if configuration is None else configuration.serialize(),
        serial,
        None if usb_configuration is None else usb_configuration.serialize(),
        iterator_timeout,
    )
