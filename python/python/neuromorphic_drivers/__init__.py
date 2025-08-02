import dataclasses
import sys
import typing

from . import serde as serde
from .generated.devices_types import *
from .mask import *
from .orientation import *
from .packet import *
from .neuromorphic_drivers import list_devices as extension_list_devices
from .udev import *


@dataclasses.dataclass
class ListedDevice:
    name: Name
    speed: Speed
    serial: typing.Optional[str]
    error: typing.Optional[str]


def list_devices() -> list[ListedDevice]:
    return [
        ListedDevice(
            name=Name(name),
            speed=Speed(speed),
            serial=serial,
            error=error,
        )
        for (name, speed, serial, error) in extension_list_devices()
    ]


def print_device_list(color: bool = True):
    table = [["Name", "Speed", "Serial", "Error"]]
    for device in list_devices():
        table.append(
            [
                device.name.value,
                device.speed.value,
                "-" if device.serial is None else device.serial,
                "-" if device.error is None else device.error,
            ]
        )
    widths = [
        max(len(row[column]) for row in table) for column in range(0, len(table[0]))
    ]
    title = lambda string: f"\033[36;1m{string}\033[0m" if color else string
    sys.stdout.write(
        "".join(
            (
                "┌",
                "┬".join("─" * (width + 2) for width in widths),
                "┐\n",
                "".join(
                    (
                        "│",
                        "│".join(
                            title(f" {column:<{width}} ")
                            for width, column in zip(widths, table[0])
                        ),
                        "│",
                    )
                ),
                *(
                    (
                        "\n├",
                        "┼".join("─" * (width + 2) for width in widths),
                        "┤\n",
                        "\n".join(
                            "".join(
                                (
                                    "│",
                                    "│".join(
                                        f" {column:<{width}} "
                                        for width, column in zip(widths, row)
                                    ),
                                    "│",
                                )
                            )
                            for row in table[1:]
                        ),
                    )
                    if len(table) > 1
                    else ()
                ),
                "\n└",
                "┴".join("─" * (width + 2) for width in widths),
                "┘\n",
            )
        )
    )
