"""
Installs the WinUSB driver for supported USB cameras on Windows.

Windows only lets programs (and therefore libusb) access a USB device that has a driver.
neuromorphic_drivers uses Microsoft's generic WinUSB driver. This module does what Zadig does
(https://zadig.akeo.ie), without user interaction: it binds WinUSB to the camera's USB ID.

The driver installation itself is done by winusb_installer.exe from the WinUSB-installer-generator
project (https://github.com/Sensing-Dev/WinUSB-installer-generator, LGPL-3.0, based on libwdi).
It is downloaded on demand and verified with a SHA-256 checksum.

The driver is installed once per computer and USB ID. Further cameras with the same USB ID
use it automatically.
"""

from __future__ import annotations

__all__ = [
    "SUPPORTED_DEVICES",
    "WindowsUsbDevice",
    "install_windows_driver",
    "install_windows_driver_program",
    "list_windows_devices",
]

import dataclasses
import pathlib
import re
import typing

INSTALLER_URL = "https://github.com/Sensing-Dev/WinUSB-installer-generator/releases/download/v24.02.02/WinUSB-installer-generator-v24.02.02.zip"
INSTALLER_SHA256 = "7a6feda72529482e367a82390dfdddefea3059670313a5dd21f689c7b3644c95"
INSTALLER_EXECUTABLE = "WinUSB-installer-generator-v24.02.02/winusb_installer.exe"


@dataclasses.dataclass(frozen=True)
class SupportedDevice:
    name: str
    vendor_id: int
    product_id_minimum: int
    product_id_maximum: int

    def matches(self, vendor_id: int, product_id: int) -> bool:
        return (
            vendor_id == self.vendor_id
            and self.product_id_minimum <= product_id <= self.product_id_maximum
        )


# USB IDs of the USB devices (VENDOR_AND_PRODUCT_IDS in drivers/src/devices/*.rs)
SUPPORTED_DEVICES: tuple[SupportedDevice, ...] = (
    SupportedDevice("SENSING SE1-S4-USB", 0x04B4, 0x00C4, 0x00C4),
    SupportedDevice("Prophesee EVK4 / EVK3 HD / EVK1 Gen4", 0x04B4, 0x00F4, 0x00F5),
    SupportedDevice("Prophesee EVK4 (CenturyArks)", 0x31F7, 0x0003, 0x0004),
    SupportedDevice("Prophesee EVK4 (IDS)", 0x1409, 0x8E00, 0x8E00),
    SupportedDevice("CenturyArks VGA", 0x31F7, 0x0002, 0x0002),
    SupportedDevice("CenturyArks VGA", 0x03FD, 0x5832, 0x5832),
    SupportedDevice("iniVation DVXplorer", 0x152A, 0x8419, 0x8419),
    SupportedDevice("iniVation DAVIS 346", 0x152A, 0x841A, 0x841A),
)

_INSTANCE_ID_PATTERN = re.compile(
    r"^USB\\VID_([0-9A-F]{4})&PID_([0-9A-F]{4})\\", re.IGNORECASE
)


@dataclasses.dataclass
class WindowsUsbDevice:
    instance_id: str
    name: str
    status: str
    service: typing.Optional[str]
    vendor_id: int
    product_id: int
    supported_device: SupportedDevice

    @property
    def usb_id(self) -> str:
        return f"{self.vendor_id:04X}:{self.product_id:04X}"

    @property
    def has_winusb(self) -> bool:
        return self.service is not None and self.service.lower() == "winusb"

    @property
    def has_driver(self) -> bool:
        return self.service is not None and len(self.service) > 0


def _powershell(script: str) -> str:
    import subprocess

    return subprocess.run(
        [
            "powershell.exe",
            "-NoProfile",
            "-NonInteractive",
            "-ExecutionPolicy",
            "Bypass",
            "-Command",
            script,
        ],
        check=True,
        capture_output=True,
        text=True,
    ).stdout


def list_windows_devices() -> list[WindowsUsbDevice]:
    """Lists the connected USB devices that neuromorphic_drivers supports, with their current driver."""
    import json

    vendor_ids = "|".join(
        sorted({f"{device.vendor_id:04X}" for device in SUPPORTED_DEVICES})
    )
    output = _powershell(
        "$ErrorActionPreference = 'SilentlyContinue'; "
        "@(Get-PnpDevice -PresentOnly | "
        f"Where-Object {{ $_.InstanceId -match '^USB\\\\VID_({vendor_ids})&PID_[0-9A-F]{{4}}\\\\' }} | "
        "ForEach-Object { [pscustomobject]@{ "
        "InstanceId = $_.InstanceId; "
        "Name = [string]$_.FriendlyName; "
        "Status = [string]$_.Status; "
        "Service = [string](Get-PnpDeviceProperty -InstanceId $_.InstanceId -KeyName 'DEVPKEY_Device_Service').Data "
        "} }) | ConvertTo-Json -Compress"
    ).strip()
    if len(output) == 0:
        return []
    entries = json.loads(output)
    if isinstance(entries, dict):
        entries = [entries]
    result = []
    for entry in entries:
        match = _INSTANCE_ID_PATTERN.match(entry["InstanceId"])
        if match is None:
            continue
        vendor_id = int(match.group(1), 16)
        product_id = int(match.group(2), 16)
        for supported_device in SUPPORTED_DEVICES:
            if supported_device.matches(vendor_id, product_id):
                result.append(
                    WindowsUsbDevice(
                        instance_id=entry["InstanceId"],
                        name=entry["Name"] or "",
                        status=entry["Status"] or "",
                        service=entry["Service"] or None,
                        vendor_id=vendor_id,
                        product_id=product_id,
                        supported_device=supported_device,
                    )
                )
                break
    return result


def _is_admin() -> bool:
    import ctypes

    try:
        return bool(ctypes.windll.shell32.IsUserAnAdmin())  # type: ignore[attr-defined]
    except Exception:
        return False


def _download_installer(directory: pathlib.Path) -> pathlib.Path:
    import hashlib
    import io
    import urllib.request
    import zipfile

    with urllib.request.urlopen(INSTALLER_URL, timeout=60) as response:
        data = response.read()
    digest = hashlib.sha256(data).hexdigest()
    if digest != INSTALLER_SHA256:
        raise RuntimeError(
            f"unexpected checksum for {INSTALLER_URL} (expected {INSTALLER_SHA256}, got {digest})"
        )
    with zipfile.ZipFile(io.BytesIO(data)) as archive:
        archive.extractall(directory)
    executable = directory / INSTALLER_EXECUTABLE
    if not executable.is_file():
        raise RuntimeError(f"{INSTALLER_EXECUTABLE} not found in the installer archive")
    return executable


def _run_installer(executable: pathlib.Path, vendor_id: int, product_id: int):
    import subprocess

    arguments = [f"{vendor_id:04x}", f"{product_id:04x}"]
    working_directory = executable.parent
    if _is_admin():
        subprocess.run(
            [str(executable), *arguments], cwd=working_directory, check=False
        )
    else:
        # Start-Process -Verb RunAs shows the UAC prompt and waits for the elevated installer
        _powershell(
            f"Start-Process -FilePath '{executable}' "
            f"-ArgumentList '{' '.join(arguments)}' "
            f"-WorkingDirectory '{working_directory}' "
            "-Verb RunAs -Wait -WindowStyle Hidden"
        )


def _wait_for_winusb(vendor_id: int, product_id: int, timeout: float) -> bool:
    import time

    deadline = time.monotonic() + timeout
    while True:
        devices = [
            device
            for device in list_windows_devices()
            if device.vendor_id == vendor_id and device.product_id == product_id
        ]
        if len(devices) > 0 and all(device.has_winusb for device in devices):
            return True
        if time.monotonic() >= deadline:
            return False
        time.sleep(1.0)


def install_windows_driver(
    usb_ids: typing.Optional[list[tuple[int, int]]] = None,
    force: bool = False,
) -> bool:
    """
    Installs WinUSB for connected supported devices.

    By default, only devices without any driver are handled. Devices that use another
    driver (for instance the camera vendor's driver) are only changed with force=True,
    since this may break the vendor's software.

    Returns True if all selected devices use WinUSB afterwards.
    """
    import sys
    import tempfile

    if sys.platform != "win32":
        raise RuntimeError("the WinUSB driver installation is only required on Windows")

    devices = list_windows_devices()
    if usb_ids is not None:
        devices = [
            device
            for device in devices
            if (device.vendor_id, device.product_id) in usb_ids
        ]
    if len(devices) == 0:
        print("No supported device found. Connect the camera and try again.")
        return False

    targets: dict[tuple[int, int], WindowsUsbDevice] = {}
    for device in devices:
        if device.has_winusb and not force:
            print(f"{device.supported_device.name} ({device.usb_id}): WinUSB already installed")
        elif device.has_driver and not device.has_winusb and not force:
            print(
                f"{device.supported_device.name} ({device.usb_id}): uses the driver '{device.service}', "
                "skipped (use --force to replace it with WinUSB, this may break the vendor's software)"
            )
        else:
            targets[(device.vendor_id, device.product_id)] = device

    if len(targets) == 0:
        return all(device.has_winusb for device in devices)

    success = True
    with tempfile.TemporaryDirectory(prefix="neuromorphic-drivers-winusb-") as directory:
        print(f"Downloading the WinUSB installer from {INSTALLER_URL}")
        executable = _download_installer(pathlib.Path(directory))
        for (vendor_id, product_id), device in targets.items():
            print(
                f"{device.supported_device.name} ({device.usb_id}): installing WinUSB, "
                "confirm the administrator prompt (this can take a few minutes)"
            )
            try:
                _run_installer(executable, vendor_id, product_id)
            except Exception as error:
                success = False
                print(f"{device.supported_device.name} ({device.usb_id}): installation cancelled or failed ({error})")
                continue
            if _wait_for_winusb(vendor_id, product_id, timeout=30.0):
                print(f"{device.supported_device.name} ({device.usb_id}): WinUSB installed")
            else:
                success = False
                print(
                    f"{device.supported_device.name} ({device.usb_id}): WinUSB is not active, "
                    "reconnect the camera and run this command again"
                )
    return success


def _parse_usb_id(value: str) -> tuple[int, int]:
    import argparse

    match = re.fullmatch(r"([0-9A-Fa-f]{4}):([0-9A-Fa-f]{4})", value)
    if match is None:
        raise argparse.ArgumentTypeError(f"expected VID:PID (for example 04b4:00c4), got '{value}'")
    return (int(match.group(1), 16), int(match.group(2), 16))


def install_windows_driver_program():
    import argparse
    import sys

    parser = argparse.ArgumentParser(
        prog="neuromorphic-drivers-install-windows-driver",
        description="Install the WinUSB driver for Neuromorphic devices on Windows (same as Zadig)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--list",
        "-l",
        action="store_true",
        help="Only list connected supported devices and their driver",
    )
    parser.add_argument(
        "--device",
        "-d",
        type=_parse_usb_id,
        action="append",
        help="Only handle this USB ID (VID:PID, can be repeated)",
    )
    parser.add_argument(
        "--force",
        "-f",
        action="store_true",
        help="Install WinUSB even if the device already uses a driver",
    )
    parser.add_argument(
        "--skip-system-warn",
        "-w",
        action="store_true",
        help="Exit silently on Linux and macOS",
    )
    args = parser.parse_args()

    if sys.platform != "win32":
        if args.skip_system_warn:
            sys.exit(0)
        sys.stdout.write(
            "a driver installation is only required on Windows (use neuromorphic-drivers-install-udev-rules on Linux)\n"
        )
        sys.exit(0)

    if args.list:
        devices = list_windows_devices()
        if len(devices) == 0:
            print("No supported device found")
        for device in devices:
            print(
                f"{device.supported_device.name} ({device.usb_id}) | {device.name} | "
                f"status: {device.status} | driver: {device.service or '(none)'}"
            )
        sys.exit(0)

    sys.exit(0 if install_windows_driver(usb_ids=args.device, force=args.force) else 1)


if __name__ == "__main__":
    install_windows_driver_program()
