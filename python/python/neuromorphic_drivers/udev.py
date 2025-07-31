import pathlib


def install_udev_rules(
    output: pathlib.Path,
    skip_system_check: bool,
    skip_system_warn: bool,
    skip_root_check: bool,
):
    import builtins
    import os
    import subprocess
    import sys

    if not skip_system_check:
        if sys.platform == "win32":
            if skip_system_warn:
                sys.exit(0)
            sys.stderr.write("udev rules are not compatible with Windows\n")
            sys.exit(1)
        if sys.platform == "darwin":
            if skip_system_warn:
                sys.exit(0)
            sys.stderr.write("udev rules are not compatible with macOS\n")
            sys.exit(1)
    if not skip_root_check:
        if os.geteuid() != 0:
            cwd = pathlib.Path.cwd()
            executable = pathlib.Path(sys.executable).resolve()
            try:
                executable = executable.relative_to(cwd)
            except ValueError:
                pass
            file = pathlib.Path(__file__).resolve()
            try:
                file = file.relative_to(cwd)
            except ValueError:
                pass
            sys.stdout.write(
                f"\033[1mrun the following command to enable non-root access to devices\033[0m\n\nsudo {executable} {file}\n\n"
            )
            sys.exit(0)
    with builtins.open(output, "w") as output_file:
        output_file.write(
            "".join(
                (
                    'SUBSYSTEM=="usb", ATTRS{idVendor}=="152a", ATTRS{idProduct}=="84[0-1]?", MODE="0666"\n',
                    'SUBSYSTEM=="usb", ATTRS{idVendor}=="04b4", ATTRS{idProduct}=="00f[4-5]", MODE="0666"\n',
                    'SUBSYSTEM=="usb", ATTRS{idVendor}=="31f7", ATTRS{idProduct}=="0003", MODE="0666"\n',
                    'SUBSYSTEM=="usb", ATTRS{idVendor}=="1409", ATTRS{idProduct}=="8E00", MODE="0666"\n',
                )
            )
        )
    subprocess.check_call(["udevadm", "control", "--reload-rules"])
    subprocess.check_call(["udevadm", "trigger"])


def install_udev_rules_program():
    import argparse

    parser = argparse.ArgumentParser(
        prog="neuromorphic-drivers-install-udev-rules",
        description="Install udev rules for Neuromorphic devices",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--output",
        "-o",
        default="/etc/udev/rules.d/65-neuromorphic-drivers.rules",
        help="Rules file path",
    )
    parser.add_argument(
        "--skip-system-check",
        "-s",
        action="store_true",
        help="Do not check the operating system type",
    )
    parser.add_argument(
        "--skip-system-warn",
        "-w",
        action="store_true",
        help="Do not print a warning on system check failure",
    )
    parser.add_argument(
        "--skip-root-check",
        "-r",
        action="store_true",
        help="Do not check for root privileges",
    )
    args = parser.parse_args()
    install_udev_rules(
        output=pathlib.Path(args.output),
        skip_system_check=args.skip_system_check,
        skip_system_warn=args.skip_system_warn,
        skip_root_check=args.skip_root_check,
    )


if __name__ == "__main__":
    install_udev_rules_program()
