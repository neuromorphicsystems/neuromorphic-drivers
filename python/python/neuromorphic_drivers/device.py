from . import status
from .generated import enums, unions
from .neuromorphic_drivers import Device as ExtensionDevice


class Device(ExtensionDevice):
    def __new__(cls, *args, **kwargs):
        return super().__new__(cls, *args, **kwargs)

    def name(self):
        return enums.Name(super().name())

    def chip_firmware_configuration(self):
        return unions.deserialize_configuration(
            self.name(),
            super().chip_firmware_configuration(),
        )

    def speed(self):
        return enums.Speed(super().speed())

    def properties(self):
        return unions.name_to_properties(self.name())

    def close(self):
        self.__exit__(None, None, None)

    def __iter__(self):
        return self

    def __next__(self):
        system_time, ring_status, packet = super().__next__()
        if ring_status is None or ring_status[5] is None:
            return (
                status.RawStatus(
                    system_time=system_time,
                    ring=(
                        None
                        if ring_status is None
                        else status.RawRingStatus(*ring_status[0:5])
                    ),
                ),
                packet,
            )
        return (
            status.Status(
                system_time=system_time,
                ring=(
                    None
                    if ring_status is None
                    else status.RingStatus(*(ring_status[0:4] + ring_status[5:6]))
                ),
            ),
            packet,
        )

    def update_configuration(self, configuration: unions.Configuration) -> None:
        return ExtensionDevice.update_configuration(
            self,
            configuration.type(),
            configuration.serialize(),
        )
