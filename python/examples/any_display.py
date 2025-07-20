import threading

import neuromorphic_drivers as nd
import numpy as np

import ui


def camera_thread_target(
    device: nd.GenericDeviceOptional,
    event_display: ui.EventDisplay,
):
    for status, packet in device:
        if packet is not None:
            if packet.polarity_events is not None:
                assert status.ring is not None and status.ring.current_t is not None
                event_display.push(
                    events=packet.polarity_events, current_t=status.ring.current_t
                )
            elif status.ring is not None and status.ring.current_t is not None:
                event_display.push(events=np.array([]), current_t=status.ring.current_t)


if __name__ == "__main__":
    nd.print_device_list()
    device = nd.open(iterator_timeout=1.0 / 60.0)
    print(device.serial(), device.properties())

    app = ui.App(
        f"""
        import QtQuick
        import NeuromorphicDrivers

        Window {{
            id: window
            width: {device.properties().width}
            height: {device.properties().height}

            EventDisplay {{
                width: window.width
                height: window.height
                sensor_size: "{device.properties().width}x{device.properties().height}"
                style: "exponential"
                tau: 100000
            }}
        }}
        """
    )

    event_display = app.event_display()
    camera_thread = threading.Thread(
        target=camera_thread_target,
        daemon=True,
        args=(device, event_display),
    )
    camera_thread.start()
    app.run()
