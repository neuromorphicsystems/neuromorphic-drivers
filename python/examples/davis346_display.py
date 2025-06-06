import threading

import neuromorphic_drivers as nd
import numpy as np

import ui


def camera_thread_target(
    device: nd.inivation_davis346.InivationDavis346DeviceOptional,
    event_displays: tuple[ui.EventDisplay, ui.EventDisplay],
    frame_display: ui.FrameDisplay,
    context: dict[str, bool],
):
    for status, packet in device:
        if not context["running"]:
            break
        if packet is not None:
            if "dvs_events" in packet:
                assert status.ring is not None and status.ring.current_t is not None
                for event_display in event_displays:
                    event_display.push(
                        events=packet["dvs_events"], current_t=status.ring.current_t
                    )
            elif status.ring is not None and status.ring.current_t is not None:
                for event_display in event_displays:
                    event_display.push(
                        events=np.array([]), current_t=status.ring.current_t
                    )
            if "frames" in packet and len(packet["frames"]) > 0:
                frame_display.push(packet["frames"][-1].pixels)


if __name__ == "__main__":
    nd.print_device_list()
    device = nd.open(
        configuration=nd.inivation_davis346.Configuration(),
        iterator_timeout=1.0 / 60.0,
    )
    print(device.serial(), device.properties())

    app = ui.App(
        f"""
        import QtQuick
        import QtQuick.Controls
        import QtQuick.Layouts 1.2
        import NeuromorphicDrivers

        Window {{
            width: 640
            height: 480
            color: "#292929"
            property var overlayEventsOnFrames: false

            ColumnLayout {{
                anchors.fill: parent
                spacing: 0

                RowLayout {{
                    spacing: 0

                    Rectangle {{
                        id: container
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        color: "transparent"

                        FrameDisplay {{
                            width: container.width
                            height: container.height
                            sensor_size: "{device.properties().width}x{device.properties().height}"
                            mode: "L"
                            dtype: "u2"
                        }}

                        EventDisplay {{
                            visible: overlayEventsOnFrames
                            width: container.width
                            height: container.height
                            objectName: "event-display-overlay"
                            sensor_size: "{device.properties().width}x{device.properties().height}"
                            style: "exponential"
                            tau: 200000
                            on_colormap: ["#00191919", "#CCFBBC05"]
                            off_colormap: ["#00191919", "#CC4285F4"]
                            clear_background: false
                        }}
                    }}

                    EventDisplay {{
                        visible: !overlayEventsOnFrames
                        objectName: "event-display-standalone"
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        sensor_size: "{device.properties().width}x{device.properties().height}"
                        style: "exponential"
                        tau: 200000
                    }}
                }}

                RowLayout {{
                    Layout.margins: 10

                    Switch {{
                        text: "Overlay events on frames"
                        checked: overlayEventsOnFrames
                        onClicked: overlayEventsOnFrames = checked
                    }}
                }}
            }}
        }}
        """
    )

    event_displays = (
        app.event_display(object_name="event-display-overlay"),
        app.event_display(object_name="event-display-standalone"),
    )
    frame_display = app.frame_display()
    context = {"running": True}
    camera_thread = threading.Thread(
        target=camera_thread_target,
        args=(device, event_displays, frame_display, context),
    )
    camera_thread.start()
    app.run()
    context["running"] = False
    camera_thread.join()
    device.__exit__(None, None, None)
