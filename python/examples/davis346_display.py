import threading
import typing

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
            if packet.polarity_events is not None:
                assert status.ring is not None and status.ring.current_t is not None
                for event_display in event_displays:
                    event_display.push(
                        events=packet.polarity_events, current_t=status.ring.current_t
                    )
            elif status.ring is not None and status.ring.current_t is not None:
                for event_display in event_displays:
                    event_display.push(
                        events=np.array([]), current_t=status.ring.current_t
                    )
            if len(packet.frames) > 0:
                frame_display.push(packet.frames[-1].pixels)


if __name__ == "__main__":
    nd.print_device_list()
    configuration = nd.inivation_davis346.Configuration()
    device = nd.open(
        configuration=configuration,
        iterator_timeout=1.0 / 60.0,
    )
    print(device.serial(), device.properties())

    transparent_on_colormap: list[str] = []
    for index, color in enumerate(ui.DEFAULT_ON_COLORMAP):
        transparent_on_colormap.append(
            '"#{:02X}{:02X}{:02X}{:02X}"'.format(
                int(round(index / (len(ui.DEFAULT_ON_COLORMAP) - 1) * 255)),
                color.red(),
                color.green(),
                color.blue(),
            )
        )
    transparent_off_colormap: list[str] = []
    for index, color in enumerate(ui.DEFAULT_OFF_COLORMAP):
        transparent_off_colormap.append(
            '"#{:02X}{:02X}{:02X}{:02X}"'.format(
                int(round(index / (len(ui.DEFAULT_OFF_COLORMAP) - 1) * 255)),
                color.red(),
                color.green(),
                color.blue(),
            )
        )

    def to_python(key: str, value: typing.Any):
        if key == "exposure":
            configuration.exposure_us = int(value)
            device.update_configuration(configuration)
        elif key == "diff_on":
            configuration.biases.onbn = int(value)
            device.update_configuration(configuration)
        elif key == "diff_off":
            configuration.biases.offbn = int(value)
            device.update_configuration(configuration)
        else:
            print(f"Unknown to_python key: {key}")

    app = ui.App(
        qml=f"""
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
                            id: eventDisplayOverlay
                            visible: overlayEventsOnFrames
                            width: container.width
                            height: container.height
                            objectName: "event-display-overlay"
                            sensor_size: "{device.properties().width}x{device.properties().height}"
                            style: "exponential"
                            tau: 200000
                            on_colormap: [{','.join(transparent_on_colormap)}]
                            off_colormap: [{','.join(transparent_off_colormap)}]
                            clear_background: false
                        }}
                    }}

                    EventDisplay {{
                        id: eventDisplayStandalone
                        visible: !overlayEventsOnFrames
                        objectName: "event-display-standalone"
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        sensor_size: "{device.properties().width}x{device.properties().height}"
                        style: "exponential"
                        tau: 200000
                    }}
                }}

                ColumnLayout {{
                    Layout.margins: 10
                    Label {{
                        text: "Display properties"
                        color: "#AAAAAA"
                    }}
                    RowLayout {{
                        spacing: 10

                        Switch {{
                            text: "Overlay events on frames"
                            checked: overlayEventsOnFrames
                            onClicked: overlayEventsOnFrames = checked
                        }}

                        RowLayout {{
                            spacing: 5
                            Label {{
                                text: "Style"
                            }}
                            ComboBox {{
                                model: ["Exponential", "Linear", "Window"]
                                currentIndex: 0
                                onCurrentIndexChanged: {{
                                    eventDisplayOverlay.style = model[currentIndex].toLowerCase()
                                    eventDisplayStandalone.style = model[currentIndex].toLowerCase()
                                }}
                            }}
                        }}

                        RowLayout {{
                            spacing: 5
                            Label {{
                                text: "𝜏 (ms)"
                            }}
                            SpinBox {{
                                from: 1
                                to: 100000
                                stepSize: 1
                                editable: true
                                value: {int(round(ui.DEFAULT_TAU / 1000))}
                                onValueChanged: {{
                                    eventDisplayOverlay.tau = value * 1000
                                    eventDisplayStandalone.tau = value * 1000
                                }}
                            }}
                        }}
                    }}

                    Label {{
                        text: "Camera properties"
                        Layout.topMargin: 10
                        color: "#AAAAAA"
                    }}

                    RowLayout {{
                        spacing: 5
                        Label {{
                            text: "Exposure (µs)"
                        }}
                        SpinBox {{
                            from: 1
                            to: 8000000
                            stepSize: 1
                            editable: true
                            value: {configuration.exposure_us}
                            onValueChanged: to_python.exposure = value
                        }}
                    }}

                    RowLayout {{
                        spacing: 5
                        Label {{
                            text: "Diff ON"
                        }}
                        SpinBox {{
                            from: 0
                            to: 2040
                            stepSize: 1
                            editable: true
                            value: {configuration.biases.onbn}
                            onValueChanged: to_python.diff_on = value
                        }}
                    }}

                    RowLayout {{
                        spacing: 5
                        Label {{
                            text: "Diff OFF"
                        }}
                        SpinBox {{
                            from: 0
                            to: 2040
                            stepSize: 1
                            editable: true
                            value: {configuration.biases.offbn}
                            onValueChanged: to_python.diff_off = value
                        }}
                    }}
                }}
            }}
        }}
        """,
        to_python=to_python,
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
    device.close()
