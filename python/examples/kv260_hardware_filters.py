"""GTK preview with per-device IMX636 sensor settings."""

from __future__ import annotations

import argparse
import dataclasses
import queue
import threading
import time

import gi  # ty: ignore[import-not-found]  # Optional GTK runtime dependency.
import neuromorphic_drivers as nd
import numpy as np

gi.require_version("Gtk", "3.0")
gi.require_version("GdkPixbuf", "2.0")
from gi.repository import GdkPixbuf, GLib, Gtk  # ty: ignore[import-not-found]


@dataclasses.dataclass
class PreviewState:
    pixels: bytes | None = None
    message: str = "Connecting camera..."
    controls: str = "Waiting for camera..."
    bounds: dict[str, tuple[int, int]] | None = None
    connected: bool = False


class Viewer:
    def __init__(self, configuration: nd.prophesee_evk4.Configuration):
        self.configuration = configuration
        self.state = PreviewState()
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.updates: queue.Queue[nd.prophesee_evk4.Configuration] = queue.Queue(
            maxsize=1
        )
        self.window = Gtk.Window(title="KV260 - IDS Camera and Sensor Settings")
        self.window.set_default_size(930, 700)
        outer = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        outer.set_border_width(10)
        self.label = Gtk.Label(label=self.state.message)
        outer.pack_start(self.label, False, False, 0)
        body = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=12)
        self.image = Gtk.Image()
        body.pack_start(self.image, True, True, 0)
        panel = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        panel.set_size_request(220, -1)
        self.biases = {}
        for key, title in [
            ("diff_on", "ON sensitivity"),
            ("diff_off", "OFF sensitivity"),
            ("fo", "Low-pass FO"),
        ]:
            panel.pack_start(Gtk.Label(label=title, xalign=0), False, False, 0)
            slider = Gtk.Scale.new_with_range(Gtk.Orientation.HORIZONTAL, -30, 30, 1)
            slider.set_digits(0)
            slider.set_value(getattr(configuration.biases, key))
            self.biases[key] = slider
            panel.pack_start(slider, False, False, 0)
        panel.pack_start(
            Gtk.Label(label="Higher ON/OFF = less sensitive.", wrap=True, xalign=0),
            False,
            False,
            0,
        )
        panel.pack_start(Gtk.Separator(), False, False, 0)
        self.afk = Gtk.CheckButton(label="Anti-Flicker (AFK)")
        self.afk.set_active(configuration.anti_flicker is not None)
        panel.pack_start(self.afk, False, False, 0)
        band = configuration.anti_flicker
        self.low = self.add_number(
            panel, "Low frequency (Hz)", 50, 520, band.low_frequency_hz if band else 50
        )
        self.high = self.add_number(
            panel,
            "High frequency (Hz)",
            50,
            520,
            band.high_frequency_hz if band else 520,
        )
        self.stc = Gtk.CheckButton(label="STC - cut trail")
        self.stc.set_active(configuration.stc is not None)
        panel.pack_start(self.stc, False, False, 0)
        self.threshold = self.add_number(
            panel,
            "STC threshold (ms)",
            1,
            100,
            configuration.stc.threshold_ms if configuration.stc else 10,
        )
        panel.pack_start(
            Gtk.Label(
                label="Polarity: ON + OFF\nHardware selection unavailable.",
                wrap=True,
                xalign=0,
            ),
            False,
            False,
            0,
        )
        self.apply = Gtk.Button(label="Apply settings")
        self.apply.set_sensitive(False)
        self.apply.connect("clicked", self.submit)
        panel.pack_start(self.apply, False, False, 0)
        self.control_label = Gtk.Label(label=self.state.controls, wrap=True, xalign=0)
        panel.pack_start(self.control_label, False, False, 0)
        body.pack_start(panel, False, False, 0)
        outer.pack_start(body, True, True, 0)
        self.window.add(outer)
        self.window.connect("destroy", self.close)

    @staticmethod
    def add_number(panel, label, minimum, maximum, value):
        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)
        row.pack_start(Gtk.Label(label=label), False, False, 0)
        spin = Gtk.SpinButton.new_with_range(minimum, maximum, 1)
        spin.set_value(value)
        row.pack_start(spin, True, True, 0)
        panel.pack_start(row, False, False, 0)
        return spin

    def submit(self, _button):
        low, high = self.low.get_value_as_int(), self.high.get_value_as_int()
        if low > high:
            with self.lock:
                self.state.controls = "Low frequency must not exceed high frequency."
            return
        configuration = nd.prophesee_evk4.Configuration(
            biases=nd.prophesee_evk4.Biases(
                diff_on=int(self.biases["diff_on"].get_value()),
                diff_off=int(self.biases["diff_off"].get_value()),
                fo=int(self.biases["fo"].get_value()),
            ),
            anti_flicker=nd.prophesee_evk4.AntiFlicker(low, high)
            if self.afk.get_active()
            else None,
            stc=nd.prophesee_evk4.Stc(self.threshold.get_value_as_int())
            if self.stc.get_active()
            else None,
        )
        try:
            self.updates.put_nowait(configuration)
        except queue.Full:
            with self.lock:
                self.state.controls = "Wait for the previous update."

    def capture(self):
        try:
            with nd.open(
                configuration=self.configuration, iterator_timeout=0.02
            ) as camera:
                bounds = camera.biases_bounds()
                with self.lock:
                    self.state.bounds = {
                        key: (
                            getattr(bounds, key).minimum,
                            getattr(bounds, key).maximum,
                        )
                        for key in self.biases
                    }
                    self.state.connected = True
                    self.state.controls = (
                        "Connected. Apply settings without restarting."
                    )
                frame = np.full((360, 640, 3), 255, dtype=np.uint8)
                last = time.monotonic()
                total = 0
                for status, packet in camera:
                    if self.stop.is_set():
                        break
                    try:
                        requested = self.updates.get_nowait()
                    except queue.Empty:
                        requested = None
                    if requested is not None:
                        camera.update_configuration(requested)
                        self.configuration = requested
                        with self.lock:
                            self.state.controls = "Settings submitted to this camera."
                    delay = status.delay()
                    if delay is not None and delay > 0.1:
                        camera.clear_backlog(until=0)
                        frame.fill(255)
                        continue
                    if packet is not None and packet.polarity_events is not None:
                        events = packet.polarity_events
                        total += len(events)
                        events = events[(events["x"] < 1280) & (events["y"] < 720)]
                        frame[events["y"] // 2, events["x"] // 2] = np.where(
                            events["on"][:, None], [255, 0, 0], [0, 0, 255]
                        )
                    if time.monotonic() - last >= 0.033:
                        with self.lock:
                            self.state.pixels = frame.tobytes()
                            self.state.message = (
                                f"IDS 1280x720 | Preview 640x360 | Events {total:,} | "
                                f"USB queue {(delay or 0) * 1000:.1f} ms"
                            )
                        frame.fill(255)
                        last = time.monotonic()
        except (RuntimeError, ValueError, OSError) as error:
            with self.lock:
                self.state.message = f"Camera error: {error}"
                self.state.controls = "Close and reopen the viewer to reconnect."
                self.state.connected = False

    def refresh(self):
        with self.lock:
            state = dataclasses.replace(self.state)
            self.state.pixels = None
            self.state.bounds = None
        self.label.set_text(state.message)
        self.control_label.set_text(state.controls)
        self.apply.set_sensitive(state.connected)
        if state.bounds:
            for key, (minimum, maximum) in state.bounds.items():
                self.biases[key].set_range(max(minimum, -30), min(maximum, 30))
        if state.pixels is not None:
            pixbuf = GdkPixbuf.Pixbuf.new_from_bytes(
                GLib.Bytes.new(state.pixels),
                GdkPixbuf.Colorspace.RGB,
                False,
                8,
                640,
                360,
                640 * 3,
            )
            self.image.set_from_pixbuf(pixbuf)
        return True

    def close(self, *_args):
        self.stop.set()
        Gtk.main_quit()

    def run(self):
        self.window.show_all()
        worker = threading.Thread(target=self.capture)
        worker.start()
        GLib.timeout_add(33, self.refresh)
        try:
            Gtk.main()
        finally:
            self.stop.set()
            worker.join(timeout=10)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--afk-off", action="store_true")
    parser.add_argument("--stc-off", action="store_true")
    parser.add_argument("--stc-ms", type=int, choices=range(1, 101), default=10)
    args = parser.parse_args()
    Viewer(
        nd.prophesee_evk4.Configuration(
            biases=nd.prophesee_evk4.Biases(diff_on=10, diff_off=10, fo=-20),
            anti_flicker=None
            if args.afk_off
            else nd.prophesee_evk4.AntiFlicker(50, 520),
            stc=None if args.stc_off else nd.prophesee_evk4.Stc(args.stc_ms),
        )
    ).run()
