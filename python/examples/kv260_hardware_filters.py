#!/usr/bin/env python3
"""Experimental IDS camera viewer with on-sensor AFK/STC for the KV260."""
import sys
from pathlib import Path
import os
import queue
import threading
import time
import numpy as np
import neuromorphic_drivers as nd
from neuromorphic_drivers.generated.devices.prophesee_evk4 import Configuration, Biases
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GLib, GdkPixbuf

stop = threading.Event()
lock = threading.Lock()
updates = queue.Queue(maxsize=1)
state = {'pixels': None, 'message': 'Connecting camera...', 'controls': 'Waiting for camera...', 'bounds': None,
         'filters': 'Checking hardware filters...'}
filter_spec = os.environ.get('KV260_FILTERS', '0,0,50,520,10').split(',')
if len(filter_spec) == 4:
    filter_spec.append('10')
if len(filter_spec) != 5:
    filter_spec = ['0', '0', '50', '520', '10']
bias_spec = os.environ.get('KV260_BIASES', '10,10,-20').split(',')
if len(bias_spec) != 3:
    bias_spec = ['10', '10', '-20']
initial_bias = dict(zip(('diff_on', 'diff_off', 'fo'), (int(value) for value in bias_spec)))
restart_env = None
window = Gtk.Window(title='KV260 - IDS Camera and Sensor Settings')
window.set_default_size(930, 650)
outer = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
outer.set_border_width(10)
label = Gtk.Label(label=state['message'])
outer.pack_start(label, False, False, 0)
body = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=12)
image = Gtk.Image()
body.pack_start(image, True, True, 0)
panel = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
panel.set_size_request(220, -1)
heading = Gtk.Label()
heading.set_markup('<b>Sensor settings</b>')
heading.set_xalign(0)
panel.pack_start(heading, False, False, 0)
controls = {}
for key, title, value in [('diff_on', 'ON sensitivity', initial_bias['diff_on']), ('diff_off', 'OFF sensitivity', initial_bias['diff_off']), ('fo', 'Low-pass FO', initial_bias['fo'])]:
    text = Gtk.Label(label=title)
    text.set_xalign(0)
    panel.pack_start(text, False, False, 0)
    slider = Gtk.Scale.new_with_range(Gtk.Orientation.HORIZONTAL, -20, 30, 1)
    slider.set_digits(0)
    slider.set_value(value)
    slider.set_sensitive(False)
    panel.pack_start(slider, False, False, 0)
    controls[key] = slider
hint = Gtk.Label(label='Higher ON/OFF = less sensitive.')
hint.set_line_wrap(True)
hint.set_xalign(0)
panel.pack_start(hint, False, False, 0)
apply_button = Gtk.Button(label='Apply sensor settings')
apply_button.set_sensitive(False)
panel.pack_start(apply_button, False, False, 0)
control_status = Gtk.Label(label=state['controls'])
control_status.set_line_wrap(True)
control_status.set_xalign(0)
panel.pack_start(control_status, False, False, 0)
separator = Gtk.Separator(orientation=Gtk.Orientation.HORIZONTAL)
panel.pack_start(separator, False, False, 0)
filter_heading = Gtk.Label()
filter_heading.set_markup('<b>Hardware filters (experimental)</b>')
filter_heading.set_xalign(0)
panel.pack_start(filter_heading, False, False, 0)
afk_check = Gtk.CheckButton(label='Anti-Flicker (AFK)')
afk_check.set_active(filter_spec[0] == '1')
panel.pack_start(afk_check, False, False, 0)
frequency = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)
frequency.pack_start(Gtk.Label(label='Hz:'), False, False, 0)
afk_low = Gtk.SpinButton.new_with_range(50, 520, 1)
afk_low.set_value(int(filter_spec[2]))
frequency.pack_start(afk_low, True, True, 0)
frequency.pack_start(Gtk.Label(label='to'), False, False, 0)
afk_high = Gtk.SpinButton.new_with_range(50, 520, 1)
afk_high.set_value(int(filter_spec[3]))
frequency.pack_start(afk_high, True, True, 0)
panel.pack_start(frequency, False, False, 0)
stc_check = Gtk.CheckButton(label='STC - cut trail')
stc_check.set_active(filter_spec[1] == '1')
panel.pack_start(stc_check, False, False, 0)
stc_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=5)
stc_row.pack_start(Gtk.Label(label='STC threshold (ms):'), False, False, 0)
stc_threshold = Gtk.SpinButton.new_with_range(1, 100, 1)
stc_threshold.set_value(int(filter_spec[4]))
stc_row.pack_start(stc_threshold, True, True, 0)
panel.pack_start(stc_row, False, False, 0)
polarity_label = Gtk.Label(label='Polarity: ON + OFF (hardware selection unavailable)')
polarity_label.set_line_wrap(True)
polarity_label.set_xalign(0)
panel.pack_start(polarity_label, False, False, 0)
filter_button = Gtk.Button(label='Apply filters')
panel.pack_start(filter_button, False, False, 0)
filter_status = Gtk.Label(label=state['filters'])
filter_status.set_line_wrap(True)
filter_status.set_xalign(0)
panel.pack_start(filter_status, False, False, 0)
body.pack_start(panel, False, False, 0)
outer.pack_start(body, True, True, 0)
window.add(outer)

def apply(_button):
    requested = {key: int(slider.get_value()) for key, slider in controls.items()}
    try:
        updates.put_nowait(requested)
        with lock:
            state['controls'] = 'Sending settings...'
        control_status.set_text('Sending settings...')
        apply_button.set_sensitive(False)
    except queue.Full:
        control_status.set_text('Wait for the previous update.')

apply_button.connect('clicked', apply)

def apply_filters(_button):
    global restart_env
    low, high = afk_low.get_value_as_int(), afk_high.get_value_as_int()
    if low > high:
        with lock:
            state['filters'] = 'Low frequency must not exceed high frequency.'
        return
    restart_env = os.environ.copy()
    restart_env['KV260_FILTERS'] = f'{int(afk_check.get_active())},{int(stc_check.get_active())},{low},{high},{stc_threshold.get_value_as_int()}'
    restart_env['KV260_BIASES'] = ','.join(str(int(controls[k].get_value())) for k in ('diff_on', 'diff_off', 'fo'))
    stop.set()
    Gtk.main_quit()

filter_button.connect('clicked', apply_filters)

def capture():
    try:
        config = Configuration(biases=Biases(**initial_bias))
        with nd.open(configuration=config, iterator_timeout=0.02) as camera:
            bounds = camera.biases_bounds()
            with lock:
                state['bounds'] = {key: (getattr(bounds, key).minimum, getattr(bounds, key).maximum) for key in controls}
                state['controls'] = 'Connected. Change values and press Apply.'
                state['filters'] = ('AFK: ' + ('ON' if filter_spec[0] == '1' else 'OFF') +
                                    ' | STC: ' + ('ON' if filter_spec[1] == '1' else 'OFF') +
                                    ' (registers checked). Resets when closed.')
            print('Camera opened:', camera.serial(), flush=True)
            frame = np.full((360, 640, 3), 255, dtype=np.uint8)
            last = time.monotonic()
            total = 0
            for status, packet in camera:
                if stop.is_set():
                    break
                try:
                    requested = updates.get_nowait()
                except queue.Empty:
                    requested = None
                if requested is not None:
                    try:
                        new_config = Configuration(biases=Biases(**requested))
                        camera.update_configuration(new_config)
                        config = new_config
                        message = 'Settings sent to driver: ON={diff_on}, OFF={diff_off}, FO={fo}'.format(**requested)
                        print(message, flush=True)
                    except Exception as exc:
                        message = 'Failed to send settings: ' + str(exc)
                        print(message, flush=True)
                    with lock:
                        state['controls'] = message
                delay = status.delay()
                if delay is not None and delay > 0.1:
                    camera.clear_backlog(until=0)
                    frame.fill(255)
                    continue
                if packet is not None and packet.polarity_events is not None:
                    ev = packet.polarity_events
                    total += len(ev)
                    ev = ev[(ev['x'] < 1280) & (ev['y'] < 720)]
                    frame[ev['y']//2, ev['x']//2] = np.where(ev['on'][:, None], [255, 0, 0], [0, 0, 255])
                if time.monotonic() - last >= 0.033:
                    with lock:
                        state['pixels'] = frame.tobytes()
                        state['message'] = f'IDS 1280x720 | Preview 640x360 | Events {total:,} | USB queue {(delay or 0)*1000:.1f} ms'
                    frame.fill(255)
                    last = time.monotonic()
    except Exception as exc:
        with lock:
            state['message'] = 'Camera error: ' + str(exc)
            state['controls'] = 'Controls unavailable.'
            state['filters'] = 'Filter initialization failed. Disable filters and apply again.'
        print(state['message'], flush=True)

def refresh():
    with lock:
        pixels = state['pixels']
        state['pixels'] = None
        message = state['message']
        control_message = state['controls']
        filter_message = state['filters']
        bounds = state['bounds']
        state['bounds'] = None
    label.set_text(message)
    control_status.set_text(control_message)
    filter_status.set_text(filter_message)
    if bounds:
        for key, (minimum, maximum) in bounds.items():
            controls[key].set_range(max(minimum, -30), min(maximum, 30))
            controls[key].set_sensitive(True)
        apply_button.set_sensitive(True)
    if control_message.startswith(('Settings sent', 'Failed to send')):
        apply_button.set_sensitive(True)
    if pixels is not None:
        pixbuf = GdkPixbuf.Pixbuf.new_from_bytes(GLib.Bytes.new(pixels), GdkPixbuf.Colorspace.RGB, False, 8, 640, 360, 640*3)
        image.set_from_pixbuf(pixbuf)
    return True

def close(*_args):
    stop.set()
    Gtk.main_quit()

window.connect('destroy', close)
window.show_all()
worker = threading.Thread(target=capture)
worker.start()
GLib.timeout_add(33, refresh)
try:
    Gtk.main()
finally:
    stop.set()
    worker.join(timeout=10)
    if restart_env is not None:
        if worker.is_alive():
            print('Restart cancelled: camera thread did not stop.', flush=True)
        else:
            os.execve(sys.executable, [sys.executable, str(Path(__file__).resolve())], restart_env)
