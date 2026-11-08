# Kria KV260: IMX636 hardware filters and viewer

This fork adds experimental on-sensor Anti-Flicker (AFK) and STC cut-trail to the `prophesee_evk4` driver. It was tested on a Kria KV260 running Ubuntu 22.04 ARM64 with an IDS UE-39B0XCP over USB 3. It is an independent driver, not the official IDS HAL plugin.

## Per-device configuration

Filters use the same serializable `Configuration` and asynchronous `update_configuration()` API as other device settings. Each camera has its own settings; the driver does not read process environment variables.

```python
import neuromorphic_drivers as nd

configuration = nd.prophesee_evk4.Configuration(
    anti_flicker=nd.prophesee_evk4.AntiFlicker(
        low_frequency_hz=50,
        high_frequency_hz=520,
    ),
    stc=nd.prophesee_evk4.Stc(threshold_ms=10),
)

with nd.open(configuration=configuration, iterator_timeout=0.02) as camera:
    # Submit a per-device change; continue consuming packets to receive driver errors.
    configuration.stc = nd.prophesee_evk4.Stc(threshold_ms=20)
    camera.update_configuration(configuration)
    for status, packet in camera:
        # Process packets here; the application decides when to stop.
        ...
```

Both fields default to `None` (bypass). Assigning `None` to one of them disables that filter. AFK requires `50 <= low_frequency_hz <= high_frequency_hz <= 520`, uses band-cut mode, 50% duty cycle, start threshold 6, and stop threshold 4. STC uses cut-trail mode with an integer threshold of 1–100 ms. Validation happens before register writes. Only changed filter settings are reconfigured, under the existing register mutex; unchanged filters are preserved during bias updates.

The driver checks initialization and relevant parameter registers for active filters, and checks the bypass bit when disabling filters. These checks run in the configuration updater for live changes; the call queues a request rather than synchronously acknowledging hardware. Keep reading from the device to receive asynchronous errors. `current_configuration()` at the Rust layer tracks requested configuration and should not be treated as hardware acknowledgement.

Hardware selection of only ON or only OFF events has not been established. Both polarities remain available, and no CPU polarity filter is implemented. AFK/STC operate in the sensor hardware, not on decoded events in the viewer.

## GTK viewer

Install GTK 3 and PyGObject for the desktop environment, then run:

```sh
python3 python/examples/kv260_hardware_filters.py --stc-ms 10
```

The viewer defaults to AFK 50–520 Hz and STC 10 ms. Use `--afk-off` or `--stc-off` to start with a filter bypassed. The English UI provides bias sliders, AFK enable and frequency band, and STC enable and threshold. **Apply settings** submits the whole configuration without restarting the camera. The UI reports submission; driver failures appear in the status. Settings are not saved after the application closes.

The preview is 640 × 360 for a 1280 × 720 sensor, with ON red, OFF blue, and a white background. It discards backlog above 100 ms and is **not a lossless recorder**. Hardware filters may remove useful information as well as noise. Static scenes with STC enabled may produce few or no events.

## Build the matching Python package

The new configuration fields change its binary serialization. **Rebuild the extension and regenerate the Python bindings together. Do not mix a published 0.18.2 extension with this fork's generated configuration classes, or vice versa.** The old `KV260_FILTERS` mechanism has been removed; migrate to `Configuration` or the viewer's command-line options.

Use the upstream development workflow described in the root README. The Python Cargo manifest uses the local `../drivers` crate for both normal and build dependencies. `python/build.rs` generates the Python classes; do not edit generated files manually.

On the tested ARM64 setup, Rust/Cargo, a C toolchain, Python development headers, `pkg-config`, and `autoconf` are required. `rusb` builds a vendored libusb. One alternative for an isolated installation, from the repository root, is:

```sh
CARGO_PROFILE_RELEASE_LTO=false CARGO_PROFILE_RELEASE_CODEGEN_UNITS=8 \
    cargo build --release -p python --jobs 2

python3 -m pip install --target "$HOME/kv260-nd" 'numpy==2.2.6'
cp -R python/python/neuromorphic_drivers "$HOME/kv260-nd/"
EXT_SUFFIX="$(python3-config --extension-suffix)"
cp target/release/libneuromorphic_drivers.so \
    "$HOME/kv260-nd/neuromorphic_drivers/neuromorphic_drivers${EXT_SUFFIX}"
export PYTHONPATH="$HOME/kv260-nd"
python3 python/examples/kv260_hardware_filters.py
```

This example assumes Linux/CPython as on the KV260. The GTK example needs a desktop session, PyGObject, and the GTK 3 typelib. Close applications using the camera before starting another. Stop the viewer before replacing package files. Keep the previous extension and generated Python files together if retaining a fallback installation.

## Checks and validation

Follow the upstream Python checks from `python/`: `ruff format --check`, `ruff check`, and `ty check`. The two GTK imports have narrowly scoped optional-dependency annotations for type checking on machines without GTK. The rest of the viewer uses typed state rather than a dictionary with unrelated value types.

Run the pure Rust filter validation/serialization tests without camera access:

```sh
cargo test --release -p neuromorphic-drivers --lib filter_tests
```

With the viewer closed and a camera connected, use:

```sh
python3 python/examples/kv260_hardware_filter_check.py --stc-ms 20
python3 python/examples/evk4_update_hardware_filters.py
```

The second example exercises AFK/STC enable, disable, and thresholds 1, 20, and 100 ms in a single open camera session. It keeps receiving packets so errors from register verification propagate. Event counts alone are not measurements of noise removal, dropped data, or output latency.

On 9 November 2026 the ARM64 release build and four Rust validation/serialization tests passed. A live test on the IDS camera completed all six update stages without reopening the camera or reporting a driver error. Earlier initialization tests also covered AFK/STC individually and together and thresholds 1, 20, and 100 ms. These are short checks on one device, not proof of long-term reliability, all threshold values on hardware, or lossless throughput. Other sensor revisions and camera models remain unverified.

The IMX636 timing table and initialization derive from Prophesee OpenEB under Apache-2.0; see [third-party notices](../THIRD_PARTY_NOTICES.md).
