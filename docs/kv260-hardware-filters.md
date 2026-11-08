# Kria KV260: IMX636 hardware filters and viewer

This fork adds **experimental on-sensor Anti-Flicker (AFK) and STC cut-trail** configuration for the `prophesee_evk4` protocol family. It was built and tested on a Kria KV260 with Ubuntu 22.04 ARM64 and an IDS UE-39B0XCP camera. It is an independent community driver, not the official IDS HAL plugin.

The changes are opt-in and configured at camera open using an environment variable. Without that variable, the original initialization remains in effect. The application must reopen the camera to change AFK or STC. This approach is process-wide and does not provide per-camera settings for multiple simultaneously opened cameras.

## Configuration

```text
KV260_FILTERS=AFK,STC,LOW_HZ,HIGH_HZ,STC_MS
```

`AFK` and `STC` are `0` or `1`; the AFK band is 50–520 Hz with low ≤ high, and the STC threshold is an integer from 1 to 100 ms. For example:

```sh
KV260_FILTERS=1,1,50,520,10 python3 python/examples/kv260_hardware_filters.py
```

An older four-value form remains valid and uses 10 ms for STC. AFK uses band-cut mode, 50% duty cycle, start threshold 6 and stop threshold 4. STC uses the cut-trail mode. The driver validates inputs and checks initialization and relevant sensor registers when enabling the filters; initialization fails rather than reporting success if these checks fail. Disabled filters use the original driver's bypass initialization.

The implementation programs the **sensor's hardware filters**; it does not apply software noise filtering to the event stream. Hardware selection of only ON or only OFF events has not been established, and this fork does not implement software polarity filtering. Both polarities remain available.

## Viewer

`python/examples/kv260_hardware_filters.py` is a GTK 3 preview for the KV260. It displays a 640 × 360 preview of the 1280 × 720 sensor, with ON events red, OFF events blue, and a white background. It provides sensor-bias sliders, AFK enable and frequency band, and STC enable and threshold. **Apply filters** briefly reopens the camera; the bias values currently shown in the panel are also applied. Settings are not saved after closing the application.

The viewer discards backlog if reported USB queue delay exceeds 100 ms. It is intended for preview and tuning, **not lossless capture**. A static scene with STC enabled can produce a mostly white preview; try moving a hand and compare with STC disabled. Hardware filters can also discard useful events.

## Build and run on the tested ARM64 setup

The following uses Linux and CPython 3.10 as on the tested KV260. Install Rust/Cargo, a C toolchain, Python development headers, GTK 3/PyGObject, `pkg-config`, and `autoconf` first. `rusb` builds a vendored libusb. Internet access is needed for uncached Cargo and pip dependencies.

The Python extension depends on the **local** `drivers` crate in both its runtime and build dependencies. This is important: using the published crate would omit this fork's modifications.

```sh
# From the root of this clone. Limit jobs if memory is constrained.
CARGO_PROFILE_RELEASE_LTO=false CARGO_PROFILE_RELEASE_CODEGEN_UNITS=8 \
    cargo build --release -p python --jobs 2

# Install the matching upstream Python package into an isolated directory,
# then replace only its extension with the one built from this fork.
python3 -m pip install --target "$HOME/kv260-nd" 'neuromorphic-drivers==0.18.2' 'numpy==2.2.6'
EXT_SUFFIX="$(python3-config --extension-suffix)"
cp target/release/libneuromorphic_drivers.so \
    "$HOME/kv260-nd/neuromorphic_drivers/neuromorphic_drivers${EXT_SUFFIX}"
export PYTHONPATH="$HOME/kv260-nd"
KV260_FILTERS=1,1,50,520,10 python3 python/examples/kv260_hardware_filters.py
```

Close any application using the camera before running another. Stop the viewer before replacing the extension file. On Linux, USB access may require an appropriate udev rule for the camera. The GTK example requires a desktop session; see the upstream documentation for driver setup and usage.

For a four-second connection check, close the viewer and run:

```sh
KV260_FILTERS=1,1,50,520,10 python3 python/examples/kv260_hardware_filter_check.py
```

The check prints the sensor register-verification result and received event count. A static scene can produce no events; move an object in front of the lens before drawing conclusions about the stream.

## Verification and limits

On 9 November 2026, an IDS UE-39B0XCP over USB 3 was tested with AFK/STC both off, each individually enabled, and both enabled. At STC 1, 20, and 100 ms, the sensor register checks passed and event packets were received while a hand moved in front of the camera. The viewer was also used to change STC from 10 to 20 ms and back, and to toggle AFK/STC individually.

These are short tests on one camera. They do **not** establish long-term reliability, lossless throughput, measured end-to-end latency, or a guaranteed percentage of noise reduction. Other sensor revisions, cameras, and operating systems are unverified. The STC timing table and initialization are derived from Apache-2.0-licensed Prophesee OpenEB; see [third-party notices](../THIRD_PARTY_NOTICES.md).
