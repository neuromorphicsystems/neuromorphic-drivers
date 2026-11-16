# IMX636 hardware anti-flicker and STC

The `prophesee_evk4` driver supports on-sensor anti-flicker (AFK) and STC cut-trail filtering for IMX636 cameras. These filters run in the sensor before events reach the host. Their configuration uses the existing per-device API and does not require a particular host board or a graphical application.

## Per-device configuration

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
    # Continue consuming packets to receive asynchronous driver errors.
    configuration.stc = nd.prophesee_evk4.Stc(threshold_ms=20)
    camera.update_configuration(configuration)
    for status, packet in camera:
        # Process packets here; the application decides when to stop.
        ...
```

Both fields default to `None` (bypass). Assigning `None` disables the corresponding filter. Each camera has its own settings; the driver does not read process environment variables for these filters.

AFK requires `50 <= low_frequency_hz <= high_frequency_hz <= 520`. It uses band-cut mode, a 50% duty cycle, start threshold 6, and stop threshold 4. STC uses cut-trail mode with an integer threshold of 1-100 ms. Hardware filters can remove useful events as well as noise; static scenes with STC enabled may produce few or no events. Both ON and OFF polarities remain available.

Filter parameters are validated before the configuration update writes registers. Only changed filters are reconfigured, under the existing register mutex; bias updates preserve unchanged filter settings. The driver checks initialization and relevant parameter registers for active filters and checks the bypass bit when disabling filters.

`update_configuration()` queues a request rather than synchronously acknowledging hardware. Keep reading from the device to receive asynchronous errors. At the Rust layer, `current_configuration()` tracks the requested configuration and should not be treated as hardware acknowledgement.

## Build the matching Python package

The added fields change binary configuration serialization. **Rebuild the extension and regenerate the Python bindings together. Do not mix an unmodified published extension with the generated configuration classes from this change, or vice versa, even if the package versions match.**

Use the [development build workflow](../README.md#contribute). The Python Cargo manifest uses the local `../drivers` crate for both normal and build dependencies, so the extension and generated classes use the same configuration schema. `python/build.rs` generates the Python classes; do not edit generated files manually.

## Checks and validation

Run the four Rust filter validation and serialization tests without a camera, from the repository root:

```sh
cargo test --release -p neuromorphic-drivers --lib filter_tests
```

With a camera connected, run the minimal live update example:

```sh
python3 python/examples/evk4_update_hardware_filters.py
```

The example exercises AFK/STC enable and disable and STC thresholds of 1, 20, and 100 ms in one camera session. It keeps consuming packets so errors from register verification propagate. Event counts alone do not measure noise removal, dropped data, or output latency.

Follow the Python checks in the root README: `ruff format --check`, `ruff check`, and `ty check` from `python/`.

On 9 November 2026, an ARM64 release build and the four Rust configuration tests passed. A live test completed all six update stages without reopening the camera or reporting a driver error. Hardware validation is limited to one IMX636-based IDS UE-39B0XCP over USB 3 on Ubuntu 22.04 ARM64, hosted by a Kria KV260. That board is the tested setup, not a driver requirement. Other camera models, sensor revisions, and long-term lossless throughput remain unverified.

The IMX636 timing table and register initialization derive from Prophesee OpenEB under Apache-2.0; see [third-party notices](../THIRD_PARTY_NOTICES.md).
