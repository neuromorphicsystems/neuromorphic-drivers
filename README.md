neuromorphic_drivers is a library to interact with USB Neuromorphic devices. The drivers were written from scratch with portability and performance in mind.

- [Supported devices and features](#supported-devices-and-features)
- [Python](#python)
  - [Get started](#get-started)
  - [Device configuration](#device-configuration)
  - [Rate limiter](#rate-limiter)
  - [Raw mode](#raw-mode)
  - [Other open options](#other-open-options)
  - [More examples](#more-examples)
  - [Contribute](#contribute)
- [Rust](#rust)
  - [Documentation](#documentation)
  - [UDEV rules](#udev-rules)
  - [Contribute](#contribute-1)
- [Performance](#performance)
  - [Event rate](#event-rate)
  - [Direct Memory Access](#direct-memory-access)

# Supported devices and features

| Name              | Type   | Resolution | Data types   | Mask | Synchronize | Rate limiter | Temperature | Illuminance |
| ----------------- | ------ | ---------- | ------------ | ---- | ----------- | ------------ | ----------- | ----------- |
| Prophesee EVK4    | Camera | 1280 × 720 | DVS, trigger | ✓    | ✓           | ✓            | ✓           | ✓           |
| Prophesee EVK3 HD | Camera | 1280 × 720 | DVS, trigger | ✓    | -           | ✓            | -           | -           |

This table lists fratures supported by this library. Some devices support unlisted features or features marked as "no" that have yet to be added to neuromorphic_drivers.

| Name              | Links                                       |
| ----------------- | ------------------------------------------- |
| Prophesee EVK4    | https://www.prophesee.ai/event-camera-evk4/ |
| Prophesee EVK3 HD | https://www.prophesee.ai/event-based-evk-3/ |

# Python

## Get started

```sh
pip install neuromorphic_drivers
```

On Linux, run the following comman after installing the package to install UDEV rules.

```sh
neuromorphic-drivers-install-udev-rules
```

The following script reads data from a connected device.

```py
import neuromorphic_drivers as nd

nd.print_device_list()  # print a table that lists connected devices

with nd.open() as device:
    for status, packet in device:
        if "dvs_events" in packet:
            # packet["dvs_events"] is a structured numpy array
            # with dtype [("t", "<u8"), ("x", "<u2"), ("y", "<u2"), ("on", "?")])
            pass
        if "trigger_events" in packet:
            # packet["trigger_events"] is a structured numpy array
            # with dtype [("t", "<u8"), ("id, "<u1"), ("rising", "?")])
            pass
```

The keys in a packet depend on the device and are only present if the associated array is not empty. Packets contain variable numbers of events (typically a few thousand to a few hundred thousand) covering a variable amount of time (typically a few dozen microseconds to a few milliseconds).

## Device configuration

The configuration, specific to each device, defines all the device's parameters – from biases to region of interest. It can be specified when opening a device and it can be updated at any time. The default configuration for device `d` is defined in _python/neuromorphic_drivers/generated/devices/d.py_.

```py
import neuromorphic_drivers as nd

configuration = nd.prophesee_evk4.Configuration(
    biases=nd.prophesee_evk4.Biases(
        diff_off=170,
        diff_on=130,
    )
)

with nd.open(configuration=configuration) as device:
    for status, packet in device:
        ...
        if not_sensitive_enough:
            configuration.biases.diff_on -= 10
            configuration.biases.diff_off -= 10
            device.update_configuration(configuration)
```

`nd.open` may open any supported device. However, it only considers matching devices if a configuration is specified. Passing the wrong configuration type to `update_configuration` raises an error.

## Rate limiter

Prophesee's EVK4 has a hardware event-rate limiter that randomly drops events when the sensor generates more than a given number per second. The limiter has two parameters. `reference_period_us` defines a count period in µs (maximum 200). `maximum_events_per_period` defines the number of events per period beyond which the limiter activates. The effective limit is `maximum_events_per_period / reference_period_us` in events/µs.

```py
import neuromorphic_drivers as nd

configuration = nd.prophesee_evk4.Configuration(
    rate_limiter=nd.prophesee_evk4.RateLimiter(
        reference_period_us=200,
        maximum_events_per_period=4000,
    )
)

with nd.open(configuration=configuration) as device:
    ...
```

## Raw mode

Converting the raw USB data into events can be an expensive operation if the data rate is high. Raw mode skips parsing and can be useful if one simply wishes to store the data into a file to be processed offline.

```py
import neuromorphic_drivers as nd

with nd.open(raw=True) as device:
    for status, packet in device:
        # in raw mode, packet is a "bytes" object
```

## Other open options

```py
def open(
    # initial device configuration
    configuration: typing.Optional[Configuration] = None,

    # timeout for each iteration (for status, packet in device)
    # By default, the iterator blocks until data is available.
    # In some cases (other time-driven calculations, graphics...),
    # it can be desirable to run an iteration of the loop even if no data is available.
    # If iterator_timeout is not None, packet may be None.
    iterator_timeout: typing.Optional[float] = None,

    # whether to skip data parsing
    raw: bool = False,

    # device serial number, None selects the first available device.
    # Use nd.list_devices() to get a list of connected devices and their serials.
    serial: typing.Optional[str] = None,

    # USB software ring configuration, None falls back to default.
    usb_configuration: typing.Optional[UsbConfiguration] = None,

    # maximum number of raw USB packets merged to create a packet returned by the iterator.
    # Under typical conditions, each iterator packet is built from one USB packet.
    # However, if more than one USB packet has already been received,
    # the library builds the iterator packet by merging the USB packets,
    # to reduce the number of GIL (Global Interpreter Lock) operations.
    # While merging speeds up processing in such cases, it can lead to large latency spikes if too many packets are merged at once.
    # iterator_maximum_raw_packets limits the number of packets that are merged, even if more USB packets have been received.
    # The remaning USB packets are not dropped but simply returned on the next iteration(s).
    iterator_maximum_raw_packets: int = 64,
): ...
```

The fields of the USB configuration are identical across devices, but the default values depend on the device. The values below are for a Prophesee EVK4.

```py
@dataclasses.dataclass
class UsbConfiguration:
    buffer_length: serde.type.uint64 = 131072 # size of each buffer in the ring, in bytes
    ring_length: serde.type.uint64 = 4096 # number of buffers in the ring, the total size is ring_length * buffer_length
    transfer_queue_length: serde.type.uint64 = 32 # number of libusb transfers submitted in parallel
    allow_dma: bool = False # whether to enable Direct Memory Access
```

## More examples

See _python/examples_ for different usage examples.

_python/examples/any_display.py_ implements a live event viewer with exponential decays caculated by the GPU. It requires PySide6 (`pip install PySide6`).

_python/examples/evk4_plot_hot_pixels_ generates plots and require Plotly (`pip install plotly pandas kaleido`).

## Contribute

Local build (first run).

```sh
cd python
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install maturin==1.9.1 patchelf numpy
maturin develop  # or maturin develop --release to build with optimizations
```

Local build (subsequent runs).

```sh
cd python
source .venv/bin/activate
maturin develop  # or maturin develop --release to build with optimizations
```

Before pushing new code, run the following to lint and format it.

```sh
cd python
isort .; black .; pyright .
```

The files in _python/python/neuromorphic_drivers/generated_ are generated by the build script (_python/build.rs_) and should not be modified directly. They are included in this repository for documentation purposes.

When making changes to the Rust library and the Python wrappers at the same time, change `neuromorphic-drivers = "x.y"` to `neuromorphic-drivers = {path = "../drivers"}` (in *python/Cargo.toml*) to use the most recent version of the Rust code.

# Rust

## Documentation

See https://docs.rs/neuromorphic-drivers/latest/neuromorphic_drivers/ for documentation.

## UDEV rules

1. Write the following content to _/etc/udev/rules.d/65-neuromorphic-drivers.rules_.

    ```txt
    SUBSYSTEM=="usb", ATTRS{idVendor}=="152a",ATTRS{idProduct}=="84[0-1]?", MODE="0666"
    SUBSYSTEM=="usb", ATTRS{idVendor}=="04b4",ATTRS{idProduct}=="00f[4-5]", MODE="0666"
    SUBSYSTEM=="usb", ATTRS{idVendor}=="31f7",ATTRS{idProduct}=="0003", MODE="0666"
    SUBSYSTEM=="usb", ATTRS{idVendor}=="1409",ATTRS{idProduct}=="8E00", MODE="0666"
    ```

2. Run the following commands (or reboot the machine).

    ```sh
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```

## Contribute

Run a specific test.

```sh
cd drivers
cargo test --release read -- --nocapture
```

Publish on crates.io.

```sh
cargo publish -p neuromorphic-types
cargo publish -p neuromorphic-drivers
```

# Performance

## Event rate

Recent event-based cameras, such as the Prophesee EVK4, can generate more data than can be processed in real-time under certain circumstances. While the data can always be moved to the computer's memory in real-time, the simplest algorithms (including converting the raw USB bytes to a `{t, x, y, polarity}` event representation) struggle to keep up during data rate peaks. This library uses seperate threads for reading (USB to memory) and processing (memory to memory or disk) with a circular buffer (ring) at the interface. Short data bursts are seemlessly absorbed by the ring and are typically not an issue, even though they brifely cause a spike in latency. However, persistent high data rates cause the ring to slowly fill and cause the program to eventually crash. Depending on the use-case, one of the following work arounds can be applied:

-   reduce the camera's sensitivity (usually by changing `diff_off` and `diff_on`)
-   enable the event rate limiter if the device supports it (the limiter randomly drops events before sensing them to the computer, reducing bandwidth issues but significantly degrading the quality of transient bursts)
-   reduce the camera's spatial resolution by masking rows and columns
-   change the environment if possible (avoid flickering lights, reduce the optical flow, remove background clutter, keep large and fast objects out of the field of view)
-   call `device.clear_backlog(until=0)` whenever the `backlog` becomes too large (the maximum backlog is the size of the ring minus the size of the transfer queue)
-   use `nd.open(raw=true)` to skip the parser and directly access the USB bytes, typically to save them to a file

## Direct Memory Access

This library relies on libusb for all USB communications. libusb supports Direct Memory Access (Linux only for now, though other platforms may be added in the future), which allows the USB controller to directly write packets to memory without requiring CPU (and OS kernel) intervention. While this can increase performance and reduce CPU usage, DMA comes without caveats that we disabled it by default. Users when knoweldge of their USB controller and its limitations may want to enable it to increase performance on embedded systems.

USB drivers have a limited number of DMA file objects (128 / 256 / 512 / 1024). This is typically not enough to accomodate event bursts (we use 4096 buffers by default for the EVK4, with 131072 bytes per buffer). The code fall backs to non-DMA buffers if all the DMA buffers are used (for instance the first 128 buffers would be DMA and the rest would be non-DMA) and may result in variable performance over time.

Using all the available DMA buffers can cause other USB transfers to fail (including control transfers to configure the device).

Using DMA thus requires one of the following workarounds:

-   Use a small number of DMA buffers and copy packets to a larger ring before processing (this somewhat defeats the purpose of DMA and increases memory copies).
-   Use larger buffers (around 1 MB) to inccrease the ring size without increasing the number of buffers, at the cost of increased latency.
-   Ensure that processing can always keep up with the data rate (sparse scenes / low-sensitivity biases / event-rate limiter / simple processing). Note that simply expanding vectorized EVT3 events to 13-bytes DVS events is not real-time during data peaks.
