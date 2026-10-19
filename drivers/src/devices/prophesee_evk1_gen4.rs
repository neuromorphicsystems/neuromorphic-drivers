use crate::adapters;
use crate::configuration;
use crate::device;
use crate::flag;
use crate::properties;
use crate::ring;
use crate::usb;

use device::Device as _;
use device::Usb as _;

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq, Eq)]
pub struct Biases {
    pub pr: u8,
    pub fo_n: u8,
    pub hpf: u8,
    pub diff_on: u8,
    pub diff: u8,
    pub diff_off: u8,
    pub refr: u8,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct BiasesBounds {
    pub pr: properties::Bounds<u8>,
    pub fo_n: properties::Bounds<u8>,
    pub hpf: properties::Bounds<u8>,
    pub diff_on: properties::Bounds<u8>,
    pub diff: properties::Bounds<u8>,
    pub diff_off: properties::Bounds<u8>,
    pub refr: properties::Bounds<u8>,
}

pub const BIASES_BOUNDS: BiasesBounds = BiasesBounds {
    pr: properties::Bounds::new(0, 255),
    fo_n: properties::Bounds::new(0, 255),
    hpf: properties::Bounds::new(0, 255),
    diff_on: properties::Bounds::new(0, 255),
    diff: properties::Bounds::new(0, 255),
    diff_off: properties::Bounds::new(0, 255),
    refr: properties::Bounds::new(0, 255),
};

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq, Eq)]
pub struct Configuration {
    pub biases: Biases,
}

#[derive(thiserror::Error, Debug, Clone)]
pub enum Error {
    #[error(transparent)]
    Usb(#[from] usb::Error),

    #[error("short write ({requested} bytes requested, {written} bytes written)")]
    ShortWrite { requested: usize, written: usize },

    #[error("short response while reading register {0}")]
    RegisterReadShortResponse(u32),

    #[error("bytes mismatch while reading register {0}")]
    RegisterReadMismatch(u32),

    #[error("register {address:#x} mismatch (expected {expected:#x}, read {read:#x})")]
    RegisterValueMismatch {
        address: u32,
        expected: u32,
        read: u32,
    },
}

impl From<rusb::Error> for Error {
    fn from(error: rusb::Error) -> Self {
        usb::Error::from(error).into()
    }
}
pub struct Device {
    handle: std::sync::Arc<rusb::DeviceHandle<rusb::Context>>,
    configuration_updater: configuration::Updater<Configuration>,
    // Stop configuration updates and the event stream before cancelling transfers.
    _shutdown: Shutdown,
    ring: usb::TransferManager,
    vendor_and_product_id: (u16, u16),
    serial: String,
}

pub const PROPERTIES: properties::Camera<Configuration> = Device::PROPERTIES;
pub const DEFAULT_CONFIGURATION: Configuration = Device::PROPERTIES.default_configuration;
pub const RING_CONFIGURATION: ring::Configuration = Device::RING_CONFIGURATION;
pub fn open<IntoError, IntoWarning>(
    serial_or_bus_number_and_address: device::Identifier,
    configuration: Configuration,
    ring_configuration: &ring::Configuration,
    event_loop: std::sync::Arc<usb::EventLoop>,
    flag: flag::Flag<IntoError, IntoWarning>,
) -> Result<Device, Error>
where
    IntoError: From<Error> + Clone + Send + 'static,
    IntoWarning: From<ring::Overflow> + Clone + Send + 'static,
{
    Device::open(
        serial_or_bus_number_and_address,
        configuration,
        ring_configuration,
        event_loop,
        flag,
    )
}

impl device::Device for Device {
    type Adapter = adapters::evt3::Adapter;

    type BiasesBounds = BiasesBounds;

    type Configuration = Configuration;

    type Error = Error;

    type Properties = properties::Camera<Self::Configuration>;

    const PROPERTIES: Self::Properties = Self::Properties {
        name: "Prophesee EVK1 Gen4",
        width: 1280,
        height: 720,
        default_configuration: Self::Configuration {
            biases: Biases {
                pr: 0x83,
                fo_n: 0x17,
                hpf: 0x30,
                diff_on: 0x70,
                diff: 0x45,
                diff_off: 0x34,
                refr: 0x2d,
            },
        },
    };

    const RING_CONFIGURATION: ring::Configuration = ring::Configuration {
        buffer_length: 1 << 17,
        ring_length: 1 << 12,
        parallel_submissions: 1 << 5,
    };

    fn default_configuration(&self) -> Self::Configuration {
        PROPERTIES.default_configuration
    }

    fn biases_bounds(&self) -> Self::BiasesBounds {
        BIASES_BOUNDS
    }

    fn current_configuration(&self) -> Self::Configuration {
        self.configuration_updater.current_configuration()
    }

    fn update_configuration(&self, configuration: Self::Configuration) {
        self.configuration_updater.update(configuration);
    }

    fn next_with_timeout(
        &'_ self,
        timeout: &std::time::Duration,
    ) -> Option<ring::ReadBufferView<'_>> {
        self.ring.next_with_timeout(timeout)
    }

    fn dropped_packets(&self) -> u64 {
        self.ring.dropped_packets()
    }

    fn backlog(&self) -> usize {
        self.ring.backlog()
    }

    fn clutch(&self) -> ring::Clutch {
        self.ring.clutch()
    }

    fn serial(&self) -> String {
        self.serial.clone()
    }

    fn connection(&self) -> crate::devices::Connection {
        usb::Speed::from(self.handle.device().speed()).into()
    }

    fn create_adapter(&self) -> Self::Adapter {
        Self::Adapter::from_dimensions(Self::PROPERTIES.width, Self::PROPERTIES.height)
    }
}

impl device::Usb for Device {
    const VENDOR_AND_PRODUCT_IDS: &'static [(u16, u16)] = &[(0x04B4, 0x00F4)];

    fn read_serial(handle: &mut rusb::DeviceHandle<rusb::Context>) -> rusb::Result<Option<String>> {
        handle.claim_interface(0)?;
        let mut type_buffer = [0u8; 4];
        if !matches!(
            handle.read_control(0xC0, 0x72, 0, 0, &mut type_buffer, TIMEOUT),
            Ok(4)
        ) || type_buffer != [0, 0, 0x1a, 0]
        {
            return Ok(None);
        }
        let serial = Serial::default()
            .read(handle)
            .map_err(|error| match error {
                Error::Usb(usb::Error::Rusb(error)) => error,
                _ => rusb::Error::Io,
            })?;
        Ok(Some(format!("{serial:08X}")))
    }

    fn open<IntoError, IntoWarning>(
        serial_or_bus_number_and_address: device::Identifier,
        configuration: Self::Configuration,
        ring_configuration: &ring::Configuration,
        event_loop: std::sync::Arc<usb::EventLoop>,
        flag: flag::Flag<IntoError, IntoWarning>,
    ) -> Result<Self, Self::Error>
    where
        IntoError: From<Self::Error> + Clone + Send + 'static,
        IntoWarning: From<crate::ring::Overflow> + Clone + Send + 'static,
    {
        let (handle, vendor_and_product_id, serial) = match serial_or_bus_number_and_address {
            device::Identifier::Serial(serial) => Self::open_serial(event_loop.context(), serial)?,
            device::Identifier::Location(device::Location::BusNumberAndAddress {
                bus_number,
                address,
            }) => Self::open_bus_number_and_address(event_loop.context(), bus_number, address)?,
            device::Identifier::Location(device::Location::Address(_)) => {
                return Err(usb::Error::Address.into())
            }
            device::Identifier::None => Self::open_any(event_loop.context())?,
        };
        let handle = std::sync::Arc::new(handle);
        let shutdown = Shutdown {
            handle: handle.clone(),
        };
        initialize(&handle, &configuration)?;
        let error_flag = flag.clone();
        let warning_flag = flag.clone();
        // Submit all transfers before enabling the event stream.
        let device = Device {
            handle: handle.clone(),
            ring: usb::TransferManager::new(
                ring_configuration,
                usb::TransferType::Bulk {
                    endpoint: 1 | libusb1_sys::constants::LIBUSB_ENDPOINT_IN,
                    timeout: std::time::Duration::ZERO,
                },
                move |usb_error| {
                    error_flag.store_error_if_not_set(Self::Error::from(usb_error));
                },
                move |overflow| {
                    warning_flag.store_warning_if_not_set(overflow);
                },
                handle.clone(),
                event_loop,
            )?,
            configuration_updater: configuration::Updater::new(
                configuration,
                ConfigurationUpdaterContext { handle, flag },
                |context, previous_configuration, configuration| {
                    if let Err(error) = update_configuration(
                        &context.handle,
                        Some(previous_configuration),
                        configuration,
                    ) {
                        context.flag.store_error_if_not_set(error);
                    }
                    context
                },
            ),
            vendor_and_product_id,
            serial,
            _shutdown: shutdown,
        };
        TimeHighRecovery { value: 1 }.write_checked(&device.handle)?;
        EventFormatter { value: 1 }.write_checked(&device.handle)?;
        SensorInterface {
            value: SensorInterface::default().read(&device.handle)? | 1,
        }
        .write_checked(&device.handle)?;
        EventMerge { value: 1 }.write_checked(&device.handle)?;
        TimeBaseControl { value: 0x4f }.write_checked(&device.handle)?;
        // Discard buffered events from the previous session and timebase startup.
        let start = std::time::Instant::now();
        while start.elapsed() < std::time::Duration::from_millis(100) {
            let _ = device
                .ring
                .next_with_timeout(&std::time::Duration::from_millis(10));
        }
        Ok(device)
    }

    fn vendor_and_product_id(&self) -> (u16, u16) {
        self.vendor_and_product_id
    }

    fn bus_number(&self) -> u8 {
        self.handle.device().bus_number()
    }

    fn address(&self) -> u8 {
        self.handle.device().address()
    }
}
const TIMEOUT: std::time::Duration = std::time::Duration::from_secs(1);

fn initialize(
    handle: &rusb::DeviceHandle<rusb::Context>,
    configuration: &Configuration,
) -> Result<(), Error> {
    SensorInterface::default().check(handle, 0x00040004)?;
    ClockControl::default().check(handle, 0)?;
    GlobalControl::default().check(handle, 8)?;

    ClockControl { value: 0x111 }.write_checked(handle)?;
    GlobalControl { value: 0x0d }.write_checked(handle)?;
    TimeBaseControl { value: 0x4e }.write_checked(handle)?;
    EventFormatter { value: 0 }.write_checked(handle)?;
    TimeHighRecovery { value: 0 }.write_checked(handle)?;
    TimestampChecker { value: 0x30d41 }.write_checked(handle)?;
    EventMerge { value: 0 }.write_checked(handle)?;

    // VDDIO, VDDC, clock, VDDA, VNEG, then release the sensor resets.
    let mut control = SensorInterface::default().read(handle)?;
    for (bit, delay_ms) in [
        (8, 50),
        (9, 50),
        (13, 10),
        (7, 50),
        (10, 100),
        (3, 10),
        (4, 10),
        (5, 70),
    ] {
        control |= 1 << bit;
        SensorInterface { value: control }.write(handle)?;
        std::thread::sleep(std::time::Duration::from_millis(delay_ms));
        SensorInterface::default().check(handle, control)?;
    }
    ChipId::default().check(handle, 0x20100801)?;
    let roi = (RoiControl::default().read(handle)? & !(1 | (1 << 10))) | 2;
    RoiControl { value: roi }.write_checked(handle)?;

    // Keep the power-on voltage and current-limit settings.
    for (address, value, expected, delay_ms) in [
        (0x1a000, 0x11, 0x91, 1),
        (0x1a008, 0x405, 0x2405, 1),
        (0x1a004, 0x11, 0x91, 1),
        (0x1a020, 0x170, 0x170, 1),
        (0x11104, 1, 1, 1),
        (0x1a010, 0x6b, 0x806b, 20),
        (0x11100, 4, 4, 20),
        (0x1a00c, 0x45, 0x445, 1),
    ] {
        let register = RuntimeRegister { address, value };
        register.write(handle)?;
        std::thread::sleep(std::time::Duration::from_millis(delay_ms));
        register.check(handle, expected)?;
    }
    update_configuration(handle, None, configuration)?;
    for address in [
        0x11004, 0x11024, 0x11028, 0x1102c, 0x11030, 0x11034, 0x11038, 0x1103c,
    ] {
        RuntimeRegister { address, value: 0 }.write_checked(handle)?;
    }
    BiasBlk {
        value: (BiasBlk::default().read(handle)? & !0xff) | 0x78,
    }
    .write_checked(handle)?;
    transfer_biases(handle)?;
    ReadoutControl { value: 0x10 }.write_checked(handle)?;
    let mut readout = TdReadoutControl::default().read(handle)?;
    for bit in [8, 14, 15, 17] {
        readout |= 1 << bit;
        TdReadoutControl { value: readout }.write_checked(handle)?;
    }
    RoiControl {
        value: roi | (1 << 5),
    }
    .write(handle)?;
    std::thread::sleep(std::time::Duration::from_millis(1));
    let shadow = RoiControl::default().read(handle)?;
    if shadow != roi && shadow != roi | (1 << 5) {
        return Err(Error::RegisterValueMismatch {
            address: 0x10004,
            expected: roi,
            read: shadow,
        });
    }
    RoiControl { value: roi }.write_checked(handle)?;
    ErcPipelineControl { value: 1 }.write_checked(handle)?;
    EdfPipelineControl { value: 0xffff0005 }.write_checked(handle)?;
    EdfControl { value: 1 }.write_checked(handle)?;
    EoiControl { value: 0x2201 }.write_checked(handle)?;
    SensorTimeBaseControl { value: 0x64a }.write_checked(handle)?;
    RoiControl {
        value: roi | (1 << 10),
    }
    .write_checked(handle)?;
    std::thread::sleep(std::time::Duration::from_millis(20));
    LdoDigital::default().check(handle, 0xc06b)?;
    TestBusControl { value: 8 }.write_checked(handle)?;
    DigitalPadControl { value: 0xc00 }.write_checked(handle)?;
    ClockSyncControl { value: 2 }.write_checked(handle)?;
    control |= 1 << 22;
    SensorInterface { value: control }.write_checked(handle)?;
    SensorTimeBaseControl { value: 0x64b }.write_checked(handle)?;
    Ok(())
}

struct Shutdown {
    handle: std::sync::Arc<rusb::DeviceHandle<rusb::Context>>,
}

impl Drop for Shutdown {
    fn drop(&mut self) {
        let _ = EventMerge { value: 0 }.write(&self.handle);
        let _ = TimeBaseControl { value: 0x4e }.write(&self.handle);
        if let Ok(mut control) = SensorInterface::default().read(&self.handle) {
            control &= !1;
            let _ = SensorInterface { value: control }.write_checked(&self.handle);
            let mut buffer = [0u8; 1 << 16];
            for _ in 0..20 {
                match self.handle.read_bulk(
                    0x81,
                    &mut buffer,
                    std::time::Duration::from_millis(100),
                ) {
                    Ok(0) | Err(_) => break,
                    Ok(_) => (),
                }
            }
            for (bit, delay_ms) in [
                (5, 10),
                (4, 10),
                (3, 10),
                (10, 50),
                (7, 50),
                (13, 10),
                (9, 50),
                (8, 50),
            ] {
                control &= !(1 << bit);
                let _ = SensorInterface { value: control }.write_checked(&self.handle);
                std::thread::sleep(std::time::Duration::from_millis(delay_ms));
            }
        }
        for value in [0x777, 0x10111, 0] {
            let _ = ClockControl { value }.write(&self.handle);
        }
    }
}

macro_rules! update_bias {
    ($name:ident, $register:ident, $handle:ident, $previous_biases:ident, $biases:expr) => {
        if match $previous_biases {
            Some(previous_biases) => previous_biases.$name != $biases.$name,
            None => true,
        } {
            $register {
                value: ($register::default().read($handle)? & !0xff) | $biases.$name as u32,
            }
            .write_checked($handle)?;
        }
    };
}

fn update_configuration(
    handle: &rusb::DeviceHandle<rusb::Context>,
    previous_configuration: Option<&Configuration>,
    configuration: &Configuration,
) -> Result<(), Error> {
    if previous_configuration == Some(configuration) {
        return Ok(());
    }
    let previous_biases = previous_configuration.map(|configuration| &configuration.biases);
    update_bias!(pr, BiasPr, handle, previous_biases, configuration.biases);
    update_bias!(fo_n, BiasFoN, handle, previous_biases, configuration.biases);
    update_bias!(hpf, BiasHpf, handle, previous_biases, configuration.biases);
    update_bias!(
        diff_on,
        BiasDiffOn,
        handle,
        previous_biases,
        configuration.biases
    );
    update_bias!(
        diff,
        BiasDiff,
        handle,
        previous_biases,
        configuration.biases
    );
    update_bias!(
        diff_off,
        BiasDiffOff,
        handle,
        previous_biases,
        configuration.biases
    );
    update_bias!(
        refr,
        BiasRefr,
        handle,
        previous_biases,
        configuration.biases
    );
    transfer_biases(handle)
}

fn transfer_biases(handle: &rusb::DeviceHandle<rusb::Context>) -> Result<(), Error> {
    BiasControl { value: 5 }.write(handle)?;
    std::thread::sleep(std::time::Duration::from_millis(10));
    BiasControl::default().check(handle, 4)
}

struct ConfigurationUpdaterContext<IntoError, IntoWarning>
where
    IntoError: From<Error> + Clone + Send,
    IntoWarning: From<crate::ring::Overflow> + Clone + Send,
{
    handle: std::sync::Arc<rusb::DeviceHandle<rusb::Context>>,
    flag: flag::Flag<IntoError, IntoWarning>,
}

struct RuntimeRegister {
    address: u32,
    value: u32,
}

trait Register {
    fn address(&self) -> u32;

    fn value(&self) -> u32;

    fn read(&self, handle: &rusb::DeviceHandle<rusb::Context>) -> Result<u32, Error> {
        let address = self.address();
        let mut buffer = [0u8; 8];
        let read = handle.read_control(
            0xc0,
            0x55,
            address as u16,
            (address >> 16) as u16,
            &mut buffer,
            TIMEOUT,
        )?;
        if read != buffer.len() {
            return Err(Error::RegisterReadShortResponse(address));
        }
        if buffer[0] & 0xc0 != 0x40 {
            return Err(Error::RegisterReadMismatch(address));
        }
        // unwrap: slice has the right number of bytes
        Ok(u32::from_be_bytes(buffer[4..8].try_into().unwrap()))
    }

    fn write(&self, handle: &rusb::DeviceHandle<rusb::Context>) -> Result<(), Error> {
        let address = self.address();
        let buffer = self.value().to_be_bytes();
        let written = handle.write_control(
            0x40,
            0x56,
            address as u16,
            (address >> 16) as u16,
            &buffer,
            TIMEOUT,
        )?;
        if written != buffer.len() {
            return Err(Error::ShortWrite {
                requested: buffer.len(),
                written,
            });
        }
        Ok(())
    }

    fn check(
        &self,
        handle: &rusb::DeviceHandle<rusb::Context>,
        expected: u32,
    ) -> Result<(), Error> {
        let read = self.read(handle)?;
        if read != expected {
            return Err(Error::RegisterValueMismatch {
                address: self.address(),
                expected,
                read,
            });
        }
        Ok(())
    }

    fn write_checked(&self, handle: &rusb::DeviceHandle<rusb::Context>) -> Result<(), Error> {
        self.write(handle)?;
        self.check(handle, self.value())
    }
}

impl Register for RuntimeRegister {
    fn address(&self) -> u32 {
        self.address
    }
    fn value(&self) -> u32 {
        self.value
    }
}

macro_rules! register {
    ($name:ident, $address:literal) => {
        #[derive(Default)]
        struct $name {
            value: u32,
        }
        impl Register for $name {
            fn address(&self) -> u32 {
                $address
            }
            fn value(&self) -> u32 {
                self.value
            }
        }
    };
}

register! { GlobalControl, 0x00000 }
register! { ClockControl, 0x00004 }
register! { TimeBaseControl, 0x00008 }
register! { EventFormatter, 0x00014 }
register! { EventMerge, 0x00018 }
register! { Serial, 0x00020 }
register! { TimeHighRecovery, 0x00044 }
register! { TimestampChecker, 0x00048 }
register! { RoiControl, 0x10004 }
register! { ReadoutControl, 0x10008 }
register! { TestBusControl, 0x1000c }
register! { ClockSyncControl, 0x10010 }
register! { ChipId, 0x10018 }
register! { SensorTimeBaseControl, 0x1002c }
register! { DigitalPadControl, 0x10038 }
register! { TdReadoutControl, 0x1003c }
register! { BiasPr, 0x11000 }
register! { BiasFoN, 0x11008 }
register! { BiasHpf, 0x1100c }
register! { BiasDiffOn, 0x11010 }
register! { BiasDiff, 0x11014 }
register! { BiasDiffOff, 0x11018 }
register! { BiasRefr, 0x11020 }
register! { BiasBlk, 0x1104c }
register! { BiasControl, 0x11100 }
register! { ErcPipelineControl, 0x16000 }
register! { EdfPipelineControl, 0x17000 }
register! { EdfControl, 0x17008 }
register! { EoiControl, 0x18000 }
register! { LdoDigital, 0x1a010 }
register! { SensorInterface, 0x1f000 }
