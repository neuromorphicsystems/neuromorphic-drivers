use crate::adapters;
use crate::device::Device as _;
use crate::device::Identifier;
use crate::flag;
use crate::ring;
use crate::usb;
use rusb::UsbContext;

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum Connection {
    UsbUnknown,
    UsbLow,
    UsbFull,
    UsbHigh,
    UsbSuper,
    UsbSuperPlus,
    Ethernet,
}

impl From<usb::Speed> for Connection {
    fn from(speed: usb::Speed) -> Self {
        match speed {
            usb::Speed::Unknown => Self::UsbUnknown,
            usb::Speed::Low => Self::UsbLow,
            usb::Speed::Full => Self::UsbFull,
            usb::Speed::High => Self::UsbHigh,
            usb::Speed::Super => Self::UsbSuper,
            usb::Speed::SuperPlus => Self::UsbSuperPlus,
        }
    }
}

impl std::fmt::Display for Connection {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UsbUnknown => write!(formatter, "{}", usb::Speed::Unknown),
            Self::UsbLow => write!(formatter, "{}", usb::Speed::Low),
            Self::UsbFull => write!(formatter, "{}", usb::Speed::Full),
            Self::UsbHigh => write!(formatter, "{}", usb::Speed::High),
            Self::UsbSuper => write!(formatter, "{}", usb::Speed::Super),
            Self::UsbSuperPlus => write!(formatter, "{}", usb::Speed::SuperPlus),
            Self::Ethernet => write!(formatter, "Ethernet"),
        }
    }
}

macro_rules! list_devices_for {
    (usb, $module:ident, $devices:expr) => {
        <$module::Device as crate::device::Usb>::list_devices($devices)?
    };
    (ethernet, $module:ident, $devices:expr) => {
        <$module::Device as crate::device::Ethernet>::list_devices()
    };
}

macro_rules! open_for {
    (usb, $module:ident, $identifier:expr, $configuration:expr, $ring_configuration:expr, $event_loop:expr, $flag:expr) => {
        <$module::Device as crate::device::Usb>::open(
            $identifier,
            $configuration,
            $ring_configuration,
            $event_loop,
            $flag,
        )
    };
    (ethernet, $module:ident, $identifier:expr, $configuration:expr, $ring_configuration:expr, $event_loop:expr, $flag:expr) => {
        <$module::Device as crate::device::Ethernet>::open(
            $identifier,
            $configuration,
            $ring_configuration,
            $flag,
        )
    };
}

macro_rules! vendor_and_product_id_for {
    (usb, $device:expr) => {
        Some(crate::device::Usb::vendor_and_product_id($device))
    };
    (ethernet, $device:expr) => {{
        let _ = $device;
        None
    }};
}

macro_rules! unpack_for {
    (usb, $module:ident, $device_type:expr, $error:expr) => {{
        let _ = $device_type;
        let _ = $error;
        None
    }};
    (ethernet, $module:ident, $device_type:expr, $error:expr) => {
        match $error {
            $module::Error::SerialNotFound(serial) => Some(Error::DeviceWithSerial {
                device_type: $device_type,
                serial: serial.clone(),
            }),
            $module::Error::NotFound => Some(Error::Device($device_type)),
            $module::Error::BusNumberAndAddress => Some(Error::AddressUnsupported($device_type)),
            _ => None,
        }
    };
}

macro_rules! register {
    ($($module:ident: $transport:ident),+ $(,)?) => {
        paste::paste! {
            $(
                pub mod $module;
            )+

            #[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
            pub enum Type {
                $(
                    [<$module:camel>],
                )+
            }

            impl std::fmt::Display for Type {
                fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                    match self {
                        $(
                            Self::[<$module:camel>] => write!(formatter, stringify!($module)),
                        )+
                    }
                }
            }

            impl Type {
                pub fn name(self) -> &'static str  {
                    match self {
                        $(
                            Type::[<$module:camel>] => $module::Device::PROPERTIES.name,
                        )+
                    }
                }
            }

            #[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
            #[serde(tag = "type", content = "biases_bounds")]
            #[allow(clippy::large_enum_variant)]
            pub enum BiasesBounds {
                $(
                    #[serde(rename = "" $module)]
                    [<$module:camel>]($module::BiasesBounds),
                )+
            }

            #[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
            #[serde(tag = "type", content = "configuration")]
            #[allow(clippy::large_enum_variant)]
            pub enum Configuration {
                $(
                    #[serde(rename = "" $module)]
                    [<$module:camel>]($module::Configuration),
                )+
            }

            impl BiasesBounds {
                pub fn serialize_bincode(&self) -> bincode::Result<Vec<u8>> {
                    match self {
                        $(
                            BiasesBounds::[<$module:camel>](biases_bounds) => bincode::serialize(biases_bounds),
                        )+
                    }
                }
            }

            impl Configuration {
                pub fn serialize_bincode(&self) -> bincode::Result<Vec<u8>> {
                    match self {
                        $(
                            Configuration::[<$module:camel>](configuration) => bincode::serialize(configuration),
                        )+
                    }
                }

                pub fn deserialize_bincode(
                    device_type: Type,
                    data: &[u8]
                ) -> bincode::Result<Configuration> {
                    match device_type {
                        $(
                            Type::[<$module:camel>] => Ok(
                                Configuration::[<$module:camel>](bincode::deserialize(data)?)
                            ),
                        )+
                    }
                }

                pub fn type_name(&self) -> &'static str {
                    match self {
                        $(
                            Configuration::[<$module:camel>](_) => Type::[<$module:camel>].name(),
                        )+
                    }
                }
            }

            #[allow(clippy::large_enum_variant)]
            pub enum Device {
                $(
                    [<$module:camel>]($module::Device),
                )+
            }

            #[derive(Debug)]
            pub struct ListedDevice {
                pub device_type: Type,
                pub location: crate::device::Location,
                pub connection: Connection,
                pub serial: Result<String, usb::Error>,
            }

            impl ListedDevice {
                pub fn open(
                    &self,
                    configuration: Option<Configuration>,
                    ring_configuration: Option<ring::Configuration>,
                    event_loop: std::sync::Arc<usb::EventLoop>,
                    flag: flag::Flag<Error, ring::Overflow>,
                ) -> Result<Device, Error> {
                    match configuration {
                        Some(configuration) => {
                            let device_type_name = self.device_type.name();
                            let configuration_type_name = match configuration {
                                $(
                                    Configuration::[<$module:camel>](_) => Type::[<$module:camel>].name(),
                                )+
                            };
                            if (device_type_name != configuration_type_name) {
                                Err(Error::ConfigurationType {
                                    device: device_type_name.to_owned(),
                                    configuration: configuration_type_name.to_owned(),
                                })
                            } else {
                                match configuration {
                                    $(
                                        Configuration::[<$module:camel>](configuration) => Ok(
                                            open_for!(
                                                $transport,
                                                $module,
                                                Identifier::Location(self.location),
                                                configuration,
                                                ring_configuration
                                                .as_ref()
                                                .unwrap_or(&$module::Device::RING_CONFIGURATION),
                                                event_loop.clone(),
                                                flag.clone()
                                            )
                                            .map(|device| paste::paste! {Device::[<$module:camel>](device)})
                                            .map_err(|error| Error::from(error).unpack())?
                                        ),
                                    )+
                                }
                            }
                        }
                        None => {
                            match self.device_type {
                                $(
                                    Type::[<$module:camel>] => Ok(
                                        open_for!(
                                            $transport,
                                            $module,
                                            Identifier::Location(self.location),
                                            $module::Device::PROPERTIES.default_configuration.clone(),
                                            ring_configuration
                                            .as_ref()
                                            .unwrap_or(&$module::Device::RING_CONFIGURATION),
                                            event_loop.clone(),
                                            flag.clone()
                                        )
                                        .map(|device| paste::paste! {Device::[<$module:camel>](device)})
                                        .map_err(|error| Error::from(error).unpack())?
                                    ),
                                )+
                            }
                        }
                    }
                }
            }

            pub fn list_devices() -> rusb::Result<Vec<ListedDevice>> {
                let context = rusb::Context::new()?;
                let devices = context.devices()?;
                let mut result = Vec::new();
                $(
                    result.extend(
                        list_devices_for!($transport, $module, &devices)
                            .into_iter()
                            .map(|listed_device| ListedDevice {
                                device_type: Type::[<$module:camel>],
                                location: listed_device.location,
                                connection: listed_device.connection,
                                serial: listed_device.serial,
                            }),
                    );
                )+
                Ok(result)
            }

            pub fn open(
                identifier: Identifier,
                configuration: Option<Configuration>,
                ring_configuration: Option<ring::Configuration>,
                event_loop: std::sync::Arc<usb::EventLoop>,
                flag: flag::Flag<Error, ring::Overflow>,
            ) -> Result<Device, Error>
            {
                match configuration {
                    Some(configuration) => {
                        match configuration {
                            $(
                                Configuration::[<$module:camel>](configuration) => Ok(
                                    open_for!(
                                        $transport,
                                        $module,
                                        identifier,
                                        configuration,
                                        ring_configuration
                                        .as_ref()
                                        .unwrap_or(&$module::Device::RING_CONFIGURATION),
                                        event_loop.clone(),
                                        flag.clone()
                                    )
                                    .map(|device| paste::paste! {Device::[<$module:camel>](device)})
                                    .map_err(|error| Error::from(error).unpack())?
                                ),
                            )+
                        }
                    },
                    None => {
                        $(
                            match open_for!(
                                $transport,
                                $module,
                                identifier,
                                $module::Device::PROPERTIES.default_configuration.clone(),
                                ring_configuration
                                .as_ref()
                                .unwrap_or(&$module::Device::RING_CONFIGURATION),
                                event_loop.clone(),
                                flag.clone()
                            ) {
                                Ok(device) => return Ok(Device::[<$module:camel>](device)),
                                Err(error) => match Error::from(error).unpack() {
                                    Error::DeviceWithSerial {device_type: _, serial: _} => (),
                                    Error::Device(_) => (),
                                    Error::AddressUnsupported(_) => (),
                                    error => return Err(error.into()),
                                }
                            };
                        )+
                        Err(match identifier {
                            Identifier::Serial(serial) => Error::Serial(serial.to_owned()),
                            Identifier::Location(crate::device::Location::BusNumberAndAddress {
                                bus_number,
                                address,
                            }) => Error::BusNumberAndAddressNotFound {bus_number, address},
                            Identifier::Location(crate::device::Location::Address(_)) => Error::NoDevice,
                            Identifier::None => Error::NoDevice
                        })
                    }
                }
            }

            #[derive(Debug, serde::Serialize)]
            pub enum Properties {
                $(
                    #[serde(rename = "" $module)]
                    [<$module:camel>](<$module::Device as crate::device::Device>::Properties),
                )+
            }

            impl Device {
                pub fn create_adapter(&self) -> adapters::Adapter {
                    match self {
                        $(
                            Self::[<$module:camel>](device) => device.create_adapter().into(),
                        )+
                    }
                }

                pub fn next_with_timeout(&self, timeout: &std::time::Duration) -> Option<ring::ReadBufferView<'_>> {
                    match self {
                        $(
                            Self::[<$module:camel>](device) => device.next_with_timeout(timeout),
                        )+
                    }
                }

                pub fn dropped_packets(&self) -> u64 {
                    match self {
                        $(
                            Self::[<$module:camel>](device) => device.dropped_packets(),
                        )+
                    }
                }

                pub fn backlog(&self) -> usize {
                    match self {
                        $(
                            Self::[<$module:camel>](device) => device.backlog(),
                        )+
                    }
                }

                pub fn properties(&self) -> Properties {
                    match self {
                        $(
                            Self::[<$module:camel>](_) => Properties::[<$module:camel>]($module::Device::PROPERTIES),
                        )+
                    }
                }

                pub fn name(&self) -> &'static str {
                    match self {
                        $(
                            Self::[<$module:camel>](_) => $module::Device::PROPERTIES.name,
                        )+
                    }
                }

                pub fn vendor_and_product_id(&self) -> Option<(u16, u16)> {
                    match self {
                        $(
                            Self::[<$module:camel>](device) => vendor_and_product_id_for!($transport, device),
                        )+
                    }
                }

                pub fn serial(&self) -> String {
                    match self {
                        $(
                            Self::[<$module:camel>](device) => device.serial(),
                        )+
                    }
                }

                pub fn connection(&self) -> Connection {
                    match self {
                        $(
                            Self::[<$module:camel>](device) => device.connection(),
                        )+
                    }
                }

                pub fn default_configuration(&self) -> Configuration {
                    match self {
                        $(
                            Self::[<$module:camel>](device) => Configuration::[<$module:camel>](device.default_configuration()),
                        )+
                    }
                }

                pub fn biases_bounds(&self) -> BiasesBounds {
                    match self {
                        $(
                            Self::[<$module:camel>](device) => BiasesBounds::[<$module:camel>](device.biases_bounds()),
                        )+
                    }
                }

                pub fn current_configuration(&self) -> Configuration {
                    match self {
                        $(
                            Self::[<$module:camel>](device) => Configuration::[<$module:camel>](device.current_configuration()),
                        )+
                    }
                }

                pub fn update_configuration(&self, configuration: Configuration) -> Result<(), Error> {
                    match self {
                        $(
                            Self::[<$module:camel>](device) => match configuration {
                                Configuration::[<$module:camel>](configuration) => {
                                    device.update_configuration(configuration);
                                    Ok(())
                                },
                                configuration => Err(Error::UpdateMismatch {
                                    configuration: configuration.type_name().to_owned(),
                                    device: $module::Device::PROPERTIES.name.to_owned(),
                                })
                            },
                        )+
                    }
                }
            }

            #[derive(Debug, PartialEq, Eq)]
            pub struct ParseTypeError {
                on: String
            }

            impl std::fmt::Display for ParseTypeError {
                fn fmt(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
                    write!(formatter, "unknow device type \"{}\"", self.on)
                }
            }

            impl std::str::FromStr for Type {
                type Err = ParseTypeError;

                fn from_str(string: &str) -> Result<Self, Self::Err> {
                    match string {
                        $(
                            stringify!($module) => paste::paste! {Ok(Self::[<$module:camel>])},
                        )+
                        _ => Err(Self::Err {on: string.to_owned()}),
                    }
                }
            }

            #[derive(thiserror::Error, Debug, Clone)]
            pub enum Error {
                #[error(transparent)]
                Usb(#[from] usb::Error),

                #[error("{device_type} with serial \"{serial}\" not found")]
                DeviceWithSerial { device_type: Type, serial: String },

                #[error("no {0} found")]
                Device(Type),

                #[error("{0} devices cannot be identified by an IP address")]
                AddressUnsupported(Type),

                #[error("serial \"{0}\" not found")]
                Serial(String),

                #[error("there is no device on bus {bus_number} at address {address}")]
                BusNumberAndAddressNotFound { bus_number: u8, address: u8 },

                #[error("unexpected configuration type (the device is a \"{device}\", got a \"{configuration}\" configuration)")]
                ConfigurationType { device: String, configuration: String },

                #[error("no device found")]
                NoDevice,

                #[error("control transfer error (expected {expected:?}, read {read:?})")]
                Mismatch { expected: Vec<u8>, read: Vec<u8> },

                #[error("configuration for {configuration:?} is not compatible with device {device:?}")]
                UpdateMismatch {
                    configuration: String,
                    device: String,
                },

                $(
                    #[error(transparent)]
                    [<$module:camel>](#[from] $module::Error),
                )+
            }

            impl Error {
                pub fn unpack(self) -> Self {
                    match self {
                        $(
                            Self::[<$module:camel>](error) => {
                                match error {
                                    $module::Error::Usb(error) => match error {
                                        usb::Error::Serial(serial) => Self::DeviceWithSerial {
                                            device_type: Type::[<$module:camel>],
                                            serial,
                                        },
                                        usb::Error::Device => Self::Device(Type::[<$module:camel>]),
                                        usb::Error::Address => {
                                            Self::AddressUnsupported(Type::[<$module:camel>])
                                        }
                                        error => Self::[<$module:camel>]($module::Error::Usb(error)),
                                    },
                                    #[allow(unreachable_patterns)]  // devices may not need extra errors besides "usb::Error"
                                    error => match unpack_for!(
                                        $transport,
                                        $module,
                                        Type::[<$module:camel>],
                                        &error
                                    ) {
                                        Some(unpacked) => unpacked,
                                        None => Self::[<$module:camel>](error),
                                    }
                                }
                            }
                        )+
                        error => error
                    }
                }
            }
        }
    };
}

register! {
    inivation_davis346: usb,
    inivation_dvxplorer: usb,
    prophesee_evk3_hd: usb,
    prophesee_evk4: usb,
    centuryarks_vga: usb,
    lucid_triton: ethernet,
}
