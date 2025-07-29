use crate::adapters;
use crate::device::SerialOrBusNumberAndAddress;
use crate::device::Usb;
use crate::flag;
use crate::usb;
use rusb::UsbContext;

macro_rules! register {
    ($($module:ident),+) => {
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

            #[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
            #[serde(tag = "type", content = "configuration")]
            #[allow(clippy::large_enum_variant)]
            pub enum Configuration {
                $(
                    #[serde(rename = "" $module)]
                    [<$module:camel>]($module::Configuration),
                )+
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
                pub bus_number: u8,
                pub address: u8,
                pub speed: usb::Speed,
                pub serial: Result<String, usb::Error>,
            }

            impl ListedDevice {
                pub fn open(
                    &self,
                    configuration: Option<Configuration>,
                    usb_configuration: Option<usb::Configuration>,
                    event_loop: std::sync::Arc<usb::EventLoop>,
                    flag: flag::Flag<Error, usb::Overflow>,
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
                                            $module::Device::open(
                                                SerialOrBusNumberAndAddress::BusNumberAndAddress((self.bus_number, self.address)),
                                                configuration,
                                                usb_configuration
                                                .as_ref()
                                                .unwrap_or(&$module::Device::DEFAULT_USB_CONFIGURATION),
                                                event_loop.clone(),
                                                flag.clone(),
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
                                        $module::Device::open(
                                            SerialOrBusNumberAndAddress::BusNumberAndAddress((self.bus_number, self.address)),
                                            $module::Device::PROPERTIES.default_configuration.clone(),
                                            usb_configuration
                                            .as_ref()
                                            .unwrap_or(&$module::Device::DEFAULT_USB_CONFIGURATION),
                                            event_loop.clone(),
                                            flag.clone(),
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
                        $module::Device::list_devices(&devices)?
                            .into_iter()
                            .map(|listed_device| ListedDevice {
                                device_type: Type::[<$module:camel>],
                                bus_number: listed_device.bus_number,
                                address: listed_device.address,
                                speed: listed_device.speed,
                                serial: listed_device.serial,
                            }),
                    );
                )+
                Ok(result)
            }

            pub fn open(
                serial_or_bus_number_and_address: SerialOrBusNumberAndAddress,
                configuration: Option<Configuration>,
                usb_configuration: Option<usb::Configuration>,
                event_loop: std::sync::Arc<usb::EventLoop>,
                flag: flag::Flag<Error, usb::Overflow>,
            ) -> Result<Device, Error>
            {
                match configuration {
                    Some(configuration) => {
                        match configuration {
                            $(
                                Configuration::[<$module:camel>](configuration) => Ok(
                                    $module::Device::open(
                                        serial_or_bus_number_and_address,
                                        configuration,
                                        usb_configuration
                                        .as_ref()
                                        .unwrap_or(&$module::Device::DEFAULT_USB_CONFIGURATION),
                                        event_loop.clone(),
                                        flag.clone(),
                                    )
                                    .map(|device| paste::paste! {Device::[<$module:camel>](device)})
                                    .map_err(|error| Error::from(error).unpack())?
                                ),
                            )+
                        }
                    },
                    None => {
                        $(
                            match $module::Device::open(
                                serial_or_bus_number_and_address,
                                $module::Device::PROPERTIES.default_configuration.clone(),
                                usb_configuration
                                .as_ref()
                                .unwrap_or(&$module::Device::DEFAULT_USB_CONFIGURATION),
                                event_loop.clone(),
                                flag.clone(),
                            ) {
                                Ok(device) => return Ok(Device::[<$module:camel>](device)),
                                Err(error) => match Error::from(error).unpack() {
                                    Error::DeviceWithSerial {device_type: _, serial: _} => (),
                                    Error::Device(_) => (),
                                    error => return Err(error.into()),
                                }
                            };
                        )+
                        Err(match serial_or_bus_number_and_address {
                            SerialOrBusNumberAndAddress::Serial(serial) => Error::Serial(serial.to_owned()),
                            SerialOrBusNumberAndAddress::BusNumberAndAddress((bus_number, address)) => {
                                Error::BusNumberAndAddressNotFound {bus_number, address}
                            },
                            SerialOrBusNumberAndAddress::None => Error::NoDevice
                        })
                    }
                }
            }

            #[derive(Debug, serde::Serialize)]
            pub enum Properties {
                $(
                    #[serde(rename = "" $module)]
                    [<$module:camel>](<$module::Device as Usb>::Properties),
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

                pub fn next_with_timeout(&self, timeout: &std::time::Duration) -> Option<usb::BufferView> {
                    match self {
                        $(
                            Self::[<$module:camel>](device) => device.next_with_timeout(timeout),
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

                pub fn vendor_and_product_id(&self) -> (u16, u16) {
                    match self {
                        $(
                            Self::[<$module:camel>](device) => device.vendor_and_product_id(),
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

                pub fn chip_firmware_configuration(&self) -> Configuration {
                    match self {
                        $(
                            Self::[<$module:camel>](device) => Configuration::[<$module:camel>](device.chip_firmware_configuration()),
                        )+
                    }
                }

                pub fn speed(&self) -> usb::Speed {
                    match self {
                        $(
                            Self::[<$module:camel>](device) => device.speed(),
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
                                        error => Self::[<$module:camel>]($module::Error::Usb(error)),
                                    },
                                    #[allow(unreachable_patterns)]  // devices may not need extra errors besides "usb::Error"
                                    error => Self::[<$module:camel>](error)
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

register! { inivation_davis346, prophesee_evk3_hd, prophesee_evk4 }
