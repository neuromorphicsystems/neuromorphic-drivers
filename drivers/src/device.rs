use crate::flag;
use crate::ring;
use crate::usb;
use rusb::UsbContext;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Location {
    BusNumberAndAddress { bus_number: u8, address: u8 },
    Address(std::net::Ipv4Addr),
}

impl std::fmt::Display for Location {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::BusNumberAndAddress {
                bus_number,
                address,
            } => write!(formatter, "{bus_number}:{address}"),
            Self::Address(address) => write!(formatter, "{address}"),
        }
    }
}

#[derive(Debug, Clone)]
pub struct ListedDevice {
    pub location: Location,
    pub connection: crate::devices::Connection,
    pub serial: Result<String, usb::Error>,
}

#[derive(Debug, Clone, Copy)]
pub struct TemperatureCelsius(pub f32);

pub type HandleAndProperties = (rusb::DeviceHandle<rusb::Context>, (u16, u16), String);

#[derive(Debug, Clone, Copy)]
pub enum Identifier<'a> {
    Serial(&'a str),
    Location(Location),
    None,
}

pub trait Device: Sized {
    type Adapter;
    type BiasesBounds;
    type Configuration;
    type Error;
    type Properties;

    const PROPERTIES: Self::Properties;

    const RING_CONFIGURATION: ring::Configuration;

    fn default_configuration(&self) -> Self::Configuration;

    fn current_configuration(&self) -> Self::Configuration;

    fn biases_bounds(&self) -> Self::BiasesBounds;

    fn update_configuration(&self, configuration: Self::Configuration);

    fn next_with_timeout(&self, timeout: &std::time::Duration) -> Option<ring::ReadBufferView<'_>>;

    fn backlog(&self) -> usize;

    fn clutch(&self) -> ring::Clutch;

    fn serial(&self) -> String;

    fn connection(&self) -> crate::devices::Connection;

    fn create_adapter(&self) -> Self::Adapter;

    /// Number of raw packets that never reached the consumer, either because the
    /// ring overflowed or because the transport rejected them.
    fn dropped_packets(&self) -> u64;
}

pub trait Ethernet: Device {
    fn list_devices() -> Vec<ListedDevice>;

    fn open<IntoError, IntoWarning>(
        identifier: Identifier,
        configuration: Self::Configuration,
        ring_configuration: &ring::Configuration,
        flag: flag::Flag<IntoError, IntoWarning>,
    ) -> Result<Self, Self::Error>
    where
        IntoError: From<Self::Error> + Clone + Send + 'static,
        IntoWarning: From<crate::ring::Overflow> + Clone + Send + 'static;
}

pub trait Usb: Device {
    const VENDOR_AND_PRODUCT_IDS: &'static [(u16, u16)];

    /// read_serial must return Ok(None) if the device is not compatible with the interface.
    /// This behaviour is required to support Prophesee EVK3 HD cameras, which share a VID/PID with
    /// EVK4 cameras.
    ///
    /// read_serial must claim bulk transfer interface(s).
    ///
    /// This is required even if read_serial does not use bulk transfers.
    fn read_serial(handle: &mut rusb::DeviceHandle<rusb::Context>) -> rusb::Result<Option<String>>;

    fn vendor_and_product_id(&self) -> (u16, u16);

    fn bus_number(&self) -> u8;

    fn address(&self) -> u8;

    fn open<IntoError, IntoWarning>(
        identifier: Identifier,
        configuration: Self::Configuration,
        ring_configuration: &ring::Configuration,
        event_loop: std::sync::Arc<usb::EventLoop>,
        flag: flag::Flag<IntoError, IntoWarning>,
    ) -> Result<Self, Self::Error>
    where
        IntoError: From<Self::Error> + Clone + Send + 'static,
        IntoWarning: From<crate::ring::Overflow> + Clone + Send + 'static;

    fn list_devices(devices: &rusb::DeviceList<rusb::Context>) -> rusb::Result<Vec<ListedDevice>> {
        let mut result = Vec::new();
        for device in devices
            .iter()
            .filter(|device| match device.device_descriptor() {
                Ok(descriptor) => {
                    let device_vendor_and_product_id =
                        (descriptor.vendor_id(), descriptor.product_id());
                    Self::VENDOR_AND_PRODUCT_IDS
                        .iter()
                        .any(|vendor_and_product_id| {
                            device_vendor_and_product_id == *vendor_and_product_id
                        })
                }
                Err(_) => false,
            })
        {
            if let Some(serial) = Self::read_serial(&mut device.open()?).transpose() {
                result.push(ListedDevice {
                    location: Location::BusNumberAndAddress {
                        bus_number: device.bus_number(),
                        address: device.address(),
                    },
                    connection: crate::usb::Speed::from(device.speed()).into(),
                    serial: serial.map_err(|error| error.into()),
                });
            }
        }
        Ok(result)
    }

    fn open_any(context: &rusb::Context) -> Result<HandleAndProperties, usb::Error> {
        match context.devices()?.iter().find_map(
            |device| -> Option<rusb::Result<HandleAndProperties>> {
                match device.device_descriptor() {
                    Ok(descriptor) => {
                        let device_vendor_and_product_id =
                            (descriptor.vendor_id(), descriptor.product_id());
                        if Self::VENDOR_AND_PRODUCT_IDS
                            .iter()
                            .any(|vendor_and_product_id| {
                                device_vendor_and_product_id == *vendor_and_product_id
                            })
                        {
                            let mut handle = match device.open() {
                                Ok(handle) => handle,
                                Err(error) => return Some(Err(error)),
                            };
                            let device_serial = match Self::read_serial(&mut handle) {
                                Ok(Some(serial)) => serial,
                                Ok(None) => return None, // ignore unsupported devices
                                Err(_) => return None, // do not raise an error if the device is already open
                            };
                            let _ = handle.set_auto_detach_kernel_driver(true);
                            Some(Ok((handle, device_vendor_and_product_id, device_serial)))
                        } else {
                            None
                        }
                    }
                    Err(_) => None,
                }
            },
        ) {
            Some(result) => Ok(result?),
            None => Err(usb::Error::Device),
        }
    }

    fn open_serial(
        context: &rusb::Context,
        serial: &str,
    ) -> Result<HandleAndProperties, usb::Error> {
        match context.devices()?.iter().find_map(
            |device| -> Option<rusb::Result<HandleAndProperties>> {
                match device.device_descriptor() {
                    Ok(descriptor) => {
                        let device_vendor_and_product_id =
                            (descriptor.vendor_id(), descriptor.product_id());
                        if Self::VENDOR_AND_PRODUCT_IDS
                            .iter()
                            .any(|vendor_and_product_id| {
                                device_vendor_and_product_id == *vendor_and_product_id
                            })
                        {
                            let mut handle = match device.open() {
                                Ok(handle) => handle,
                                Err(error) => return Some(Err(error)),
                            };
                            let device_serial = match Self::read_serial(&mut handle) {
                                Ok(Some(serial)) => serial,
                                Ok(None) => return None, // ignore unsupported devices
                                Err(_) => return None, // do not raise an error if the device is already open
                            };
                            if *serial == device_serial {
                                let _ = handle.set_auto_detach_kernel_driver(true);
                                Some(Ok((handle, device_vendor_and_product_id, device_serial)))
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    }
                    Err(_) => None,
                }
            },
        ) {
            Some(result) => Ok(result?),
            None => Err(usb::Error::Serial((*serial).to_owned())),
        }
    }

    fn open_bus_number_and_address(
        context: &rusb::Context,
        bus_number: u8,
        address: u8,
    ) -> Result<HandleAndProperties, usb::Error> {
        match context.devices()?.iter().find_map(
            |device| -> Option<Result<HandleAndProperties, usb::Error>> {
                if device.bus_number() == bus_number && device.address() == address {
                    Some(match device.device_descriptor() {
                        Ok(descriptor) => {
                            let device_vendor_and_product_id =
                                (descriptor.vendor_id(), descriptor.product_id());
                            if Self::VENDOR_AND_PRODUCT_IDS
                                .iter()
                                .any(|vendor_and_product_id| {
                                    device_vendor_and_product_id == *vendor_and_product_id
                                })
                            {
                                let mut handle = match device.open() {
                                    Ok(handle) => handle,
                                    Err(error) => return Some(Err(error.into())),
                                };
                                match Self::read_serial(&mut handle) {
                                    Ok(Some(serial)) => {
                                        let _ = handle.set_auto_detach_kernel_driver(true);
                                        Ok((handle, device_vendor_and_product_id, serial))
                                    }
                                    Ok(None) => {
                                        Err(usb::Error::BusNumberAndAddressUnsupportedDevice {
                                            bus_number,
                                            address,
                                        })
                                    }
                                    Err(error) => Err(usb::Error::BusNumberAndAddressAccessError {
                                        bus_number,
                                        address,
                                        error,
                                    }),
                                }
                            } else {
                                Err(usb::Error::BusNumberAndAddressUnexpectedIds {
                                    bus_number,
                                    address,
                                    vendor_id: device_vendor_and_product_id.0,
                                    product_id: device_vendor_and_product_id.1,
                                })
                            }
                        }
                        Err(error) => Err(usb::Error::BusNumberAndAddressAccessError {
                            bus_number,
                            address,
                            error,
                        }),
                    })
                } else {
                    None
                }
            },
        ) {
            Some(result) => Ok(result?),
            None => Err(usb::Error::BusNumberAndAddressNotFound {
                bus_number,
                address,
            }),
        }
    }
}
