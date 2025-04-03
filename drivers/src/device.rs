use crate::flag;
use crate::usb;
use rusb::UsbContext;

pub struct ListedDevice {
    pub speed: usb::Speed,
    pub serial: Result<String, usb::Error>,
}

#[derive(Debug, Clone, Copy)]
pub struct TemperatureCelsius(pub f32);

pub type HandleAndProperties = (rusb::DeviceHandle<rusb::Context>, (u16, u16), String);

pub trait Usb: Sized {
    type Adapter;
    type Configuration;
    type Error;
    type Properties;

    const VENDOR_AND_PRODUCT_IDS: &'static [(u16, u16)];

    const PROPERTIES: Self::Properties;

    const DEFAULT_USB_CONFIGURATION: usb::Configuration;

    /// read_serial must return Ok(None) if the device is not compatible with the interface.
    /// This behaviour is required to support Prophesee EVK3 HD cameras, which share a VID/PID with
    /// EVK4 cameras.
    ///
    /// read_serial must claim bulk transfer interface(s).
    ///
    /// This is required even if read_serial does not use bulk transfers.
    fn read_serial(handle: &mut rusb::DeviceHandle<rusb::Context>) -> rusb::Result<Option<String>>;

    fn update_configuration(&self, configuration: Self::Configuration);

    fn open<IntoError, IntoWarning>(
        serial: &Option<&str>,
        configuration: Self::Configuration,
        usb_configuration: &usb::Configuration,
        event_loop: std::sync::Arc<usb::EventLoop>,
        flag: flag::Flag<IntoError, IntoWarning>,
    ) -> Result<Self, Self::Error>
    where
        IntoError: From<Self::Error> + Clone + Send + 'static,
        IntoWarning: From<crate::usb::Overflow> + Clone + Send + 'static;

    fn next_with_timeout(&self, timeout: &std::time::Duration) -> Option<usb::BufferView>;

    fn backlog(&self) -> usize;

    fn clutch(&self) -> usb::Clutch;

    fn vendor_and_product_id(&self) -> (u16, u16);

    fn serial(&self) -> String;

    fn chip_firmware_configuration(&self) -> Self::Configuration;

    fn speed(&self) -> usb::Speed;

    fn create_adapter(&self) -> Self::Adapter;

    fn temperature_celsius(&self) -> Result<TemperatureCelsius, Self::Error>;

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
                    speed: device.speed().into(),
                    serial: serial.map_err(|error| error.into()),
                });
            }
        }
        Ok(result)
    }

    fn open_serial(
        context: &rusb::Context,
        serial: &Option<&str>,
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
                                Err(_) => return None, // ignore errors to support devices that are already open
                            };
                            match serial {
                                Some(serial) => {
                                    if *serial == device_serial {
                                        let _ = handle.set_auto_detach_kernel_driver(true);
                                        Some(Ok((
                                            handle,
                                            device_vendor_and_product_id,
                                            device_serial,
                                        )))
                                    } else {
                                        None
                                    }
                                }
                                None => {
                                    let _ = handle.set_auto_detach_kernel_driver(true);
                                    Some(Ok((handle, device_vendor_and_product_id, device_serial)))
                                }
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
            None => Err(match serial {
                Some(serial) => usb::Error::Serial((*serial).to_owned()),
                None => usb::Error::Device,
            }),
        }
    }
}
