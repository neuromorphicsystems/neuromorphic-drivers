use crate::adapters;
use crate::configuration;
use crate::device;
use crate::flag;
use crate::properties;
use crate::usb;

use device::Usb;

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq, Eq)]
pub struct Biases {
    pub pr: u8,
    pub fo_p: u8,
    pub fo_n: u8,
    pub hpf: u8,
    pub diff_on: u8,
    pub diff: u8,
    pub diff_off: u8,
    pub refr: u8,
    pub reqpuy: u8,
    pub blk: u8,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq, Eq)]
pub struct RateLimiter {
    pub reference_period_us: u16,
    pub maximum_events_per_period: u32,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq, Eq)]
pub struct Configuration {
    pub biases: Biases,
    pub x_mask: [u64; 20],
    pub y_mask: [u64; 12],
    pub mask_intersection_only: bool,
    pub rate_limiter: Option<RateLimiter>,
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
}

impl From<rusb::Error> for Error {
    fn from(error: rusb::Error) -> Self {
        usb::Error::from(error).into()
    }
}
pub struct Device {
    handle: std::sync::Arc<rusb::DeviceHandle<rusb::Context>>,
    ring: usb::Ring,
    configuration_updater: configuration::Updater<Configuration>,
    vendor_and_product_id: (u16, u16),
    serial: String,
}

pub const PROPERTIES: properties::Camera<Configuration> = Device::PROPERTIES;
pub const DEFAULT_CONFIGURATION: Configuration = Device::PROPERTIES.default_configuration;
pub const DEFAULT_USB_CONFIGURATION: usb::Configuration = Device::DEFAULT_USB_CONFIGURATION;
pub fn open<IntoError, IntoWarning>(
    serial_or_bus_number_and_address: device::SerialOrBusNumberAndAddress,
    configuration: Configuration,
    usb_configuration: &usb::Configuration,
    event_loop: std::sync::Arc<usb::EventLoop>,
    flag: flag::Flag<IntoError, IntoWarning>,
) -> Result<Device, Error>
where
    IntoError: From<Error> + Clone + Send + 'static,
    IntoWarning: From<usb::Overflow> + Clone + Send + 'static,
{
    Device::open(
        serial_or_bus_number_and_address,
        configuration,
        usb_configuration,
        event_loop,
        flag,
    )
}

impl device::Usb for Device {
    type Adapter = adapters::evt3::Adapter;

    type Configuration = Configuration;

    type Error = Error;

    type Properties = properties::Camera<Self::Configuration>;

    const VENDOR_AND_PRODUCT_IDS: &'static [(u16, u16)] = &[(0x04B4, 0x00F4), (0x04B4, 0x00F5)];

    const PROPERTIES: Self::Properties = Self::Properties {
        name: "Prophesee EVK3 HD",
        width: 1280,
        height: 720,
        default_configuration: Self::Configuration {
            biases: Biases {
                pr: 0x69,
                fo_p: 0x4a,
                fo_n: 0x00,
                hpf: 0x00,
                diff_on: 0x73,
                diff: 0x50,
                diff_off: 0x34,
                refr: 0x44,
                reqpuy: 0x94,
                blk: 0x78,
            },
            x_mask: [0; 20],
            y_mask: [0; 12],
            mask_intersection_only: false,
            rate_limiter: None,
        },
    };

    const DEFAULT_USB_CONFIGURATION: usb::Configuration = usb::Configuration {
        buffer_length: 1 << 17,
        ring_length: 1 << 12,
        transfer_queue_length: 1 << 5,
        allow_dma: false,
    };

    fn read_serial(handle: &mut rusb::DeviceHandle<rusb::Context>) -> rusb::Result<Option<String>> {
        handle.claim_interface(0)?;
        let mut type_buffer = [0u8; 2];
        if handle.read_control(
            0xC0,
            0x72,
            0x00,
            0x00,
            &mut type_buffer,
            std::time::Duration::from_secs(1),
        )? != 2
        {
            return Ok(None);
        }
        if type_buffer[0] != 0x30 {
            return Ok(None);
        }
        handle.write_bulk(
            0x02,
            &[0x72, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            std::time::Duration::from_secs(1),
        )?;
        let mut buffer = vec![0u8; 16];
        handle.read_bulk(0x82, &mut buffer, std::time::Duration::from_secs(1))?;
        Ok(Some(format!(
            "{:02X}{:02X}{:02X}{:02X}",
            buffer[11], buffer[10], buffer[9], buffer[8]
        )))
    }

    fn default_configuration(&self) -> Self::Configuration {
        PROPERTIES.default_configuration
    }

    fn current_configuration(&self) -> Self::Configuration {
        self.configuration_updater.current_configuration()
    }

    fn update_configuration(&self, configuration: Self::Configuration) {
        self.configuration_updater.update(configuration);
    }

    fn open<IntoError, IntoWarning>(
        serial_or_bus_number_and_address: device::SerialOrBusNumberAndAddress,
        configuration: Self::Configuration,
        usb_configuration: &usb::Configuration,
        event_loop: std::sync::Arc<usb::EventLoop>,
        flag: flag::Flag<IntoError, IntoWarning>,
    ) -> Result<Self, Self::Error>
    where
        IntoError: From<Self::Error> + Clone + Send + 'static,
        IntoWarning: From<crate::usb::Overflow> + Clone + Send + 'static,
    {
        let (handle, vendor_and_product_id, serial) = match serial_or_bus_number_and_address {
            device::SerialOrBusNumberAndAddress::Serial(serial) => {
                Self::open_serial(event_loop.context(), serial)?
            }
            device::SerialOrBusNumberAndAddress::BusNumberAndAddress((bus_number, address)) => {
                Self::open_bus_number_and_address(event_loop.context(), bus_number, address)?
            }
            device::SerialOrBusNumberAndAddress::None => Self::open_any(event_loop.context())?,
        };
        std::thread::sleep(std::time::Duration::from_millis(150));
        request(
            &handle,
            &[0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            TIMEOUT,
        )?;
        request(
            &handle,
            &[0x55, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00],
            TIMEOUT,
        )?;
        std::thread::sleep(std::time::Duration::from_millis(5));
        Unknown7008 { value: 0x00000001 }.write(&handle)?;
        EdfPipelineControl { value: 0xffff0001 }.write(&handle)?;
        Unknown8000 { value: 0x0001e085 }.write(&handle)?;
        RoTimeBaseCtrl { value: 0x00000644 }.write(&handle)?;
        Unknown0064 { value: 0x00000001 }.write(&handle)?;
        UnknownB074 { value: 0x00000002 }.write(&handle)?;
        UnknownB078 { value: 0x000000a0 }.write(&handle)?;
        Unknown0000 { value: 0x10006442 }.write(&handle)?;
        Unknown0000 { value: 0x10001442 }.write(&handle)?;
        Unknown0000 { value: 0x10001442 }.write(&handle)?;
        UnknownB068 { value: 0x00000004 }.write(&handle)?;
        UnknownB07C { value: 0x00000003 }.write(&handle)?;
        Unknown0000 { value: 0x50001442 }.write(&handle)?;
        Unknown0000 { value: 0x5000144a }.write(&handle)?;
        Unknown0000 { value: 0x5000140a }.write(&handle)?;
        Unknown0000 { value: 0x5000640a }.write(&handle)?;
        Unknown0000 { value: 0x5000644a }.write(&handle)?;
        UnknownB080 { value: 0x00000077 }.write(&handle)?;
        UnknownB084 { value: 0x0000000f }.write(&handle)?;
        UnknownB088 { value: 0x00000037 }.write(&handle)?;
        UnknownB08C { value: 0x00000037 }.write(&handle)?;
        UnknownB090 { value: 0x000000df }.write(&handle)?;
        UnknownB094 { value: 0x00000057 }.write(&handle)?;
        UnknownB098 { value: 0x00000037 }.write(&handle)?;
        UnknownB09C { value: 0x00000067 }.write(&handle)?;
        UnknownB0A0 { value: 0x00000037 }.write(&handle)?;
        UnknownB0A4 { value: 0x0000002f }.write(&handle)?;
        UnknownB0AC { value: 0x00000028 }.write(&handle)?;
        UnknownB0CC { value: 0x00000001 }.write(&handle)?;
        UnknownB000 { value: 0x000002d8 }.write(&handle)?;
        UnknownB004 { value: 0x0000008a }.write(&handle)?;
        UnknownB01C { value: 0x00000030 }.write(&handle)?;
        UnknownB020 { value: 0x00002000 }.write(&handle)?;
        UnknownB02C { value: 0x000000ff }.write(&handle)?;
        UnknownB030 { value: 0x00003e80 }.write(&handle)?;
        UnknownB028 { value: 0x00000fa0 }.write(&handle)?;
        UnknownB040 { value: 0x00000007 }.write(&handle)?;
        UnknownA000 { value: 0x000000a1 }.write(&handle)?;
        UnknownA008 { value: 0x00002401 }.write(&handle)?;
        UnknownA004 { value: 0x000000a1 }.write(&handle)?;
        UnknownA020 { value: 0x00000160 }.write(&handle)?;
        UnknownB040 { value: 0x0000000f }.write(&handle)?;
        UnknownB004 { value: 0x0000008a }.write(&handle)?;
        UnknownB0C8 { value: 0x00000003 }.write(&handle)?;
        UnknownB044 { value: 0x00000001 }.write(&handle)?;
        UnknownB000 { value: 0x000002dd }.write(&handle)?;
        RoTimeBaseCtrl { value: 0x00000640 }.write(&handle)?;
        Unknown8000 { value: 0x0001e085 }.write(&handle)?;
        Unknown7008 { value: 0x00000001 }.write(&handle)?;
        EdfPipelineControl { value: 0x00070001 }.write(&handle)?;
        ErcReserved6000 { value: 0x00155403 }.write(&handle)?;
        StcPipelineControl { value: 0x00000005 }.write(&handle)?;
        AfkPipelineControl { value: 0x00000005 }.write(&handle)?;

        // Event Rate Controler (ERC)
        ErcReserved6000 { value: 0x00155400 }.write(&handle)?;
        match &configuration.rate_limiter {
            Some(rate_limiter) => {
                ErcInDropRateControl {
                    enable: 1,
                    reserved_1_32: 0,
                }
                .write(&handle)?;
                ErcReferencePeriod {
                    duration_us: rate_limiter.reference_period_us as u32,
                    reserved_10_32: 0,
                }
                .write(&handle)?;
                ErcTdTargetEventRate {
                    maximum_per_period: rate_limiter.maximum_events_per_period,
                    reserved_22_32: 0,
                }
                .write(&handle)?;
                ErcControl {
                    enable: 1,
                    reserved_1_32: 1,
                }
                .write(&handle)?;
            }
            None => {
                ErcInDropRateControl {
                    enable: 0,
                    reserved_1_32: 0,
                }
                .write(&handle)?;
                ErcControl {
                    enable: 0,
                    reserved_1_32: 1,
                }
                .write(&handle)?;
            }
        }
        ErcReserved602C { value: 0x00000001 }.write(&handle)?;
        for offset in 0..230 {
            ErcReserved6800 { value: 0x08080808 }
                .offset(offset)
                .write(&handle)?;
        }
        ErcReserved602C { value: 0x00000000 }.write(&handle)?;
        for offset in 0..256 {
            TDropLut {
                value: ((offset * 2 + 1) << 16) | (offset * 2),
            }
            .offset(offset)
            .write(&handle)?;
        }
        ErcTDroppingControl {
            enable: configuration.rate_limiter.is_some() as u32,
            reserved_1_32: 0,
        }
        .write(&handle)?;
        ErcHDroppingControl {
            enable: 0,
            reserved_1_32: 0,
        }
        .write(&handle)?;
        ErcVDroppingControl {
            enable: 0,
            reserved_1_32: 0,
        }
        .write(&handle)?;
        ErcReserved6000 { value: 0x00155401 }.write(&handle)?;
        RoReadoutCtrl { value: 0x00000208 }.write(&handle)?;
        Unknown7008 { value: 0x00000001 }.write(&handle)?;
        EdfPipelineControl { value: 0x00070001 }.write(&handle)?;
        Unknown8000 { value: 0x0001e085 }.write(&handle)?;
        RoTimeBaseCtrl { value: 0x00000644 }.write(&handle)?;
        RoiCtrl {
            reserved_0_1: 0,
            td_enable: 1,
            reserved_2_5: 0,
            td_shadow_trigger: 0,
            td_roni_n_en: 1,
            reserved_7_10: 0,
            td_rstn: 0,
            reserved_11_32: 0x1e000a,
        }
        .write(&handle)?;
        Unknown002C { value: 0x0022c324 }.write(&handle)?;
        UnknownA000 { value: 0x000002a1 }.write(&handle)?;
        UnknownA000 { value: 0x000002a1 }.write(&handle)?;
        UnknownA008 { value: 0x00082401 }.write(&handle)?;
        UnknownA004 { value: 0x000002a1 }.write(&handle)?;
        UnknownA004 { value: 0x000002a1 }.write(&handle)?;
        UnknownA020 { value: 0x00000160 }.write(&handle)?;
        UnknownA020 { value: 0x00000160 }.write(&handle)?;
        UnknownA008 { value: 0x00082401 }.write(&handle)?;
        Unknown004C { value: 0x00007141 }.write(&handle)?;
        AdcMiscCtrl { value: 0x00000210 }.write(&handle)?;
        Unknown0008 { value: 0x60000000 }.write(&handle)?;
        Unknown1104 { value: 0x00000001 }.write(&handle)?;
        UnknownA010 { value: 0x0000a06b }.write(&handle)?;
        BgenCtrl { value: 0x00000004 }.write(&handle)?;
        UnknownA010 { value: 0x0180a063 }.write(&handle)?;
        UnknownA00C { value: 0x00000400 }.write(&handle)?;
        UnknownA00C { value: 0x00000401 }.write(&handle)?;
        UnknownA00C { value: 0x00020401 }.write(&handle)?;
        Unknown0070 { value: 0x00400000 }.write(&handle)?;
        Unknown006C { value: 0x0ee47117 }.write(&handle)?;
        Unknown006C { value: 0x0ee4711f }.write(&handle)?;
        Unknown0070 { value: 0x00480000 }.write(&handle)?;
        update_configuration(&handle, None, &configuration)?;
        BgenCtrl { value: 0x00000005 }.write(&handle)?;
        Unknown002C { value: 0x0022c724 }.write(&handle)?;
        Unknown0018 { value: 0x00000200 }.write(&handle)?;
        request(
            &handle,
            &[0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            TIMEOUT,
        )?;
        UnknownB000 { value: 0x000002dd }.write(&handle)?;
        RoCtrl { value: 0x00000000 }.write(&handle)?;
        RoTimeBaseCtrl { value: 0x00000645 }.write(&handle)?;
        Unknown002C { value: 0x0022c724 }.write(&handle)?;
        RoiCtrl {
            reserved_0_1: 0,
            td_enable: 1,
            reserved_2_5: 0,
            td_shadow_trigger: 0,
            td_roni_n_en: (!configuration.mask_intersection_only) as u32,
            reserved_7_10: 0,
            td_rstn: 1,
            reserved_11_32: 0x1e000a,
        }
        .write(&handle)?;

        let handle = std::sync::Arc::new(handle);
        let error_flag = flag.clone();
        let warning_flag = flag.clone();
        Ok(Device {
            handle: handle.clone(),
            ring: usb::Ring::new(
                handle.clone(),
                usb_configuration,
                move |usb_error| {
                    error_flag.store_error_if_not_set(Self::Error::from(usb_error));
                },
                move |overflow| {
                    warning_flag.store_warning_if_not_set(overflow);
                },
                event_loop,
                usb::TransferType::Bulk {
                    endpoint: 1 | libusb1_sys::constants::LIBUSB_ENDPOINT_IN,
                    timeout: std::time::Duration::ZERO, // @DEV this was 100 ms but the EVK4 uses 0, does this matter?
                },
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
        })
    }

    fn next_with_timeout(&self, timeout: &std::time::Duration) -> Option<usb::BufferView> {
        self.ring.next_with_timeout(timeout)
    }

    fn backlog(&self) -> usize {
        self.ring.backlog()
    }

    fn clutch(&self) -> usb::Clutch {
        self.ring.clutch()
    }

    fn vendor_and_product_id(&self) -> (u16, u16) {
        self.vendor_and_product_id
    }

    fn serial(&self) -> String {
        self.serial.clone()
    }

    fn chip_firmware_configuration(&self) -> Self::Configuration {
        Self::PROPERTIES.default_configuration.clone()
    }

    fn bus_number(&self) -> u8 {
        self.handle.device().bus_number()
    }

    fn address(&self) -> u8 {
        self.handle.device().address()
    }

    fn speed(&self) -> usb::Speed {
        self.handle.device().speed().into()
    }

    fn create_adapter(&self) -> Self::Adapter {
        Self::Adapter::from_dimensions(Self::PROPERTIES.width, Self::PROPERTIES.height)
    }
}

impl Drop for Device {
    fn drop(&mut self) {
        let _ = RoiCtrl {
            reserved_0_1: 0,
            td_enable: 1,
            reserved_2_5: 0,
            td_shadow_trigger: 0,
            td_roni_n_en: 1,
            reserved_7_10: 0,
            td_rstn: 0,
            reserved_11_32: 0x1e000a,
        }
        .write(&self.handle);
        let _ = Unknown002C { value: 0x0022C324 }.write(&self.handle);
        let _ = AfkPipelineControl { value: 0x00000002 }.write(&self.handle);
        let _ = RoCtrl { value: 0x00000002 }.write(&self.handle);
        let _ = AfkPipelineControl { value: 0x00000005 }.write(&self.handle);
        std::thread::sleep(std::time::Duration::from_millis(1));
        let _ = RoTimeBaseCtrl { value: 0x00000644 }.write(&self.handle);
        let _ = UnknownB000 { value: 0x000002D8 }.write(&self.handle);
        std::thread::sleep(std::time::Duration::from_micros(300));
        let _ = Unknown0070 { value: 0x00400000 }.write(&self.handle);
        let _ = Unknown006C { value: 0x0EE47114 }.write(&self.handle);
        std::thread::sleep(std::time::Duration::from_micros(500));
        let _ = UnknownA00C { value: 0x00000400 }.write(&self.handle);
        std::thread::sleep(std::time::Duration::from_micros(500));
        let _ = UnknownA010 { value: 0x00008068 }.write(&self.handle);
        std::thread::sleep(std::time::Duration::from_micros(200));
        let _ = Unknown1104 { value: 0x00000000 }.write(&self.handle);
        std::thread::sleep(std::time::Duration::from_micros(200));
        let _ = UnknownA020 { value: 0x00000060 }.write(&self.handle);
        std::thread::sleep(std::time::Duration::from_micros(200));
        let _ = UnknownA004 { value: 0x000002A0 }.write(&self.handle);
        std::thread::sleep(std::time::Duration::from_micros(200));
        let _ = UnknownA008 { value: 0x00002400 }.write(&self.handle);
        std::thread::sleep(std::time::Duration::from_micros(200));
        let _ = UnknownA000 { value: 0x000002A0 }.write(&self.handle);
        let _ = UnknownB044 { value: 0x00000002 }.write(&self.handle);
        let _ = UnknownB004 { value: 0x0000000a }.write(&self.handle);
        let _ = UnknownB040 { value: 0x0000000e }.write(&self.handle);
        let _ = UnknownB0C8 { value: 0x00000000 }.write(&self.handle);
        let _ = UnknownB040 { value: 0x00000006 }.write(&self.handle);
        let _ = UnknownB040 { value: 0x00000004 }.write(&self.handle);
        let _ = Unknown0000 { value: 0x50006442 }.write(&self.handle);
        let _ = Unknown0000 { value: 0x10006442 }.write(&self.handle);
        let _ = UnknownB07C { value: 0x00000000 }.write(&self.handle);
    }
}

const TIMEOUT: std::time::Duration = std::time::Duration::from_millis(100);

fn request(
    handle: &rusb::DeviceHandle<rusb::Context>,
    buffer: &[u8],
    timeout: std::time::Duration,
) -> Result<Vec<u8>, Error> {
    let written = handle.write_bulk(0x02, buffer, timeout)?;
    if buffer.len() != written {
        return Err(Error::ShortWrite {
            requested: buffer.len(),
            written,
        });
    }
    let mut buffer = vec![0; 16];
    let read = handle.read_bulk(0x82, &mut buffer, timeout)?;
    buffer.truncate(read);
    Ok(buffer)
}

struct BiasConfiguration {
    vdac_ctl: u32,
    buf_stg: u32,
    ibtype_sel: u32,
    mux_sel: u32,
    mux_en: u32,
    vdac_en: u32,
    buf_en: u32,
    idac_en: u32,
    single: u32,
}

macro_rules! update_bias {
    ($name:ident, $register:ident, $configuration:expr, $handle:ident, $previous_biases:ident, $biases:expr) => {
        if match $previous_biases {
            Some(previous_biases) => previous_biases.$name != $biases.$name,
            None => true,
        } {
            let configuration = $configuration;
            $register {
                idac_ctl: $biases.$name as u32,
                vdac_ctl: configuration.vdac_ctl,
                buf_stg: configuration.buf_stg,
                ibtype_sel: configuration.ibtype_sel,
                mux_sel: configuration.mux_sel,
                mux_en: configuration.mux_en,
                vdac_en: configuration.vdac_en,
                buf_en: configuration.buf_en,
                idac_en: configuration.idac_en,
                reserved_25_28: 0,
                single: configuration.single,
            }
            .write($handle)?;
        }
    };
}

fn update_configuration(
    handle: &rusb::DeviceHandle<rusb::Context>,
    previous_configuration: Option<&Configuration>,
    configuration: &Configuration,
) -> Result<(), Error> {
    {
        let previous_biases = previous_configuration.map(|configuration| &configuration.biases);
        update_bias!(
            pr,
            BiasPr,
            BiasConfiguration {
                vdac_ctl: 0xc4,
                buf_stg: 1,
                ibtype_sel: 0,
                mux_sel: 0,
                mux_en: 1,
                vdac_en: 0,
                buf_en: 1,
                idac_en: 1,
                single: 0,
            },
            handle,
            previous_biases,
            configuration.biases
        );
        update_bias!(
            fo_p,
            BiasFoP,
            BiasConfiguration {
                vdac_ctl: 0xe8,
                buf_stg: 1,
                ibtype_sel: 0,
                mux_sel: 0,
                mux_en: 1,
                vdac_en: 0,
                buf_en: 1,
                idac_en: 1,
                single: 0,
            },
            handle,
            previous_biases,
            configuration.biases
        );
        update_bias!(
            fo_n,
            BiasFoN,
            BiasConfiguration {
                vdac_ctl: 0x00,
                buf_stg: 1,
                ibtype_sel: 0,
                mux_sel: 0,
                mux_en: 1,
                vdac_en: 0,
                buf_en: 0,
                idac_en: 1,
                single: 0,
            },
            handle,
            previous_biases,
            configuration.biases
        );
        update_bias!(
            hpf,
            BiasHpf,
            BiasConfiguration {
                vdac_ctl: 0xff,
                buf_stg: 1,
                ibtype_sel: 0,
                mux_sel: 0,
                mux_en: 1,
                vdac_en: 0,
                buf_en: 1,
                idac_en: 1,
                single: 0,
            },
            handle,
            previous_biases,
            configuration.biases
        );
        update_bias!(
            diff_on,
            BiasDiffOn,
            BiasConfiguration {
                vdac_ctl: 0x63,
                buf_stg: 1,
                ibtype_sel: 0,
                mux_sel: 0,
                mux_en: 1,
                vdac_en: 0,
                buf_en: 1,
                idac_en: 1,
                single: 0,
            },
            handle,
            previous_biases,
            configuration.biases
        );
        update_bias!(
            diff,
            BiasDiff,
            BiasConfiguration {
                vdac_ctl: 0x50,
                buf_stg: 1,
                ibtype_sel: 0,
                mux_sel: 0,
                mux_en: 1,
                vdac_en: 0,
                buf_en: 1,
                idac_en: 1,
                single: 0,
            },
            handle,
            previous_biases,
            configuration.biases
        );
        update_bias!(
            diff_off,
            BiasDiffOff,
            BiasConfiguration {
                vdac_ctl: 0x37,
                buf_stg: 1,
                ibtype_sel: 0,
                mux_sel: 0,
                mux_en: 1,
                vdac_en: 0,
                buf_en: 1,
                idac_en: 1,
                single: 0,
            },
            handle,
            previous_biases,
            configuration.biases
        );
        update_bias!(
            refr,
            BiasRefr,
            BiasConfiguration {
                vdac_ctl: 0xcd,
                buf_stg: 1,
                ibtype_sel: 1,
                mux_sel: 0,
                mux_en: 1,
                vdac_en: 0,
                buf_en: 1,
                idac_en: 1,
                single: 0,
            },
            handle,
            previous_biases,
            configuration.biases
        );
        update_bias!(
            reqpuy,
            BiasReqpuy,
            BiasConfiguration {
                vdac_ctl: 0x8a,
                buf_stg: 1,
                ibtype_sel: 1,
                mux_sel: 0,
                mux_en: 1,
                vdac_en: 0,
                buf_en: 1,
                idac_en: 1,
                single: 0,
            },
            handle,
            previous_biases,
            configuration.biases
        );
        update_bias!(
            blk,
            BiasBlk,
            BiasConfiguration {
                vdac_ctl: 0x92,
                buf_stg: 1,
                ibtype_sel: 0,
                mux_sel: 0,
                mux_en: 1,
                vdac_en: 0,
                buf_en: 1,
                idac_en: 1,
                single: 0,
            },
            handle,
            previous_biases,
            configuration.biases
        );
    }
    if match previous_configuration {
        Some(previous_configuration) => {
            previous_configuration.x_mask != configuration.x_mask
                || previous_configuration.y_mask != configuration.y_mask
                || previous_configuration.mask_intersection_only
                    != configuration.mask_intersection_only
        }
        None => true,
    } {
        for offset in 0..((configuration.x_mask.len() as u32) * 2) {
            let register = TdRoiX {
                value: if (offset % 2) == 0 {
                    (configuration.x_mask[(offset / 2) as usize] & 0xffffffffu64) as u32
                } else {
                    ((configuration.x_mask[(offset / 2) as usize] & 0xffffffff00000000u64) >> 32)
                        as u32
                },
            }
            .offset(offset);
            register.write(handle)?;
        }
        for offset in 0..((configuration.y_mask.len() as u32) * 2 - 1) {
            let register = TdRoiY {
                value: if (offset % 2) == 0 {
                    let [byte2, byte3, _, _, _, _, _, _] = configuration.y_mask
                        [configuration.y_mask.len() - 1 - (offset / 2) as usize]
                        .to_le_bytes();
                    if offset < (configuration.y_mask.len() as u32) * 2 - 2 {
                        let [_, _, _, _, _, _, byte0, byte1] = configuration.y_mask
                            [configuration.y_mask.len() - 2 - (offset / 2) as usize]
                            .to_le_bytes();
                        u32::from_le_bytes([
                            byte3.reverse_bits(),
                            byte2.reverse_bits(),
                            byte1.reverse_bits(),
                            byte0.reverse_bits(),
                        ])
                    } else {
                        u32::from_le_bytes([byte3.reverse_bits(), byte2.reverse_bits(), 0xff, 0x00])
                    }
                } else {
                    let [_, _, byte0, byte1, byte2, byte3, _, _] = configuration.y_mask
                        [configuration.y_mask.len() - 2 - (offset / 2) as usize]
                        .to_le_bytes();
                    u32::from_le_bytes([
                        byte3.reverse_bits(),
                        byte2.reverse_bits(),
                        byte1.reverse_bits(),
                        byte0.reverse_bits(),
                    ])
                },
            }
            .offset(offset);
            register.write(handle)?;
        }
        RoiCtrl {
            reserved_0_1: 0,
            td_enable: 1,
            reserved_2_5: 0,
            td_shadow_trigger: 1,
            td_roni_n_en: (!configuration.mask_intersection_only) as u32,
            reserved_7_10: 0,
            td_rstn: previous_configuration.is_some() as u32,
            reserved_11_32: 0x1e000a,
        }
        .write(handle)?;
    }
    Ok(())
}

struct ConfigurationUpdaterContext<IntoError, IntoWarning>
where
    IntoError: From<Error> + Clone + Send,
    IntoWarning: From<crate::usb::Overflow> + Clone + Send,
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

    fn offset(&self, registers: u32) -> RuntimeRegister;

    #[allow(dead_code)]
    fn read(&self, handle: &rusb::DeviceHandle<rusb::Context>) -> Result<u32, Error> {
        let address = self.address();
        let buffer = [
            0x02,
            0x01,
            0x01,
            0x00,
            0x0c,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            (address & 0xff) as u8,
            ((address >> 8) & 0xff) as u8,
            ((address >> 16) & 0xff) as u8,
            ((address >> 24) & 0xff) as u8,
            0x01,
            0x00,
            0x00,
            0x00,
        ];
        let result = request(handle, &buffer, std::time::Duration::from_millis(1000))?;
        if result.len() != buffer.len() {
            return Err(Error::RegisterReadShortResponse(address));
        }
        if result[0..16] != buffer[0..16] {
            return Err(Error::RegisterReadMismatch(address));
        }
        // unwrap: slice has the right number of bytes
        Ok(u32::from_le_bytes(result[16..20].try_into().unwrap()))
    }

    fn write(&self, handle: &rusb::DeviceHandle<rusb::Context>) -> Result<(), Error> {
        let address = self.address();
        let value = self.value();
        request(
            handle,
            &[
                0x56,
                0x00,
                0x00,
                0x00,
                (address & 0xff) as u8,
                ((address >> 8) & 0xff) as u8,
                ((address >> 16) & 0xff) as u8,
                ((address >> 24) & 0xff) as u8,
                (value & 0xff) as u8,
                ((value >> 8) & 0xff) as u8,
                ((value >> 16) & 0xff) as u8,
                ((value >> 24) & 0xff) as u8,
            ],
            TIMEOUT,
        )?;
        Ok(())
    }
}

impl Register for RuntimeRegister {
    fn address(&self) -> u32 {
        self.address
    }
    fn value(&self) -> u32 {
        self.value
    }
    fn offset(&self, registers: u32) -> RuntimeRegister {
        RuntimeRegister {
            address: self.address + registers * 4,
            value: self.value,
        }
    }
}

macro_rules! register {
    ($name:ident, $address:literal, {$($subname:ident: $substart:literal..$subend:literal),+ $(,)?}) => {
        #[allow(dead_code)]
        #[derive(Default)]
        struct $name {
            $(
                $subname: u32,
            )+
        }
        $(
            const _: () = assert!($substart < $subend);
        )+
        impl Register for $name {
            fn address(&self) -> u32 {
                $address
            }
            fn value(&self) -> u32 {
                0u32
                $(
                    | ((self.$subname & (((1u64 << ($subend - $substart)) - 1) as u32)) << $substart)
                )+
            }
            fn offset(&self, registers: u32) -> RuntimeRegister {
                RuntimeRegister  {
                    address: $address + registers * 4,
                    value: self.value(),
                }
            }
        }
    };
}

register! { Unknown0000, 0x0000, { value: 0..32 } }
register! { Unknown0008, 0x0008, { value: 0..32 } }
register! { Unknown0018, 0x0018, { value: 0..32 } }
register! { Unknown002C, 0x002C, { value: 0..32 } }
register! { Unknown004C, 0x004C, { value: 0..32 } }
register! { Unknown0064, 0x0064, { value: 0..32 } }
register! { Unknown006C, 0x006C, { value: 0..32 } }
register! { Unknown0070, 0x0070, { value: 0..32 } }
register! { Unknown1104, 0x1104, { value: 0..32 } }
register! { UnknownA000, 0xA000, { value: 0..32 } }
register! { UnknownA004, 0xA004, { value: 0..32 } }
register! { UnknownA008, 0xA008, { value: 0..32 } }
register! { UnknownA00C, 0xA00C, { value: 0..32 } }
register! { UnknownA010, 0xA010, { value: 0..32 } }
register! { UnknownA020, 0xA020, { value: 0..32 } }
register! { UnknownB000, 0xB000, { value: 0..32 } }
register! { UnknownB004, 0xB004, { value: 0..32 } }
register! { UnknownB01C, 0xB01C, { value: 0..32 } }
register! { UnknownB020, 0xB020, { value: 0..32 } }
register! { UnknownB028, 0xB028, { value: 0..32 } }
register! { UnknownB02C, 0xB02C, { value: 0..32 } }
register! { UnknownB030, 0xB030, { value: 0..32 } }
register! { UnknownB040, 0xB040, { value: 0..32 } }
register! { UnknownB044, 0xB044, { value: 0..32 } }
register! { UnknownB068, 0xB068, { value: 0..32 } }
register! { UnknownB074, 0xB074, { value: 0..32 } }
register! { UnknownB078, 0xB078, { value: 0..32 } }
register! { UnknownB07C, 0xB07C, { value: 0..32 } }
register! { UnknownB080, 0xB080, { value: 0..32 } }
register! { UnknownB084, 0xB084, { value: 0..32 } }
register! { UnknownB088, 0xB088, { value: 0..32 } }
register! { UnknownB08C, 0xB08C, { value: 0..32 } }
register! { UnknownB090, 0xB090, { value: 0..32 } }
register! { UnknownB094, 0xB094, { value: 0..32 } }
register! { UnknownB098, 0xB098, { value: 0..32 } }
register! { UnknownB09C, 0xB09C, { value: 0..32 } }
register! { UnknownB0A0, 0xB0A0, { value: 0..32 } }
register! { UnknownB0A4, 0xB0A4, { value: 0..32 } }
register! { UnknownB0AC, 0xB0AC, { value: 0..32 } }
register! { UnknownB0C8, 0xB0C8, { value: 0..32 } }
register! { UnknownB0CC, 0xB0CC, { value: 0..32 } }
register! { Unknown7008, 0x7008, { value: 0..32 } }
register! { Unknown8000, 0x8000, { value: 0..32 } }
register! { RoiCtrl, 0x0004, {
    reserved_0_1: 0..1,
    td_enable: 1..2,
    reserved_2_5: 2..5,
    td_shadow_trigger: 5..6,
    td_roni_n_en: 6..7,
    reserved_7_10: 7..10,
    td_rstn: 10..11,
    reserved_11_32: 11..32,
} }
register! { LifoCtrl, 0x000C, { value: 0..32 } }
register! { LifoStatus, 0x0010, { value: 0..32 } }
register! { Reserved0014, 0x0014, { value: 0..32 } }
register! { RefractoryCtrl, 0x0020, { value: 0..32 } }
register! { RoiWinCtrl, 0x0034, { value: 0..32 } }
register! { RoiWinStartAddr, 0x0038, { value: 0..32 } }
register! { RoiWinEndAddr, 0x003C, { value: 0..32 } }
register! { DigPad2Ctrl, 0x0044, { value: 0..32 } }
register! { AdcControl, 0x004C, { value: 0..32 } }
register! { AdcStatus, 0x0050, { value: 0..32 } }
register! { AdcMiscCtrl, 0x0054, { value: 0..32 } }
register! { TempCtrl, 0x005C, { value: 0..32 } }
register! { IphMirrCtrl, 0x0074, { value: 0..32 } }
register! { ReqyQmonCtrl, 0x0088, { value: 0..32 } }
register! { ReqyQmonStatus, 0x008C, { value: 0..32 } }
register! { BiasPr, 0x1000, {
    idac_ctl: 0..8,
    vdac_ctl: 8..16,
    buf_stg: 16..19,
    ibtype_sel: 19..20,
    mux_sel: 20..21,
    mux_en: 21..22,
    vdac_en: 22..23,
    buf_en: 23..24,
    idac_en: 24..25,
    reserved_25_28: 25..28,
    single: 28..29,
} }
register! { BiasFoP, 0x1004, {
    idac_ctl: 0..8,
    vdac_ctl: 8..16,
    buf_stg: 16..19,
    ibtype_sel: 19..20,
    mux_sel: 20..21,
    mux_en: 21..22,
    vdac_en: 22..23,
    buf_en: 23..24,
    idac_en: 24..25,
    reserved_25_28: 25..28,
    single: 28..29,
} }
register! { BiasFoN, 0x1008, {
    idac_ctl: 0..8,
    vdac_ctl: 8..16,
    buf_stg: 16..19,
    ibtype_sel: 19..20,
    mux_sel: 20..21,
    mux_en: 21..22,
    vdac_en: 22..23,
    buf_en: 23..24,
    idac_en: 24..25,
    reserved_25_28: 25..28,
    single: 28..29,
} }
register! { BiasHpf, 0x100C, {
    idac_ctl: 0..8,
    vdac_ctl: 8..16,
    buf_stg: 16..19,
    ibtype_sel: 19..20,
    mux_sel: 20..21,
    mux_en: 21..22,
    vdac_en: 22..23,
    buf_en: 23..24,
    idac_en: 24..25,
    reserved_25_28: 25..28,
    single: 28..29,
} }
register! { BiasDiffOn, 0x1010, {
    idac_ctl: 0..8,
    vdac_ctl: 8..16,
    buf_stg: 16..19,
    ibtype_sel: 19..20,
    mux_sel: 20..21,
    mux_en: 21..22,
    vdac_en: 22..23,
    buf_en: 23..24,
    idac_en: 24..25,
    reserved_25_28: 25..28,
    single: 28..29,
} }
register! { BiasDiff, 0x1014, {
    idac_ctl: 0..8,
    vdac_ctl: 8..16,
    buf_stg: 16..19,
    ibtype_sel: 19..20,
    mux_sel: 20..21,
    mux_en: 21..22,
    vdac_en: 22..23,
    buf_en: 23..24,
    idac_en: 24..25,
    reserved_25_28: 25..28,
    single: 28..29,
} }
register! { BiasDiffOff, 0x1018, {
    idac_ctl: 0..8,
    vdac_ctl: 8..16,
    buf_stg: 16..19,
    ibtype_sel: 19..20,
    mux_sel: 20..21,
    mux_en: 21..22,
    vdac_en: 22..23,
    buf_en: 23..24,
    idac_en: 24..25,
    reserved_25_28: 25..28,
    single: 28..29,
} }
register! { BiasRefr, 0x1020, {
    idac_ctl: 0..8,
    vdac_ctl: 8..16,
    buf_stg: 16..19,
    ibtype_sel: 19..20,
    mux_sel: 20..21,
    mux_en: 21..22,
    vdac_en: 22..23,
    buf_en: 23..24,
    idac_en: 24..25,
    reserved_25_28: 25..28,
    single: 28..29,
} }
register! { BiasReqpuy, 0x1040, {
    idac_ctl: 0..8,
    vdac_ctl: 8..16,
    buf_stg: 16..19,
    ibtype_sel: 19..20,
    mux_sel: 20..21,
    mux_en: 21..22,
    vdac_en: 22..23,
    buf_en: 23..24,
    idac_en: 24..25,
    reserved_25_28: 25..28,
    single: 28..29,
} }
register! { BiasBlk, 0x104C, {
    idac_ctl: 0..8,
    vdac_ctl: 8..16,
    buf_stg: 16..19,
    ibtype_sel: 19..20,
    mux_sel: 20..21,
    mux_en: 21..22,
    vdac_en: 22..23,
    buf_en: 23..24,
    idac_en: 24..25,
    reserved_25_28: 25..28,
    single: 28..29,
} }
register! { BgenCtrl, 0x1100, { value: 0..32 } }
register! { TdRoiX, 0x2000, { value: 0..32 } }
register! { TdRoiY, 0x4000, { value: 0..32 } }
register! { ErcReserved6000, 0x6000, { value: 0..32 } }
register! { ErcInDropRateControl, 0x6004, {
    enable: 0..1,
    reserved_1_32: 1..32,
} }
register! { ErcReferencePeriod, 0x6008, {
    duration_us: 0..10,
    reserved_10_32: 10..32,
} }
register! { ErcTdTargetEventRate, 0x600C, {
    maximum_per_period: 0..22,
    reserved_22_32: 22..32,
} }
register! { ErcControl, 0x6028, {
    enable: 0..1,
    reserved_1_32: 1..32,
} }
register! { ErcReserved602C, 0x602C, { value: 0..32 } }
register! { ErcTDroppingControl, 0x6050, {
    enable: 0..1,
    reserved_1_32: 1..32,
} }
register! { ErcHDroppingControl, 0x6060, {
    enable: 0..1,
    reserved_1_32: 1..32,
} }
register! { ErcVDroppingControl, 0x6070, {
    enable: 0..1,
    reserved_1_32: 1..32,
} }
register! { HDropLut, 0x6080, { value: 0..32 } }
register! { TDropLut, 0x6400, { value: 0..32 } }
register! { ErcReserved6800, 0x6800, { value: 0..32 } }
register! { EdfPipelineControl, 0x7000, { value: 0..32 } }
register! { EdfReserved7004, 0x7004, { value: 0..32 } }
register! { RoReadoutCtrl, 0x9000, { value: 0..32 } }
register! { RoTimeBaseCtrl, 0x9008, { value: 0..32 } }
register! { RoDigCtrl, 0x900C, { value: 0..32 } }
register! { RoDigStartPos, 0x9010, { value: 0..32 } }
register! { RoDigEndPos, 0x9014, { value: 0..32 } }
register! { RoCtrl, 0x9028, { value: 0..32 } }
register! { RoAreaX0Addr, 0x902C, { value: 0..32 } }
register! { RoAreaX1Addr, 0x9030, { value: 0..32 } }
register! { RoAreaX2Addr, 0x9034, { value: 0..32 } }
register! { RoAreaX3Addr, 0x9038, { value: 0..32 } }
register! { RoAreaX4Addr, 0x903C, { value: 0..32 } }
register! { RoAreaY0Addr, 0x9040, { value: 0..32 } }
register! { RoAreaY1Addr, 0x9044, { value: 0..32 } }
register! { RoAreaY2Addr, 0x9048, { value: 0..32 } }
register! { RoAreaY3Addr, 0x904C, { value: 0..32 } }
register! { RoAreaY4Addr, 0x9050, { value: 0..32 } }
register! { RoCounterCtrl, 0x9054, { value: 0..32 } }
register! { RoCounterTimerThreshold, 0x9058, { value: 0..32 } }
register! { RoDigitalMaskPixel00, 0x9100, { value: 0..32 } }
register! { RoDigitalMaskPixel63, 0x91FC, { value: 0..32 } }
register! { RoAreaCnt00, 0x9200, { value: 0..32 } }
register! { RoAreaCnt15, 0x923C, { value: 0..32 } }
register! { AfkPipelineControl, 0xC000, { value: 0..32 } }
register! { AfkReservedC004, 0xC004, { value: 0..32 } }
register! { AfkFilterPeriod, 0xC008, { value: 0..32 } }
register! { AfkInvalidation, 0xC0C0, { value: 0..32 } }
register! { AfkInitialization, 0xC0C4, { value: 0..32 } }
register! { StcPipelineControl, 0xD000, { value: 0..32 } }
register! { StcParam, 0xD004, { value: 0..32 } }
register! { StcTrailParam, 0xD008, { value: 0..32 } }
register! { StcTimestamping, 0xD00C, { value: 0..32 } }
register! { StcReservedD0C0, 0xD0C0, { value: 0..32 } }
register! { StcInitialization, 0xD0C4, { value: 0..32 } }
