use crate::adapters;
use crate::configuration;
use crate::device;
use crate::flag;
use crate::properties;
use crate::usb;

use device::Usb;

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq, Eq)]
pub struct Biases {
    localbufbn: u16,    // in [0, 2047]
    padfollbn: u16,     // in [0, 2047]
    diffbn: u16,        // in [0, 2047]
    onbn: u16,          // in [0, 2047]
    offbn: u16,         // in [0, 2047]
    pixinvbn: u16,      // in [0, 2047]
    prbp: u16,          // in [0, 2047]
    prsfbp: u16,        // in [0, 2047]
    refrbp: u16,        // in [0, 2047]
    readoutbufbp: u16,  // in [0, 2047]
    apsrosfbn: u16,     // in [0, 2047]
    adccompbp: u16,     // in [0, 2047]
    colsellowbn: u16,   // in [0, 2047]
    dacbufbp: u16,      // in [0, 2047]
    lcoltimeoutbn: u16, // in [0, 2047]
    aepdbn: u16,        // in [0, 2047]
    aepuxbp: u16,       // in [0, 2047]
    aepuybp: u16,       // in [0, 2047]
    ifrefrbn: u16,      // in [0, 2047]
    ifthrbn: u16,       // in [0, 2047]
    biasbuffer: u16,    // in [0, 2047]
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq, Eq)]
pub struct RegionOfInterest {
    left: u16,
    top: u16,
    width: u16,
    height: u16,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq, Eq)]
pub struct ActivityFilter {
    mask_isolated_enable: bool,
    mask_isolated_tau: u32, // in 250 µs increments
    refractory_period_enable: bool,
    refractory_period_tau: u32, // in 250 µs increments
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq, Eq)]
pub enum PolarityFilter {
    Disabled = 0,
    Flatten = 1, // all the polarities are set to OFF
    MaskOn = 2,  // implies flatten
    MaskOff = 3,
    MaskOffFlatten = 4,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq, Eq)]
pub struct Configuration {
    pub biases: Biases,
    pub region_of_interest: RegionOfInterest,
    pub pixel_mask: [u32; 8],
    pub activity_filter: ActivityFilter,
    pub skip_events_every: u32,
    pub polarity_filter: PolarityFilter,
    pub exposure_us: u32,
    pub frame_interval_us: u32,
}

pub struct Device {
    handle: std::sync::Arc<rusb::DeviceHandle<rusb::Context>>,
    ring: usb::Ring,
    configuration_updater: configuration::Updater<Configuration>,
    vendor_and_product_id: (u16, u16),
    serial: String,
}

#[derive(thiserror::Error, Debug, Clone)]
pub enum Error {
    #[error(transparent)]
    Usb(#[from] usb::Error),

    #[error("Firmware version not supported (the software expects version 0.6 or later, the camera uses {minor}.{subminor})")]
    FirmwareVersion { minor: u8, subminor: u8 },

    #[error("Logic version not supported (the software expects version 18, the camera uses {0})")]
    LogicVersion(u32),

    #[error("Logic patch not supported (the software expects version 1, the camera uses {0})")]
    LogicPatch(u32),

    #[error("Short SPI read to {module_address:#04X}:{parameter_address:#04X} (expected {expected} bytes, read {read} bytes)")]
    ShortRead {
        module_address: u16,
        parameter_address: u16,
        expected: usize,
        read: usize,
    },

    #[error("Short SPI write to {module_address:#04X}:{parameter_address:#04X} (expected {expected} bytes, wrote {wrote} bytes)")]
    ShortWrite {
        module_address: u16,
        parameter_address: u16,
        expected: usize,
        wrote: usize,
    },

    #[error("{0} is not implemented for the DAVIS 346")]
    NotImplemented(String),
}

impl From<rusb::Error> for Error {
    fn from(error: rusb::Error) -> Self {
        usb::Error::from(error).into()
    }
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
    type Adapter = adapters::davis346::Adapter;

    type Configuration = Configuration;

    type Error = Error;

    type Properties = properties::Camera<Self::Configuration>;

    const VENDOR_AND_PRODUCT_IDS: &'static [(u16, u16)] = &[(0x152A, 0x841A)];

    const PROPERTIES: Self::Properties = Self::Properties {
        name: "iniVation DAVIS 346",
        width: 346,
        height: 260,
        default_configuration: Self::Configuration {
            biases: Biases {
                localbufbn: 1444,
                padfollbn: 2007,
                diffbn: 1063,
                onbn: 1535,
                offbn: 1025,
                pixinvbn: 1680,
                prbp: 570,
                prsfbp: 272,
                refrbp: 1049,
                readoutbufbp: 1556,
                apsrosfbn: 1755,
                adccompbp: 1300,
                colsellowbn: 1,
                dacbufbp: 1596,
                lcoltimeoutbn: 1329,
                aepdbn: 1627,
                aepuxbp: 1104,
                aepuybp: 1944,
                ifrefrbn: 1535,
                ifthrbn: 1535,
                biasbuffer: 1534,
            },
            pixel_mask: [0; 8],
            region_of_interest: RegionOfInterest {
                left: 0,
                top: 0,
                width: 346,
                height: 260,
            },
            activity_filter: ActivityFilter {
                mask_isolated_enable: true,
                mask_isolated_tau: 8,
                refractory_period_enable: false,
                refractory_period_tau: 1,
            },
            polarity_filter: PolarityFilter::Disabled,
            skip_events_every: 0,
            exposure_us: 4000,
            frame_interval_us: 40000,
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
        let descriptor = handle.device().device_descriptor()?;
        if let Some(serial_number_string_index) = descriptor.serial_number_string_index() {
            Ok(Some(handle.read_string_descriptor_ascii(
                serial_number_string_index,
            )?))
        } else {
            Ok(Some("00000000".to_owned()))
        }
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
        IntoWarning: From<usb::Overflow> + Clone + Send + 'static,
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
        let device_version = handle.device().device_descriptor()?.device_version();
        if device_version.minor() == 0 && device_version.sub_minor() < 6 {
            return Err(Self::Error::FirmwareVersion {
                minor: device_version.minor(),
                subminor: device_version.sub_minor(),
            });
        }
        let logic_version = LOGIC_VERSION.get(&handle)?;
        if logic_version != 18 {
            return Err(Error::LogicVersion(logic_version));
        }
        let logic_patch = LOGIC_PATCH.get(&handle)?;
        if logic_patch != 1 {
            return Err(Error::LogicPatch(logic_patch));
        }
        let logic_clock_register = LOGIC_CLOCK.get(&handle)?;
        let adc_clock_register = ADC_CLOCK.get(&handle)?;
        let usb_clock_register = USB_CLOCK.get(&handle)?;
        let clock_deviation = CLOCK_DEVIATION.get(&handle)?;
        let logic_clock = (logic_clock_register as f64) * (clock_deviation as f64 / 1000.0);
        let adc_clock = (adc_clock_register as f64) * (clock_deviation as f64 / 1000.0);
        let usb_clock = (usb_clock_register as f64) * (clock_deviation as f64 / 1000.0);
        let has_pixel_filter = DVS_HAS_PIXEL_FILTER.get(&handle)? == 1;
        let has_activity_filter = DVS_HAS_BACKGROUND_ACTIVITY_FILTER.get(&handle)? == 1;
        let has_roi_filter = DVS_HAS_ROI_FILTER.get(&handle)? == 1;
        let has_skip_filter = DVS_HAS_SKIP_FILTER.get(&handle)? == 1;
        let has_polarity_filter = DVS_HAS_POLARITY_FILTER.get(&handle)? == 1;
        let has_global_shutter = APS_HAS_GLOBAL_SHUTTER.get(&handle)? == 1;

        // @DEV {
        println!("DVS_SIZE_COLUMNS={}", DVS_SIZE_COLUMNS.get(&handle)?);
        println!("DVS_SIZE_ROWS={}", DVS_SIZE_ROWS.get(&handle)?);
        println!("DVS_ORIENTATION={}", DVS_ORIENTATION.get(&handle)?);
        println!(
            "DVS_HAS_PIXEL_FILTER={}",
            DVS_HAS_PIXEL_FILTER.get(&handle)?
        );
        println!(
            "DVS_HAS_BACKGROUND_ACTIVITY_FILTER={}",
            DVS_HAS_BACKGROUND_ACTIVITY_FILTER.get(&handle)?
        );
        println!("DVS_HAS_ROI_FILTER={}", DVS_HAS_ROI_FILTER.get(&handle)?);
        println!("DVS_HAS_SKIP_FILTER={}", DVS_HAS_SKIP_FILTER.get(&handle)?);
        println!(
            "DVS_HAS_POLARITY_FILTER={}",
            DVS_HAS_POLARITY_FILTER.get(&handle)?
        );
        println!("APS_SIZE_COLUMNS={}", APS_SIZE_COLUMNS.get(&handle)?);
        println!("APS_SIZE_ROWS={}", APS_SIZE_ROWS.get(&handle)?);
        println!("APS_ORIENTATION={}", APS_ORIENTATION.get(&handle)?);
        println!(
            "APS_HAS_GLOBAL_SHUTTER={}",
            APS_HAS_GLOBAL_SHUTTER.get(&handle)?
        );
        println!("IMU_TYPE={}", IMU_TYPE.get(&handle)?);
        println!("IMU_ORIENTATION={}", IMU_ORIENTATION.get(&handle)?);
        println!("LOGIC_CLOCK={}", LOGIC_CLOCK.get(&handle)?);
        println!("ADC_CLOCK={}", ADC_CLOCK.get(&handle)?);
        println!("USB_CLOCK={}", USB_CLOCK.get(&handle)?);
        println!("CLOCK_DEVIATION={}", CLOCK_DEVIATION.get(&handle)?);
        // }

        // reset + start sequence
        DVS_RUN.set(&handle, 0)?;
        APS_RUN.set(&handle, 0)?;
        IMU_ACCELEROMETER_RUN.set(&handle, 0)?;
        IMU_GYROSCOPE_RUN.set(&handle, 0)?;
        IMU_TEMPERATURE_RUN.set(&handle, 0)?;
        EXTERNAL_INPUT_DETECTOR_RUN.set(&handle, 0)?;
        MULTIPLEXER_RUN.set(&handle, 0)?;
        MULTIPLEXER_TIMESTAMP_RUN.set(&handle, 0)?;
        USB_RUN.set(&handle, 0)?;
        MULTIPLEXER_CHIP_RUN.set(&handle, 0)?;

        update_configuration(
            &handle,
            None,
            &configuration,
            has_pixel_filter,
            has_activity_filter,
            has_roi_filter,
            has_skip_filter,
            has_polarity_filter,
            has_global_shutter,
            logic_clock,
            adc_clock,
            usb_clock,
        )?;

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
                    endpoint: 2 | libusb1_sys::constants::LIBUSB_ENDPOINT_IN,
                    timeout: std::time::Duration::ZERO,
                },
            )?,
            configuration_updater: configuration::Updater::new(
                configuration,
                ConfigurationUpdaterContext {
                    handle,
                    flag,
                    has_pixel_filter,
                    has_activity_filter,
                    has_roi_filter,
                    has_skip_filter,
                    has_polarity_filter,
                    has_global_shutter,
                    logic_clock,
                    adc_clock,
                    usb_clock,
                },
                |context, previous_configuration, configuration| {
                    let result = {
                        update_configuration(
                            &context.handle,
                            Some(previous_configuration),
                            configuration,
                            context.has_pixel_filter,
                            context.has_activity_filter,
                            context.has_roi_filter,
                            context.has_skip_filter,
                            context.has_polarity_filter,
                            context.has_global_shutter,
                            context.logic_clock,
                            context.adc_clock,
                            context.usb_clock,
                        )
                    };
                    if let Err(error) = result {
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
        Self::Adapter::new()
    }

    fn temperature_celsius(&self) -> Result<device::TemperatureCelsius, Self::Error> {
        Err(Error::NotImplemented("temperature_celsius".to_owned()))
    }
}

impl Drop for Device {
    fn drop(&mut self) {
        let _ = DVS_RUN.set(&self.handle, 0);
        let _ = APS_RUN.set(&self.handle, 0);
        let _ = IMU_ACCELEROMETER_RUN.set(&self.handle, 0);
        let _ = IMU_GYROSCOPE_RUN.set(&self.handle, 0);
        let _ = IMU_TEMPERATURE_RUN.set(&self.handle, 0);
        let _ = EXTERNAL_INPUT_DETECTOR_RUN.set(&self.handle, 0);
        let _ = MULTIPLEXER_RUN.set(&self.handle, 0);
        let _ = MULTIPLEXER_TIMESTAMP_RUN.set(&self.handle, 0);
        let _ = USB_RUN.set(&self.handle, 0);
        let _ = MULTIPLEXER_CHIP_RUN.set(&self.handle, 0);
    }
}

const TIMEOUT: std::time::Duration = std::time::Duration::from_secs(1);

const BIAS_ENABLED: u32 = 0b1;
const BIAS_SEX_N: u32 = 0b10;
const BIAS_TYPE_NORMAL: u32 = 0b100;
const BIAS_CURRENT_LEVEL_NORMAL: u32 = 0b1000;
const BIAS_P_TYPE: u32 = BIAS_ENABLED | BIAS_TYPE_NORMAL | BIAS_CURRENT_LEVEL_NORMAL;
const BIAS_N_TYPE: u32 = BIAS_ENABLED | BIAS_SEX_N | BIAS_TYPE_NORMAL | BIAS_CURRENT_LEVEL_NORMAL;
const BIAS_N_TYPE_OFF: u32 = BIAS_SEX_N | BIAS_TYPE_NORMAL | BIAS_CURRENT_LEVEL_NORMAL;

macro_rules! update_bias {
    ($name:ident, $register:ident, $bias_type:ident, $handle:ident, $previous_biases:ident, $biases:expr) => {
        if match $previous_biases {
            Some(previous_biases) => previous_biases.$name != $biases.$name,
            None => true,
        } {
            let (coarse, fine) = if $biases.$name < 2048 {
                ($biases.$name / 256, $biases.$name % 256)
            } else {
                (7, 255)
            };
            $register.set(
                $handle,
                $bias_type | ((fine as u32) << 4) | ((coarse as u32) << 12),
            )?;
        }
    };
}

// @TODO move non-parameters to the init function (and confirm that the change in order does not change the behaviour)
fn update_configuration(
    handle: &rusb::DeviceHandle<rusb::Context>,
    previous_configuration: Option<&Configuration>,
    configuration: &Configuration,
    has_pixel_filter: bool,
    has_activity_filter: bool,
    has_roi_filter: bool,
    has_skip_filter: bool,
    has_polarity_filter: bool,
    has_global_shutter: bool,
    logic_clock: f64,
    adc_clock: f64,
    usb_clock: f64,
) -> Result<(), Error> {
    // voltage-current (VDAC) biases
    if previous_configuration.is_none() {
        CHIP_BIAS_APSOVERFLOWLEVEL.set(handle, 27 | (6 << 6))?;
        CHIP_BIAS_APSCAS.set(handle, 21 | (6 << 6))?;
        CHIP_BIAS_ADCREFHIGH.set(handle, 32 | (7 << 6))?;
        CHIP_BIAS_ADCREFLOW.set(handle, 1 | (7 << 6))?;
        CHIP_BIAS_ADCTESTVOLTAGE.set(handle, 21 | (7 << 6))?;
    }

    // coarse / fine biases
    let previous_biases = previous_configuration.map(|configuration| &configuration.biases);
    update_bias!(
        localbufbn,
        CHIP_BIAS_LOCALBUFBN,
        BIAS_N_TYPE,
        handle,
        previous_biases,
        configuration.biases
    );
    update_bias!(
        padfollbn,
        CHIP_BIAS_PADFOLLBN,
        BIAS_N_TYPE_OFF,
        handle,
        previous_biases,
        configuration.biases
    );
    update_bias!(
        diffbn,
        CHIP_BIAS_DIFFBN,
        BIAS_N_TYPE,
        handle,
        previous_biases,
        configuration.biases
    );
    update_bias!(
        onbn,
        CHIP_BIAS_ONBN,
        BIAS_N_TYPE,
        handle,
        previous_biases,
        configuration.biases
    );
    update_bias!(
        offbn,
        CHIP_BIAS_OFFBN,
        BIAS_N_TYPE,
        handle,
        previous_biases,
        configuration.biases
    );
    update_bias!(
        pixinvbn,
        CHIP_BIAS_PIXINVBN,
        BIAS_N_TYPE,
        handle,
        previous_biases,
        configuration.biases
    );
    update_bias!(
        prbp,
        CHIP_BIAS_PRBP,
        BIAS_P_TYPE,
        handle,
        previous_biases,
        configuration.biases
    );
    update_bias!(
        prsfbp,
        CHIP_BIAS_PRSFBP,
        BIAS_P_TYPE,
        handle,
        previous_biases,
        configuration.biases
    );
    update_bias!(
        refrbp,
        CHIP_BIAS_REFRBP,
        BIAS_P_TYPE,
        handle,
        previous_biases,
        configuration.biases
    );
    update_bias!(
        readoutbufbp,
        CHIP_BIAS_READOUTBUFBP,
        BIAS_P_TYPE,
        handle,
        previous_biases,
        configuration.biases
    );
    update_bias!(
        apsrosfbn,
        CHIP_BIAS_APSROSFBN,
        BIAS_N_TYPE,
        handle,
        previous_biases,
        configuration.biases
    );
    update_bias!(
        adccompbp,
        CHIP_BIAS_ADCCOMPBP,
        BIAS_P_TYPE,
        handle,
        previous_biases,
        configuration.biases
    );
    update_bias!(
        colsellowbn,
        CHIP_BIAS_COLSELLOWBN,
        BIAS_N_TYPE,
        handle,
        previous_biases,
        configuration.biases
    );
    update_bias!(
        dacbufbp,
        CHIP_BIAS_DACBUFBP,
        BIAS_P_TYPE,
        handle,
        previous_biases,
        configuration.biases
    );
    update_bias!(
        lcoltimeoutbn,
        CHIP_BIAS_LCOLTIMEOUTBN,
        BIAS_N_TYPE,
        handle,
        previous_biases,
        configuration.biases
    );
    update_bias!(
        aepdbn,
        CHIP_BIAS_AEPDBN,
        BIAS_N_TYPE,
        handle,
        previous_biases,
        configuration.biases
    );
    update_bias!(
        aepuxbp,
        CHIP_BIAS_AEPUXBP,
        BIAS_P_TYPE,
        handle,
        previous_biases,
        configuration.biases
    );
    update_bias!(
        aepuybp,
        CHIP_BIAS_AEPUYBP,
        BIAS_P_TYPE,
        handle,
        previous_biases,
        configuration.biases
    );
    update_bias!(
        ifrefrbn,
        CHIP_BIAS_IFREFRBN,
        BIAS_N_TYPE,
        handle,
        previous_biases,
        configuration.biases
    );
    update_bias!(
        ifthrbn,
        CHIP_BIAS_IFTHRBN,
        BIAS_N_TYPE,
        handle,
        previous_biases,
        configuration.biases
    );
    update_bias!(
        biasbuffer,
        CHIP_BIAS_BUFFER,
        BIAS_N_TYPE,
        handle,
        previous_biases,
        configuration.biases
    );

    // shifted source biases and other init settings
    if previous_configuration.is_none() {
        CHIP_BIAS_SSP.set(handle, (1 << 4) | (33 << 10))?;
        CHIP_BIAS_SSN.set(handle, (1 << 4) | (33 << 10))?;
        CHIP_DIGITALMUX0.set(handle, 0)?;
        CHIP_DIGITALMUX1.set(handle, 0)?;
        CHIP_DIGITALMUX2.set(handle, 0)?;
        CHIP_DIGITALMUX3.set(handle, 0)?;
        CHIP_ANALOGMUX0.set(handle, 0)?;
        CHIP_ANALOGMUX1.set(handle, 0)?;
        CHIP_ANALOGMUX2.set(handle, 0)?;
        CHIP_BIASMUX0.set(handle, 0)?;
        CHIP_RESETCALIBNEURON.set(handle, 1)?;
        CHIP_TYPENCALIBNEURON.set(handle, 0)?;
        CHIP_RESETTESTPIXEL.set(handle, 1)?;
        CHIP_AERNAROW.set(handle, 0)?;
        CHIP_USEAOUT.set(handle, 0)?;
        CHIP_SELECTGRAYCOUNTER.set(handle, 1)?;
        CHIP_TESTADC.set(handle, 0)?;
        std::thread::sleep(std::time::Duration::from_millis(10));
        MULTIPLEXER_CHIP_RUN.set(&handle, 1)?;
        USB_EARLY_PACKET_DELAY.set(handle, (1000.0 * usb_clock).round() as u32)?;
        std::thread::sleep(std::time::Duration::from_millis(200));
        USB_RUN.set(&handle, 1)?;
        MULTIPLEXER_DROP_DVS_ON_STALL.set(&handle, 1)?;
        MULTIPLEXER_DROP_EXTERNAL_INPUT_ON_STALL.set(&handle, 1)?;
        MULTIPLEXER_TIMESTAMP_RUN.set(&handle, 1)?;
        MULTIPLEXER_RUN.set(&handle, 1)?;
        std::thread::sleep(std::time::Duration::from_millis(50));
        DVS_WAIT_ON_STALL.set(&handle, 0)?;
        DVS_EXTERNAL_AER_CONTROL.set(&handle, 0)?;
    }

    if has_pixel_filter {
        if match previous_configuration {
            Some(previous_configuration) => {
                previous_configuration.pixel_mask != configuration.pixel_mask
            }
            None => true,
        } {
            for (index, code) in configuration.pixel_mask.iter().enumerate() {
                let (x_value, y_value) = if *code == 0 {
                    (346, 260)
                } else {
                    ((code - 1) % 346, (code - 1) / 346)
                };
                SpiRegister::new(DVS_MODULE_ADDRESS, (11 + 2 * index) as u16)
                    .set(handle, y_value)?;
                SpiRegister::new(DVS_MODULE_ADDRESS, (12 + 2 * index) as u16)
                    .set(handle, x_value)?;
            }
        }
    }

    if has_activity_filter {
        if match previous_configuration {
            Some(previous_configuration) => {
                previous_configuration.activity_filter != configuration.activity_filter
            }
            None => true,
        } {
            DVS_FILTER_BACKGROUND_ACTIVITY.set(
                handle,
                configuration.activity_filter.mask_isolated_enable as u32,
            )?;
            DVS_FILTER_BACKGROUND_ACTIVITY_TIME
                .set(handle, configuration.activity_filter.mask_isolated_tau)?;
            DVS_FILTER_REFRACTORY_PERIOD.set(
                handle,
                configuration.activity_filter.refractory_period_enable as u32,
            )?;
            DVS_FILTER_REFRACTORY_PERIOD_TIME
                .set(handle, configuration.activity_filter.refractory_period_tau)?;
        }
    }

    if has_roi_filter {
        if match previous_configuration {
            Some(previous_configuration) => {
                previous_configuration.region_of_interest != configuration.region_of_interest
            }
            None => true,
        } {
            DVS_FILTER_ROI_START_COLUMN
                .set(handle, configuration.region_of_interest.left as u32)?;
            DVS_FILTER_ROI_START_ROW.set(handle, configuration.region_of_interest.top as u32)?;
            DVS_FILTER_ROI_END_COLUMN.set(
                handle,
                (configuration.region_of_interest.left + configuration.region_of_interest.width)
                    .min(345) as u32,
            )?;
            DVS_FILTER_ROI_END_ROW.set(
                handle,
                (configuration.region_of_interest.top + configuration.region_of_interest.height)
                    .min(259) as u32,
            )?;
        }
    }

    if has_skip_filter {
        if match previous_configuration {
            Some(previous_configuration) => {
                previous_configuration.skip_events_every != configuration.skip_events_every
            }
            None => true,
        } {
            DVS_FILTER_SKIP_EVENTS.set(handle, (configuration.skip_events_every > 0) as u32)?;
            DVS_FILTER_SKIP_EVENTS_EVERY.set(handle, configuration.skip_events_every)?;
        }
    }

    if has_polarity_filter {
        DVS_FILTER_POLARITY_FLATTEN.set(
            handle,
            match configuration.polarity_filter {
                PolarityFilter::Flatten | PolarityFilter::MaskOffFlatten => 1,
                _ => 0,
            },
        )?;
        DVS_FILTER_POLARITY_SUPPRESS.set(
            handle,
            match configuration.polarity_filter {
                PolarityFilter::MaskOn
                | PolarityFilter::MaskOff
                | PolarityFilter::MaskOffFlatten => 1,
                _ => 0,
            },
        )?;
        DVS_FILTER_POLARITY_SUPPRESS_TYPE.set(
            handle,
            match configuration.polarity_filter {
                PolarityFilter::MaskOn => 1,
                _ => 0,
            },
        )?;
    }

    if previous_configuration.is_none() {
        DVS_RUN.set(&handle, 1)?;
        APS_WAIT_ON_STALL.set(&handle, 1)?;
        CHIP_GLOBAL_SHUTTER.set(&handle, has_global_shutter as u32)?;
        APS_GLOBAL_SHUTTER.set(handle, has_global_shutter as u32)?;
    }

    if has_roi_filter {
        if match previous_configuration {
            Some(previous_configuration) => {
                previous_configuration.region_of_interest != configuration.region_of_interest
            }
            None => true,
        } {
            APS_START_COLUMN_0.set(handle, configuration.region_of_interest.left as u32)?;
            APS_START_ROW_0.set(handle, configuration.region_of_interest.top as u32)?;
            APS_END_COLUMN_0.set(
                handle,
                (configuration.region_of_interest.left + configuration.region_of_interest.width - 1)
                    .min(345) as u32,
            )?;
            APS_END_ROW_0.set(
                handle,
                (configuration.region_of_interest.top + configuration.region_of_interest.height - 1)
                    .min(259) as u32,
            )?;
        }
    }

    if match previous_configuration {
        Some(previous_configuration) => {
            previous_configuration.exposure_us != configuration.exposure_us
        }
        None => true,
    } {
        println!(
            "set exposure to {} µs ({} cycles)",
            configuration.exposure_us,
            (configuration.exposure_us as f64 * adc_clock).round() as u32
        ); // @DEV
        APS_EXPOSURE.set(
            handle,
            (configuration.exposure_us as f64 * adc_clock).round() as u32,
        )?;
    }
    if match previous_configuration {
        Some(previous_configuration) => {
            previous_configuration.frame_interval_us != configuration.frame_interval_us
        }
        None => true,
    } {
        println!(
            "set frame interval to {} µs ({} cycles)",
            configuration.frame_interval_us,
            (configuration.frame_interval_us as f64 * adc_clock).round() as u32
        ); // @DEV
        APS_FRAME_INTERVAL.set(
            handle,
            (configuration.frame_interval_us as f64 * adc_clock).round() as u32,
        )?;
    }

    if previous_configuration.is_none() {
        APS_RUN.set(&handle, 1)?;
        IMU_SAMPLE_RATE_DIVIDER.set(handle, 0)?;
        IMU_ACCELEROMETER_DIGITAL_LOW_PASS_FILTER.set(handle, 1)?;
        IMU_ACCELEROMETER_FULL_SCALE.set(handle, 1)?;
        IMU_GYROSCOPE_DIGITAL_LOW_PASS_FILTER.set(handle, 1)?;
        IMU_GYROSCOPE_FULL_SCALE.set(handle, 1)?;
        IMU_GYROSCOPE_RUN.set(&handle, 1)?;
        IMU_TEMPERATURE_RUN.set(&handle, 1)?;
        EXTERNAL_INPUT_DETECT_RISING_EDGES.set(handle, 0)?;
        EXTERNAL_INPUT_DETECT_FALLING_EDGES.set(handle, 0)?;
        EXTERNAL_INPUT_DETECT_PULSES.set(handle, 1)?;
        EXTERNAL_INPUT_DETECT_PULSE_POLARITY.set(handle, 1)?;
        EXTERNAL_INPUT_DETECT_PULSE_LENGTH.set(handle, (10.0 * logic_clock).round() as u32)?;
        EXTERNAL_INPUT_DETECTOR_RUN.set(&handle, 0)?;
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
    has_pixel_filter: bool,
    has_activity_filter: bool,
    has_roi_filter: bool,
    has_skip_filter: bool,
    has_polarity_filter: bool,
    has_global_shutter: bool,
    logic_clock: f64,
    adc_clock: f64,
    usb_clock: f64,
}

struct SpiRegister {
    module_address: u16,
    parameter_address: u16,
}

struct SpiRegister64 {
    module_address: u16,
    parameter_address: u16,
}

// module addresses
const MULTIPLEXER_MODULE_ADDRESS: u16 = 0;
const DVS_MODULE_ADDRESS: u16 = 1;
const APS_MODULE_ADDRESS: u16 = 2;
const IMU_MODULE_ADDRESS: u16 = 3;
const EXTERNAL_INPUT_MODULE_ADDRESS: u16 = 4;
const CHIP_MODULE_ADDRESS: u16 = 5;
const SYSTEM_INFORMATION_MODULE_ADDRESS: u16 = 6;
const USB_MODULE_ADDRESS: u16 = 9;

// multiplexer module registers
const MULTIPLEXER_RUN: SpiRegister = SpiRegister::new(MULTIPLEXER_MODULE_ADDRESS, 0);
const MULTIPLEXER_TIMESTAMP_RUN: SpiRegister = SpiRegister::new(MULTIPLEXER_MODULE_ADDRESS, 1);
const MULTIPLEXER_TIMESTAMP_RESET: SpiRegister = SpiRegister::new(MULTIPLEXER_MODULE_ADDRESS, 2);
const MULTIPLEXER_CHIP_RUN: SpiRegister = SpiRegister::new(MULTIPLEXER_MODULE_ADDRESS, 3);
const MULTIPLEXER_DROP_EXTERNAL_INPUT_ON_STALL: SpiRegister =
    SpiRegister::new(MULTIPLEXER_MODULE_ADDRESS, 4);
const MULTIPLEXER_DROP_DVS_ON_STALL: SpiRegister = SpiRegister::new(MULTIPLEXER_MODULE_ADDRESS, 5);
const MULTIPLEXER_HAS_STATISTICS: SpiRegister = SpiRegister::new(MULTIPLEXER_MODULE_ADDRESS, 80);
const MULTIPLEXER_STATISTICS_EXTERNAL_INPUT_DROPPED: SpiRegister64 =
    SpiRegister64::new(MULTIPLEXER_MODULE_ADDRESS, 81);
const MULTIPLEXER_STATISTICS_DVS_DROPPED: SpiRegister64 =
    SpiRegister64::new(MULTIPLEXER_MODULE_ADDRESS, 83);

// dvs module registers
const DVS_SIZE_COLUMNS: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 0);
const DVS_SIZE_ROWS: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 1);
const DVS_ORIENTATION: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 2);
const DVS_RUN: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 3);
const DVS_WAIT_ON_STALL: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 4);
const DVS_EXTERNAL_AER_CONTROL: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 5);
const DVS_HAS_PIXEL_FILTER: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 10);
const DVS_HAS_BACKGROUND_ACTIVITY_FILTER: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 30);
const DVS_FILTER_BACKGROUND_ACTIVITY: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 31);
const DVS_FILTER_BACKGROUND_ACTIVITY_TIME: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 32);
const DVS_FILTER_REFRACTORY_PERIOD: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 33);
const DVS_FILTER_REFRACTORY_PERIOD_TIME: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 34);
const DVS_HAS_ROI_FILTER: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 40);
const DVS_FILTER_ROI_START_COLUMN: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 41);
const DVS_FILTER_ROI_START_ROW: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 42);
const DVS_FILTER_ROI_END_COLUMN: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 43);
const DVS_FILTER_ROI_END_ROW: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 44);
const DVS_HAS_SKIP_FILTER: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 50);
const DVS_FILTER_SKIP_EVENTS: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 51);
const DVS_FILTER_SKIP_EVENTS_EVERY: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 52);
const DVS_HAS_POLARITY_FILTER: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 60);
const DVS_FILTER_POLARITY_FLATTEN: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 61);
const DVS_FILTER_POLARITY_SUPPRESS: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 62);
const DVS_FILTER_POLARITY_SUPPRESS_TYPE: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 63);
const DVS_HAS_STATISTICS: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 80);
const DVS_STATISTICS_EVENTS_ROW: SpiRegister64 = SpiRegister64::new(DVS_MODULE_ADDRESS, 81);
const DVS_STATISTICS_EVENTS_COLUMN: SpiRegister64 = SpiRegister64::new(DVS_MODULE_ADDRESS, 83);
const DVS_STATISTICS_EVENTS_DROPPED: SpiRegister64 = SpiRegister64::new(DVS_MODULE_ADDRESS, 85);
const DVS_STATISTICS_FILTERED_PIXELS: SpiRegister64 = SpiRegister64::new(DVS_MODULE_ADDRESS, 87);
const DVS_STATISTICS_FILTERED_BACKGROUND_ACTIVITY: SpiRegister64 =
    SpiRegister64::new(DVS_MODULE_ADDRESS, 89);
const DVS_STATISTICS_FILTERED_REFRACTORY_PERIOD: SpiRegister64 =
    SpiRegister64::new(DVS_MODULE_ADDRESS, 91);
const DVS_FILTER_PIXEL_AUTO_TRAIN: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 100);

// aps module registers
const APS_SIZE_COLUMNS: SpiRegister = SpiRegister::new(APS_MODULE_ADDRESS, 0);
const APS_SIZE_ROWS: SpiRegister = SpiRegister::new(APS_MODULE_ADDRESS, 1);
const APS_ORIENTATION: SpiRegister = SpiRegister::new(APS_MODULE_ADDRESS, 2);
const APS_COLOR_FILTER: SpiRegister = SpiRegister::new(APS_MODULE_ADDRESS, 3);
const APS_RUN: SpiRegister = SpiRegister::new(APS_MODULE_ADDRESS, 4);
const APS_WAIT_ON_STALL: SpiRegister = SpiRegister::new(APS_MODULE_ADDRESS, 5);
const APS_HAS_GLOBAL_SHUTTER: SpiRegister = SpiRegister::new(APS_MODULE_ADDRESS, 6);
const APS_GLOBAL_SHUTTER: SpiRegister = SpiRegister::new(APS_MODULE_ADDRESS, 7);
const APS_START_COLUMN_0: SpiRegister = SpiRegister::new(APS_MODULE_ADDRESS, 8);
const APS_START_ROW_0: SpiRegister = SpiRegister::new(APS_MODULE_ADDRESS, 9);
const APS_END_COLUMN_0: SpiRegister = SpiRegister::new(APS_MODULE_ADDRESS, 10);
const APS_END_ROW_0: SpiRegister = SpiRegister::new(APS_MODULE_ADDRESS, 11);
const APS_EXPOSURE: SpiRegister = SpiRegister::new(APS_MODULE_ADDRESS, 12);
const APS_FRAME_INTERVAL: SpiRegister = SpiRegister::new(APS_MODULE_ADDRESS, 13);
const APS_AUTOEXPOSURE: SpiRegister = SpiRegister::new(APS_MODULE_ADDRESS, 101);
const APS_FRAME_MODE: SpiRegister = SpiRegister::new(APS_MODULE_ADDRESS, 102);

// imu module registers
const IMU_TYPE: SpiRegister = SpiRegister::new(IMU_MODULE_ADDRESS, 0);
const IMU_ORIENTATION: SpiRegister = SpiRegister::new(IMU_MODULE_ADDRESS, 1);
const IMU_ACCELEROMETER_RUN: SpiRegister = SpiRegister::new(IMU_MODULE_ADDRESS, 2);
const IMU_GYROSCOPE_RUN: SpiRegister = SpiRegister::new(IMU_MODULE_ADDRESS, 3);
const IMU_TEMPERATURE_RUN: SpiRegister = SpiRegister::new(IMU_MODULE_ADDRESS, 4);
const IMU_SAMPLE_RATE_DIVIDER: SpiRegister = SpiRegister::new(IMU_MODULE_ADDRESS, 5);
const IMU_ACCELEROMETER_DIGITAL_LOW_PASS_FILTER: SpiRegister =
    SpiRegister::new(IMU_MODULE_ADDRESS, 6);
const IMU_ACCELEROMETER_FULL_SCALE: SpiRegister = SpiRegister::new(IMU_MODULE_ADDRESS, 7);
const IMU_GYROSCOPE_DIGITAL_LOW_PASS_FILTER: SpiRegister = SpiRegister::new(IMU_MODULE_ADDRESS, 9);
const IMU_GYROSCOPE_FULL_SCALE: SpiRegister = SpiRegister::new(IMU_MODULE_ADDRESS, 10);

// external input module registers
const EXTERNAL_INPUT_DETECTOR_RUN: SpiRegister = SpiRegister::new(EXTERNAL_INPUT_MODULE_ADDRESS, 0);
const EXTERNAL_INPUT_DETECT_RISING_EDGES: SpiRegister =
    SpiRegister::new(EXTERNAL_INPUT_MODULE_ADDRESS, 1);
const EXTERNAL_INPUT_DETECT_FALLING_EDGES: SpiRegister =
    SpiRegister::new(EXTERNAL_INPUT_MODULE_ADDRESS, 2);
const EXTERNAL_INPUT_DETECT_PULSES: SpiRegister =
    SpiRegister::new(EXTERNAL_INPUT_MODULE_ADDRESS, 3);
const EXTERNAL_INPUT_DETECT_PULSE_POLARITY: SpiRegister =
    SpiRegister::new(EXTERNAL_INPUT_MODULE_ADDRESS, 4);
const EXTERNAL_INPUT_DETECT_PULSE_LENGTH: SpiRegister =
    SpiRegister::new(EXTERNAL_INPUT_MODULE_ADDRESS, 5);
const EXTERNAL_INPUT_HAS_GENERATOR: SpiRegister =
    SpiRegister::new(EXTERNAL_INPUT_MODULE_ADDRESS, 10);
const EXTERNAL_INPUT_RUN_GENERATOR: SpiRegister =
    SpiRegister::new(EXTERNAL_INPUT_MODULE_ADDRESS, 11);
const EXTERNAL_INPUT_GENERATE_PULSE_POLARITY: SpiRegister =
    SpiRegister::new(EXTERNAL_INPUT_MODULE_ADDRESS, 12);
const EXTERNAL_INPUT_GENERATE_PULSE_INTERVAL: SpiRegister =
    SpiRegister::new(EXTERNAL_INPUT_MODULE_ADDRESS, 13);
const EXTERNAL_INPUT_GENERATE_PULSE_LENGTH: SpiRegister =
    SpiRegister::new(EXTERNAL_INPUT_MODULE_ADDRESS, 14);
const EXTERNAL_INPUT_GENERATE_INJECT_ON_RISING_EDGE: SpiRegister =
    SpiRegister::new(EXTERNAL_INPUT_MODULE_ADDRESS, 15);
const EXTERNAL_INPUT_GENERATE_INJECT_ON_FALLING_EDGE: SpiRegister =
    SpiRegister::new(EXTERNAL_INPUT_MODULE_ADDRESS, 16);

// chip module registers
const CHIP_BIAS_APSOVERFLOWLEVEL: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 0);
const CHIP_BIAS_APSCAS: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 1);
const CHIP_BIAS_ADCREFHIGH: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 2);
const CHIP_BIAS_ADCREFLOW: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 3);
const CHIP_BIAS_ADCTESTVOLTAGE: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 4);
const CHIP_BIAS_LOCALBUFBN: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 8);
const CHIP_BIAS_PADFOLLBN: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 9);
const CHIP_BIAS_DIFFBN: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 10);
const CHIP_BIAS_ONBN: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 11);
const CHIP_BIAS_OFFBN: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 12);
const CHIP_BIAS_PIXINVBN: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 13);
const CHIP_BIAS_PRBP: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 14);
const CHIP_BIAS_PRSFBP: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 15);
const CHIP_BIAS_REFRBP: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 16);
const CHIP_BIAS_READOUTBUFBP: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 17);
const CHIP_BIAS_APSROSFBN: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 18);
const CHIP_BIAS_ADCCOMPBP: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 19);
const CHIP_BIAS_COLSELLOWBN: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 20);
const CHIP_BIAS_DACBUFBP: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 21);
const CHIP_BIAS_LCOLTIMEOUTBN: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 22);
const CHIP_BIAS_AEPDBN: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 23);
const CHIP_BIAS_AEPUXBP: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 24);
const CHIP_BIAS_AEPUYBP: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 25);
const CHIP_BIAS_IFREFRBN: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 26);
const CHIP_BIAS_IFTHRBN: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 27);
const CHIP_BIAS_BUFFER: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 34);
const CHIP_BIAS_SSP: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 35);
const CHIP_BIAS_SSN: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 36);
const CHIP_DIGITALMUX0: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 128);
const CHIP_DIGITALMUX1: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 129);
const CHIP_DIGITALMUX2: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 130);
const CHIP_DIGITALMUX3: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 131);
const CHIP_ANALOGMUX0: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 132);
const CHIP_ANALOGMUX1: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 133);
const CHIP_ANALOGMUX2: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 134);
const CHIP_BIASMUX0: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 135);
const CHIP_RESETCALIBNEURON: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 136);
const CHIP_TYPENCALIBNEURON: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 137);
const CHIP_RESETTESTPIXEL: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 138);
const CHIP_SPECIALPIXELCONTROL: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 139);
const CHIP_AERNAROW: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 140);
const CHIP_USEAOUT: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 141);
const CHIP_GLOBAL_SHUTTER: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 142);
const CHIP_SELECTGRAYCOUNTER: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 143);
const CHIP_TESTADC: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 144);

// system information module registers
const LOGIC_VERSION: SpiRegister = SpiRegister::new(SYSTEM_INFORMATION_MODULE_ADDRESS, 0);
const CHIP_IDENTIFIER: SpiRegister = SpiRegister::new(SYSTEM_INFORMATION_MODULE_ADDRESS, 1);
const CHIP_IS_PRIMARY: SpiRegister = SpiRegister::new(SYSTEM_INFORMATION_MODULE_ADDRESS, 2);
const LOGIC_CLOCK: SpiRegister = SpiRegister::new(SYSTEM_INFORMATION_MODULE_ADDRESS, 3);
const ADC_CLOCK: SpiRegister = SpiRegister::new(SYSTEM_INFORMATION_MODULE_ADDRESS, 4);
const USB_CLOCK: SpiRegister = SpiRegister::new(SYSTEM_INFORMATION_MODULE_ADDRESS, 5);
const CLOCK_DEVIATION: SpiRegister = SpiRegister::new(SYSTEM_INFORMATION_MODULE_ADDRESS, 6);
const LOGIC_PATCH: SpiRegister = SpiRegister::new(SYSTEM_INFORMATION_MODULE_ADDRESS, 7);

// usb module registers
const USB_RUN: SpiRegister = SpiRegister::new(USB_MODULE_ADDRESS, 0);
const USB_EARLY_PACKET_DELAY: SpiRegister = SpiRegister::new(USB_MODULE_ADDRESS, 1);

impl SpiRegister {
    fn get(&self, handle: &rusb::DeviceHandle<rusb::Context>) -> Result<u32, Error> {
        let mut buffer = [0; 4];
        let read = handle.read_control(
            0xC0,
            0xBF,
            self.module_address,
            self.parameter_address,
            &mut buffer,
            TIMEOUT,
        )?;
        if read != 4 {
            return Err(Error::ShortRead {
                module_address: self.module_address,
                parameter_address: self.parameter_address,
                expected: 4,
                read,
            });
        }

        println!("libusb_control_transfer(request_type=0x{:02X}, request=0x{:02X}, value=0x{:04X}, index=0x{:04X}) -> {} ({:02x}:{:02x}:{:02x}:{:02x})",
            0xC0,
            0xBF,
            self.module_address,
            self.parameter_address,
            u32::from_be_bytes(buffer),
            buffer[0],
            buffer[1],
            buffer[2],
            buffer[3],
        ); // @DEV

        Ok(u32::from_be_bytes(buffer))
    }

    fn set(&self, handle: &rusb::DeviceHandle<rusb::Context>, value: u32) -> Result<(), Error> {
        let buffer = value.to_be_bytes();

        println!("libusb_control_transfer(request_type=0x{:02X}, request=0x{:02X}, value=0x{:04X}, index=0x{:04X}, data={} ({:02x}:{:02x}:{:02x}:{:02x}))",
            0x40,
            0xBF,
            self.module_address,
            self.parameter_address,
            value,
            buffer[0],
            buffer[1],
            buffer[2],
            buffer[3],
        ); // @DEV

        let wrote = handle.write_control(
            0x40,
            0xBF,
            self.module_address,
            self.parameter_address,
            &buffer,
            TIMEOUT,
        )?;
        if wrote != 4 {
            return Err(Error::ShortWrite {
                module_address: self.module_address,
                parameter_address: self.parameter_address,
                expected: 4,
                wrote,
            });
        }
        Ok(())
    }

    const fn new(module_address: u16, parameter_address: u16) -> Self {
        Self {
            module_address,
            parameter_address,
        }
    }
}

impl SpiRegister64 {
    fn get(&self, handle: &rusb::DeviceHandle<rusb::Context>) -> Result<u64, Error> {
        let msb = SpiRegister {
            module_address: self.module_address,
            parameter_address: self.parameter_address,
        }
        .get(handle)?;
        let lsb = SpiRegister {
            module_address: self.module_address,
            parameter_address: self.parameter_address + 1,
        }
        .get(handle)?;
        Ok(((msb as u64) << 32) | (lsb as u64))
    }

    const fn new(module_address: u16, parameter_address: u16) -> Self {
        Self {
            module_address,
            parameter_address,
        }
    }
}
