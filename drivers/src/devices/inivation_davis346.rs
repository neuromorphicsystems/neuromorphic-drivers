use crate::adapters;
use crate::configuration;
use crate::device;
use crate::flag;
use crate::properties;
use crate::usb;

use device::Usb;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ApsOrientation {
    invert_xy: bool,
    flip_x: bool,
    flip_y: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ImuOrientation {
    flip_x: bool,
    flip_y: bool,
    flip_z: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImuType {
    None,
    InvenSense6050Or6150,
    InvenSense9250,
}

impl ImuType {
    pub fn temperature_celsius(&self, value: i16) -> f32 {
        match self {
            ImuType::None => value as f32,
            ImuType::InvenSense6050Or6150 => (value as f32 / 340.0) + 35.0,
            ImuType::InvenSense9250 => (value as f32 / 333.87) + 21.0,
        }
    }
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq, Eq)]
pub struct Biases {
    localbufbn: u16,    // in [0, 2040]
    padfollbn: u16,     // in [0, 2040]
    diffbn: u16,        // in [0, 2040]
    onbn: u16,          // in [0, 2040]
    offbn: u16,         // in [0, 2040]
    pixinvbn: u16,      // in [0, 2040]
    prbp: u16,          // in [0, 2040]
    prsfbp: u16,        // in [0, 2040]
    refrbp: u16,        // in [0, 2040]
    readoutbufbp: u16,  // in [0, 2040]
    apsrosfbn: u16,     // in [0, 2040]
    adccompbp: u16,     // in [0, 2040]
    colsellowbn: u16,   // in [0, 2040]
    dacbufbp: u16,      // in [0, 2040]
    lcoltimeoutbn: u16, // in [0, 2040]
    aepdbn: u16,        // in [0, 2040]
    aepuxbp: u16,       // in [0, 2040]
    aepuybp: u16,       // in [0, 2040]
    ifrefrbn: u16,      // in [0, 2040]
    ifthrbn: u16,       // in [0, 2040]
    biasbuffer: u16,    // in [0, 2040]
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
    dvs_invert_xy: bool,
    aps_orientation: ApsOrientation,
    imu_orientation: ImuOrientation,
    imu_type: ImuType,
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

    #[error("Short SPI read to {module_address:#04X}:{parameter_address:#04X} (expected {expected} bytes, read {count} bytes)")]
    ShortRead {
        module_address: u16,
        parameter_address: u16,
        expected: usize,
        count: usize,
    },

    #[error("Short SPI write to {module_address:#04X}:{parameter_address:#04X} (expected {expected} bytes, wrote {count} bytes)")]
    ShortWrite {
        module_address: u16,
        parameter_address: u16,
        expected: usize,
        count: usize,
    },
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
                localbufbn: 1461,
                padfollbn: 2000,
                diffbn: 1063,
                onbn: 1564,
                offbn: 583,
                pixinvbn: 1691,
                prbp: 575,
                prsfbp: 153,
                refrbp: 993,
                readoutbufbp: 1457,
                apsrosfbn: 1776,
                adccompbp: 1202,
                colsellowbn: 1,
                dacbufbp: 1597,
                lcoltimeoutbn: 1330,
                aepdbn: 1632,
                aepuxbp: 1110,
                aepuybp: 1937,
                ifrefrbn: 1564,
                ifthrbn: 1564,
                biasbuffer: 1563,
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
        let dvs_invert_xy = ((DVS_ORIENTATION.get(&handle)? >> 2) & 1) == 1;
        let aps_orientation = {
            let aps_orientation = APS_ORIENTATION.get(&handle)?;
            ApsOrientation {
                invert_xy: ((aps_orientation >> 2) & 1) == 1,
                flip_x: ((aps_orientation >> 1) & 1) == 1,
                flip_y: (aps_orientation & 1) == 1,
            }
        };
        let imu_orientation = {
            let imu_orientation = IMU_ORIENTATION.get(&handle)?;
            ImuOrientation {
                flip_x: ((imu_orientation >> 2) & 1) == 1,
                flip_y: ((imu_orientation >> 1) & 1) == 1,
                flip_z: (imu_orientation & 1) == 1,
            }
        };
        let imu_type = match IMU_TYPE.get(&handle)? {
            1 => ImuType::InvenSense6050Or6150,
            2 => ImuType::InvenSense9250,
            _ => ImuType::None,
        };
        DVS_RUN.set(&handle, 0)?;
        APS_RUN.set(&handle, 0)?;
        IMU_ACCELEROMETER_RUN.set(&handle, 0)?;
        IMU_GYROSCOPE_RUN.set(&handle, 0)?;
        IMU_TEMPERATURE_RUN.set(&handle, 0)?;
        EXTERNAL_INPUT_DETECTOR_RUN.set(&handle, 0)?;
        EXTERNAL_INPUT_RUN_GENERATOR.set(&handle, 0)?;
        MULTIPLEXER_RUN.set(&handle, 0)?;
        MULTIPLEXER_TIMESTAMP_RUN.set(&handle, 0)?;
        USB_RUN.set(&handle, 0)?;
        MULTIPLEXER_CHIP_RUN.set(&handle, 0)?;
        std::thread::sleep(std::time::Duration::from_millis(10));
        handle.clear_halt(0x82)?;
        CHIP_BIAS_APSOVERFLOWLEVEL.set(&handle, 27 | (6 << 6))?;
        CHIP_BIAS_APSCAS.set(&handle, 21 | (6 << 6))?;
        CHIP_BIAS_ADCREFHIGH.set(&handle, 32 | (7 << 6))?;
        CHIP_BIAS_ADCREFLOW.set(&handle, 1 | (7 << 6))?;
        CHIP_BIAS_ADCTESTVOLTAGE.set(&handle, 21 | (7 << 6))?;

        update_configuration(
            &handle,
            None,
            &configuration,
            has_pixel_filter,
            has_activity_filter,
            has_roi_filter,
            has_skip_filter,
            has_polarity_filter,
            adc_clock,
        )?;

        CHIP_BIAS_SSP.set(&handle, (1 << 4) | (33 << 10))?;
        CHIP_BIAS_SSN.set(&handle, (1 << 4) | (33 << 10))?;
        CHIP_DIGITALMUX0.set(&handle, 0)?;
        CHIP_DIGITALMUX1.set(&handle, 0)?;
        CHIP_DIGITALMUX2.set(&handle, 0)?;
        CHIP_DIGITALMUX3.set(&handle, 0)?;
        CHIP_ANALOGMUX0.set(&handle, 0)?;
        CHIP_ANALOGMUX1.set(&handle, 0)?;
        CHIP_ANALOGMUX2.set(&handle, 0)?;
        CHIP_BIASMUX0.set(&handle, 0)?;
        CHIP_RESETCALIBNEURON.set(&handle, 1)?;
        CHIP_TYPENCALIBNEURON.set(&handle, 0)?;
        CHIP_RESETTESTPIXEL.set(&handle, 1)?;
        CHIP_AERNAROW.set(&handle, 0)?;
        CHIP_USEAOUT.set(&handle, 0)?;
        CHIP_SELECTGRAYCOUNTER.set(&handle, 1)?;
        CHIP_TESTADC.set(&handle, 0)?;
        std::thread::sleep(std::time::Duration::from_millis(10));
        MULTIPLEXER_CHIP_RUN.set(&handle, 1)?;
        USB_EARLY_PACKET_DELAY.set(&handle, (1000.0 * usb_clock).round() as u32)?;
        std::thread::sleep(std::time::Duration::from_millis(200));
        USB_RUN.set(&handle, 1)?;
        MULTIPLEXER_DROP_DVS_ON_STALL.set(&handle, 1)?;
        MULTIPLEXER_DROP_EXTERNAL_INPUT_ON_STALL.set(&handle, 1)?;
        MULTIPLEXER_TIMESTAMP_RUN.set(&handle, 1)?;
        MULTIPLEXER_RUN.set(&handle, 1)?;
        std::thread::sleep(std::time::Duration::from_millis(50));
        DVS_WAIT_ON_STALL.set(&handle, 0)?;
        DVS_EXTERNAL_AER_CONTROL.set(&handle, 0)?;
        DVS_RUN.set(&handle, 1)?;
        APS_WAIT_ON_STALL.set(&handle, 1)?;
        APS_GLOBAL_SHUTTER.set(&handle, has_global_shutter as u32)?;
        CHIP_GLOBAL_SHUTTER.set(&handle, has_global_shutter as u32)?;
        APS_RUN.set(&handle, 1)?;
        IMU_SAMPLE_RATE_DIVIDER.set(&handle, 0)?;
        IMU_ACCELEROMETER_DIGITAL_LOW_PASS_FILTER.set(&handle, 1)?;
        IMU_ACCELEROMETER_FULL_SCALE.set(&handle, 1)?;
        if matches!(imu_type, ImuType::InvenSense9250) {
            IMU_GYROSCOPE_DIGITAL_LOW_PASS_FILTER.set(&handle, 1)?;
        }
        IMU_GYROSCOPE_FULL_SCALE.set(&handle, 1)?;
        IMU_ACCELEROMETER_RUN.set(&handle, 1)?;
        IMU_GYROSCOPE_RUN.set(&handle, 1)?;
        IMU_TEMPERATURE_RUN.set(&handle, 1)?;
        EXTERNAL_INPUT_DETECT_RISING_EDGES.set(&handle, 0)?;
        EXTERNAL_INPUT_DETECT_FALLING_EDGES.set(&handle, 0)?;
        EXTERNAL_INPUT_DETECT_PULSES.set(&handle, 1)?;
        EXTERNAL_INPUT_DETECT_PULSE_POLARITY.set(&handle, 1)?;
        EXTERNAL_INPUT_DETECT_PULSE_LENGTH.set(&handle, (10.0 * logic_clock).round() as u32)?;
        EXTERNAL_INPUT_DETECTOR_RUN.set(&handle, 0)?;

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
                    adc_clock,
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
                            context.adc_clock,
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
            dvs_invert_xy,
            aps_orientation,
            imu_orientation,
            imu_type,
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
        Self::Adapter::new(
            self.dvs_invert_xy,
            self.aps_orientation,
            self.imu_orientation,
            self.imu_type,
        )
    }
}

impl Device {
    pub fn dvs_invert_xy(&self) -> bool {
        self.dvs_invert_xy
    }

    pub fn aps_orientation(&self) -> ApsOrientation {
        self.aps_orientation
    }

    pub fn imu_orientation(&self) -> ImuOrientation {
        self.imu_orientation
    }

    pub fn imu_type(&self) -> ImuType {
        self.imu_type
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
            let (coarse, fine) =
                COARSE_FINE_MAP[($biases.$name as usize).min(COARSE_FINE_MAP.len() - 1)];
            $register.set(
                $handle,
                $bias_type | ((fine as u32) << 4) | ((coarse as u32) << 12),
            )?;
        }
    };
}

fn update_configuration(
    handle: &rusb::DeviceHandle<rusb::Context>,
    previous_configuration: Option<&Configuration>,
    configuration: &Configuration,
    has_pixel_filter: bool,
    has_activity_filter: bool,
    has_roi_filter: bool,
    has_skip_filter: bool,
    has_polarity_filter: bool,
    adc_clock: f64,
) -> Result<(), Error> {
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
        if match previous_configuration {
            Some(previous_configuration) => {
                previous_configuration.polarity_filter != configuration.polarity_filter
            }
            None => true,
        } {
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
    }

    if has_roi_filter {
        if match previous_configuration {
            Some(previous_configuration) => {
                previous_configuration.region_of_interest != configuration.region_of_interest
            }
            None => true,
        } {
            set_many(
                handle,
                &[
                    (&APS_RUN, 0),
                    (
                        &APS_START_COLUMN_0,
                        configuration.region_of_interest.left as u32,
                    ),
                    (
                        &APS_START_ROW_0,
                        configuration.region_of_interest.top as u32,
                    ),
                    (
                        &APS_END_COLUMN_0,
                        (configuration.region_of_interest.left
                            + configuration.region_of_interest.width
                            - 1)
                        .min(345) as u32,
                    ),
                    (
                        &APS_END_ROW_0,
                        (configuration.region_of_interest.top
                            + configuration.region_of_interest.height
                            - 1)
                        .min(259) as u32,
                    ),
                    (&APS_RUN, 0),
                ],
            )?;
        }
    }

    if match previous_configuration {
        Some(previous_configuration) => {
            previous_configuration.exposure_us != configuration.exposure_us
        }
        None => true,
    } {
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
        APS_FRAME_INTERVAL.set(
            handle,
            (configuration.frame_interval_us as f64 * adc_clock).round() as u32,
        )?;
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
    adc_clock: f64,
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
#[allow(dead_code)]
const MULTIPLEXER_TIMESTAMP_RESET: SpiRegister = SpiRegister::new(MULTIPLEXER_MODULE_ADDRESS, 2);
const MULTIPLEXER_CHIP_RUN: SpiRegister = SpiRegister::new(MULTIPLEXER_MODULE_ADDRESS, 3);
const MULTIPLEXER_DROP_EXTERNAL_INPUT_ON_STALL: SpiRegister =
    SpiRegister::new(MULTIPLEXER_MODULE_ADDRESS, 4);
const MULTIPLEXER_DROP_DVS_ON_STALL: SpiRegister = SpiRegister::new(MULTIPLEXER_MODULE_ADDRESS, 5);
#[allow(dead_code)]
const MULTIPLEXER_HAS_STATISTICS: SpiRegister = SpiRegister::new(MULTIPLEXER_MODULE_ADDRESS, 80);
#[allow(dead_code)]
const MULTIPLEXER_STATISTICS_EXTERNAL_INPUT_DROPPED: SpiRegister64 =
    SpiRegister64::new(MULTIPLEXER_MODULE_ADDRESS, 81);
#[allow(dead_code)]
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
#[allow(dead_code)]
const DVS_HAS_STATISTICS: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 80);
#[allow(dead_code)]
const DVS_STATISTICS_EVENTS_ROW: SpiRegister64 = SpiRegister64::new(DVS_MODULE_ADDRESS, 81);
#[allow(dead_code)]
const DVS_STATISTICS_EVENTS_COLUMN: SpiRegister64 = SpiRegister64::new(DVS_MODULE_ADDRESS, 83);
#[allow(dead_code)]
const DVS_STATISTICS_EVENTS_DROPPED: SpiRegister64 = SpiRegister64::new(DVS_MODULE_ADDRESS, 85);
#[allow(dead_code)]
const DVS_STATISTICS_FILTERED_PIXELS: SpiRegister64 = SpiRegister64::new(DVS_MODULE_ADDRESS, 87);
#[allow(dead_code)]
const DVS_STATISTICS_FILTERED_BACKGROUND_ACTIVITY: SpiRegister64 =
    SpiRegister64::new(DVS_MODULE_ADDRESS, 89);
#[allow(dead_code)]
const DVS_STATISTICS_FILTERED_REFRACTORY_PERIOD: SpiRegister64 =
    SpiRegister64::new(DVS_MODULE_ADDRESS, 91);
#[allow(dead_code)]
const DVS_FILTER_PIXEL_AUTO_TRAIN: SpiRegister = SpiRegister::new(DVS_MODULE_ADDRESS, 100);

// aps module registers
const APS_SIZE_COLUMNS: SpiRegister = SpiRegister::new(APS_MODULE_ADDRESS, 0);
const APS_SIZE_ROWS: SpiRegister = SpiRegister::new(APS_MODULE_ADDRESS, 1);
const APS_ORIENTATION: SpiRegister = SpiRegister::new(APS_MODULE_ADDRESS, 2);
#[allow(dead_code)]
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
#[allow(dead_code)]
const APS_AUTOEXPOSURE: SpiRegister = SpiRegister::new(APS_MODULE_ADDRESS, 101);
#[allow(dead_code)]
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
#[allow(dead_code)]
const EXTERNAL_INPUT_HAS_GENERATOR: SpiRegister =
    SpiRegister::new(EXTERNAL_INPUT_MODULE_ADDRESS, 10);
const EXTERNAL_INPUT_RUN_GENERATOR: SpiRegister =
    SpiRegister::new(EXTERNAL_INPUT_MODULE_ADDRESS, 11);
#[allow(dead_code)]
const EXTERNAL_INPUT_GENERATE_PULSE_POLARITY: SpiRegister =
    SpiRegister::new(EXTERNAL_INPUT_MODULE_ADDRESS, 12);
#[allow(dead_code)]
const EXTERNAL_INPUT_GENERATE_PULSE_INTERVAL: SpiRegister =
    SpiRegister::new(EXTERNAL_INPUT_MODULE_ADDRESS, 13);
#[allow(dead_code)]
const EXTERNAL_INPUT_GENERATE_PULSE_LENGTH: SpiRegister =
    SpiRegister::new(EXTERNAL_INPUT_MODULE_ADDRESS, 14);
#[allow(dead_code)]
const EXTERNAL_INPUT_GENERATE_INJECT_ON_RISING_EDGE: SpiRegister =
    SpiRegister::new(EXTERNAL_INPUT_MODULE_ADDRESS, 15);
#[allow(dead_code)]
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
#[allow(dead_code)]
const CHIP_SPECIALPIXELCONTROL: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 139);
const CHIP_AERNAROW: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 140);
const CHIP_USEAOUT: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 141);
const CHIP_GLOBAL_SHUTTER: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 142);
const CHIP_SELECTGRAYCOUNTER: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 143);
const CHIP_TESTADC: SpiRegister = SpiRegister::new(CHIP_MODULE_ADDRESS, 144);

// system information module registers
const LOGIC_VERSION: SpiRegister = SpiRegister::new(SYSTEM_INFORMATION_MODULE_ADDRESS, 0);
#[allow(dead_code)]
const CHIP_IDENTIFIER: SpiRegister = SpiRegister::new(SYSTEM_INFORMATION_MODULE_ADDRESS, 1);
#[allow(dead_code)]
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
        let count = handle.read_control(
            0xC0,
            0xBF,
            self.module_address,
            self.parameter_address,
            &mut buffer,
            TIMEOUT,
        )?;
        if count != 4 {
            return Err(Error::ShortRead {
                module_address: self.module_address,
                parameter_address: self.parameter_address,
                expected: 4,
                count,
            });
        }
        Ok(u32::from_be_bytes(buffer))
    }

    fn set(&self, handle: &rusb::DeviceHandle<rusb::Context>, value: u32) -> Result<(), Error> {
        let buffer = value.to_be_bytes();
        let count = handle.write_control(
            0x40,
            0xBF,
            self.module_address,
            self.parameter_address,
            &buffer,
            TIMEOUT,
        )?;
        if count != 4 {
            return Err(Error::ShortWrite {
                module_address: self.module_address,
                parameter_address: self.parameter_address,
                expected: 4,
                count,
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
    #[allow(dead_code)]
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

fn set_many(
    handle: &rusb::DeviceHandle<rusb::Context>,
    registers_and_values: &[(&SpiRegister, u32)],
) -> Result<(), Error> {
    let mut buffer = vec![0; 6 * registers_and_values.len()];
    for (index, (register, value)) in registers_and_values.iter().enumerate() {
        buffer[index * 6] = register.module_address as u8;
        buffer[index * 6 + 1] = register.parameter_address as u8;
        buffer[index * 6 + 2..index * 6 + 6].copy_from_slice(&value.to_be_bytes());
    }
    let count = handle.write_control(0x40, 0xC2, 0x0006, 0x0000, &buffer, TIMEOUT)?;
    if count != buffer.len() {
        return Err(Error::ShortWrite {
            module_address: 0x0006,
            parameter_address: 0x0000,
            expected: buffer.len(),
            count,
        });
    }
    Ok(())
}

// The bias current i is given by
//     i = C[coarse] * (fine / 255.0)
// where:
//     - C = [11, 94, 756, 6054, 48437, 387500, 3100000, 24800000] (in pA)
//     - coarse, in [0, 7], is the bias coarse value sent to the camera
//     - fine, in [0, 255], is the bias fine value sent to the camera
// "fine" may only be 0 if "coarse" is 0 as well
//
// Bias currents are monotonic for a given coarse value,
// but not monotonic across them (https://doi.org/10.1109/ISCAS.2012.6271979)
// To simplify the user interface,
// we re-order the 2041 allowed pairs (coarse, fine) so that the output currents are monotonic.
// Users thus have a single interger field to tweak.
#[rustfmt::skip]
const COARSE_FINE_MAP: [(u8, u8); 2041] = [
    (0,   0), (0,   1), (0,   2), (0,   3), (0,   4), (0,   5), (0,   6), (0,   7),
    (0,   8), (1,   1), (0,   9), (0,  10), (0,  11), (0,  12), (0,  13), (0,  14),
    (0,  15), (0,  16), (0,  17), (1,   2), (0,  18), (0,  19), (0,  20), (0,  21),
    (0,  22), (0,  23), (0,  24), (0,  25), (1,   3), (0,  26), (0,  27), (0,  28),
    (0,  29), (0,  30), (0,  31), (0,  32), (0,  33), (0,  34), (1,   4), (0,  35),
    (0,  36), (0,  37), (0,  38), (0,  39), (0,  40), (0,  41), (0,  42), (1,   5),
    (0,  43), (0,  44), (0,  45), (0,  46), (0,  47), (0,  48), (0,  49), (0,  50),
    (0,  51), (1,   6), (0,  52), (0,  53), (0,  54), (0,  55), (0,  56), (0,  57),
    (0,  58), (0,  59), (1,   7), (0,  60), (0,  61), (0,  62), (0,  63), (0,  64),
    (0,  65), (0,  66), (0,  67), (0,  68), (1,   8), (2,   1), (0,  69), (0,  70),
    (0,  71), (0,  72), (0,  73), (0,  74), (0,  75), (0,  76), (1,   9), (0,  77),
    (0,  78), (0,  79), (0,  80), (0,  81), (0,  82), (0,  83), (0,  84), (0,  85),
    (1,  10), (0,  86), (0,  87), (0,  88), (0,  89), (0,  90), (0,  91), (0,  92),
    (0,  93), (0,  94), (1,  11), (0,  95), (0,  96), (0,  97), (0,  98), (0,  99),
    (0, 100), (0, 101), (0, 102), (1,  12), (0, 103), (0, 104), (0, 105), (0, 106),
    (0, 107), (0, 108), (0, 109), (0, 110), (0, 111), (1,  13), (0, 112), (0, 113),
    (0, 114), (0, 115), (0, 116), (0, 117), (0, 118), (0, 119), (1,  14), (0, 120),
    (0, 121), (0, 122), (0, 123), (0, 124), (0, 125), (0, 126), (0, 127), (0, 128),
    (1,  15), (0, 129), (0, 130), (0, 131), (0, 132), (0, 133), (0, 134), (0, 135),
    (0, 136), (1,  16), (0, 137), (2,   2), (0, 138), (0, 139), (0, 140), (0, 141),
    (0, 142), (0, 143), (0, 144), (0, 145), (1,  17), (0, 146), (0, 147), (0, 148),
    (0, 149), (0, 150), (0, 151), (0, 152), (0, 153), (1,  18), (0, 154), (0, 155),
    (0, 156), (0, 157), (0, 158), (0, 159), (0, 160), (0, 161), (0, 162), (1,  19),
    (0, 163), (0, 164), (0, 165), (0, 166), (0, 167), (0, 168), (0, 169), (0, 170),
    (1,  20), (0, 171), (0, 172), (0, 173), (0, 174), (0, 175), (0, 176), (0, 177),
    (0, 178), (0, 179), (1,  21), (0, 180), (0, 181), (0, 182), (0, 183), (0, 184),
    (0, 185), (0, 186), (0, 187), (0, 188), (1,  22), (0, 189), (0, 190), (0, 191),
    (0, 192), (0, 193), (0, 194), (0, 195), (0, 196), (1,  23), (0, 197), (0, 198),
    (0, 199), (0, 200), (0, 201), (0, 202), (0, 203), (0, 204), (0, 205), (1,  24),
    (0, 206), (2,   3), (0, 207), (0, 208), (0, 209), (0, 210), (0, 211), (0, 212),
    (0, 213), (1,  25), (0, 214), (0, 215), (0, 216), (0, 217), (0, 218), (0, 219),
    (0, 220), (0, 221), (0, 222), (1,  26), (0, 223), (0, 224), (0, 225), (0, 226),
    (0, 227), (0, 228), (0, 229), (0, 230), (1,  27), (0, 231), (0, 232), (0, 233),
    (0, 234), (0, 235), (0, 236), (0, 237), (0, 238), (0, 239), (1,  28), (0, 240),
    (0, 241), (0, 242), (0, 243), (0, 244), (0, 245), (0, 246), (0, 247), (1,  29),
    (0, 248), (0, 249), (0, 250), (0, 251), (0, 252), (0, 253), (0, 254), (0, 255),
    (1,  30), (1,  31), (1,  32), (2,   4), (1,  33), (1,  34), (1,  35), (1,  36),
    (1,  37), (1,  38), (1,  39), (1,  40), (2,   5), (1,  41), (1,  42), (1,  43),
    (1,  44), (1,  45), (1,  46), (1,  47), (1,  48), (2,   6), (1,  49), (1,  50),
    (1,  51), (1,  52), (1,  53), (1,  54), (1,  55), (1,  56), (2,   7), (1,  57),
    (1,  58), (1,  59), (1,  60), (1,  61), (1,  62), (1,  63), (1,  64), (2,   8),
    (3,   1), (1,  65), (1,  66), (1,  67), (1,  68), (1,  69), (1,  70), (1,  71),
    (1,  72), (2,   9), (1,  73), (1,  74), (1,  75), (1,  76), (1,  77), (1,  78),
    (1,  79), (1,  80), (2,  10), (1,  81), (1,  82), (1,  83), (1,  84), (1,  85),
    (1,  86), (1,  87), (1,  88), (2,  11), (1,  89), (1,  90), (1,  91), (1,  92),
    (1,  93), (1,  94), (1,  95), (1,  96), (2,  12), (1,  97), (1,  98), (1,  99),
    (1, 100), (1, 101), (1, 102), (1, 103), (1, 104), (2,  13), (1, 105), (1, 106),
    (1, 107), (1, 108), (1, 109), (1, 110), (1, 111), (1, 112), (2,  14), (1, 113),
    (1, 114), (1, 115), (1, 116), (1, 117), (1, 118), (1, 119), (1, 120), (2,  15),
    (1, 121), (1, 122), (1, 123), (1, 124), (1, 125), (1, 126), (1, 127), (1, 128),
    (2,  16), (3,   2), (1, 129), (1, 130), (1, 131), (1, 132), (1, 133), (1, 134),
    (1, 135), (1, 136), (2,  17), (1, 137), (1, 138), (1, 139), (1, 140), (1, 141),
    (1, 142), (1, 143), (1, 144), (2,  18), (1, 145), (1, 146), (1, 147), (1, 148),
    (1, 149), (1, 150), (1, 151), (1, 152), (2,  19), (1, 153), (1, 154), (1, 155),
    (1, 156), (1, 157), (1, 158), (1, 159), (1, 160), (2,  20), (1, 161), (1, 162),
    (1, 163), (1, 164), (1, 165), (1, 166), (1, 167), (1, 168), (2,  21), (1, 169),
    (1, 170), (1, 171), (1, 172), (1, 173), (1, 174), (1, 175), (1, 176), (2,  22),
    (1, 177), (1, 178), (1, 179), (1, 180), (1, 181), (1, 182), (1, 183), (1, 184),
    (2,  23), (1, 185), (1, 186), (1, 187), (1, 188), (1, 189), (1, 190), (1, 191),
    (1, 192), (1, 193), (2,  24), (3,   3), (1, 194), (1, 195), (1, 196), (1, 197),
    (1, 198), (1, 199), (1, 200), (1, 201), (2,  25), (1, 202), (1, 203), (1, 204),
    (1, 205), (1, 206), (1, 207), (1, 208), (1, 209), (2,  26), (1, 210), (1, 211),
    (1, 212), (1, 213), (1, 214), (1, 215), (1, 216), (1, 217), (2,  27), (1, 218),
    (1, 219), (1, 220), (1, 221), (1, 222), (1, 223), (1, 224), (1, 225), (2,  28),
    (1, 226), (1, 227), (1, 228), (1, 229), (1, 230), (1, 231), (1, 232), (1, 233),
    (2,  29), (1, 234), (1, 235), (1, 236), (1, 237), (1, 238), (1, 239), (1, 240),
    (1, 241), (2,  30), (1, 242), (1, 243), (1, 244), (1, 245), (1, 246), (1, 247),
    (1, 248), (1, 249), (2,  31), (1, 250), (1, 251), (1, 252), (1, 253), (1, 254),
    (1, 255), (2,  32), (3,   4), (2,  33), (2,  34), (2,  35), (2,  36), (2,  37),
    (2,  38), (2,  39), (2,  40), (3,   5), (2,  41), (2,  42), (2,  43), (2,  44),
    (2,  45), (2,  46), (2,  47), (2,  48), (3,   6), (2,  49), (2,  50), (2,  51),
    (2,  52), (2,  53), (2,  54), (2,  55), (2,  56), (3,   7), (2,  57), (2,  58),
    (2,  59), (2,  60), (2,  61), (2,  62), (2,  63), (2,  64), (3,   8), (4,   1),
    (2,  65), (2,  66), (2,  67), (2,  68), (2,  69), (2,  70), (2,  71), (2,  72),
    (3,   9), (2,  73), (2,  74), (2,  75), (2,  76), (2,  77), (2,  78), (2,  79),
    (2,  80), (3,  10), (2,  81), (2,  82), (2,  83), (2,  84), (2,  85), (2,  86),
    (2,  87), (2,  88), (3,  11), (2,  89), (2,  90), (2,  91), (2,  92), (2,  93),
    (2,  94), (2,  95), (2,  96), (3,  12), (2,  97), (2,  98), (2,  99), (2, 100),
    (2, 101), (2, 102), (2, 103), (2, 104), (3,  13), (2, 105), (2, 106), (2, 107),
    (2, 108), (2, 109), (2, 110), (2, 111), (2, 112), (3,  14), (2, 113), (2, 114),
    (2, 115), (2, 116), (2, 117), (2, 118), (2, 119), (2, 120), (3,  15), (2, 121),
    (2, 122), (2, 123), (2, 124), (2, 125), (2, 126), (2, 127), (2, 128), (3,  16),
    (4,   2), (2, 129), (2, 130), (2, 131), (2, 132), (2, 133), (2, 134), (2, 135),
    (2, 136), (3,  17), (2, 137), (2, 138), (2, 139), (2, 140), (2, 141), (2, 142),
    (2, 143), (2, 144), (3,  18), (2, 145), (2, 146), (2, 147), (2, 148), (2, 149),
    (2, 150), (2, 151), (2, 152), (3,  19), (2, 153), (2, 154), (2, 155), (2, 156),
    (2, 157), (2, 158), (2, 159), (2, 160), (3,  20), (2, 161), (2, 162), (2, 163),
    (2, 164), (2, 165), (2, 166), (2, 167), (2, 168), (3,  21), (2, 169), (2, 170),
    (2, 171), (2, 172), (2, 173), (2, 174), (2, 175), (2, 176), (3,  22), (2, 177),
    (2, 178), (2, 179), (2, 180), (2, 181), (2, 182), (2, 183), (2, 184), (3,  23),
    (2, 185), (2, 186), (2, 187), (2, 188), (2, 189), (2, 190), (2, 191), (2, 192),
    (3,  24), (4,   3), (2, 193), (2, 194), (2, 195), (2, 196), (2, 197), (2, 198),
    (2, 199), (2, 200), (3,  25), (2, 201), (2, 202), (2, 203), (2, 204), (2, 205),
    (2, 206), (2, 207), (2, 208), (3,  26), (2, 209), (2, 210), (2, 211), (2, 212),
    (2, 213), (2, 214), (2, 215), (2, 216), (3,  27), (2, 217), (2, 218), (2, 219),
    (2, 220), (2, 221), (2, 222), (2, 223), (2, 224), (3,  28), (2, 225), (2, 226),
    (2, 227), (2, 228), (2, 229), (2, 230), (2, 231), (2, 232), (3,  29), (2, 233),
    (2, 234), (2, 235), (2, 236), (2, 237), (2, 238), (2, 239), (2, 240), (3,  30),
    (2, 241), (2, 242), (2, 243), (2, 244), (2, 245), (2, 246), (2, 247), (2, 248),
    (3,  31), (2, 249), (2, 250), (2, 251), (2, 252), (2, 253), (2, 254), (2, 255),
    (3,  32), (4,   4), (3,  33), (3,  34), (3,  35), (3,  36), (3,  37), (3,  38),
    (3,  39), (3,  40), (4,   5), (3,  41), (3,  42), (3,  43), (3,  44), (3,  45),
    (3,  46), (3,  47), (3,  48), (4,   6), (3,  49), (3,  50), (3,  51), (3,  52),
    (3,  53), (3,  54), (3,  55), (3,  56), (4,   7), (3,  57), (3,  58), (3,  59),
    (3,  60), (3,  61), (3,  62), (3,  63), (3,  64), (4,   8), (5,   1), (3,  65),
    (3,  66), (3,  67), (3,  68), (3,  69), (3,  70), (3,  71), (3,  72), (4,   9),
    (3,  73), (3,  74), (3,  75), (3,  76), (3,  77), (3,  78), (3,  79), (3,  80),
    (4,  10), (3,  81), (3,  82), (3,  83), (3,  84), (3,  85), (3,  86), (3,  87),
    (3,  88), (4,  11), (3,  89), (3,  90), (3,  91), (3,  92), (3,  93), (3,  94),
    (3,  95), (3,  96), (4,  12), (3,  97), (3,  98), (3,  99), (3, 100), (3, 101),
    (3, 102), (3, 103), (3, 104), (4,  13), (3, 105), (3, 106), (3, 107), (3, 108),
    (3, 109), (3, 110), (3, 111), (3, 112), (4,  14), (3, 113), (3, 114), (3, 115),
    (3, 116), (3, 117), (3, 118), (3, 119), (3, 120), (4,  15), (3, 121), (3, 122),
    (3, 123), (3, 124), (3, 125), (3, 126), (3, 127), (3, 128), (4,  16), (5,   2),
    (3, 129), (3, 130), (3, 131), (3, 132), (3, 133), (3, 134), (3, 135), (3, 136),
    (4,  17), (3, 137), (3, 138), (3, 139), (3, 140), (3, 141), (3, 142), (3, 143),
    (3, 144), (4,  18), (3, 145), (3, 146), (3, 147), (3, 148), (3, 149), (3, 150),
    (3, 151), (3, 152), (4,  19), (3, 153), (3, 154), (3, 155), (3, 156), (3, 157),
    (3, 158), (3, 159), (3, 160), (4,  20), (3, 161), (3, 162), (3, 163), (3, 164),
    (3, 165), (3, 166), (3, 167), (3, 168), (4,  21), (3, 169), (3, 170), (3, 171),
    (3, 172), (3, 173), (3, 174), (3, 175), (3, 176), (4,  22), (3, 177), (3, 178),
    (3, 179), (3, 180), (3, 181), (3, 182), (3, 183), (3, 184), (4,  23), (3, 185),
    (3, 186), (3, 187), (3, 188), (3, 189), (3, 190), (3, 191), (3, 192), (4,  24),
    (5,   3), (3, 193), (3, 194), (3, 195), (3, 196), (3, 197), (3, 198), (3, 199),
    (3, 200), (4,  25), (3, 201), (3, 202), (3, 203), (3, 204), (3, 205), (3, 206),
    (3, 207), (3, 208), (4,  26), (3, 209), (3, 210), (3, 211), (3, 212), (3, 213),
    (3, 214), (3, 215), (3, 216), (4,  27), (3, 217), (3, 218), (3, 219), (3, 220),
    (3, 221), (3, 222), (3, 223), (3, 224), (4,  28), (3, 225), (3, 226), (3, 227),
    (3, 228), (3, 229), (3, 230), (3, 231), (3, 232), (4,  29), (3, 233), (3, 234),
    (3, 235), (3, 236), (3, 237), (3, 238), (3, 239), (3, 240), (4,  30), (3, 241),
    (3, 242), (3, 243), (3, 244), (3, 245), (3, 246), (3, 247), (3, 248), (4,  31),
    (3, 249), (3, 250), (3, 251), (3, 252), (3, 253), (3, 254), (3, 255), (4,  32),
    (5,   4), (4,  33), (4,  34), (4,  35), (4,  36), (4,  37), (4,  38), (4,  39),
    (4,  40), (5,   5), (4,  41), (4,  42), (4,  43), (4,  44), (4,  45), (4,  46),
    (4,  47), (4,  48), (5,   6), (4,  49), (4,  50), (4,  51), (4,  52), (4,  53),
    (4,  54), (4,  55), (4,  56), (5,   7), (4,  57), (4,  58), (4,  59), (4,  60),
    (4,  61), (4,  62), (4,  63), (4,  64), (5,   8), (6,   1), (4,  65), (4,  66),
    (4,  67), (4,  68), (4,  69), (4,  70), (4,  71), (4,  72), (5,   9), (4,  73),
    (4,  74), (4,  75), (4,  76), (4,  77), (4,  78), (4,  79), (4,  80), (5,  10),
    (4,  81), (4,  82), (4,  83), (4,  84), (4,  85), (4,  86), (4,  87), (4,  88),
    (5,  11), (4,  89), (4,  90), (4,  91), (4,  92), (4,  93), (4,  94), (4,  95),
    (4,  96), (5,  12), (4,  97), (4,  98), (4,  99), (4, 100), (4, 101), (4, 102),
    (4, 103), (4, 104), (5,  13), (4, 105), (4, 106), (4, 107), (4, 108), (4, 109),
    (4, 110), (4, 111), (4, 112), (5,  14), (4, 113), (4, 114), (4, 115), (4, 116),
    (4, 117), (4, 118), (4, 119), (4, 120), (5,  15), (4, 121), (4, 122), (4, 123),
    (4, 124), (4, 125), (4, 126), (4, 127), (4, 128), (5,  16), (6,   2), (4, 129),
    (4, 130), (4, 131), (4, 132), (4, 133), (4, 134), (4, 135), (4, 136), (5,  17),
    (4, 137), (4, 138), (4, 139), (4, 140), (4, 141), (4, 142), (4, 143), (4, 144),
    (5,  18), (4, 145), (4, 146), (4, 147), (4, 148), (4, 149), (4, 150), (4, 151),
    (4, 152), (5,  19), (4, 153), (4, 154), (4, 155), (4, 156), (4, 157), (4, 158),
    (4, 159), (4, 160), (5,  20), (4, 161), (4, 162), (4, 163), (4, 164), (4, 165),
    (4, 166), (4, 167), (4, 168), (5,  21), (4, 169), (4, 170), (4, 171), (4, 172),
    (4, 173), (4, 174), (4, 175), (4, 176), (5,  22), (4, 177), (4, 178), (4, 179),
    (4, 180), (4, 181), (4, 182), (4, 183), (4, 184), (5,  23), (4, 185), (4, 186),
    (4, 187), (4, 188), (4, 189), (4, 190), (4, 191), (4, 192), (5,  24), (6,   3),
    (4, 193), (4, 194), (4, 195), (4, 196), (4, 197), (4, 198), (4, 199), (4, 200),
    (5,  25), (4, 201), (4, 202), (4, 203), (4, 204), (4, 205), (4, 206), (4, 207),
    (4, 208), (5,  26), (4, 209), (4, 210), (4, 211), (4, 212), (4, 213), (4, 214),
    (4, 215), (4, 216), (5,  27), (4, 217), (4, 218), (4, 219), (4, 220), (4, 221),
    (4, 222), (4, 223), (4, 224), (5,  28), (4, 225), (4, 226), (4, 227), (4, 228),
    (4, 229), (4, 230), (4, 231), (4, 232), (5,  29), (4, 233), (4, 234), (4, 235),
    (4, 236), (4, 237), (4, 238), (4, 239), (4, 240), (5,  30), (4, 241), (4, 242),
    (4, 243), (4, 244), (4, 245), (4, 246), (4, 247), (4, 248), (5,  31), (4, 249),
    (4, 250), (4, 251), (4, 252), (4, 253), (4, 254), (4, 255), (5,  32), (6,   4),
    (5,  33), (5,  34), (5,  35), (5,  36), (5,  37), (5,  38), (5,  39), (5,  40),
    (6,   5), (5,  41), (5,  42), (5,  43), (5,  44), (5,  45), (5,  46), (5,  47),
    (5,  48), (6,   6), (5,  49), (5,  50), (5,  51), (5,  52), (5,  53), (5,  54),
    (5,  55), (5,  56), (6,   7), (5,  57), (5,  58), (5,  59), (5,  60), (5,  61),
    (5,  62), (5,  63), (5,  64), (6,   8), (7,   1), (5,  65), (5,  66), (5,  67),
    (5,  68), (5,  69), (5,  70), (5,  71), (5,  72), (6,   9), (5,  73), (5,  74),
    (5,  75), (5,  76), (5,  77), (5,  78), (5,  79), (5,  80), (6,  10), (5,  81),
    (5,  82), (5,  83), (5,  84), (5,  85), (5,  86), (5,  87), (5,  88), (6,  11),
    (5,  89), (5,  90), (5,  91), (5,  92), (5,  93), (5,  94), (5,  95), (5,  96),
    (6,  12), (5,  97), (5,  98), (5,  99), (5, 100), (5, 101), (5, 102), (5, 103),
    (5, 104), (6,  13), (5, 105), (5, 106), (5, 107), (5, 108), (5, 109), (5, 110),
    (5, 111), (5, 112), (6,  14), (5, 113), (5, 114), (5, 115), (5, 116), (5, 117),
    (5, 118), (5, 119), (5, 120), (6,  15), (5, 121), (5, 122), (5, 123), (5, 124),
    (5, 125), (5, 126), (5, 127), (5, 128), (6,  16), (7,   2), (5, 129), (5, 130),
    (5, 131), (5, 132), (5, 133), (5, 134), (5, 135), (5, 136), (6,  17), (5, 137),
    (5, 138), (5, 139), (5, 140), (5, 141), (5, 142), (5, 143), (5, 144), (6,  18),
    (5, 145), (5, 146), (5, 147), (5, 148), (5, 149), (5, 150), (5, 151), (5, 152),
    (6,  19), (5, 153), (5, 154), (5, 155), (5, 156), (5, 157), (5, 158), (5, 159),
    (5, 160), (6,  20), (5, 161), (5, 162), (5, 163), (5, 164), (5, 165), (5, 166),
    (5, 167), (5, 168), (6,  21), (5, 169), (5, 170), (5, 171), (5, 172), (5, 173),
    (5, 174), (5, 175), (5, 176), (6,  22), (5, 177), (5, 178), (5, 179), (5, 180),
    (5, 181), (5, 182), (5, 183), (5, 184), (6,  23), (5, 185), (5, 186), (5, 187),
    (5, 188), (5, 189), (5, 190), (5, 191), (5, 192), (6,  24), (7,   3), (5, 193),
    (5, 194), (5, 195), (5, 196), (5, 197), (5, 198), (5, 199), (5, 200), (6,  25),
    (5, 201), (5, 202), (5, 203), (5, 204), (5, 205), (5, 206), (5, 207), (5, 208),
    (6,  26), (5, 209), (5, 210), (5, 211), (5, 212), (5, 213), (5, 214), (5, 215),
    (5, 216), (6,  27), (5, 217), (5, 218), (5, 219), (5, 220), (5, 221), (5, 222),
    (5, 223), (5, 224), (6,  28), (5, 225), (5, 226), (5, 227), (5, 228), (5, 229),
    (5, 230), (5, 231), (5, 232), (6,  29), (5, 233), (5, 234), (5, 235), (5, 236),
    (5, 237), (5, 238), (5, 239), (5, 240), (6,  30), (5, 241), (5, 242), (5, 243),
    (5, 244), (5, 245), (5, 246), (5, 247), (5, 248), (6,  31), (5, 249), (5, 250),
    (5, 251), (5, 252), (5, 253), (5, 254), (5, 255), (6,  32), (7,   4), (6,  33),
    (6,  34), (6,  35), (6,  36), (6,  37), (6,  38), (6,  39), (6,  40), (7,   5),
    (6,  41), (6,  42), (6,  43), (6,  44), (6,  45), (6,  46), (6,  47), (6,  48),
    (7,   6), (6,  49), (6,  50), (6,  51), (6,  52), (6,  53), (6,  54), (6,  55),
    (6,  56), (7,   7), (6,  57), (6,  58), (6,  59), (6,  60), (6,  61), (6,  62),
    (6,  63), (6,  64), (7,   8), (6,  65), (6,  66), (6,  67), (6,  68), (6,  69),
    (6,  70), (6,  71), (6,  72), (7,   9), (6,  73), (6,  74), (6,  75), (6,  76),
    (6,  77), (6,  78), (6,  79), (6,  80), (7,  10), (6,  81), (6,  82), (6,  83),
    (6,  84), (6,  85), (6,  86), (6,  87), (6,  88), (7,  11), (6,  89), (6,  90),
    (6,  91), (6,  92), (6,  93), (6,  94), (6,  95), (6,  96), (7,  12), (6,  97),
    (6,  98), (6,  99), (6, 100), (6, 101), (6, 102), (6, 103), (6, 104), (7,  13),
    (6, 105), (6, 106), (6, 107), (6, 108), (6, 109), (6, 110), (6, 111), (6, 112),
    (7,  14), (6, 113), (6, 114), (6, 115), (6, 116), (6, 117), (6, 118), (6, 119),
    (6, 120), (7,  15), (6, 121), (6, 122), (6, 123), (6, 124), (6, 125), (6, 126),
    (6, 127), (6, 128), (7,  16), (6, 129), (6, 130), (6, 131), (6, 132), (6, 133),
    (6, 134), (6, 135), (6, 136), (7,  17), (6, 137), (6, 138), (6, 139), (6, 140),
    (6, 141), (6, 142), (6, 143), (6, 144), (7,  18), (6, 145), (6, 146), (6, 147),
    (6, 148), (6, 149), (6, 150), (6, 151), (6, 152), (7,  19), (6, 153), (6, 154),
    (6, 155), (6, 156), (6, 157), (6, 158), (6, 159), (6, 160), (7,  20), (6, 161),
    (6, 162), (6, 163), (6, 164), (6, 165), (6, 166), (6, 167), (6, 168), (7,  21),
    (6, 169), (6, 170), (6, 171), (6, 172), (6, 173), (6, 174), (6, 175), (6, 176),
    (7,  22), (6, 177), (6, 178), (6, 179), (6, 180), (6, 181), (6, 182), (6, 183),
    (6, 184), (7,  23), (6, 185), (6, 186), (6, 187), (6, 188), (6, 189), (6, 190),
    (6, 191), (6, 192), (7,  24), (6, 193), (6, 194), (6, 195), (6, 196), (6, 197),
    (6, 198), (6, 199), (6, 200), (7,  25), (6, 201), (6, 202), (6, 203), (6, 204),
    (6, 205), (6, 206), (6, 207), (6, 208), (7,  26), (6, 209), (6, 210), (6, 211),
    (6, 212), (6, 213), (6, 214), (6, 215), (6, 216), (7,  27), (6, 217), (6, 218),
    (6, 219), (6, 220), (6, 221), (6, 222), (6, 223), (6, 224), (7,  28), (6, 225),
    (6, 226), (6, 227), (6, 228), (6, 229), (6, 230), (6, 231), (6, 232), (7,  29),
    (6, 233), (6, 234), (6, 235), (6, 236), (6, 237), (6, 238), (6, 239), (6, 240),
    (7,  30), (6, 241), (6, 242), (6, 243), (6, 244), (6, 245), (6, 246), (6, 247),
    (6, 248), (7,  31), (6, 249), (6, 250), (6, 251), (6, 252), (6, 253), (6, 254),
    (6, 255), (7,  32), (7,  33), (7,  34), (7,  35), (7,  36), (7,  37), (7,  38),
    (7,  39), (7,  40), (7,  41), (7,  42), (7,  43), (7,  44), (7,  45), (7,  46),
    (7,  47), (7,  48), (7,  49), (7,  50), (7,  51), (7,  52), (7,  53), (7,  54),
    (7,  55), (7,  56), (7,  57), (7,  58), (7,  59), (7,  60), (7,  61), (7,  62),
    (7,  63), (7,  64), (7,  65), (7,  66), (7,  67), (7,  68), (7,  69), (7,  70),
    (7,  71), (7,  72), (7,  73), (7,  74), (7,  75), (7,  76), (7,  77), (7,  78),
    (7,  79), (7,  80), (7,  81), (7,  82), (7,  83), (7,  84), (7,  85), (7,  86),
    (7,  87), (7,  88), (7,  89), (7,  90), (7,  91), (7,  92), (7,  93), (7,  94),
    (7,  95), (7,  96), (7,  97), (7,  98), (7,  99), (7, 100), (7, 101), (7, 102),
    (7, 103), (7, 104), (7, 105), (7, 106), (7, 107), (7, 108), (7, 109), (7, 110),
    (7, 111), (7, 112), (7, 113), (7, 114), (7, 115), (7, 116), (7, 117), (7, 118),
    (7, 119), (7, 120), (7, 121), (7, 122), (7, 123), (7, 124), (7, 125), (7, 126),
    (7, 127), (7, 128), (7, 129), (7, 130), (7, 131), (7, 132), (7, 133), (7, 134),
    (7, 135), (7, 136), (7, 137), (7, 138), (7, 139), (7, 140), (7, 141), (7, 142),
    (7, 143), (7, 144), (7, 145), (7, 146), (7, 147), (7, 148), (7, 149), (7, 150),
    (7, 151), (7, 152), (7, 153), (7, 154), (7, 155), (7, 156), (7, 157), (7, 158),
    (7, 159), (7, 160), (7, 161), (7, 162), (7, 163), (7, 164), (7, 165), (7, 166),
    (7, 167), (7, 168), (7, 169), (7, 170), (7, 171), (7, 172), (7, 173), (7, 174),
    (7, 175), (7, 176), (7, 177), (7, 178), (7, 179), (7, 180), (7, 181), (7, 182),
    (7, 183), (7, 184), (7, 185), (7, 186), (7, 187), (7, 188), (7, 189), (7, 190),
    (7, 191), (7, 192), (7, 193), (7, 194), (7, 195), (7, 196), (7, 197), (7, 198),
    (7, 199), (7, 200), (7, 201), (7, 202), (7, 203), (7, 204), (7, 205), (7, 206),
    (7, 207), (7, 208), (7, 209), (7, 210), (7, 211), (7, 212), (7, 213), (7, 214),
    (7, 215), (7, 216), (7, 217), (7, 218), (7, 219), (7, 220), (7, 221), (7, 222),
    (7, 223), (7, 224), (7, 225), (7, 226), (7, 227), (7, 228), (7, 229), (7, 230),
    (7, 231), (7, 232), (7, 233), (7, 234), (7, 235), (7, 236), (7, 237), (7, 238),
    (7, 239), (7, 240), (7, 241), (7, 242), (7, 243), (7, 244), (7, 245), (7, 246),
    (7, 247), (7, 248), (7, 249), (7, 250), (7, 251), (7, 252), (7, 253), (7, 254),
    (7, 255),
];
