use crate::adapters;
use crate::configuration;
use crate::device;
use crate::flag;
use crate::properties;
use crate::usb;

use device::Usb;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DvsOrientation {
    pub flip_x: bool,
    pub flip_y: bool,
    pub invert_xy: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ImuOrientation {
    pub flip_x: bool,
    pub flip_y: bool,
    pub flip_z: bool,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq, Eq)]
pub struct Biases {
    pub amp: u8, //in [0, 8]
    pub on: u8,  //in [0, 17]
    pub off: u8, //in [0, 17]
    pub sf: u8,  // in [0, 3]
    pub nrst: bool,
    pub log: bool,
    pub log_a: bool,
    pub log_d: u8, // in [0, 2]
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq, Eq)]
pub enum ReadoutFramesPerSecond {
    Constant100,
    Constant200,
    Constant500,
    Constant1000,
    ConstantLossy2000,
    ConstantLossy5000,
    ConstantLossy10000,
    Variable2000,
    Variable5000,
    Variable10000,
    Variable15000,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq, Eq)]
pub struct Configuration {
    pub biases: Biases,
    readout_frames_per_second: ReadoutFramesPerSecond,
}

pub struct Device {
    handle: std::sync::Arc<rusb::DeviceHandle<rusb::Context>>,
    ring: usb::Ring,
    configuration_updater: configuration::Updater<Configuration>,
    vendor_and_product_id: (u16, u16),
    serial: String,
    dvs_orientation: DvsOrientation,
    imu_orientation: ImuOrientation,
}

#[derive(thiserror::Error, Debug, Clone)]
pub enum Error {
    #[error(transparent)]
    Usb(#[from] usb::Error),

    #[error("Firmware version not supported (neuromorphic-drivers expects version 0.9 or later, the camera runs on version {minor}.{subminor})")]
    FirmwareVersion { minor: u8, subminor: u8 },

    #[error("Logic version not supported (neuromorphic-drivers expects version 18, the camera runs on version {0})")]
    LogicVersion(u32),

    #[error("Logic patch not supported (neuromorphic-drivers expects version 1, the camera runs on version {0})")]
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
    type Adapter = adapters::dvxplorer::Adapter;

    type Configuration = Configuration;

    type Error = Error;

    type Properties = properties::Camera<Self::Configuration>;

    const VENDOR_AND_PRODUCT_IDS: &'static [(u16, u16)] = &[(0x152A, 0x8419)];

    const PROPERTIES: Self::Properties = Self::Properties {
        name: "iniVation DVXplorer",
        width: 640,
        height: 480,
        default_configuration: Self::Configuration {
            biases: Biases {
                amp: 4,
                on: 8,
                off: 8,
                sf: 0,
                nrst: false,
                log: false,
                log_a: true,
                log_d: 1,
            },
            readout_frames_per_second: ReadoutFramesPerSecond::Variable5000,
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
        if device_version.minor() == 0 && device_version.sub_minor() < 9 {
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
        if logic_patch != 4 {
            return Err(Error::LogicPatch(logic_patch));
        }

        // @DEV @TEMP {
        println!("DVS_SIZE_COLUMNS={}", DVS_SIZE_COLUMNS.get(&handle)?);
        println!("DVS_SIZE_ROWS={}", DVS_SIZE_ROWS.get(&handle)?);
        println!("IMU_TYPE={}", IMU_TYPE.get(&handle)?);
        // }

        let dvs_orientation = {
            let dvs_orientation = DVS_ORIENTATION.get(&handle)?;
            DvsOrientation {
                flip_x: ((dvs_orientation >> 1) & 1) == 1,
                flip_y: (dvs_orientation & 1) == 1,
                invert_xy: ((dvs_orientation >> 2) & 1) == 1,
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

        let logic_clock_register = LOGIC_CLOCK.get(&handle)?;
        let usb_clock_register = USB_CLOCK.get(&handle)?;
        let clock_deviation = CLOCK_DEVIATION.get(&handle)?;
        let logic_clock = (logic_clock_register as f64) * (clock_deviation as f64 / 1000.0);
        let usb_clock = (usb_clock_register as f64) * (clock_deviation as f64 / 1000.0);

        DVS_RUN.set(&handle, 0)?;
        DEVICE_CONTROL_MODE.set(&handle, 0)?;
        IMU_ACCELEROMETER_RUN.set(&handle, 0)?;
        IMU_GYROSCOPE_RUN.set(&handle, 0)?;
        IMU_TEMPERATURE_RUN.set(&handle, 0)?;
        EXTERNAL_INPUT_RUN.set(&handle, 0)?;
        EXTERNAL_INPUT_GENERATOR_RUN.set(&handle, 0)?;
        MULTIPLEXER_RUN.set(&handle, 0)?;
        MULTIPLEXER_TIMESTAMP_RUN.set(&handle, 0)?;
        USB_RUN.set(&handle, 0)?;
        MULTIPLEXER_CHIP_RUN.set(&handle, 0)?;
        std::thread::sleep(std::time::Duration::from_millis(100));
        handle.clear_halt(0x82)?;
        MULTIPLEXER_CHIP_RUN.set(&handle, 1)?;
        std::thread::sleep(std::time::Duration::from_millis(10));
        DEVICE_BIAS_OTP_TRIM.set(&handle, 0x24)?;
        DEVICE_BIAS_PINS_DBGP.set(&handle, 0x00)?;
        DEVICE_BIAS_PINS_DBGN.set(&handle, 0x00)?;
        DEVICE_BIAS_PINS_BUFP.set(&handle, 0x03)?;
        DEVICE_BIAS_PINS_BUFN.set(&handle, 0x7F)?;
        DEVICE_BIAS_PINS_DOB.set(&handle, 0)?;

        // @DEV use config to set (move into update configuration) {
        DEVICE_RANGE_SF_ON_RST.set(
            &handle,
            (if configuration.biases.nrst { 0b1 } else { 0 })
                | (if configuration.biases.on > 8 { 0b10 } else { 0 })
                | (if configuration.biases.sf > 1 {
                    0b100
                } else {
                    0
                })
                | (if configuration.biases.log { 0b1000 } else { 0 }),
        )?;
        DEVICE_RANGE_LOGA_LOGD_MONITOR.set(
            &handle,
            ((configuration.biases.log_d & 0b11) << 2)
                | (if configuration.biases.log_a {
                    0b10000
                } else {
                    0
                }),
        )?;
        DEVICE_LEVEL_SF_OFF.set(
            &handle,
            0b01101101
                | (if configuration.biases.off < 9 {
                    0b10
                } else {
                    0
                })
                | (if configuration.biases.sf == 1 || configuration.biases.sf > 2 {
                    0b10000
                } else {
                    0
                }),
        )?;
        DEVICE_BIAS_AMP.set(&handle, configuration.biases.amp.max(8))?;
        DEVICE_BIAS_ON.set(
            &handle,
            if configuration.biases.on < 9 {
                configuration.biases.on
            } else {
                (configuration.biases.on - 9).max(8)
            },
        )?;
        DEVICE_BIAS_OFF.set(
            &handle,
            if configuration.biases.off < 9 {
                configuration.biases.off
            } else {
                (configuration.biases.off - 9).max(8)
            },
        )?;
        // }

        DEVICE_CONTROL_CLOCK_DIVIDER_SYS.set(&handle, 0xA0)?;
        DEVICE_CONTROL_PARALLEL_OUT_CONTROL.set(&handle, 0)?;
        DEVICE_CONTROL_PARALLEL_OUT_ENABLE.set(&handle, 1)?;
        DEVICE_CONTROL_PACKET_FORMAT.set(&handle, 0x80)?;
        DEVICE_DIGITAL_MODE_CONTROL.set(&handle, 0b1100)?;
        DEVICE_DIGITAL_BOOT_SEQUENCE.set(&handle, 0x08)?;
        DEVICE_DIGITAL_TIMESTAMP_REFUNIT.set(&handle, 999)?;
        DEVICE_DIGITAL_TIMESTAMP_SUBUNIT.set(&handle, SYSTEM_CLOCK_FREQUENCY_MHZ - 1)?;
        DEVICE_DIGITAL_DTAG_REFERENCE.set(&handle, SYSTEM_CLOCK_FREQUENCY_MHZ)?;
        DEVICE_TIMING_GH_COUNT_FINE.set(&handle, SYSTEM_CLOCK_FREQUENCY_MHZ)?;
        DEVICE_TIMING_GRS_COUNT_FINE.set(&handle, SYSTEM_CLOCK_FREQUENCY_MHZ)?;
        DEVICE_TIMING_GRS_END_FINE.set(&handle, SYSTEM_CLOCK_FREQUENCY_MHZ)?;
        DEVICE_TIMING_GRS_COUNT.set(&handle, 0)?;
        DEVICE_TIMING_GRS_END.set(&handle, 1)?;
        DEVICE_TIMING_FIRST_SELX_START.set(&handle, 4)?;
        DEVICE_TIMING_SELX_WIDTH.set(&handle, 6)?;
        DEVICE_TIMING_AY_START.set(&handle, 4)?;
        DEVICE_TIMING_AY_END.set(&handle, 6)?;
        DEVICE_TIMING_R_START.set(&handle, 8)?;
        DEVICE_TIMING_R_END.set(&handle, 10)?;
        DEVICE_TIMING_NEXT_GH_CNT.set(&handle, 4)?;
        DEVICE_ACTIVITY_DECISION_BYPASS.set(&handle, 1)?;
        DEVICE_SPATIAL_HISTOGRAM_OFF.set(&handle, 1)?;
        DEVICE_DIGITAL_EXTERNAL_TRIGGER.set(&handle, 0)?;
        DEVICE_DIGITAL_GLOBAL_RESET_READOUT.set(&handle, 0)?;
        DEVICE_DIGITAL_ENABLE.set(&handle, 0b10)?;
        DEVICE_DIGITAL_DUAL_BINNING.set(&handle, 0)?;
        DEVICE_DIGITAL_MODE_CONTROL.set(&handle, 0b1101)?;
        DEVICE_DIGITAL_SUBSAMPLE_RATIO.set(&handle, 0b000000)?;

        DEVICE_DIGITAL_RESTART.set(&handle, 0)?;
        DEVICE_DIGITAL_FIXED_READ_TIME.set(&handle, 0)?;
        DEVICE_TIMING_READ_TIME_INTERVAL.set(&handle, SYSTEM_CLOCK_FREQUENCY_MHZ as u16 * 900)?;
        DEVICE_TIMING_GH_COUNT_THOUSANDS.set(&handle, 0)?;
        DEVICE_TIMING_GH_COUNT.set(&handle, 7)?;
        DEVICE_DIGITAL_RESTART.set(&handle, 1)?; // start

        // @DEV ROI should go in update configuration too {
        DEVICE_CROPPER_BYPASS.set(&handle, 1)?;
        DEVICE_CROPPER_X_START_ADDRESS.set(&handle, 0)?;
        DEVICE_CROPPER_X_END_ADDRESS.set(&handle, PROPERTIES.width - 1)?;
        DEVICE_CROPPER_Y_START_GROUP.set(&handle, 0)?;
        DEVICE_CROPPER_Y_START_MASK.set(&handle, 0b11111111)?;
        DEVICE_CROPPER_Y_END_GROUP.set(&handle, ((PROPERTIES.height - 1) / 8) as u8)?;
        DEVICE_CROPPER_Y_END_MASK.set(&handle, 0b11111111)?;
        DEVICE_CROPPER_BYPASS.set(&handle, 0)?;
        // }

        DEVICE_DIGITAL_RESTART.set(&handle, 2)?; // restart
        MULTIPLEXER_DROP_EXTERNAL_INPUT_ON_STALL.set(&handle, 1)?;
        MULTIPLEXER_DROP_DVS_ON_STALL.set(&handle, 0)?;

        IMU_ACCELEROMETER_DATA_RATE.set(&handle, 6)?;
        IMU_ACCELEROMETER_FILTER.set(&handle, 2)?;
        IMU_ACCELEROMETER_RANGE.set(&handle, 1)?;
        IMU_GYROSCOPE_DATA_RATE.set(&handle, 5)?;
        IMU_GYROSCOPE_FILTER.set(&handle, 2)?;
        IMU_GYROSCOPE_RANGE.set(&handle, 2)?;
        EXTERNAL_INPUT_DETECT_RISING_EDGES.set(&handle, 0)?;
        EXTERNAL_INPUT_DETECT_FALLING_EDGES.set(&handle, 0)?;
        EXTERNAL_INPUT_GENERATE_PULSE_LENGTH.set(&handle, (5.0 * logic_clock).round() as u32)?;
        EXTERNAL_INPUT_GENERATE_PULSE_INTERVAL.set(&handle, (5.0 * logic_clock).round() as u32)?;
        EXTERNAL_INPUT_GENERATE_INJECT_ON_RISING_EDGE.set(&handle, 0)?;
        EXTERNAL_INPUT_GENERATE_INJECT_ON_FALLING_EDGE.set(&handle, 0)?;
        USB_EARLY_PACKET_DELAY.set(&handle, (1000.0 * usb_clock).round() as u32)?;

        USB_RUN.set(&handle, 1)?;
        MULTIPLEXER_TIMESTAMP_RUN.set(&handle, 1)?;
        MULTIPLEXER_RUN.set(&handle, 1)?;
        std::thread::sleep(std::time::Duration::from_millis(10));
        DVS_RUN.set(&handle, 1)?;
        DEVICE_CONTROL_MODE.set(&handle, 2)?; // stream
        IMU_ACCELEROMETER_RUN.set(&handle, 1)?;
        IMU_GYROSCOPE_RUN.set(&handle, 1)?;
        IMU_TEMPERATURE_RUN.set(&handle, 1)?;
        EXTERNAL_INPUT_RUN.set(&handle, 1)?;
        EXTERNAL_INPUT_GENERATOR_RUN.set(&handle, 1)?;

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
                ConfigurationUpdaterContext { handle, flag },
                |context, previous_configuration, configuration| {
                    let result = {
                        update_configuration(
                            &context.handle,
                            Some(previous_configuration),
                            configuration,
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
            dvs_orientation,
            imu_orientation,
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
        Self::Adapter::new(self.dvs_orientation(), self.imu_orientation())
    }
}

impl Device {
    pub fn dvs_orientation(&self) -> DvsOrientation {
        self.dvs_orientation
    }

    pub fn imu_orientation(&self) -> ImuOrientation {
        self.imu_orientation
    }
}

impl Drop for Device {
    fn drop(&mut self) {
        let _ = DVS_RUN.set(&self.handle, 0);
        let _ = DEVICE_CONTROL_MODE.set(&self.handle, 0);
        let _ = IMU_ACCELEROMETER_RUN.set(&self.handle, 0);
        let _ = IMU_GYROSCOPE_RUN.set(&self.handle, 0);
        let _ = IMU_TEMPERATURE_RUN.set(&self.handle, 0);
        let _ = EXTERNAL_INPUT_RUN.set(&self.handle, 0);
        let _ = EXTERNAL_INPUT_GENERATOR_RUN.set(&self.handle, 0);
        let _ = MULTIPLEXER_RUN.set(&self.handle, 0);
        let _ = MULTIPLEXER_TIMESTAMP_RUN.set(&self.handle, 0);
        let _ = USB_RUN.set(&self.handle, 0);
        let _ = MULTIPLEXER_CHIP_RUN.set(&self.handle, 0);
    }
}

const TIMEOUT: std::time::Duration = std::time::Duration::from_secs(1);

fn update_configuration(
    handle: &rusb::DeviceHandle<rusb::Context>,
    previous_configuration: Option<&Configuration>,
    configuration: &Configuration,
) -> Result<(), Error> {
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

#[repr(u16)]
#[derive(Clone, Copy)]
enum ModuleAddress {
    Multiplexer = 0,
    Dvs = 1,
    Imu = 3,
    ExternalInput = 4,
    Device = 5,
    SystemInformation = 6,
    Usb = 9,
}

struct SpiRegister {
    module_address: ModuleAddress,
    parameter_address: u16,
}

impl SpiRegister {
    fn get(&self, handle: &rusb::DeviceHandle<rusb::Context>) -> Result<u8, Error> {
        let mut buffer = [0; 4];
        let count = handle.read_control(
            0xC0,
            0xBF,
            self.module_address as u16,
            self.parameter_address,
            &mut buffer,
            TIMEOUT,
        )?;
        if count != 4 {
            return Err(Error::ShortRead {
                module_address: self.module_address as u16,
                parameter_address: self.parameter_address,
                expected: 4,
                count,
            });
        }
        Ok(buffer[3])
    }

    fn set(&self, handle: &rusb::DeviceHandle<rusb::Context>, value: u8) -> Result<(), Error> {
        let buffer = (value as u32).to_be_bytes();
        let count = handle.write_control(
            0x40,
            0xBF,
            self.module_address as u16,
            self.parameter_address,
            &buffer,
            TIMEOUT,
        )?;
        if count != 4 {
            return Err(Error::ShortWrite {
                module_address: self.module_address as u16,
                parameter_address: self.parameter_address,
                expected: 4,
                count,
            });
        }
        Ok(())
    }

    const fn new(module_address: ModuleAddress, parameter_address: u16) -> Self {
        Self {
            module_address,
            parameter_address,
        }
    }
}

struct SpiRegister16 {
    module_address: ModuleAddress,
    parameter_address: u16,
}

impl SpiRegister16 {
    fn set(&self, handle: &rusb::DeviceHandle<rusb::Context>, value: u16) -> Result<(), Error> {
        let bytes = value.to_be_bytes();
        SpiRegister::new(self.module_address, self.parameter_address).set(handle, bytes[0])?;
        SpiRegister::new(self.module_address, self.parameter_address + 1).set(handle, bytes[1])?;
        Ok(())
    }

    const fn new(module_address: ModuleAddress, parameter_address: u16) -> Self {
        Self {
            module_address,
            parameter_address,
        }
    }
}

struct SpiRegister24 {
    module_address: ModuleAddress,
    parameter_address: u16,
}

impl SpiRegister24 {
    fn set(&self, handle: &rusb::DeviceHandle<rusb::Context>, value: u32) -> Result<(), Error> {
        let bytes = value.to_be_bytes();
        SpiRegister::new(self.module_address, self.parameter_address).set(handle, bytes[1])?;
        SpiRegister::new(self.module_address, self.parameter_address + 1).set(handle, bytes[2])?;
        SpiRegister::new(self.module_address, self.parameter_address + 2).set(handle, bytes[3])?;
        Ok(())
    }

    const fn new(module_address: ModuleAddress, parameter_address: u16) -> Self {
        Self {
            module_address,
            parameter_address,
        }
    }
}

struct SpiRegister32 {
    module_address: ModuleAddress,
    parameter_address: u16,
}

impl SpiRegister32 {
    fn get(&self, handle: &rusb::DeviceHandle<rusb::Context>) -> Result<u32, Error> {
        let mut buffer = [0; 4];
        let count = handle.read_control(
            0xC0,
            0xBF,
            self.module_address as u16,
            self.parameter_address,
            &mut buffer,
            TIMEOUT,
        )?;
        if count != 4 {
            return Err(Error::ShortRead {
                module_address: self.module_address as u16,
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
            self.module_address as u16,
            self.parameter_address,
            &buffer,
            TIMEOUT,
        )?;
        if count != 4 {
            return Err(Error::ShortWrite {
                module_address: self.module_address as u16,
                parameter_address: self.parameter_address,
                expected: 4,
                count,
            });
        }
        Ok(())
    }

    const fn new(module_address: ModuleAddress, parameter_address: u16) -> Self {
        Self {
            module_address,
            parameter_address,
        }
    }
}

const SYSTEM_CLOCK_FREQUENCY_MHZ: u8 = 50;

// multiplexer module registers
const MULTIPLEXER_RUN: SpiRegister32 = SpiRegister32::new(ModuleAddress::Multiplexer, 0);
const MULTIPLEXER_TIMESTAMP_RUN: SpiRegister32 = SpiRegister32::new(ModuleAddress::Multiplexer, 1);
const MULTIPLEXER_CHIP_RUN: SpiRegister32 = SpiRegister32::new(ModuleAddress::Multiplexer, 3);
const MULTIPLEXER_DROP_EXTERNAL_INPUT_ON_STALL: SpiRegister32 =
    SpiRegister32::new(ModuleAddress::Multiplexer, 4);
const MULTIPLEXER_DROP_DVS_ON_STALL: SpiRegister32 =
    SpiRegister32::new(ModuleAddress::Multiplexer, 5);

// device registers
const DEVICE_RANGE_SF_ON_RST: SpiRegister = SpiRegister::new(ModuleAddress::Device, 11);
const DEVICE_RANGE_LOGA_LOGD_MONITOR: SpiRegister = SpiRegister::new(ModuleAddress::Device, 12);
const DEVICE_BIAS_OTP_TRIM: SpiRegister = SpiRegister::new(ModuleAddress::Device, 13);
const DEVICE_BIAS_PINS_DBGP: SpiRegister = SpiRegister::new(ModuleAddress::Device, 15);
const DEVICE_BIAS_PINS_DBGN: SpiRegister = SpiRegister::new(ModuleAddress::Device, 16);
const DEVICE_LEVEL_SF_OFF: SpiRegister = SpiRegister::new(ModuleAddress::Device, 18);
const DEVICE_BIAS_PINS_BUFP: SpiRegister = SpiRegister::new(ModuleAddress::Device, 19);
const DEVICE_BIAS_PINS_BUFN: SpiRegister = SpiRegister::new(ModuleAddress::Device, 20);
const DEVICE_BIAS_PINS_DOB: SpiRegister = SpiRegister::new(ModuleAddress::Device, 21);
const DEVICE_BIAS_AMP: SpiRegister = SpiRegister::new(ModuleAddress::Device, 24);
const DEVICE_BIAS_ON: SpiRegister = SpiRegister::new(ModuleAddress::Device, 28);
const DEVICE_BIAS_OFF: SpiRegister = SpiRegister::new(ModuleAddress::Device, 30);
const DEVICE_CONTROL_MODE: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x3000);
const DEVICE_CONTROL_CLOCK_DIVIDER_SYS: SpiRegister =
    SpiRegister::new(ModuleAddress::Device, 0x3011);
const DEVICE_CONTROL_PARALLEL_OUT_CONTROL: SpiRegister =
    SpiRegister::new(ModuleAddress::Device, 0x3019);
const DEVICE_CONTROL_PARALLEL_OUT_ENABLE: SpiRegister =
    SpiRegister::new(ModuleAddress::Device, 0x301E);
const DEVICE_CONTROL_PACKET_FORMAT: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x3067);
const DEVICE_DIGITAL_ENABLE: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x3200);
const DEVICE_DIGITAL_RESTART: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x3201);
const DEVICE_DIGITAL_DUAL_BINNING: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x3202);
const DEVICE_DIGITAL_SUBSAMPLE_RATIO: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x3204);
const DEVICE_DIGITAL_TIMESTAMP_SUBUNIT: SpiRegister =
    SpiRegister::new(ModuleAddress::Device, 0x3234);
const DEVICE_DIGITAL_TIMESTAMP_REFUNIT: SpiRegister16 =
    SpiRegister16::new(ModuleAddress::Device, 0x3235);
const DEVICE_TIMING_FIRST_SELX_START: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x323C);
const DEVICE_DIGITAL_DTAG_REFERENCE: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x323D);
const DEVICE_TIMING_GH_COUNT_THOUSANDS: SpiRegister =
    SpiRegister::new(ModuleAddress::Device, 0x3240);
const DEVICE_TIMING_GH_COUNT: SpiRegister16 = SpiRegister16::new(ModuleAddress::Device, 0x3241);
const DEVICE_TIMING_GH_COUNT_FINE: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x3243);
const DEVICE_TIMING_GRS_COUNT: SpiRegister24 = SpiRegister24::new(ModuleAddress::Device, 0x3244);
const DEVICE_TIMING_GRS_COUNT_FINE: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x3247);
const DEVICE_DIGITAL_GLOBAL_RESET_READOUT: SpiRegister =
    SpiRegister::new(ModuleAddress::Device, 0x3248);
const DEVICE_TIMING_NEXT_GH_CNT: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x324B);
const DEVICE_TIMING_SELX_WIDTH: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x324C);
const DEVICE_TIMING_AY_START: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x324E);
const DEVICE_TIMING_AY_END: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x324F);
const DEVICE_TIMING_MAX_EVENT_NUM: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x3251);
const DEVICE_TIMING_R_START: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x3253);
const DEVICE_TIMING_R_END: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x3254);
const DEVICE_DIGITAL_MODE_CONTROL: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x3255);
const DEVICE_TIMING_GRS_END: SpiRegister24 = SpiRegister24::new(ModuleAddress::Device, 0x3256);
const DEVICE_TIMING_GRS_END_FINE: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x3259);
const DEVICE_DIGITAL_FIXED_READ_TIME: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x325C);
const DEVICE_TIMING_READ_TIME_INTERVAL: SpiRegister16 =
    SpiRegister16::new(ModuleAddress::Device, 0x325D);
const DEVICE_DIGITAL_EXTERNAL_TRIGGER: SpiRegister =
    SpiRegister::new(ModuleAddress::Device, 0x3260);
const DEVICE_TIMING_NEXT_SELX_START: SpiRegister16 =
    SpiRegister16::new(ModuleAddress::Device, 0x3261);
const DEVICE_DIGITAL_BOOT_SEQUENCE: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x3266);
const DEVICE_CROPPER_BYPASS: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x3300);
const DEVICE_CROPPER_Y_START_GROUP: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x3301);
const DEVICE_CROPPER_Y_START_MASK: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x3302);
const DEVICE_CROPPER_Y_END_GROUP: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x3303);
const DEVICE_CROPPER_Y_END_MASK: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x3304);
const DEVICE_CROPPER_X_START_ADDRESS: SpiRegister16 =
    SpiRegister16::new(ModuleAddress::Device, 0x3305);
const DEVICE_CROPPER_X_END_ADDRESS: SpiRegister16 =
    SpiRegister16::new(ModuleAddress::Device, 0x3307);
const DEVICE_ACTIVITY_DECISION_BYPASS: SpiRegister =
    SpiRegister::new(ModuleAddress::Device, 0x3500);
const DEVICE_SPATIAL_HISTOGRAM_OFF: SpiRegister = SpiRegister::new(ModuleAddress::Device, 0x3600);

// dvs module registers
const DVS_SIZE_COLUMNS: SpiRegister32 = SpiRegister32::new(ModuleAddress::Dvs, 0);
const DVS_SIZE_ROWS: SpiRegister32 = SpiRegister32::new(ModuleAddress::Dvs, 1);
const DVS_ORIENTATION: SpiRegister32 = SpiRegister32::new(ModuleAddress::Dvs, 2);
const DVS_RUN: SpiRegister32 = SpiRegister32::new(ModuleAddress::Dvs, 3);

// imu module registers
const IMU_TYPE: SpiRegister32 = SpiRegister32::new(ModuleAddress::Imu, 0);
const IMU_ORIENTATION: SpiRegister32 = SpiRegister32::new(ModuleAddress::Imu, 1);
const IMU_ACCELEROMETER_RUN: SpiRegister32 = SpiRegister32::new(ModuleAddress::Imu, 2);
const IMU_GYROSCOPE_RUN: SpiRegister32 = SpiRegister32::new(ModuleAddress::Imu, 3);
const IMU_TEMPERATURE_RUN: SpiRegister32 = SpiRegister32::new(ModuleAddress::Imu, 4);
const IMU_ACCELEROMETER_DATA_RATE: SpiRegister32 = SpiRegister32::new(ModuleAddress::Imu, 5);
const IMU_ACCELEROMETER_FILTER: SpiRegister32 = SpiRegister32::new(ModuleAddress::Imu, 6);
const IMU_ACCELEROMETER_RANGE: SpiRegister32 = SpiRegister32::new(ModuleAddress::Imu, 7);
const IMU_GYROSCOPE_DATA_RATE: SpiRegister32 = SpiRegister32::new(ModuleAddress::Imu, 8);
const IMU_GYROSCOPE_FILTER: SpiRegister32 = SpiRegister32::new(ModuleAddress::Imu, 9);
const IMU_GYROSCOPE_RANGE: SpiRegister32 = SpiRegister32::new(ModuleAddress::Imu, 10);

// external input module registers
const EXTERNAL_INPUT_RUN: SpiRegister32 = SpiRegister32::new(ModuleAddress::ExternalInput, 0);
const EXTERNAL_INPUT_DETECT_RISING_EDGES: SpiRegister32 =
    SpiRegister32::new(ModuleAddress::ExternalInput, 1);
const EXTERNAL_INPUT_DETECT_FALLING_EDGES: SpiRegister32 =
    SpiRegister32::new(ModuleAddress::ExternalInput, 2);
const EXTERNAL_INPUT_GENERATOR_RUN: SpiRegister32 =
    SpiRegister32::new(ModuleAddress::ExternalInput, 11);
const EXTERNAL_INPUT_GENERATE_PULSE_INTERVAL: SpiRegister32 =
    SpiRegister32::new(ModuleAddress::ExternalInput, 13);
const EXTERNAL_INPUT_GENERATE_PULSE_LENGTH: SpiRegister32 =
    SpiRegister32::new(ModuleAddress::ExternalInput, 14);
const EXTERNAL_INPUT_GENERATE_INJECT_ON_RISING_EDGE: SpiRegister32 =
    SpiRegister32::new(ModuleAddress::ExternalInput, 15);
const EXTERNAL_INPUT_GENERATE_INJECT_ON_FALLING_EDGE: SpiRegister32 =
    SpiRegister32::new(ModuleAddress::ExternalInput, 16);

// system information module registers
const LOGIC_VERSION: SpiRegister32 = SpiRegister32::new(ModuleAddress::SystemInformation, 0);
const CHIP_IDENTIFIER: SpiRegister32 = SpiRegister32::new(ModuleAddress::SystemInformation, 1);
const DEVICE_IS_MASTER: SpiRegister32 = SpiRegister32::new(ModuleAddress::SystemInformation, 2);
const LOGIC_CLOCK: SpiRegister32 = SpiRegister32::new(ModuleAddress::SystemInformation, 3);
const USB_CLOCK: SpiRegister32 = SpiRegister32::new(ModuleAddress::SystemInformation, 5);
const CLOCK_DEVIATION: SpiRegister32 = SpiRegister32::new(ModuleAddress::SystemInformation, 6);
const LOGIC_PATCH: SpiRegister32 = SpiRegister32::new(ModuleAddress::SystemInformation, 7);

// usb module registers
const USB_RUN: SpiRegister32 = SpiRegister32::new(ModuleAddress::Usb, 0);
const USB_EARLY_PACKET_DELAY: SpiRegister32 = SpiRegister32::new(ModuleAddress::Usb, 1);
