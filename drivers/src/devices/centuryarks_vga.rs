use crate::adapters;
use crate::configuration;
use crate::device;
use crate::flag;
use crate::properties;
use crate::usb;

use device::Usb;

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq, Eq)]
pub struct Biases {
    // constrast
    pub diff_on: u16,
    pub diff: u16,
    pub diff_off: u16,

    //Bandwidth
    pub fo: u16,
    pub hpf: u16,

    //Advanced
    pub pr: u16,
    pub refr: u16,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq, Eq)]
pub struct RateLimiter {
    pub reference_period_us: u16,
    pub maximum_events_per_period: u32,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq, Eq)]
pub struct Configuration {
    pub biases: Biases,
    pub x_mask: [u64; 10],
    pub y_mask: [u64; 8],
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

const TIMEOUT: std::time::Duration = std::time::Duration::from_secs(1);

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

    const VENDOR_AND_PRODUCT_IDS: &'static [(u16, u16)] = &[(0x31F7, 0x0002),(0x03fd,0x5832)]; // here you could add the vendor id of the prophesee  vga camera

    const PROPERTIES: Self::Properties = Self::Properties {
        name: "CenturyArks VGA",
        width: 640,
        height: 480,
        default_configuration: Self::Configuration {
            biases: Biases {
                // constrast
                diff_on: 0x180,
                diff: 0x128,
                diff_off: 0x0DE,

                //Bandwidth
                fo: 0x5c5,
                hpf: 0x5db,

                //Advanced
                pr: 0x4e2,
                refr: 0x5dc,
            },
            x_mask: [0; 10],
            y_mask: [0; 8],
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
        if type_buffer[0] != 0x00 {
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
        usb::assert_control_transfer(
            &handle,
            0x80,
            0x06,
            0x0300,
            0x0000,
            &[0x04, 0x03, 0x09, 0x04],
            TIMEOUT,
        )?;
        usb::assert_string_descriptor_any(
            &handle,
            0x80,
            0x06,
            0x0301,
            0x0409,
            &[
                // "Prophesee" (UTF-16)
                &[
                    b'P', 0x00, b'r', 0x00, b'o', 0x00, b'p', 0x00, b'h', 0x00, b'e', 0x00, b's',
                    0x00, b'e', 0x00, b'e', 0x00,
                ],
                // "CenturyArks" (UTF-16)
                &[
                    b'C', 0x00, b'e', 0x00, b'n', 0x00, b't', 0x00, b'u', 0x00, b'r', 0x00, b'y',
                    0x00, b'A', 0x00, b'r', 0x00, b'k', 0x00, b's', 0x00,
                ],
                // "SilkyEvCam"
            ],
            TIMEOUT,
        )?;
        usb::assert_control_transfer(
            &handle,
            0x80,
            0x06,
            0x0300,
            0x0000,
            &[0x04, 0x03, 0x09, 0x04],
            TIMEOUT,
        )?;

        usb::assert_string_descriptor_any(
            &handle,
            0x80,
            0x06,
            0x0302,
            0x0409,
            &[
                // "SilkyEvCam Gen3.1 v03.09.00C"
                &[
                    b'S', 0x00, b'i', 0x00, b'l', 0x00, b'k', 0x00, b'y', 0x00, b'E', 0x00, b'v',
                    0x00, b'C', 0x00, b'a', 0x00, b'm', 0x00, b' ', 0x00, b'G', 0x00, b'e', 0x00,
                    b'n', 0x00, b'3', 0x00, b'.', 0x00, b'1', 0x00, b' ', 0x00, b'v', 0x00, b'0',
                    0x00, b'3', 0x00, b'.', 0x00, b'0', 0x00, b'9', 0x00, b'.', 0x00, b'0', 0x00,
                    b'0', 0x00, b'C', 0x00,
                ],
            ],
            TIMEOUT,
        )?;
        request(
            &handle,
            &[0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            TIMEOUT,
        )?; // read release version
        request(
            &handle,
            &[0x7a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            TIMEOUT,
        )?; // read build date
        request(
            &handle,
            &[0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00],
            TIMEOUT,
        )?; // ?
        request(
            &handle,
            &[
                0x03, 0x00, 0x01, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            ],
            TIMEOUT,
        )?; // psee,ccam5_imx636 psee,ccam5_gen42 psee,ccam5_fpga

        request(
            &handle,
            &[0x72, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            TIMEOUT,
        )?; // serial request
        request(
            &handle,
            &[0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00],
            TIMEOUT,
        )?; // ?
        request(
            &handle,
            &[
                0x01, 0x00, 0x01, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            ],
            TIMEOUT,
        )?; // CCam5 FPGA Event-Based Camera
        request(
            &handle,
            &[
                0x03, 0x00, 0x01, 0x00, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
            ],
            TIMEOUT,
        )?; // ti, tmp103
            //
        request(
            &handle,
            &[
                0x01, 0x00, 0x01, 0x00, 0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
            ],
            TIMEOUT,
        )?; // temperature sensor

        // from
        // 1. connection test
        // Ref: Read(0x00000800...) x 3
        for _ in 0..3 {
            let _ = Unknown0800 { value: 0 }.read(&handle);
        }

        // 2. first shutdown and Reset
        // Ref: Write(0x00000000, 0x00000000)
        // AtisControl { value: 0 }.write(&handle)?;
        AtisControl {
            en_vdda: 0,
            en_vddc: 0,
            en_vddd: 0,
            sensor_soft_reset: 0,
            in_evt_no_blocking_mode: 0,
            em_rstn: 0,
            en_ext_ctrl_rstb: 0,
            flip_x_en: 0,
            flip_y_en: 0,
            master_mode: 0,
            sensor_tb_iobuf_en_n: 0,
            sensor_tb_pe_rst_n: 0,
            sisley_hvga_remap_bypass: 0,
            td_rstn: 0,
            use_ext_start: 0,
        }
        .write(&handle)?;

 
        // Ref: Write(0x0000000C, 0x00000000) -> 0x00000001
        Ccam2Trigger { soft_reset: 0 }.write(&handle)?;
        Ccam2Trigger { soft_reset: 1 }.write(&handle)?;

        // 3. (Power-Up Sequence)

        // Ref: Write(0x00000000, 0x00000000)
        AtisControl {
            en_vdda: 0,
            en_vddc: 0,
            en_vddd: 0,
            sensor_soft_reset: 0,
            in_evt_no_blocking_mode: 0,
            em_rstn: 0,
            en_ext_ctrl_rstb: 0,
            flip_x_en: 0,
            flip_y_en: 0,
            master_mode: 0,
            sensor_tb_iobuf_en_n: 0,
            sensor_tb_pe_rst_n: 0,
            sisley_hvga_remap_bypass: 0,
            td_rstn: 0,
            use_ext_start: 0,
        }
        .write(&handle)?;

        // reapet in log
        // AtisControl { value: 0 }.write(&handle)?;

        // Ref: Write(0x00000000, 0x00000008) -> Bit 3 ON (SENSOR_SOFT_RESET)
        AtisControl {
            en_vdda: 0,
            en_vddc: 0,
            en_vddd: 0,
            sensor_soft_reset: 1,
            in_evt_no_blocking_mode: 0,
            em_rstn: 0,
            en_ext_ctrl_rstb: 0,
            flip_x_en: 0,
            flip_y_en: 0,
            master_mode: 0,
            sensor_tb_iobuf_en_n: 0,
            sensor_tb_pe_rst_n: 0,
            sisley_hvga_remap_bypass: 0,
            td_rstn: 0,
            use_ext_start: 0,
        }
        .write(&handle)?;

        AtisControl {
            en_vdda: 0,
            en_vddc: 1,
            en_vddd: 0,
            sensor_soft_reset: 1,
            in_evt_no_blocking_mode: 0,
            em_rstn: 0,
            en_ext_ctrl_rstb: 0,
            flip_x_en: 0,
            flip_y_en: 0,
            master_mode: 0,
            sensor_tb_iobuf_en_n: 0,
            sensor_tb_pe_rst_n: 0,
            sisley_hvga_remap_bypass: 0,
            td_rstn: 0,
            use_ext_start: 0,
        }
        .write(&handle)?;
        AtisControl {
            en_vdda: 0,
            en_vddc: 1,
            en_vddd: 1,
            sensor_soft_reset: 1,
            in_evt_no_blocking_mode: 0,
            em_rstn: 0,
            en_ext_ctrl_rstb: 0,
            flip_x_en: 0,
            flip_y_en: 0,
            master_mode: 0,
            sensor_tb_iobuf_en_n: 0,
            sensor_tb_pe_rst_n: 0,
            sisley_hvga_remap_bypass: 0,
            td_rstn: 0,
            use_ext_start: 0,
        }
        .write(&handle)?;

        AtisControl {
            en_vdda: 0,
            en_vddc: 1,
            en_vddd: 1,
            sensor_soft_reset: 0,
            in_evt_no_blocking_mode: 0,
            em_rstn: 0,
            en_ext_ctrl_rstb: 0,
            flip_x_en: 0,
            flip_y_en: 0,
            master_mode: 0,
            sensor_tb_iobuf_en_n: 0,
            sensor_tb_pe_rst_n: 0,
            sisley_hvga_remap_bypass: 0,
            td_rstn: 0,
            use_ext_start: 0,
        }
        .write(&handle)?;

        AtisControl {
            en_vdda: 0,
            en_vddc: 1,
            en_vddd: 1,
            sensor_soft_reset: 0,
            in_evt_no_blocking_mode: 0,
            em_rstn: 0,
            en_ext_ctrl_rstb: 0,
            flip_x_en: 0,
            flip_y_en: 0,
            master_mode: 1,
            sensor_tb_iobuf_en_n: 0,
            sensor_tb_pe_rst_n: 0,
            sisley_hvga_remap_bypass: 0,
            td_rstn: 0,
            use_ext_start: 0,
        }
        .write(&handle)?;

        // Ref: Write(0x00000000, 0x00001006) -> Bit 12 ON (MASTER_MODE)

        // 4.  clocks config (PLL) - Bloque 0x07xx
        Unknown0768 { value: 0x08010000 }.write(&handle)?;
        Unknown076C { value: 0x00000000 }.write(&handle)?;
        Unknown0770 { value: 0x00000000 }.write(&handle)?;
        Unknown0774 { value: 0x00000000 }.write(&handle)?;
        Unknown0778 { value: 0x00000000 }.write(&handle)?;
        Unknown077C { value: 0x00000000 }.write(&handle)?;
        Unknown0798 { value: 0x003FFFFF }.write(&handle)?;
        Unknown0764 { value: 0x00FFFFFF }.write(&handle)?;

        // 5. Enable Interfaz Host (CCAM2_CONTROL 0x0008)
        // Ref: Write(0x00000008, 0x00000100) -> Bit 8 (HOST_IF_EN)
        Ccam2Control {
            host_if_en: 1,
            stereo_merge_enable: 0,
            enable_out_of_fov: 0,
            th_recovery_bypass: 0,
            ccam_id: 0,
        }
        .write(&handle)?;

        // 6. configuration PHY and sistem
        Unknown1500 { value: 0x0000000D }.write(&handle)?;

        // Ref: Write(0x00000000, 0x00001106)
        // Bits on: VDDC(1), VDDD(2), SISLEY_HVGA_REMAP_BYPASS(8), MASTER_MODE(12)
        AtisControl {
            en_vdda: 0,
            en_vddc: 1,
            en_vddd: 1,
            sensor_soft_reset: 0,
            in_evt_no_blocking_mode: 0,
            em_rstn: 0,
            en_ext_ctrl_rstb: 0,
            flip_x_en: 0,
            flip_y_en: 0,
            master_mode: 1,
            sensor_tb_iobuf_en_n: 0,
            sensor_tb_pe_rst_n: 0,
            sisley_hvga_remap_bypass: 1,
            td_rstn: 0,
            use_ext_start: 0,
        }
        .write(&handle)?;

        AtisControl {
            en_vdda: 0,
            en_vddc: 1,
            en_vddd: 1,
            sensor_soft_reset: 0,
            in_evt_no_blocking_mode: 0,
            em_rstn: 0,
            en_ext_ctrl_rstb: 0,
            flip_x_en: 0,
            flip_y_en: 0,
            master_mode: 1,
            sensor_tb_iobuf_en_n: 0,
            sensor_tb_pe_rst_n: 0,
            sisley_hvga_remap_bypass: 1,
            td_rstn: 0,
            use_ext_start: 0,
        }
        .write(&handle)?;

        Unknown074C { value: 0x00000002 }.write(&handle)?;

        Unknown1508 { value: 0x00000FA0 }.write(&handle)?;
        Unknown1520 { value: 0x00003E80 }.write(&handle)?;
        Unknown150C { value: 0x000001F8 }.write(&handle)?;
        Unknown1510 { value: 0x00002000 }.write(&handle)?;

        Unknown0238 { value: 0x00000001 }.write(&handle)?;
        Unknown0204 { value: 0x0000000B }.write(&handle)?;
        Unknown021C { value: 0x00000100 }.write(&handle)?;

        Unknown0200 { value: 0x00000010 }.write(&handle)?;
        Unknown0200 { value: 0x00000018 }.write(&handle)?;
        Unknown0200 { value: 0x0000001A }.write(&handle)?;

        let _ = Unknown0218 { value: 0 }.read(&handle);

        Unknown0240 { value: 0x00140501 }.write(&handle)?;

        Unknown0208 { value: 0x40020BFF }.write(&handle)?;
        Unknown0208 { value: 0x40020BFF }.write(&handle)?;
        Unknown0208 { value: 0x400203FF }.write(&handle)?;

        Unknown0244 { value: 0x400203FF }.write(&handle)?;
        Unknown0244 { value: 0x400203FF }.write(&handle)?;
        Unknown0244 { value: 0x400203FF }.write(&handle)?;

        Unknown021C { value: 0x10000100 }.write(&handle)?;
        Unknown021C { value: 0x10000101 }.write(&handle)?;

        // 7. load BIASES ( Analogic config - Base 0x0300) I dont know if those are the real biases registers
        // hex exctracted from log C++
        Unknown0300 { value: 0x5903E1B0 }.write(&handle)?;
        Unknown0304 { value: 0x5903E1B0 }.write(&handle)?;
        Unknown0308 { value: 0x5900629B }.write(&handle)?;
        Unknown030C { value: 0x59001BA9 }.write(&handle)?;
        Unknown0310 { value: 0x59014D8C }.write(&handle)?;
        Unknown0314 { value: 0x790DB770 }.write(&handle)?;
        Unknown0318 { value: 0x59014D8C }.write(&handle)?;
        Unknown031C { value: 0x59014DBE }.write(&handle)?;
        Unknown0320 { value: 0x79002C3E }.write(&handle)?;
        Unknown0324 { value: 0x79005436 }.write(&handle)?;
        Unknown0328 { value: 0x59166CB7 }.write(&handle)?;
        Unknown032C { value: 0x79000000 }.write(&handle)?;
        Unknown0330 { value: 0x590000FF }.write(&handle)?;
        Unknown0334 { value: 0x7100002A }.write(&handle)?;
        Unknown0338 { value: 0x7100002F }.write(&handle)?;
        Unknown033C { value: 0x71000021 }.write(&handle)?;
        Unknown0340 { value: 0x590004FF }.write(&handle)?;
        Unknown0344 { value: 0x590000FF }.write(&handle)?;
        Unknown0348 { value: 0x59014DFF }.write(&handle)?;
        BiasDiffOff { value: 0x7900571E }.write(&handle)?;
        BiasDiffOn  { value: 0x79125C35 }.write(&handle)?;
        BiasDiff    { value: 0x7902CE29 }.write(&handle)?;
        BiasFO      { value: 0x71003DD6 }.write(&handle)?;
        BiasPr      { value: 0x51034FB1 }.write(&handle)?;
        BiasRefr    { value: 0x51004FD4 }.write(&handle)?;
        BiasHpf     { value: 0x510006D4 }.write(&handle)?;
        Unknown0368 { value: 0x71000054 }.write(&handle)?;

        // 8. final enable
        Unknown0238 { value: 0x00000001 }.write(&handle)?;
        Unknown0220 { value: 0x00000001 }.write(&handle)?;
        Unknown0204 { value: 0x00000003 }.write(&handle)?;
        Unknown0204 { value: 0x00000003 }.write(&handle)?;

        Unknown0450 { value: 0x00000000 }.write(&handle)?;
        Unknown053C { value: 0x00000000 }.write(&handle)?;

        Unknown0204 { value: 0x00000023 }.write(&handle)?;
        Unknown024C { value: 0x0002BFFF }.write(&handle)?;
        Unknown0248 { value: 0x000000C9 }.write(&handle)?;

        // --- START SEQUENCE ---
        // from openeb/hal_psee_plugins/include/devices/gen31/gen31_evk3_issd.h

        // Ref: WriteField(0x0000, 0x00001107, 0x1) -> activate VDDA (Bit 0)
        // Ref: WriteField(..., 0x00041107, 0x40000) -> activate TD_RSTN (Bit 18)
        // Ref: WriteField(..., 0x000C1107, 0x80000) -> activate EM_RSTN (Bit 19)
        // Valor final acumulado: 0x1106 | 1 | 0x40000 | 0x80000 = 0x000C1107
        AtisControl {
            en_vdda: 1,
            en_vddc: 1,
            en_vddd: 1,
            sensor_soft_reset: 0,
            in_evt_no_blocking_mode: 0,
            em_rstn: 1,
            en_ext_ctrl_rstb: 0,
            flip_x_en: 0,
            flip_y_en: 0,
            master_mode: 1,
            sensor_tb_iobuf_en_n: 0,
            sensor_tb_pe_rst_n: 0,
            sisley_hvga_remap_bypass: 1,
            td_rstn: 1,
            use_ext_start: 0,
        }
        .write(&handle)?;

        // Ref: Write(0x0008, 0x00000300) -> HOST_IF_EN (Bit 8) + STEREO_MERGE (Bit 9)
        Ccam2Control {
            host_if_en: 1,
            stereo_merge_enable: 1,
            th_recovery_bypass: 0,
            enable_out_of_fov: 0,
            ccam_id: 0,
        }
        .write(&handle)?;

        Unknown0238 { value: 0x00000001 }.write(&handle)?;
        Unknown0204 { value: 0x0000002B }.write(&handle)?;

        std::thread::sleep(std::time::Duration::from_micros(250));

        Unknown0248 { value: 0x000000C8 }.write(&handle)?;

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

    fn next_with_timeout(&'_ self, timeout: &std::time::Duration) -> Option<usb::BufferView<'_>> {
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

impl From<rusb::Error> for Error {
    fn from(error: rusb::Error) -> Self {
        usb::Error::from(error).into()
    }
}

// Simplified structure for Reverse Engineering implementation.
// We only need the "Magic Header" and the bit-shift amount.
//
struct BiasGen31Config {
    header_mask: u32, // Example: 0x79000000
    shift: u8,        // Example: 0 for Diffs, 8 for PR/FO/HPF
}

macro_rules! update_bias {
    ($name:ident, $register:ident, $config:expr, $handle:ident, $previous_biases:ident, $biases:expr) => {
        // 1. Change Detection (Optimization)
        // Only write to USB if the value has actually changed since the last update.
        if match $previous_biases {
            Some(prev) => prev.$name != $biases.$name,
            None => true,
        } {
            // 2. Construct Raw Value
            let val = $biases.$name as u32;
            
            // Formula: Header | (Value << Shift)
            // Example Diff: 0x79000000 | (300 << 0) = 0x7900012C
            // Example PR:   0x51000000 | (12 << 8)  = 0x51000C00
            let final_value = $config.header_mask | (val << $config.shift);

            // 3. USB Write
            // Use generic 32-bit 'value' field to allow full payload control.
            $register { value: final_value }.write($handle)?;
        }
    };
}

fn update_configuration(
    handle: &rusb::DeviceHandle<rusb::Context>,
    previous_configuration: Option<&Configuration>,
    configuration: &Configuration,
) -> Result<(), Error> {
    let previous_biases = previous_configuration.map(|c| &c.biases);
    let biases = &configuration.biases;

    // --- COMPARATOR GROUP (Header 0x79, Shift 0) ---
    // These registers accept the value directly in the lower bits (0-11).
    
    update_bias!(
        diff_off, BiasDiffOff,
        BiasGen31Config { header_mask: 0x79000000, shift: 0 },
        handle, previous_biases, biases
    );

    update_bias!(
        diff_on, BiasDiffOn,
        BiasGen31Config { header_mask: 0x79000000, shift: 0 },
        handle, previous_biases, biases
    );

    update_bias!(
        diff, BiasDiff,
        BiasGen31Config { header_mask: 0x79000000, shift: 0 },
        handle, previous_biases, biases
    );

    // --- FILTER GROUP (Header 0x71, Shift 8) ---
    // Sniffing data (e.g., value 0x3D00) showed the value resides in the 2nd byte.
    
    update_bias!(
        fo, BiasFO, // Assumes your 'biases' struct has a 'fo' field
        BiasGen31Config { header_mask: 0x71000000, shift: 8 },
        handle, previous_biases, biases
    );

    // --- FRONT-END GROUP (Header 0x51, Shift 8) ---
    // PR, Refr, and HPF all demonstrated 0x51 headers with shifted values in the logs.

    update_bias!(
        pr, BiasPr,
        BiasGen31Config { header_mask: 0x51000000, shift: 8 },
        handle, previous_biases, biases
    );

    update_bias!(
        refr, BiasRefr,
        BiasGen31Config { header_mask: 0x51000000, shift: 8 },
        handle, previous_biases, biases
    );

    update_bias!(
        hpf, BiasHpf,
        BiasGen31Config { header_mask: 0x51000000, shift: 8 },
        handle, previous_biases, biases
    );

    // NOTE: 'reqpuy' and 'blk' were omitted as they were not confirmed via sniffing.
    // Using Gen4 addresses (0x1XXX) for them would likely fail on this hardware.
    Ok(())
}

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
    let mut buffer = vec![0; 1024];
    let read = handle.read_bulk(0x82, &mut buffer, timeout)?;
    buffer.truncate(read);
    Ok(buffer)
}

#[allow(dead_code)]
struct RuntimeRegister {
    address: u32,
    value: u32,
}

trait Register {
    fn address(&self) -> u32;

    fn value(&self) -> u32;

    #[allow(dead_code)]
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

struct ConfigurationUpdaterContext<IntoError, IntoWarning>
where
    IntoError: From<Error> + Clone + Send,
    IntoWarning: From<crate::usb::Overflow> + Clone + Send,
{
    handle: std::sync::Arc<rusb::DeviceHandle<rusb::Context>>,
    flag: flag::Flag<IntoError, IntoWarning>,
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

// --- System Control Block (Base 0x0000) ---
// Map: ccam5_single_gen31_SystemControlRegisterMap
register! { AtisControl, 0x0000, {
    en_vdda: 0..1,                  // 0: EN_VDDA
    en_vddc: 1..2,                  // 1: EN_VDDC
    en_vddd: 2..3,                  // 2: EN_VDDD
    sensor_soft_reset: 3..4,        // 3: SENSOR_SOFT_RESET
    in_evt_no_blocking_mode: 4..5,  // 4: IN_EVT_NO_BLOCKING_MODE
    // Bits 5-7: reserved/ hollow map
    sisley_hvga_remap_bypass: 8..9, // 8: SISLEY_HVGA_REMAP_BYPASS
    // Bits 9-11: reserved
    master_mode: 12..13,            // 12: MASTER_MODE
    // Bit 13: reserved
    use_ext_start: 14..15,          // 14: USE_EXT_START
    sensor_tb_iobuf_en_n: 15..16,   // 15: SENSOR_TB_IOBUF_EN_N
    sensor_tb_pe_rst_n: 16..17,     // 16: SENSOR_TB_PE_RST_N
    // Bit 17: reserved
    td_rstn: 18..19,                // 18: TD_RSTN
    em_rstn: 19..20,                // 19: EM_RSTN
    en_ext_ctrl_rstb: 20..21,       // 20: EN_EXT_CTRL_RSTB
    flip_x_en: 21..22,              // 21: FLIP_X_EN
    flip_y_en: 22..23,              // 22: FLIP_Y_EN
    // Resto hasta 32 reserved
} }

register! { BoardControlStatus, 0x0004, {
    version: 0..2,                  // 0-1: VERSION
} }

register! { Ccam2Control, 0x0008, {
    // Bits 0-7 reserved
    host_if_en: 8..9,               // 8: HOST_IF_EN
    stereo_merge_enable: 9..10,     // 9: STEREO_MERGE_ENABLE
    // Bit 10 reserved
    enable_out_of_fov: 11..12,      // 11: ENABLE_OUT_OF_FOV
    th_recovery_bypass: 12..13,     // 12: TH_RECOVERY_BYPASS
    ccam_id: 13..14,                // 13: CCAM_ID
} }

register! { Ccam2Trigger, 0x000C, {
    soft_reset: 0..1,               // 0: SOFT_RESET
} }

register! { OutOfFovFilterSize, 0x0010, {
    width: 0..11,                   // 0-10: WIDTH
    // Bits 11-15 reserved
    value: 16..27,                  // 16-26: VALUE (Height)
} }

register! { OutOfFovFilterOrigin, 0x0014, {
    x: 0..11,                       // 0-10: X
    // Bits 11-15 reserved
    y: 16..27,                      // 16-26: Y
} }

register! { EvtRateControl, 0x0018, {
    enable: 0..1,                   // 0: ENABLE
    // Bits 11-15 reserved
    t_drop_factor: 16..32,          // 16-31: T_DROP_FACTOR
} }

// --- graphic registers / Unknown for inicialización ---
// needed for Blocks 0x02xx, 0x07xx, 0x15xx and Biases (0x03xx)
register! { Unknown0800, 0x0800, { value: 0..32 } }
register! { Unknown0768, 0x0768, { value: 0..32 } }
register! { Unknown076C, 0x076C, { value: 0..32 } }
register! { Unknown0770, 0x0770, { value: 0..32 } }
register! { Unknown0774, 0x0774, { value: 0..32 } }
register! { Unknown0778, 0x0778, { value: 0..32 } }
register! { Unknown077C, 0x077C, { value: 0..32 } }
register! { Unknown0798, 0x0798, { value: 0..32 } }
register! { Unknown0764, 0x0764, { value: 0..32 } }
register! { Unknown074C, 0x074C, { value: 0..32 } }

register! { Unknown1500, 0x1500, { value: 0..32 } }
register! { Unknown1508, 0x1508, { value: 0..32 } }
register! { Unknown1520, 0x1520, { value: 0..32 } }
register! { Unknown150C, 0x150C, { value: 0..32 } }
register! { Unknown1510, 0x1510, { value: 0..32 } }

register! { Unknown0238, 0x0238, { value: 0..32 } }
register! { Unknown0220, 0x0220, { value: 0..32 } }
register! { Unknown0204, 0x0204, { value: 0..32 } }
register! { Unknown021C, 0x021C, { value: 0..32 } }
register! { Unknown0200, 0x0200, { value: 0..32 } }
register! { Unknown0218, 0x0218, { value: 0..32 } }
register! { Unknown0240, 0x0240, { value: 0..32 } }
register! { Unknown0208, 0x0208, { value: 0..32 } }
register! { Unknown0244, 0x0244, { value: 0..32 } }
register! { Unknown0248, 0x0248, { value: 0..32 } }
register! { Unknown024C, 0x024C, { value: 0..32 } }
register! { Unknown0450, 0x0450, { value: 0..32 } }
register! { Unknown053C, 0x053C, { value: 0..32 } }

// Bias (Base 0x0300), I dont really know if those are the biases registers
register! { Unknown0300, 0x0300, { value: 0..32 } }
register! { Unknown0304, 0x0304, { value: 0..32 } }
register! { Unknown0308, 0x0308, { value: 0..32 } }
register! { Unknown030C, 0x030C, { value: 0..32 } }
register! { Unknown0310, 0x0310, { value: 0..32 } }
register! { Unknown0314, 0x0314, { value: 0..32 } }
register! { Unknown0318, 0x0318, { value: 0..32 } }
register! { Unknown031C, 0x031C, { value: 0..32 } }
register! { Unknown0320, 0x0320, { value: 0..32 } }
register! { Unknown0324, 0x0324, { value: 0..32 } }
register! { Unknown0328, 0x0328, { value: 0..32 } }
register! { Unknown032C, 0x032C, { value: 0..32 } }
register! { Unknown0330, 0x0330, { value: 0..32 } }
register! { Unknown0334, 0x0334, { value: 0..32 } }
register! { Unknown0338, 0x0338, { value: 0..32 } }
register! { Unknown033C, 0x033c, { value: 0..32 } }
register! { Unknown0340, 0x0340, { value: 0..32 } }
register! { Unknown0344, 0x0344, { value: 0..32 } }
register! { Unknown0348, 0x0348, { value: 0..32 } }
register! { Unknown0368, 0x0368, { value: 0..32 } }

// Defined with 'value: 0..32' to allow full control over the 32-bit payload.
// --- COMPARATOR GROUP (Header 0x79) ---
register! { BiasDiffOff, 0x034C, { value: 0..32 } } // Negative Contrast Threshold
register! { BiasDiffOn,  0x0350, { value: 0..32 } } // Positive Contrast Threshold
register! { BiasDiff,    0x0354, { value: 0..32 } } // Reference Level

// --- FILTER GROUP (Header 0x71) ---
register! { BiasFO,      0x0358, { value: 0..32 } } // Source Follower / Low Pass Filter

// --- FRONT-END GROUP (Header 0x51) ---
register! { BiasPr,      0x035C, { value: 0..32 } } // Photoreceptor Gain
register! { BiasRefr,    0x0360, { value: 0..32 } } // Refractory Period (Dead time)
register! { BiasHpf,     0x0364, { value: 0..32 } } // High Pass Filter
