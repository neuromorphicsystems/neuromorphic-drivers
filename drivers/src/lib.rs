pub mod adapters;
pub mod configuration;
pub mod device;
pub mod devices;
pub mod flag;
pub mod properties;
pub mod usb;

pub use adapters::Adapter;
pub use device::Usb as UsbDevice;
pub use devices::list_devices;
pub use devices::open;
pub use devices::Configuration;
pub use devices::Device;
pub use devices::Error;
pub use devices::Properties;
pub use devices::Type;
pub use flag::Flag;
pub use usb::Configuration as UsbConfiguration;
pub use usb::Overflow as UsbOverflow;

pub use devices::prophesee_evk3_hd;
pub use devices::prophesee_evk4;

pub use bincode;
pub use libc;
pub use libusb1_sys;
pub use neuromorphic_types as types;
pub use rusb;

pub fn event_loop_and_flag(
) -> Result<(Flag<Error, usb::Overflow>, std::sync::Arc<usb::EventLoop>), usb::Error> {
    let flag = Flag::new();
    let event_loop = std::sync::Arc::new(usb::EventLoop::new(
        std::time::Duration::from_millis(100),
        flag.clone(),
    )?);
    Ok((flag, event_loop))
}
