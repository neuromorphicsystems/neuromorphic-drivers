use crate::adapters;
use crate::configuration;
use crate::device;
use crate::ethernet;
use crate::flag;
use crate::properties;
use crate::ring;
use crate::usb;

use device::Device as _;

use std::os::fd::AsRawFd;

const CONTROL_CHANNEL_PRIVILEGE_CONTROL: i32 = 2;
const STREAM_CHANNEL_PACKET_SIZE_ADDRESS: u32 = 0x0000_0D04;
const DEVICE_TEMPERATURE_ADDRESS: u32 = 0x1010_0030;
const DEVICE_TEMPERATURE_DIVIDER: f32 = 16.0;
const HEARTBEAT_TIMEOUT_VALUE: i32 = 6000;

const GVSP_PAYLOAD_PACKET_FORMAT: u8 = 0x83;
const GVSP_HEADER_LENGTH: usize = 20;
const GVSP_FORMAT_INDEX: usize = 4;
const GVSP_BLOCK_ID_RANGE: std::ops::Range<usize> = 8..16;
const GVSP_PACKET_ID_RANGE: std::ops::Range<usize> = 16..20;
const GVSP_RECEIVE_BUFFER_SIZE: usize = 1 << 24;
const IPV4_HEADER_LENGTH: usize = 20;
const UDP_HEADER_LENGTH: usize = 8;
const GVSP_PACKET_OVERHEAD: usize = IPV4_HEADER_LENGTH + UDP_HEADER_LENGTH + GVSP_HEADER_LENGTH;
const GVSP_MINIMUM_PACKET_SIZE: usize = 576;
const GVSP_PARALLEL_SUBMISSIONS: usize = 1;
const GVSP_READ_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(100);

const START_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(250);
const HEARTBEAT_PERIOD: std::time::Duration = std::time::Duration::from_secs(1);

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq, Eq)]
pub struct Biases {
    pub fo: i16,
    pub hpf: i16,
    pub diff_on: i16,
    pub diff: i16,
    pub diff_off: i16,
    pub refr: i16,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct BiasesBounds {
    pub fo: properties::Bounds<i16>,
    pub hpf: properties::Bounds<i16>,
    pub diff_on: properties::Bounds<i16>,
    pub diff: properties::Bounds<i16>,
    pub diff_off: properties::Bounds<i16>,
    pub refr: properties::Bounds<i16>,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq, Eq)]
pub struct Configuration {
    pub biases: Biases,
    pub enable_output: bool,
}

#[derive(thiserror::Error, Debug, Clone)]
pub enum Error {
    #[error(transparent)]
    Usb(#[from] usb::Error),
    #[error("network error: {0}")]
    Io(String),
    #[error("ethernet error: {0}")]
    Ethernet(String),
    #[error("\"{0}\" is not a valid IPv4 address")]
    Address(String),
    #[error("no Lucid Triton found")]
    NotFound,
    #[error("Lucid Triton cameras cannot be identified by a USB bus number and address")]
    BusNumberAndAddress,
    #[error("no Lucid Triton found with the serial number \"{0}\"")]
    SerialNotFound(String),
    #[error("the buffer length ({0}) is the GVSP packet size and must be at least {1}")]
    PacketSizeTooSmall(usize, usize),
    #[error("the camera rejected the GVSP packet size {requested} (it reports {actual})")]
    PacketSizeRejected { requested: usize, actual: usize },
    #[error(
        "GVSP packets are read one at a time, the number of parallel submissions must be {1} (got {0})"
    )]
    ParallelSubmissions(usize, usize),
}

impl From<std::io::Error> for Error {
    fn from(error: std::io::Error) -> Self {
        Self::Io(error.to_string())
    }
}

impl From<ethernet::Error> for Error {
    fn from(error: ethernet::Error) -> Self {
        Self::Ethernet(error.to_string())
    }
}

pub const PROPERTIES: properties::Camera<Configuration> = Device::PROPERTIES;

pub const DEFAULT_CONFIGURATION: Configuration = Configuration {
    biases: Biases {
        fo: 0,
        hpf: 0,
        diff_on: 0,
        diff: 0,
        diff_off: 0,
        refr: 0,
    },
    enable_output: true,
};

pub const RING_CONFIGURATION: ring::Configuration = Device::RING_CONFIGURATION;

struct DiscoveredCamera {
    local_address: std::net::Ipv4Addr,
    discovery: ethernet::Discovery,
}

struct ControlChannel {
    gvcp: std::sync::Arc<std::sync::Mutex<ethernet::Ethernet>>,
    camera_address: std::net::Ipv4Addr,
}

impl ControlChannel {
    fn take(
        local_address: std::net::Ipv4Addr,
        camera_address: std::net::Ipv4Addr,
    ) -> Result<Self, Error> {
        let mut gvcp = ethernet::Ethernet::new(local_address, false)?;
        ControlChannelPrivilege {
            value: CONTROL_CHANNEL_PRIVILEGE_CONTROL,
        }
        .write(&mut gvcp, camera_address)?;
        Ok(Self {
            gvcp: std::sync::Arc::new(std::sync::Mutex::new(gvcp)),
            camera_address,
        })
    }

    fn gvcp(&self) -> std::sync::MutexGuard<'_, ethernet::Ethernet> {
        self.gvcp.lock().expect("gvcp mutex is not poisoned")
    }

    fn share(&self) -> std::sync::Arc<std::sync::Mutex<ethernet::Ethernet>> {
        self.gvcp.clone()
    }
}

impl Drop for ControlChannel {
    fn drop(&mut self) {
        let _ = ControlChannelPrivilege { value: 0 }.write(&mut self.gvcp(), self.camera_address);
    }
}

struct Streaming {
    running: std::sync::Arc<std::sync::atomic::AtomicBool>,
    heartbeat_thread: Option<std::thread::JoinHandle<()>>,
    gvsp_thread: Option<std::thread::JoinHandle<()>>,
    control_channel: ControlChannel,
}

impl Drop for Streaming {
    fn drop(&mut self) {
        self.running
            .store(false, std::sync::atomic::Ordering::Release);
        if let Some(thread) = self.gvsp_thread.take() {
            let _ = thread.join();
        }
        if let Some(thread) = self.heartbeat_thread.take() {
            let _ = thread.join();
        }
        let _ = AcquisitionStop { value: 1 }.write_with_timeout(
            &mut self.control_channel.gvcp(),
            self.control_channel.camera_address,
            START_TIMEOUT,
        );
    }
}

pub struct Device {
    streaming: Streaming,
    dropped_packets: std::sync::Arc<std::sync::atomic::AtomicU64>,
    biases_bounds: BiasesBounds,
    configuration_updater: configuration::Updater<Configuration>,
    ring: ring::SharedRing,
    serial: String,
    address: std::net::Ipv4Addr,
}

pub fn open<IntoError, IntoWarning>(
    identifier: device::Identifier,
    configuration: Configuration,
    ring_configuration: &ring::Configuration,
    flag: flag::Flag<IntoError, IntoWarning>,
) -> Result<Device, Error>
where
    IntoError: From<Error> + Clone + Send + 'static,
    IntoWarning: From<ring::Overflow> + Clone + Send + 'static,
{
    <Device as device::Ethernet>::open(identifier, configuration, ring_configuration, flag)
}

impl Device {
    pub fn address(&self) -> std::net::Ipv4Addr {
        self.address
    }

    pub fn temperature_celsius(&self) -> Result<device::TemperatureCelsius, Error> {
        Ok(device::TemperatureCelsius(
            self.streaming.control_channel.gvcp().read_register(
                DEVICE_TEMPERATURE_ADDRESS,
                self.address,
                ethernet::DEFAULT_TIMEOUT,
                ethernet::ATTEMPTS,
            )? as i32 as f32
                / DEVICE_TEMPERATURE_DIVIDER,
        ))
    }
}

impl device::Device for Device {
    type Adapter = adapters::evt3::Adapter;

    type BiasesBounds = BiasesBounds;

    type Configuration = Configuration;

    type Error = Error;

    type Properties = properties::Camera<Self::Configuration>;

    const PROPERTIES: Self::Properties = Self::Properties {
        name: "Lucid Triton",
        width: 1280,
        height: 720,
        default_configuration: DEFAULT_CONFIGURATION,
    };

    const RING_CONFIGURATION: ring::Configuration = ring::Configuration {
        buffer_length: 1400,
        ring_length: 1 << 17,
        parallel_submissions: 1,
    };

    fn default_configuration(&self) -> Self::Configuration {
        PROPERTIES.default_configuration.clone()
    }

    fn biases_bounds(&self) -> Self::BiasesBounds {
        self.biases_bounds
    }

    fn current_configuration(&self) -> Self::Configuration {
        self.configuration_updater.current_configuration()
    }

    fn update_configuration(&self, configuration: Self::Configuration) {
        self.configuration_updater.update(configuration);
    }


    fn next_with_timeout(&self, timeout: &std::time::Duration) -> Option<ring::ReadBufferView<'_>> {
        self.ring.next_with_timeout(timeout)
    }

    fn backlog(&self) -> usize {
        self.ring.backlog()
    }

    fn clutch(&self) -> ring::Clutch {
        self.ring.clutch()
    }

    fn serial(&self) -> String {
        self.serial.clone()
    }

    fn connection(&self) -> crate::devices::Connection {
        crate::devices::Connection::Ethernet
    }

    fn dropped_packets(&self) -> u64 {
        self.ring.dropped_packets().saturating_add(
            self.dropped_packets
                .load(std::sync::atomic::Ordering::Acquire),
        )
    }

    fn create_adapter(&self) -> Self::Adapter {
        Self::Adapter::from_dimensions(Self::PROPERTIES.width, Self::PROPERTIES.height)
    }
}

impl device::Ethernet for Device {
    fn list_devices() -> Vec<device::ListedDevice> {
        discover()
            .into_iter()
            .map(|camera| device::ListedDevice {
                location: device::Location::Address(camera.discovery.ip_address),
                connection: crate::devices::Connection::Ethernet,
                serial: Ok(camera.discovery.serial_number),
            })
            .collect()
    }
    fn open<IntoError, IntoWarning>(
        identifier: device::Identifier,
        configuration: Self::Configuration,
        ring_configuration: &ring::Configuration,
        flag: flag::Flag<IntoError, IntoWarning>,
    ) -> Result<Self, Self::Error>
    where
        IntoError: From<Self::Error> + Clone + Send + 'static,
        IntoWarning: From<crate::ring::Overflow> + Clone + Send + 'static,
    {
        if ring_configuration.parallel_submissions != GVSP_PARALLEL_SUBMISSIONS {
            return Err(Error::ParallelSubmissions(
                ring_configuration.parallel_submissions,
                GVSP_PARALLEL_SUBMISSIONS,
            ));
        }
        if ring_configuration.buffer_length < GVSP_MINIMUM_PACKET_SIZE {
            return Err(Error::PacketSizeTooSmall(
                ring_configuration.buffer_length,
                GVSP_MINIMUM_PACKET_SIZE,
            ));
        }

        let camera = match identifier {
            device::Identifier::Location(device::Location::Address(address)) => {
                discover_at(address).ok_or(Error::NotFound)?
            }
            device::Identifier::Location(device::Location::BusNumberAndAddress { .. }) => {
                return Err(Error::BusNumberAndAddress)
            }
            device::Identifier::Serial(serial) => discover()
                .into_iter()
                .find(|camera| camera.discovery.serial_number == serial)
                .ok_or_else(|| Error::SerialNotFound(serial.to_owned()))?,
            device::Identifier::None => discover().into_iter().next().ok_or(Error::NotFound)?,
        };
        let camera_address = camera.discovery.ip_address;
        let serial = camera.discovery.serial_number.clone();

        let control_channel = ControlChannel::take(camera.local_address, camera_address)?;

        let biases_bounds = {
            let mut gvcp = control_channel.gvcp();
            BiasesBounds {
                fo: BiasFo::bounds(&mut gvcp, camera_address)?,
                hpf: BiasHpf::bounds(&mut gvcp, camera_address)?,
                diff_on: BiasDiffOn::bounds(&mut gvcp, camera_address)?,
                diff: BiasDiff::bounds(&mut gvcp, camera_address)?,
                diff_off: BiasDiffOff::bounds(&mut gvcp, camera_address)?,
                refr: BiasRefr::bounds(&mut gvcp, camera_address)?,
            }
        };

        StreamChannelPacketSize {
            value: ring_configuration.buffer_length as i32,
        }
        .write(&mut control_channel.gvcp(), camera_address)?;
        let packet_size = control_channel.gvcp().read_register(
            STREAM_CHANNEL_PACKET_SIZE_ADDRESS,
            camera_address,
            ethernet::DEFAULT_TIMEOUT,
            ethernet::ATTEMPTS,
        )? as usize;
        if packet_size != ring_configuration.buffer_length {
            return Err(Error::PacketSizeRejected {
                requested: ring_configuration.buffer_length,
                actual: packet_size,
            });
        }

        let gvsp_socket = std::net::UdpSocket::bind((camera.local_address, 0))?;
        gvsp_socket.set_read_timeout(Some(GVSP_READ_TIMEOUT))?;
        set_receive_buffer_size(&gvsp_socket, GVSP_RECEIVE_BUFFER_SIZE);
        let gvsp_local = match gvsp_socket.local_addr()? {
            std::net::SocketAddr::V4(address) => address,
            std::net::SocketAddr::V6(_) => {
                return Err(Error::Io("GVSP socket is not IPv4".to_owned()))
            }
        };
        StreamChannelDestinationAddress {
            value: u32::from(*gvsp_local.ip()) as i32,
        }
        .write(&mut control_channel.gvcp(), camera_address)?;
        StreamChannelHostPort {
            value: gvsp_local.port() as i32,
        }
        .write(&mut control_channel.gvcp(), camera_address)?;

        update_configuration(&mut control_channel.gvcp(), camera_address, None, &configuration)?;
        AcquisitionStart { value: 1 }.write_with_timeout(
            &mut control_channel.gvcp(),
            camera_address,
            START_TIMEOUT,
        )?;

        let warning_flag = flag.clone();
        let (ring, mut write_buffer_views) = ring::SharedRing::new(&ring::Configuration {
            buffer_length: ring_configuration.buffer_length - GVSP_PACKET_OVERHEAD,
            ring_length: ring_configuration.ring_length,
            parallel_submissions: GVSP_PARALLEL_SUBMISSIONS,
        })
        .map_err(usb::Error::from)?;
        let mut producer = Producer {
            ring: ring.clone(),
            write_buffer_view: write_buffer_views
                .pop()
                .expect("the ring yields one write buffer view per parallel submission"),
            on_overflow: Box::new(move || {
                warning_flag.store_warning_if_not_set(crate::ring::Overflow(()));
            }),
        };

        let dropped_packets = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0));
        let gvsp_dropped_packets = dropped_packets.clone();
        let running = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
        let heartbeat_running = running.clone();
        let heartbeat_gvcp = control_channel.share();
        let updater_gvcp = control_channel.share();
        let heartbeat_flag = flag.clone();
        let heartbeat_thread = std::thread::spawn(move || {
            while heartbeat_running.load(std::sync::atomic::Ordering::Acquire) {
                std::thread::sleep(HEARTBEAT_PERIOD);
                if !heartbeat_running.load(std::sync::atomic::Ordering::Acquire) {
                    break;
                }
                let result = HeartbeatTimeout {
                    value: HEARTBEAT_TIMEOUT_VALUE,
                }
                .write_with_timeout(
                    &mut heartbeat_gvcp.lock().expect("gvcp mutex is not poisoned"),
                    camera_address,
                    START_TIMEOUT,
                );
                if let Err(error) = result {
                    heartbeat_flag.store_error_if_not_set(error);
                    break;
                }
            }
        });

        let gvsp_running = running.clone();
        let gvsp_flag = flag.clone();
        let gvsp_thread = std::thread::spawn(move || {
            let mut header = [0u8; GVSP_HEADER_LENGTH];
            let mut previous_identifier: Option<(u64, u32)> = None;
            while gvsp_running.load(std::sync::atomic::Ordering::Acquire) {
                let count = {
                    let buffer = producer.buffer();
                    let mut scattered = [
                        libc::iovec {
                            iov_base: header.as_mut_ptr() as *mut libc::c_void,
                            iov_len: header.len(),
                        },
                        libc::iovec {
                            iov_base: buffer.as_mut_ptr() as *mut libc::c_void,
                            iov_len: buffer.len(),
                        },
                    ];
                    // unsafe: recvmsg writes at most iov_len bytes to each destination,
                    // which are both borrowed for the duration of the call
                    unsafe {
                        let mut message: libc::msghdr = std::mem::zeroed();
                        message.msg_iov = scattered.as_mut_ptr();
                        message.msg_iovlen = scattered.len() as _;
                        libc::recvmsg(gvsp_socket.as_raw_fd(), &mut message, 0)
                    }
                };
                if count < 0 {
                    let error = std::io::Error::last_os_error();
                    if matches!(
                        error.kind(),
                        std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock
                    ) {
                        continue;
                    }
                    gvsp_flag.store_error_if_not_set(Error::from(error));
                    break;
                }
                let count = count as usize;
                if count < GVSP_HEADER_LENGTH
                    || header[GVSP_FORMAT_INDEX] != GVSP_PAYLOAD_PACKET_FORMAT
                {
                    continue;
                }
                let identifier = (
                    u64::from_be_bytes(header[GVSP_BLOCK_ID_RANGE].try_into().expect("8 bytes")),
                    u32::from_be_bytes(header[GVSP_PACKET_ID_RANGE].try_into().expect("4 bytes")),
                );
                if follows(previous_identifier, identifier) {
                    previous_identifier = Some(identifier);
                    producer.commit(
                        count - GVSP_HEADER_LENGTH,
                        std::time::SystemTime::now(),
                        std::time::Instant::now(),
                    );
                } else {
                    gvsp_dropped_packets.fetch_add(1, std::sync::atomic::Ordering::AcqRel);
                }
            }
        });

        Ok(Device {
            dropped_packets,
            biases_bounds,
            streaming: Streaming {
                running,
                heartbeat_thread: Some(heartbeat_thread),
                gvsp_thread: Some(gvsp_thread),
                control_channel,
            },
            configuration_updater: configuration::Updater::new(
                configuration,
                ConfigurationUpdaterContext {
                    gvcp: updater_gvcp,
                    camera_address,
                    flag,
                },
                |context, previous_configuration, configuration| {
                    let result = {
                        let mut gvcp = context.gvcp.lock().expect("gvcp mutex is not poisoned");
                        update_configuration(
                            &mut gvcp,
                            context.camera_address,
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
            ring,
            serial,
            address: camera_address,
        })
    }
}

struct ConfigurationUpdaterContext<IntoError, IntoWarning>
where
    IntoError: From<Error> + Clone + Send,
    IntoWarning: From<ring::Overflow> + Clone + Send,
{
    gvcp: std::sync::Arc<std::sync::Mutex<ethernet::Ethernet>>,
    camera_address: std::net::Ipv4Addr,
    flag: flag::Flag<IntoError, IntoWarning>,
}

macro_rules! update_bias {
    ($name:ident, $register:ident, $gvcp:ident, $camera_address:ident, $previous_biases:ident, $biases:expr) => {
        if match $previous_biases {
            Some(previous_biases) => previous_biases.$name != $biases.$name,
            None => true,
        } {
            $register {
                value: $biases.$name as i32,
            }
            .write($gvcp, $camera_address)?;
        }
    };
}

fn update_configuration(
    gvcp: &mut ethernet::Ethernet,
    camera_address: std::net::Ipv4Addr,
    previous_configuration: Option<&Configuration>,
    configuration: &Configuration,
) -> Result<(), Error> {
    let previous_biases = previous_configuration.map(|configuration| &configuration.biases);
    update_bias!(diff_on, BiasDiffOn, gvcp, camera_address, previous_biases, configuration.biases);
    update_bias!(diff_off, BiasDiffOff, gvcp, camera_address, previous_biases, configuration.biases);
    update_bias!(diff, BiasDiff, gvcp, camera_address, previous_biases, configuration.biases);
    update_bias!(refr, BiasRefr, gvcp, camera_address, previous_biases, configuration.biases);
    update_bias!(fo, BiasFo, gvcp, camera_address, previous_biases, configuration.biases);
    update_bias!(hpf, BiasHpf, gvcp, camera_address, previous_biases, configuration.biases);
    Ok(())
}

fn discover() -> Vec<DiscoveredCamera> {
    let mut result: Vec<DiscoveredCamera> = Vec::new();
    for (local_address, netmask) in local_ipv4_interfaces() {
        let mut gvcp = match ethernet::Ethernet::new(local_address, true) {
            Ok(gvcp) => gvcp,
            Err(_) => continue,
        };
        let subnet_broadcast =
            std::net::Ipv4Addr::from(u32::from(local_address) | !u32::from(netmask));
        for broadcast in [
            Some(std::net::Ipv4Addr::BROADCAST),
            (subnet_broadcast != std::net::Ipv4Addr::BROADCAST
                && subnet_broadcast != local_address)
                .then_some(subnet_broadcast),
        ]
        .into_iter()
        .flatten()
        {
            for discovery in gvcp.discovery(broadcast, ethernet::DISCOVERY_TIMEOUT) {
                if !result
                    .iter()
                    .any(|camera| camera.discovery.mac_address == discovery.mac_address)
                {
                    result.push(DiscoveredCamera {
                        local_address,
                        discovery,
                    });
                }
            }
        }
    }
    result
}

fn discover_at(address: std::net::Ipv4Addr) -> Option<DiscoveredCamera> {
    let local_address = local_address_for(address)?;
    ethernet::Ethernet::new(local_address, false)
        .ok()?
        .discovery(address, ethernet::DISCOVERY_TIMEOUT)
        .into_iter()
        .next()
        .map(|discovery| DiscoveredCamera {
            local_address,
            discovery,
        })
}

fn local_address_for(remote_address: std::net::Ipv4Addr) -> Option<std::net::Ipv4Addr> {
    let socket = std::net::UdpSocket::bind((std::net::Ipv4Addr::UNSPECIFIED, 0)).ok()?;
    socket.connect((remote_address, ethernet::GVCP_PORT)).ok()?;
    match socket.local_addr().ok()? {
        std::net::SocketAddr::V4(address) => Some(*address.ip()),
        std::net::SocketAddr::V6(_) => None,
    }
}

struct Producer {
    ring: ring::SharedRing,
    write_buffer_view: ring::WriteBufferView,
    on_overflow: Box<dyn Fn() + Send>,
}

unsafe impl Send for Producer {}

impl Producer {
    fn buffer(&mut self) -> &mut [u8] {
        // unsafe:
        // - the ring guarantees that this buffer is not read whilst the producer holds it
        // - the buffer outlives the slice (the producer owns an arc to the ring data)
        unsafe {
            std::slice::from_raw_parts_mut(
                self.write_buffer_view.data,
                self.write_buffer_view.capacity,
            )
        }
    }

    fn commit(
        &mut self,
        length: usize,
        system_time: std::time::SystemTime,
        instant: std::time::Instant,
    ) {
        let next_write_buffer_view = self.ring.data().transfer_complete(
            self.write_buffer_view.clutch,
            system_time,
            instant,
            length,
        );
        if matches!(self.write_buffer_view.clutch, ring::Clutch::Disengaged) {
            self.ring.notify_one();
        }
        if next_write_buffer_view.clutch_changed
            && matches!(next_write_buffer_view.clutch, ring::Clutch::Engaged)
        {
            (self.on_overflow)();
        }
        self.write_buffer_view = next_write_buffer_view;
    }
}

fn follows(previous: Option<(u64, u32)>, current: (u64, u32)) -> bool {
    match previous {
        Some((previous_block_id, previous_packet_id)) => {
            if current.0 == previous_block_id {
                current.1.wrapping_sub(previous_packet_id) as i32 > 0
            } else {
                current.0.wrapping_sub(previous_block_id) as i64 > 0
            }
        }
        None => true,
    }
}

fn set_receive_buffer_size(socket: &std::net::UdpSocket, size: usize) {
    let value = size as libc::c_int;
    unsafe {
        libc::setsockopt(
            socket.as_raw_fd(),
            libc::SOL_SOCKET,
            libc::SO_RCVBUF,
            &value as *const libc::c_int as *const libc::c_void,
            std::mem::size_of::<libc::c_int>() as libc::socklen_t,
        );
    }
}

fn local_ipv4_interfaces() -> Vec<(std::net::Ipv4Addr, std::net::Ipv4Addr)> {
    let mut result = Vec::new();
    let mut ifap: *mut libc::ifaddrs = std::ptr::null_mut();
    unsafe {
        if libc::getifaddrs(&mut ifap) != 0 {
            return result;
        }
        let mut current = ifap;
        while !current.is_null() {
            let interface = &*current;
            if !interface.ifa_addr.is_null()
                && (*interface.ifa_addr).sa_family as i32 == libc::AF_INET
                && interface.ifa_flags & (libc::IFF_LOOPBACK as u32) == 0
            {
                let socket_address = &*(interface.ifa_addr as *const libc::sockaddr_in);
                let address =
                    std::net::Ipv4Addr::from(u32::from_be(socket_address.sin_addr.s_addr));
                let netmask = if interface.ifa_netmask.is_null() {
                    std::net::Ipv4Addr::BROADCAST
                } else {
                    let socket_netmask = &*(interface.ifa_netmask as *const libc::sockaddr_in);
                    std::net::Ipv4Addr::from(u32::from_be(socket_netmask.sin_addr.s_addr))
                };
                if !address.is_loopback() {
                    result.push((address, netmask));
                }
            }
            current = interface.ifa_next;
        }
        libc::freeifaddrs(ifap);
    }
    result
}

trait Register {
    fn address(&self) -> u32;

    fn value(&self) -> u32;

    fn write(
        &self,
        gvcp: &mut ethernet::Ethernet,
        camera_address: std::net::Ipv4Addr,
    ) -> Result<(), Error> {
        self.write_with_timeout(gvcp, camera_address, ethernet::DEFAULT_TIMEOUT)
    }

    fn write_with_timeout(
        &self,
        gvcp: &mut ethernet::Ethernet,
        camera_address: std::net::Ipv4Addr,
        timeout: std::time::Duration,
    ) -> Result<(), Error> {
        gvcp.write_register(
            self.address(),
            self.value(),
            camera_address,
            timeout,
            ethernet::ATTEMPTS,
        )?;
        Ok(())
    }
}

macro_rules! register {
    ($name:ident, $address:literal) => {
        struct $name {
            value: i32,
        }
        impl Register for $name {
            fn address(&self) -> u32 {
                $address
            }
            fn value(&self) -> u32 {
                self.value as u32
            }
        }
    };
}

macro_rules! bias_register {
    ($name:ident, $address:literal) => {
        register! { $name, $address }
        impl $name {
            fn bounds(
                gvcp: &mut ethernet::Ethernet,
                camera_address: std::net::Ipv4Addr,
            ) -> Result<properties::Bounds<i16>, Error> {
                Ok(properties::Bounds::new(
                    read_bound(gvcp, camera_address, $address + 4)?,
                    read_bound(gvcp, camera_address, $address + 8)?,
                ))
            }
        }
    };
}

fn read_bound(
    gvcp: &mut ethernet::Ethernet,
    camera_address: std::net::Ipv4Addr,
    address: u32,
) -> Result<i16, Error> {
    Ok((gvcp.read_register(
        address,
        camera_address,
        ethernet::DEFAULT_TIMEOUT,
        ethernet::ATTEMPTS,
    )? as i32)
        .clamp(i16::MIN as i32, i16::MAX as i32) as i16)
}

register! { ControlChannelPrivilege, 0x0000_0A00 }
register! { HeartbeatTimeout, 0x0000_0938 }
register! { StreamChannelHostPort, 0x0000_0D00 }
register! { StreamChannelPacketSize, 0x0000_0D04 }
register! { StreamChannelDestinationAddress, 0x0000_0D18 }
register! { AcquisitionStart, 0x1030_0004 }
register! { AcquisitionStop, 0x1030_0008 }
bias_register! { BiasDiffOn, 0x13F3_0000 }
bias_register! { BiasDiffOff, 0x13F3_0014 }
bias_register! { BiasDiff, 0x13F3_0028 }
bias_register! { BiasRefr, 0x13F3_003C }
bias_register! { BiasFo, 0x13F3_0050 }
bias_register! { BiasHpf, 0x13F3_0064 }
