pub const GVCP_PORT: u16 = 3956;
pub const DEFAULT_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(25);
pub const DISCOVERY_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(200);
pub const ATTEMPTS: u8 = 3;
pub const PENDING_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(200);

const HEADER_LENGTH: usize = 8;
const MAXIMUM_MESSAGE_LENGTH: usize = 65507;
const PENDING_ACKNOWLEDGE: u16 = 0x0089;
const ERROR_ACKNOWLEDGE: u16 = 0x0041;
const DISCOVERY_PAYLOAD_LENGTH: usize = 248;

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error(transparent)]
    Io(#[from] std::io::Error),

    #[error(
        "non-zero status ({status:#06X}, {status_string}) in message type {message_type:#06X}"
    )]
    Status {
        status: u16,
        status_string: String,
        message_type: u16,
    },

    #[error("short message ({0} bytes)")]
    ShortMessage(usize),

    #[error("invalid message type ({0:#06X})")]
    InvalidMessageType(u16),

    #[error("unexpected message type (expected {expected:#06X}, got {actual:#06X})")]
    UnexpectedMessageType { expected: u16, actual: u16 },

    #[error("unexpected request id (expected {expected:#06X}, got {actual:#06X})")]
    UnexpectedRequestId { expected: u16, actual: u16 },

    #[error("the header length ({header}) and the message length ({message}) are different")]
    LengthMismatch { header: usize, message: usize },

    #[error("unexpected length (expected {expected}, got {actual}) for message type {message_type:#06X}")]
    UnexpectedLength {
        expected: usize,
        actual: usize,
        message_type: u16,
    },

    #[error("timeout for message type {message_type:#06X} ({attempts} attempts)")]
    Timeout { message_type: u16, attempts: u8 },

    #[error(
        "the device kept requesting acknowledge extensions for more than {timeout:?} for message type {message_type:#06X}"
    )]
    PendingTimeout {
        message_type: u16,
        timeout: std::time::Duration,
    },

    #[error("short read mem message ({0} bytes)")]
    ShortReadMemMessage(usize),

    #[error("invalid read mem address (expected {expected:#010X}, got {actual:#010X})")]
    InvalidReadMemAddress { expected: u32, actual: u32 },
}

impl Error {
    fn is_timeout(&self) -> bool {
        match self {
            Self::Io(error) => matches!(
                error.kind(),
                std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock
            ),
            _ => false,
        }
    }
}

#[repr(u16)]
#[derive(Clone, Copy, Debug)]
pub enum Command {
    Discovery = 0x0002,
    ForceIp = 0x0004,
    PacketResend = 0x0040,
    ReadReg = 0x0080,
    WriteReg = 0x0082,
    ReadMem = 0x0084,
    WriteMem = 0x0086,
    Event = 0x00C0,
    EventData = 0x00C2,
    Action = 0x0100,
}

fn status_to_string(status: u16) -> String {
    (match status {
        0x0000 => "success",
        0x0100 => "packet resend",
        0x8001 => "not implemented",
        0x8002 => "invalid parameter",
        0x8003 => "invalid address",
        0x8004 => "write protect",
        0x8005 => "bad alignment",
        0x8006 => "access denied",
        0x8007 => "status busy",
        0x8008..=0x800B => "deprecated",
        0x800C => "packet unavailable",
        0x800D => "data overrun",
        0x800E => "invalid header",
        0x800F => "deprecated",
        0x8010 => "packet not yet available",
        0x8011 => "packet and previous removed from memory",
        0x8012 => "packet removed from memory",
        0x8013 => "no reference time",
        0x8014 => "packet temporarily unavailable",
        0x8015 => "overflow",
        0x8016 => "action late",
        0x8017 => "leader trailer overflow",
        0x8FFF => "generic error",
        _ => "unknown",
    })
    .to_string()
}

#[derive(Clone, Debug)]
pub struct Discovery {
    pub spec_version: (u16, u16),
    pub mode: u32,
    pub mac_address: [u8; 6],
    pub ip_configuration_options: u32,
    pub ip_configuration: u32,
    pub ip_address: std::net::Ipv4Addr,
    pub subnet_mask: std::net::Ipv4Addr,
    pub gateway: std::net::Ipv4Addr,
    pub manufacturer_name: String,
    pub model_name: String,
    pub device_version: String,
    pub manufacturer_specific_information: String,
    pub serial_number: String,
    pub user_defined_name: String,
}

impl Discovery {
    fn from_payload(payload: &[u8]) -> Self {
        Self {
            spec_version: (
                u16::from_be_bytes(payload[0..2].try_into().expect("2 bytes")),
                u16::from_be_bytes(payload[2..4].try_into().expect("2 bytes")),
            ),
            mode: u32::from_be_bytes(payload[4..8].try_into().expect("4 bytes")),
            mac_address: payload[10..16].try_into().expect("6 bytes"),
            ip_configuration_options: u32::from_be_bytes(
                payload[16..20].try_into().expect("4 bytes"),
            ),
            ip_configuration: u32::from_be_bytes(payload[20..24].try_into().expect("4 bytes")),
            ip_address: std::net::Ipv4Addr::from(
                <[u8; 4]>::try_from(&payload[36..40]).expect("4 bytes"),
            ),
            subnet_mask: std::net::Ipv4Addr::from(
                <[u8; 4]>::try_from(&payload[52..56]).expect("4 bytes"),
            ),
            gateway: std::net::Ipv4Addr::from(
                <[u8; 4]>::try_from(&payload[68..72]).expect("4 bytes"),
            ),
            manufacturer_name: String::from_utf8_lossy(trim_null_end(&payload[72..104]))
                .to_string(),
            model_name: String::from_utf8_lossy(trim_null_end(&payload[104..136])).to_string(),
            device_version: String::from_utf8_lossy(trim_null_end(&payload[136..168])).to_string(),
            manufacturer_specific_information: String::from_utf8_lossy(trim_null_end(
                &payload[168..216],
            ))
            .to_string(),
            serial_number: String::from_utf8_lossy(trim_null_end(&payload[216..232])).to_string(),
            user_defined_name: String::from_utf8_lossy(trim_null_end(&payload[232..248]))
                .to_string(),
        }
    }
}

const fn trim_null_end(slice: &[u8]) -> &[u8] {
    let mut result = slice;
    while let [rest @ .., last] = result {
        if *last == 0u8 {
            result = rest;
        } else {
            break;
        }
    }
    result
}

pub struct Ethernet {
    socket: std::net::UdpSocket,
    buffer: Vec<u8>,
    request_id: u16,
}

impl Ethernet {
    pub fn new(local_address: std::net::Ipv4Addr, broadcast: bool) -> Result<Self, std::io::Error> {
        let socket = std::net::UdpSocket::bind((local_address, 0))?;
        socket.set_nonblocking(false)?;
        if broadcast {
            socket.set_broadcast(true)?;
        }
        Ok(Self {
            socket,
            buffer: vec![0u8; MAXIMUM_MESSAGE_LENGTH],
            request_id: 0xFFFF,
        })
    }

    fn payload(&self, length: usize) -> &[u8] {
        &self.buffer[HEADER_LENGTH..HEADER_LENGTH + length]
    }

    fn send(
        &mut self,
        command: Command,
        payload: &[u8],
        remote_address: std::net::Ipv4Addr,
        timeout: std::time::Duration,
    ) -> Result<(), std::io::Error> {
        self.socket.set_write_timeout(Some(timeout))?;
        self.request_id = if self.request_id == 0xFFFF {
            1
        } else {
            self.request_id + 1
        };
        self.buffer[0] = 0x42;
        self.buffer[1] = 0x01;
        self.buffer[2..4].copy_from_slice(&(command as u16).to_be_bytes());
        self.buffer[4..6].copy_from_slice(&(payload.len() as u16).to_be_bytes());
        self.buffer[6..8].copy_from_slice(&self.request_id.to_be_bytes());
        self.buffer[HEADER_LENGTH..HEADER_LENGTH + payload.len()].copy_from_slice(payload);
        self.socket.send_to(
            &self.buffer[0..HEADER_LENGTH + payload.len()],
            (remote_address, GVCP_PORT),
        )?;
        Ok(())
    }

    fn receive(&mut self, command: Command, timeout: std::time::Duration) -> Result<usize, Error> {
        let begin = std::time::Instant::now();
        let mut timeout = timeout;
        loop {
            self.socket.set_read_timeout(Some(timeout))?;
            let count = self.socket.recv(&mut self.buffer)?;
            if count < HEADER_LENGTH {
                return Err(Error::ShortMessage(count));
            }
            let status = u16::from_be_bytes(self.buffer[0..2].try_into().expect("2 bytes"));
            let message_type = u16::from_be_bytes(self.buffer[2..4].try_into().expect("2 bytes"));
            if status != 0 {
                return Err(Error::Status {
                    status,
                    status_string: status_to_string(status),
                    message_type,
                });
            }
            if message_type == ERROR_ACKNOWLEDGE {
                return Err(Error::InvalidMessageType(message_type));
            }
            let length =
                u16::from_be_bytes(self.buffer[4..6].try_into().expect("2 bytes")) as usize;
            if length != count - HEADER_LENGTH {
                return Err(Error::LengthMismatch {
                    header: length,
                    message: count - HEADER_LENGTH,
                });
            }
            let request_id = u16::from_be_bytes(self.buffer[6..8].try_into().expect("2 bytes"));
            if request_id != self.request_id {
                return Err(Error::UnexpectedRequestId {
                    expected: self.request_id,
                    actual: request_id,
                });
            }
            if message_type == PENDING_ACKNOWLEDGE {
                if length != 4 {
                    return Err(Error::UnexpectedLength {
                        expected: 4,
                        actual: length,
                        message_type,
                    });
                }
                let elapsed = begin.elapsed();
                if elapsed >= PENDING_TIMEOUT {
                    return Err(Error::PendingTimeout {
                        message_type: command as u16,
                        timeout: PENDING_TIMEOUT,
                    });
                }
                timeout = std::time::Duration::from_millis(u16::from_be_bytes(
                    self.buffer[10..12].try_into().expect("2 bytes"),
                ) as u64)
                .min(PENDING_TIMEOUT - elapsed);
                continue;
            }
            if message_type != command as u16 + 1 {
                return Err(Error::UnexpectedMessageType {
                    expected: command as u16 + 1,
                    actual: message_type,
                });
            }
            return Ok(length);
        }
    }

    fn request(
        &mut self,
        command: Command,
        payload: &[u8],
        remote_address: std::net::Ipv4Addr,
        timeout: std::time::Duration,
        attempts: u8,
    ) -> Result<usize, Error> {
        for _ in 0..attempts {
            if let Err(error) = self.send(command, payload, remote_address, timeout) {
                let error = Error::from(error);
                if error.is_timeout() {
                    continue;
                }
                return Err(error);
            }
            match self.receive(command, timeout) {
                Ok(length) => return Ok(length),
                Err(error) => {
                    if !error.is_timeout() {
                        return Err(error);
                    }
                }
            }
        }
        Err(Error::Timeout {
            message_type: command as u16,
            attempts,
        })
    }

    fn expect_length(&self, command: Command, length: usize, expected: usize) -> Result<(), Error> {
        if length == expected {
            Ok(())
        } else {
            Err(Error::UnexpectedLength {
                expected,
                actual: length,
                message_type: command as u16,
            })
        }
    }

    pub fn read_register(
        &mut self,
        address: u32,
        remote_address: std::net::Ipv4Addr,
        timeout: std::time::Duration,
        attempts: u8,
    ) -> Result<u32, Error> {
        let length = self.request(
            Command::ReadReg,
            &address.to_be_bytes(),
            remote_address,
            timeout,
            attempts,
        )?;
        self.expect_length(Command::ReadReg, length, 4)?;
        Ok(u32::from_be_bytes(
            self.payload(4).try_into().expect("4 bytes"),
        ))
    }

    pub fn write_register(
        &mut self,
        address: u32,
        value: u32,
        remote_address: std::net::Ipv4Addr,
        timeout: std::time::Duration,
        attempts: u8,
    ) -> Result<(), Error> {
        let mut payload = [0u8; 8];
        payload[0..4].copy_from_slice(&address.to_be_bytes());
        payload[4..8].copy_from_slice(&value.to_be_bytes());
        let length = self.request(
            Command::WriteReg,
            &payload,
            remote_address,
            timeout,
            attempts,
        )?;
        self.expect_length(Command::WriteReg, length, 4)
    }

    #[allow(dead_code)]
    pub fn read_memory(
        &mut self,
        address: u32,
        length: u16,
        remote_address: std::net::Ipv4Addr,
        timeout: std::time::Duration,
        attempts: u8,
    ) -> Result<Vec<u8>, Error> {
        let mut request_payload = [0u8; 8];
        request_payload[0..4].copy_from_slice(&address.to_be_bytes());
        request_payload[6..8].copy_from_slice(&length.to_be_bytes());
        let received = self.request(
            Command::ReadMem,
            &request_payload,
            remote_address,
            timeout,
            attempts,
        )?;
        if received < 4 {
            return Err(Error::ShortReadMemMessage(received));
        }
        let payload = self.payload(received);
        let actual_address = u32::from_be_bytes(payload[0..4].try_into().expect("4 bytes"));
        if address != actual_address {
            return Err(Error::InvalidReadMemAddress {
                expected: address,
                actual: actual_address,
            });
        }
        Ok(payload[4..].to_vec())
    }

    pub fn discovery(
        &mut self,
        remote_address: std::net::Ipv4Addr,
        timeout: std::time::Duration,
    ) -> Vec<Discovery> {
        let mut result = Vec::new();
        if self
            .send(Command::Discovery, &[], remote_address, timeout)
            .is_ok()
        {
            let start = std::time::Instant::now();
            while let Some(remaining) = timeout.checked_sub(start.elapsed()) {
                match self.receive(Command::Discovery, remaining) {
                    Ok(length) => {
                        if length == DISCOVERY_PAYLOAD_LENGTH {
                            result.push(Discovery::from_payload(self.payload(length)));
                        }
                    }
                    Err(error) => {
                        if error.is_timeout() {
                            break;
                        }
                    }
                }
            }
        }
        result
    }
}
