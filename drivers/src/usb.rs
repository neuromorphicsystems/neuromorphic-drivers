use crate::flag;
use crate::ring;
use rusb::UsbContext;

#[derive(thiserror::Error, Debug, Clone)]
pub enum Error {
    #[error(transparent)]
    Rusb(#[from] rusb::Error),

    #[error("device with serial not found")]
    Serial(String),

    #[error("there is no device on bus {bus_number} at address {address}")]
    BusNumberAndAddressNotFound { bus_number: u8, address: u8 },

    #[error("unsupported device on bus {bus_number} at address {address}")]
    BusNumberAndAddressUnsupportedDevice { bus_number: u8, address: u8 },

    #[error("could not access device on bus {bus_number} at address {address} ({error:?})")]
    BusNumberAndAddressAccessError {
        bus_number: u8,
        address: u8,
        error: rusb::Error,
    },

    #[error("the device on bus {bus_number} at address {address} has an unsupported VID:PID ({vendor_id:04X}:{product_id:04X})")]
    BusNumberAndAddressUnexpectedIds {
        bus_number: u8,
        address: u8,
        vendor_id: u16,
        product_id: u16,
    },

    #[error("device not found")]
    Device,

    #[error("this device is connected over USB and cannot be identified by an IP address")]
    Address,

    #[error(transparent)]
    Configuration(#[from] ring::ConfigurationError),

    #[error("control transfer error (expected {expected:?}, read {read:?})")]
    Mismatch { expected: Vec<u8>, read: Vec<u8> },

    #[error("control transfer error (expected one of {expected:?}, read {read:?})")]
    MismatchAny {
        expected: Vec<Vec<u8>>,
        read: Vec<u8>,
    },

    #[error("the device is already used by another program")]
    Busy,

    #[error("USB transfer allocation failed (successfully allocated {0} transfers)")]
    TransferAllocationFailed(usize),
}

#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub enum Speed {
    Unknown,
    Low,
    Full,
    High,
    Super,
    SuperPlus,
}

impl From<rusb::Speed> for Speed {
    fn from(speed: rusb::Speed) -> Self {
        match speed {
            rusb::Speed::Low => Self::Low,
            rusb::Speed::Full => Self::Full,
            rusb::Speed::High => Self::High,
            rusb::Speed::Super => Self::Super,
            rusb::Speed::SuperPlus => Self::SuperPlus,
            _ => Self::Unknown,
        }
    }
}

impl std::fmt::Display for Speed {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Unknown => write!(formatter, "USB Unknown speed"),
            Self::Low => write!(formatter, "USB 1.0 Low Speed (1.5 Mb/s)"),
            Self::Full => write!(formatter, "USB 1.1 Full Speed (12 Mb/s)"),
            Self::High => write!(formatter, "USB 2.0 High Speed (480 Mb/s)"),
            Self::Super => write!(formatter, "USB 3.0 SuperSpeed (5.0 Gb/s)"),
            Self::SuperPlus => write!(formatter, "USB 3.1 SuperSpeed+ (10.0 Gb/s)"),
        }
    }
}

pub fn assert_control_transfer(
    handle: &rusb::DeviceHandle<rusb::Context>,
    request_type: u8,
    request: u8,
    value: u16,
    index: u16,
    expected_buffer: &[u8],
    timeout: std::time::Duration,
) -> Result<(), Error> {
    let mut buffer = vec![0; expected_buffer.len()];
    let read = handle.read_control(request_type, request, value, index, &mut buffer, timeout)?;
    buffer.truncate(read);
    if expected_buffer == &buffer[..] {
        Ok(())
    } else {
        Err(Error::Mismatch {
            expected: Vec::from(expected_buffer),
            read: buffer,
        })
    }
}

pub fn assert_string_descriptor_any(
    handle: &rusb::DeviceHandle<rusb::Context>,
    request_type: u8,
    request: u8,
    value: u16,
    index: u16,
    expected_buffers: &[&[u8]],
    timeout: std::time::Duration,
) -> Result<(), Error> {
    let mut buffer = vec![
        0;
        expected_buffers
            .iter()
            .fold(0, |maximum, expected_buffer| maximum
                .max(expected_buffer.len())
                + 2)
    ];
    let read = handle.read_control(request_type, request, value, index, &mut buffer, timeout)?;
    buffer.truncate(read);
    for expected_buffer in expected_buffers {
        if *expected_buffer == &buffer[2..] {
            return Ok(());
        }
    }
    buffer.drain(0..2);
    Err(Error::MismatchAny {
        expected: expected_buffers
            .iter()
            .map(|expected_buffer| Vec::from(*expected_buffer))
            .collect(),
        read: buffer,
    })
}

pub struct EventLoop {
    context: rusb::Context,
    running: std::sync::Arc<std::sync::atomic::AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl EventLoop {
    pub fn new<IntoError, IntoWarning>(
        timeout: std::time::Duration,
        flag: flag::Flag<IntoError, IntoWarning>,
    ) -> Result<Self, Error>
    where
        IntoError: From<Error> + Clone + Send + 'static,
        IntoWarning: From<ring::Overflow> + Clone + Send + 'static,
    {
        let context = rusb::Context::new()?;
        let running = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
        let thread_running = running.clone();
        let thread_context = context.clone();
        Ok(Self {
            context,
            thread: Some(std::thread::spawn(move || {
                while thread_running.load(std::sync::atomic::Ordering::Acquire) {
                    if let Err(handle_events_error) = thread_context.handle_events(Some(timeout)) {
                        flag.store_error_if_not_set(Error::from(handle_events_error));
                    }
                }
            })),
            running,
        })
    }

    pub fn context(&self) -> &rusb::Context {
        &self.context
    }
}

impl Drop for EventLoop {
    fn drop(&mut self) {
        self.running
            .store(false, std::sync::atomic::Ordering::Release);
        if let Some(thread) = self.thread.take() {
            thread.join().expect("event loop joined self");
        }
    }
}

struct LibusbTransfer(*mut libusb1_sys::libusb_transfer);

impl LibusbTransfer {
    fn cancel(&self) -> libc::c_int {
        unsafe { libusb1_sys::libusb_cancel_transfer(self.0) }
    }
}

// unsafe: *mut libusb1_sys::libusb_transfer is thread-safe.
unsafe impl Send for LibusbTransfer {}
unsafe impl Sync for LibusbTransfer {}

pub struct TransferManager {
    transfers: Vec<LibusbTransfer>,
    ring: ring::SharedRing,
    #[allow(dead_code)]
    handle: std::sync::Arc<rusb::DeviceHandle<rusb::Context>>,
    #[allow(dead_code)]
    event_loop: std::sync::Arc<EventLoop>,
}

pub enum TransferType {
    Control(std::time::Duration),
    Isochronous {
        endpoint: u8,
        packets: u32,
        timeout: std::time::Duration,
    },
    Bulk {
        endpoint: u8,
        timeout: std::time::Duration,
    },
    Interrupt {
        endpoint: u8,
        timeout: std::time::Duration,
    },
    BulkStream {
        endpoint: u8,
        stream_id: u32,
        timeout: std::time::Duration,
    },
}

trait CallbacksTrait: Send + Sync + 'static {
    fn on_error(&self, error: Error);
    fn on_overflow(&self, overflow: ring::Overflow);
}

struct Callbacks<OnError, OnOverflow> {
    on_error: OnError,
    on_overflow: OnOverflow,
}

impl<OnError, OnOverflow> CallbacksTrait for Callbacks<OnError, OnOverflow>
where
    OnError: Fn(Error) + Send + Sync + 'static,
    OnOverflow: Fn(ring::Overflow) + Send + Sync + 'static,
{
    fn on_error(&self, error: Error) {
        (self.on_error)(error)
    }
    fn on_overflow(&self, overflow: ring::Overflow) {
        (self.on_overflow)(overflow)
    }
}

#[derive(Clone)]
struct SharedCallbacks(std::sync::Arc<std::sync::Mutex<dyn CallbacksTrait>>);

impl SharedCallbacks {
    fn on_error(&self, error: Error) {
        self.0
            .lock()
            .expect("callbacks is not poisoned")
            .on_error(error)
    }

    fn on_overflow(&self, overflow: ring::Overflow) {
        self.0
            .lock()
            .expect("callbacks is not poisoned")
            .on_overflow(overflow)
    }
}

struct TransferContext {
    ring: ring::SharedRing,
    callbacks: SharedCallbacks,
    transfer_index: usize,
    clutch: ring::Clutch,
}

#[no_mangle]
extern "system" fn usb_transfer_callback(transfer_pointer: *mut libusb1_sys::libusb_transfer) {
    let system_time = std::time::SystemTime::now();
    let instant = std::time::Instant::now();

    // unsafe: transfer is not null (libusb callback)
    let transfer = unsafe { &mut *transfer_pointer };
    let context = transfer.user_data;
    assert!(!context.is_null(), "context is null");
    // unsafe: context is a *mut TransferContext
    let context = unsafe { &mut *(context as *mut TransferContext) };
    let mut ring_data = context.ring.data();
    match ring_data.transfer_status(context.transfer_index) {
        ring::TransferStatus::Active => match transfer.status {
            libusb1_sys::constants::LIBUSB_TRANSFER_COMPLETED
            | libusb1_sys::constants::LIBUSB_TRANSFER_TIMED_OUT => {
                let buffer_view = ring_data.transfer_complete(
                    context.clutch,
                    system_time,
                    instant,
                    transfer.actual_length as usize,
                );
                if matches!(context.clutch, ring::Clutch::Disengaged) {
                    context.ring.notify_one();
                }
                if buffer_view.clutch_changed && matches!(buffer_view.clutch, ring::Clutch::Engaged)
                {
                    context.callbacks.on_overflow(ring::Overflow(()));
                }
                context.clutch = buffer_view.clutch;
                transfer.buffer = buffer_view.data;
                transfer.length = buffer_view.capacity as i32;
                // unsafe: libusb_alloc_transfer succeeded (transfer_pointer points to a valid transfer)
                match unsafe { libusb1_sys::libusb_submit_transfer(transfer_pointer) } {
                    0 => (), // success
                    submit_transfer_status => {
                        context.callbacks.on_error(
                            match submit_transfer_status {
                                libusb1_sys::constants::LIBUSB_ERROR_IO => rusb::Error::Io,
                                libusb1_sys::constants::LIBUSB_ERROR_INVALID_PARAM => {
                                    rusb::Error::InvalidParam
                                }
                                libusb1_sys::constants::LIBUSB_ERROR_ACCESS => rusb::Error::Access,
                                libusb1_sys::constants::LIBUSB_ERROR_NO_DEVICE => {
                                    rusb::Error::NoDevice
                                }
                                libusb1_sys::constants::LIBUSB_ERROR_NOT_FOUND => {
                                    rusb::Error::NotFound
                                }
                                libusb1_sys::constants::LIBUSB_ERROR_BUSY => rusb::Error::Busy,
                                libusb1_sys::constants::LIBUSB_ERROR_TIMEOUT => {
                                    rusb::Error::Timeout
                                }
                                libusb1_sys::constants::LIBUSB_ERROR_OVERFLOW => {
                                    rusb::Error::Overflow
                                }
                                libusb1_sys::constants::LIBUSB_ERROR_PIPE => rusb::Error::Pipe,
                                libusb1_sys::constants::LIBUSB_ERROR_INTERRUPTED => {
                                    rusb::Error::Interrupted
                                }
                                libusb1_sys::constants::LIBUSB_ERROR_NO_MEM => rusb::Error::NoMem,
                                libusb1_sys::constants::LIBUSB_ERROR_NOT_SUPPORTED => {
                                    rusb::Error::NotSupported
                                }
                                _ => rusb::Error::Other,
                            }
                            .into(),
                        );
                    }
                }
            }
            status @ (libusb1_sys::constants::LIBUSB_TRANSFER_ERROR
            | libusb1_sys::constants::LIBUSB_TRANSFER_CANCELLED
            | libusb1_sys::constants::LIBUSB_TRANSFER_STALL
            | libusb1_sys::constants::LIBUSB_TRANSFER_NO_DEVICE
            | libusb1_sys::constants::LIBUSB_TRANSFER_OVERFLOW) => {
                ring_data.transfer_complete_without_next(
                    context.clutch,
                    system_time,
                    instant,
                    transfer.actual_length as usize,
                );
                if matches!(context.clutch, ring::Clutch::Disengaged) {
                    context.ring.notify_one();
                }
                ring_data
                    .update_transfer_status(context.transfer_index, ring::TransferStatus::Complete);
                context.callbacks.on_error(
                    match status {
                        libusb1_sys::constants::LIBUSB_TRANSFER_ERROR
                        | libusb1_sys::constants::LIBUSB_TRANSFER_CANCELLED => rusb::Error::Io,
                        libusb1_sys::constants::LIBUSB_TRANSFER_STALL => rusb::Error::Pipe,
                        libusb1_sys::constants::LIBUSB_TRANSFER_NO_DEVICE => rusb::Error::NoDevice,
                        libusb1_sys::constants::LIBUSB_TRANSFER_OVERFLOW => rusb::Error::Overflow,
                        _ => rusb::Error::Other,
                    }
                    .into(),
                );
            }
            unknown_transfer_status => {
                panic!("unknown transfer status {unknown_transfer_status}")
            }
        },
        ring::TransferStatus::Cancelling => match transfer.status {
            libusb1_sys::constants::LIBUSB_TRANSFER_COMPLETED
            | libusb1_sys::constants::LIBUSB_TRANSFER_TIMED_OUT
            | libusb1_sys::constants::LIBUSB_TRANSFER_ERROR
            | libusb1_sys::constants::LIBUSB_TRANSFER_CANCELLED
            | libusb1_sys::constants::LIBUSB_TRANSFER_STALL
            | libusb1_sys::constants::LIBUSB_TRANSFER_NO_DEVICE
            | libusb1_sys::constants::LIBUSB_TRANSFER_OVERFLOW => {
                ring_data.transfer_complete_without_next(
                    context.clutch,
                    system_time,
                    instant,
                    transfer.actual_length as usize,
                );
                ring_data
                    .update_transfer_status(context.transfer_index, ring::TransferStatus::Complete);
            }
            unknown_transfer_status => {
                panic!("unknown transfer status {unknown_transfer_status}")
            }
        },
        ring::TransferStatus::Complete => {
            panic!("callback called for a transfer marked as complete")
        }
        ring::TransferStatus::Deallocated => {
            panic!("callback called for a transfer marked as deallocated")
        }
    }
}

impl TransferManager {
    pub fn new<OnError, OnOverflow>(
        configuration: &ring::Configuration,
        transfer_type: TransferType,
        on_error: OnError,
        on_overflow: OnOverflow,
        handle: std::sync::Arc<rusb::DeviceHandle<rusb::Context>>,
        event_loop: std::sync::Arc<EventLoop>,
    ) -> Result<Self, Error>
    where
        OnError: Fn(Error) + Send + Sync + 'static,
        OnOverflow: Fn(ring::Overflow) + Send + Sync + 'static,
    {
        assert!(
            handle.context() == event_loop.context(),
            "handle and event_loop must have the same context"
        );
        let (ring, write_buffer_views) = ring::SharedRing::new(configuration)?;
        let mut manager = TransferManager {
            transfers: Vec::new(),
            ring,
            handle: handle.clone(),
            event_loop,
        };
        let callbacks = SharedCallbacks(std::sync::Arc::new(std::sync::Mutex::new(Callbacks {
            on_error,
            on_overflow,
        })));
        {
            manager
                .transfers
                .reserve_exact(configuration.parallel_submissions);
            for (index, write_buffer_view) in write_buffer_views.iter().enumerate() {
                // unsafe: libusb1_sys wrapper
                let libusb_transfer = unsafe { libusb1_sys::libusb_alloc_transfer(0) };
                if libusb_transfer.is_null() {
                    return Err(Error::TransferAllocationFailed(index));
                } else {
                    let transfer_context = Box::new(TransferContext {
                        ring: manager.ring.clone(),
                        callbacks: callbacks.clone(),
                        transfer_index: index,
                        clutch: ring::Clutch::Disengaged,
                    });
                    let transfer_context_pointer = Box::into_raw(transfer_context);
                    match transfer_type {
                        // unsafe: libusb_alloc_transfer succeeded
                        TransferType::Control(timeout) => unsafe {
                            libusb1_sys::libusb_fill_control_transfer(
                                libusb_transfer,
                                handle.as_raw(),
                                write_buffer_view.data,
                                usb_transfer_callback,
                                transfer_context_pointer as *mut libc::c_void,
                                timeout.as_millis() as libc::c_uint,
                            )
                        },
                        // unsafe: libusb_alloc_transfer succeeded
                        TransferType::Isochronous {
                            endpoint,
                            packets,
                            timeout,
                        } => unsafe {
                            libusb1_sys::libusb_fill_iso_transfer(
                                libusb_transfer,
                                handle.as_raw(),
                                endpoint,
                                write_buffer_view.data,
                                write_buffer_view.capacity as libc::c_int,
                                packets as libc::c_int,
                                usb_transfer_callback,
                                transfer_context_pointer as *mut libc::c_void,
                                timeout.as_millis() as libc::c_uint,
                            )
                        },
                        // unsafe: libusb_alloc_transfer succeeded
                        TransferType::Bulk { endpoint, timeout } => unsafe {
                            libusb1_sys::libusb_fill_bulk_transfer(
                                libusb_transfer,
                                handle.as_raw(),
                                endpoint,
                                write_buffer_view.data,
                                write_buffer_view.capacity as libc::c_int,
                                usb_transfer_callback,
                                transfer_context_pointer as *mut libc::c_void,
                                timeout.as_millis() as libc::c_uint,
                            )
                        },
                        // unsafe: libusb_alloc_transfer succeeded
                        TransferType::Interrupt { endpoint, timeout } => unsafe {
                            libusb1_sys::libusb_fill_interrupt_transfer(
                                libusb_transfer,
                                handle.as_raw(),
                                endpoint,
                                write_buffer_view.data,
                                write_buffer_view.capacity as libc::c_int,
                                usb_transfer_callback,
                                transfer_context_pointer as *mut libc::c_void,
                                timeout.as_millis() as libc::c_uint,
                            )
                        },
                        // unsafe: libusb_alloc_transfer succeeded
                        TransferType::BulkStream {
                            endpoint,
                            stream_id,
                            timeout,
                        } => unsafe {
                            libusb1_sys::libusb_fill_bulk_stream_transfer(
                                libusb_transfer,
                                handle.as_raw(),
                                endpoint,
                                stream_id,
                                write_buffer_view.data,
                                write_buffer_view.capacity as libc::c_int,
                                usb_transfer_callback,
                                transfer_context_pointer as *mut libc::c_void,
                                timeout.as_millis() as libc::c_uint,
                            )
                        },
                    }
                    // unsafe: libusb_alloc_transfer succeeded
                    unsafe {
                        (*libusb_transfer).flags = 0; // !LIBUSB_TRANSFER_SHORT_NOT_OK
                                                      // !LIBUSB_TRANSFER_FREE_BUFFER
                                                      // !LIBUSB_TRANSFER_FREE_TRANSFER
                                                      // !LIBUSB_TRANSFER_ADD_ZERO_PACKET
                    }
                    manager.transfers.push(LibusbTransfer(libusb_transfer));
                }
            }
            for (index, transfer) in manager.transfers.iter().enumerate() {
                // unsafe: libusb_alloc_transfer succeeded (transfer.data points to a valid transfer)
                match unsafe { libusb1_sys::libusb_submit_transfer(transfer.0) } {
                    0 => (), // success
                    submit_transfer_status => {
                        {
                            let mut ring_data = manager.ring.data();
                            for rest_index in index..manager.transfers.len() {
                                // dropping 'manager' cancels transfers
                                // mark unscheduled transfers as complete to prevent un-needed cancelling
                                ring_data.update_transfer_status(
                                    rest_index,
                                    ring::TransferStatus::Complete,
                                );
                            }
                        }
                        return Err(match submit_transfer_status {
                            libusb1_sys::constants::LIBUSB_ERROR_IO => rusb::Error::Io,
                            libusb1_sys::constants::LIBUSB_ERROR_INVALID_PARAM => {
                                rusb::Error::InvalidParam
                            }
                            libusb1_sys::constants::LIBUSB_ERROR_ACCESS => rusb::Error::Access,
                            libusb1_sys::constants::LIBUSB_ERROR_NO_DEVICE => rusb::Error::NoDevice,
                            libusb1_sys::constants::LIBUSB_ERROR_NOT_FOUND => rusb::Error::NotFound,
                            libusb1_sys::constants::LIBUSB_ERROR_BUSY => rusb::Error::Busy,
                            libusb1_sys::constants::LIBUSB_ERROR_TIMEOUT => rusb::Error::Timeout,
                            libusb1_sys::constants::LIBUSB_ERROR_OVERFLOW => rusb::Error::Overflow,
                            libusb1_sys::constants::LIBUSB_ERROR_PIPE => rusb::Error::Pipe,
                            libusb1_sys::constants::LIBUSB_ERROR_INTERRUPTED => {
                                rusb::Error::Interrupted
                            }
                            libusb1_sys::constants::LIBUSB_ERROR_NO_MEM => rusb::Error::NoMem,
                            libusb1_sys::constants::LIBUSB_ERROR_NOT_SUPPORTED => {
                                rusb::Error::NotSupported
                            }
                            _ => rusb::Error::Other,
                        }
                        .into());
                    }
                }
            }
        }
        Ok(manager)
    }

    pub fn dropped_packets(&self) -> u64 {
        self.ring.dropped_packets()
    }

    pub fn backlog(&self) -> usize {
        self.ring.backlog()
    }

    pub fn clutch(&self) -> ring::Clutch {
        self.ring.clutch()
    }

    pub fn next_with_timeout(
        &self,
        duration: &std::time::Duration,
    ) -> Option<ring::ReadBufferView<'_>> {
        self.ring.next_with_timeout(duration)
    }
}

impl Drop for TransferManager {
    fn drop(&mut self) {
        let before_dealloc_transfers = std::time::Instant::now();
        #[cfg(target_os = "macos")]
        {
            // on macOS, cancelling any transfer for a device cancels all transfers for that device
            let _ = self.transfers[0].cancel();
            let mut ring_data = self.ring.data();
            for index in 0..self.transfers.len() {
                ring_data.update_transfer_status(index, ring::TransferStatus::Cancelling);
            }
        }
        loop {
            let mut deallocated_transfers: usize = 0;
            {
                let mut ring_data = self.ring.data();
                for index in 0..self.transfers.len() {
                    match ring_data.transfer_status(index) {
                        ring::TransferStatus::Active => {
                            if self.transfers[index].cancel() == 0 {
                                ring_data.update_transfer_status(
                                    index,
                                    ring::TransferStatus::Cancelling,
                                );
                            } else {
                                ring_data
                                    .update_transfer_status(index, ring::TransferStatus::Complete);
                            }
                        }
                        ring::TransferStatus::Complete => {
                            // unsafe: transfer is allocated and user_data is an allocated *mut TransferContext
                            let _transfer_context = unsafe {
                                Box::from_raw(
                                    (*(self.transfers[index]).0).user_data as *mut TransferContext,
                                )
                            };
                            // unsafe: transfer is allocated
                            unsafe { libusb1_sys::libusb_free_transfer(self.transfers[index].0) };
                            ring_data
                                .update_transfer_status(index, ring::TransferStatus::Deallocated);
                            deallocated_transfers += 1;
                        }
                        ring::TransferStatus::Cancelling => (),
                        ring::TransferStatus::Deallocated => {
                            deallocated_transfers += 1;
                        }
                    }
                }
            }
            if deallocated_transfers == self.transfers.len() {
                break;
            }
            // give up if the transfers are not freed after one second
            // this may cause segfaults if libusb tries to use the buffers again,
            // as the buffers will be freed by their Drop function
            if std::time::Instant::now() - before_dealloc_transfers
                > std::time::Duration::from_secs(1)
            {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
    }
}
