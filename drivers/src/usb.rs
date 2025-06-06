use crate::flag;
use rusb::UsbContext;

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Configuration {
    pub buffer_length: usize,
    pub ring_length: usize,
    pub transfer_queue_length: usize,
    pub allow_dma: bool,
}

impl Configuration {
    pub fn deserialize_bincode(data: &[u8]) -> bincode::Result<Configuration> {
        bincode::deserialize(data)
    }
}

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

    #[error("ring size is smaller than or equal to transfer queue size")]
    ConfigurationSizes,

    #[error("ring overflow")]
    Overflow,

    #[error("control transfer error (expected {expected:?}, read {read:?})")]
    Mismatch { expected: Vec<u8>, read: Vec<u8> },

    #[error("the device is already used by another program")]
    Busy,
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

extern "system" {
    pub fn libusb_dev_mem_alloc(
        dev_handle: *mut libusb1_sys::libusb_device_handle,
        length: libc::ssize_t,
    ) -> *mut libc::c_uchar;

    pub fn libusb_dev_mem_free(
        dev_handle: *mut libusb1_sys::libusb_device_handle,
        buffer: *mut libc::c_uchar,
        length: libc::ssize_t,
    ) -> *mut libc::c_int;
}

struct BufferData(std::ptr::NonNull<u8>);

unsafe impl Send for BufferData {}
unsafe impl Sync for BufferData {}

impl BufferData {
    fn as_ptr(&self) -> *mut u8 {
        self.0.as_ptr()
    }
}

struct Buffer {
    system_time: std::time::SystemTime,
    instant: std::time::Instant,
    first_after_overflow: bool,
    data: BufferData,
    length: usize,
    capacity: usize,
    dma: bool,
}

pub struct EventLoop {
    context: rusb::Context,
    running: std::sync::Arc<std::sync::atomic::AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}

#[derive(Debug, Clone, Copy)]
pub struct Overflow(());

impl EventLoop {
    pub fn new<IntoError, IntoWarning>(
        timeout: std::time::Duration,
        flag: flag::Flag<IntoError, IntoWarning>,
    ) -> Result<Self, Error>
    where
        IntoError: From<Error> + Clone + Send + 'static,
        IntoWarning: From<Overflow> + Clone + Send + 'static,
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

enum TransferStatus {
    Active,
    Complete,
    Cancelling,
    Deallocated,
}

#[derive(Clone)]
pub struct WriteRange {
    pub start: usize,
    pub end: usize,
    pub ring_length: usize,
}

impl WriteRange {
    fn increment_start(&mut self) {
        self.start = (self.start + 1) % self.ring_length;
    }

    fn increment_end(&mut self) {
        self.end = (self.end + 1) % self.ring_length;
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Clutch {
    Disengaged,
    Engaged,
}

struct RingContext {
    read: usize,
    write_range: WriteRange,
    transfer_statuses: Vec<TransferStatus>,
    buffers: Vec<Buffer>,
    freewheel_buffers: Vec<Buffer>,
    clutch: Clutch,
}

struct SharedRingContext {
    on_error: Box<dyn Fn(Error) + Send + Sync + 'static>,
    on_overflow: Box<dyn Fn(Overflow) + Send + Sync + 'static>,
    shared: std::sync::Mutex<RingContext>,
    shared_condvar: std::sync::Condvar,
}

struct LibusbTransfer(std::ptr::NonNull<libusb1_sys::libusb_transfer>);

unsafe impl Send for LibusbTransfer {}

impl LibusbTransfer {
    unsafe fn as_mut(&mut self) -> &mut libusb1_sys::libusb_transfer {
        self.0.as_mut()
    }

    fn as_ptr(&self) -> *mut libusb1_sys::libusb_transfer {
        self.0.as_ptr()
    }
}

pub struct Ring {
    transfers: Vec<LibusbTransfer>,
    handle: std::sync::Arc<rusb::DeviceHandle<rusb::Context>>,
    active_buffer_view: std::sync::Arc<std::sync::atomic::AtomicBool>,
    #[allow(dead_code)]
    event_loop: std::sync::Arc<EventLoop>,
    context: std::sync::Arc<SharedRingContext>,
}

unsafe impl Send for Ring {}
unsafe impl Sync for Ring {}

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

pub struct TransferProperties {
    pub transfer_type: TransferType,
    pub timeout: std::time::Duration,
}

enum TransferClutch {
    Disengaged,
    DisengagedFirst,
    Engaged,
}

struct TransferContext {
    ring: std::sync::Arc<SharedRingContext>,
    transfer_index: usize,
    clutch: TransferClutch,
}

#[no_mangle]
extern "system" fn usb_transfer_callback(transfer_pointer: *mut libusb1_sys::libusb_transfer) {
    let system_time = std::time::SystemTime::now();
    let now = std::time::Instant::now();
    let mut resubmit = false;
    {
        // unsafe: transfer is not null (libusb callback)
        let transfer = unsafe { &mut *transfer_pointer };
        let context = transfer.user_data;
        assert!(!context.is_null(), "context is null");
        // unsafe: context is a *mut TransferContext
        let context = unsafe { &mut *(context as *mut TransferContext) };
        let mut error = None;
        {
            let mut shared = context
                .ring
                .shared
                .lock()
                .expect("ring context's lock is not poisoned");
            match shared.transfer_statuses[context.transfer_index] {
                TransferStatus::Active => match transfer.status {
                    libusb1_sys::constants::LIBUSB_TRANSFER_COMPLETED
                    | libusb1_sys::constants::LIBUSB_TRANSFER_TIMED_OUT => {
                        if !matches!(context.clutch, TransferClutch::Engaged) {
                            let active_buffer = shared.write_range.start;
                            shared.buffers[active_buffer].system_time = system_time;
                            shared.buffers[active_buffer].instant = now;
                            shared.buffers[active_buffer].first_after_overflow =
                                matches!(context.clutch, TransferClutch::DisengagedFirst);
                            shared.buffers[active_buffer].length = transfer.actual_length as usize;
                            shared.write_range.increment_start();
                            context.ring.shared_condvar.notify_one();
                        }
                        if shared.write_range.end == shared.read {
                            if matches!(shared.clutch, Clutch::Disengaged) {
                                shared.clutch = Clutch::Engaged;
                                (context.ring.on_overflow)(Overflow(()));
                            }
                            context.clutch = TransferClutch::Engaged;
                            transfer.buffer = shared.freewheel_buffers[context.transfer_index]
                                .data
                                .as_ptr();
                            transfer.length =
                                shared.freewheel_buffers[context.transfer_index].capacity as i32;
                        } else {
                            match shared.clutch {
                                Clutch::Disengaged => {
                                    context.clutch = TransferClutch::Disengaged;
                                }
                                Clutch::Engaged => {
                                    shared.clutch = Clutch::Disengaged;
                                    context.clutch = TransferClutch::DisengagedFirst;
                                }
                            }
                            transfer.buffer = shared.buffers[shared.write_range.end].data.as_ptr();
                            transfer.length =
                                shared.buffers[shared.write_range.end].capacity as i32;
                            shared.write_range.increment_end();
                        }
                        resubmit = true;
                    }
                    status @ (libusb1_sys::constants::LIBUSB_TRANSFER_ERROR
                    | libusb1_sys::constants::LIBUSB_TRANSFER_CANCELLED
                    | libusb1_sys::constants::LIBUSB_TRANSFER_STALL
                    | libusb1_sys::constants::LIBUSB_TRANSFER_NO_DEVICE
                    | libusb1_sys::constants::LIBUSB_TRANSFER_OVERFLOW) => {
                        if !matches!(context.clutch, TransferClutch::Engaged) {
                            let active_buffer = shared.write_range.start;
                            shared.buffers[active_buffer].system_time = system_time;
                            shared.buffers[active_buffer].instant = now;
                            shared.buffers[active_buffer].length = transfer.actual_length as usize;
                            shared.write_range.increment_start();
                            context.ring.shared_condvar.notify_one();
                        }
                        // set clutch to report a packet drop
                        shared.clutch = Clutch::Engaged;
                        context.clutch = TransferClutch::Disengaged;
                        shared.transfer_statuses[context.transfer_index] = TransferStatus::Complete;
                        error = Some(
                            match status {
                                libusb1_sys::constants::LIBUSB_TRANSFER_ERROR
                                | libusb1_sys::constants::LIBUSB_TRANSFER_CANCELLED => {
                                    rusb::Error::Io
                                }
                                libusb1_sys::constants::LIBUSB_TRANSFER_STALL => rusb::Error::Pipe,
                                libusb1_sys::constants::LIBUSB_TRANSFER_NO_DEVICE => {
                                    rusb::Error::NoDevice
                                }
                                libusb1_sys::constants::LIBUSB_TRANSFER_OVERFLOW => {
                                    rusb::Error::Overflow
                                }
                                _ => rusb::Error::Other,
                            }
                            .into(),
                        );
                    }
                    unknown_transfer_status => {
                        panic!("unknown transfer status {unknown_transfer_status}")
                    }
                },
                TransferStatus::Cancelling => match transfer.status {
                    libusb1_sys::constants::LIBUSB_TRANSFER_COMPLETED
                    | libusb1_sys::constants::LIBUSB_TRANSFER_TIMED_OUT
                    | libusb1_sys::constants::LIBUSB_TRANSFER_ERROR
                    | libusb1_sys::constants::LIBUSB_TRANSFER_CANCELLED
                    | libusb1_sys::constants::LIBUSB_TRANSFER_STALL
                    | libusb1_sys::constants::LIBUSB_TRANSFER_NO_DEVICE => {
                        if !matches!(context.clutch, TransferClutch::Engaged) {
                            let active_buffer = shared.write_range.start;
                            shared.buffers[active_buffer].system_time = system_time;
                            shared.buffers[active_buffer].instant = now;
                            shared.buffers[active_buffer].length = transfer.actual_length as usize;
                            shared.write_range.increment_start();
                            context.ring.shared_condvar.notify_one();
                        }
                        // set clutch to report a packet drop
                        shared.clutch = Clutch::Engaged;
                        context.clutch = TransferClutch::Disengaged;
                        shared.transfer_statuses[context.transfer_index] = TransferStatus::Complete;
                    }
                    unknown_transfer_status => {
                        panic!("unknown transfer status {unknown_transfer_status}")
                    }
                },
                TransferStatus::Complete => {
                    panic!("callback called for a transfer marked as complete")
                }
                TransferStatus::Deallocated => {
                    panic!("callback called for a transfer marked as deallocated")
                }
            }
        }
        if let Some(error) = error {
            (context.ring.on_error)(error);
        }
    }
    if resubmit {
        // unsafe: libusb_alloc_transfer succeeded
        match unsafe { libusb1_sys::libusb_submit_transfer(transfer_pointer) } {
            0 => (),
            submit_transfer_status => {
                // unsafe: transfer is not null (libusb callback)
                let transfer = unsafe { &mut *transfer_pointer };
                transfer.flags = 0;
                let context = transfer.user_data;
                assert!(!context.is_null(), "context is null");
                // unsafe: context is a *mut TransferContext
                let context = unsafe { &mut *(context as *mut TransferContext) };
                (context.ring.on_error)(
                    match submit_transfer_status {
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
                    .into(),
                );
            }
        }
    }
}

impl Ring {
    pub fn new<OnError, OnOverflow>(
        handle: std::sync::Arc<rusb::DeviceHandle<rusb::Context>>,
        configuration: &Configuration,
        on_error: OnError,
        on_overflow: OnOverflow,
        event_loop: std::sync::Arc<EventLoop>,
        transfer_type: TransferType,
    ) -> Result<Self, Error>
    where
        OnError: Fn(Error) + Send + Sync + 'static,
        OnOverflow: Fn(Overflow) + Send + Sync + 'static,
    {
        assert!(
            handle.context() == event_loop.context(),
            "handle and event_loop must have the same context"
        );
        if configuration.ring_length <= configuration.transfer_queue_length {
            return Err(Error::ConfigurationSizes);
        }
        let mut buffers = Vec::new();
        buffers.reserve_exact(configuration.ring_length);
        let mut freewheel_buffers = Vec::new();
        freewheel_buffers.reserve_exact(configuration.transfer_queue_length);
        for index in 0..configuration.ring_length + configuration.transfer_queue_length {
            let dma_buffer = if configuration.allow_dma {
                // unsafe: libusb wrapper
                unsafe {
                    libusb_dev_mem_alloc(
                        handle.as_raw(),
                        configuration.buffer_length as libc::ssize_t,
                    )
                }
            } else {
                std::ptr::null_mut()
            };
            if dma_buffer.is_null() {
                (if index < configuration.ring_length {
                    &mut buffers
                } else {
                    &mut freewheel_buffers
                })
                .push(Buffer {
                    system_time: std::time::SystemTime::now(),
                    instant: std::time::Instant::now(),
                    first_after_overflow: false,
                    data: BufferData(
                        std::ptr::NonNull::new(
                            // unsafe: alloc wrapper
                            // std::alloc::Layout::from_length_align_unchecked
                            // - align must not be zero
                            // - align must be a power of two
                            // - size, when rounded up to the nearest multiple of align, must not overflow isize
                            unsafe {
                                std::alloc::alloc(std::alloc::Layout::from_size_align_unchecked(
                                    configuration.buffer_length,
                                    1,
                                ))
                            },
                        )
                        .ok_or(rusb::Error::NoMem)?,
                    ),
                    length: 0,
                    capacity: configuration.buffer_length,
                    dma: false,
                });
            } else {
                (if index < configuration.ring_length {
                    &mut buffers
                } else {
                    &mut freewheel_buffers
                })
                .push(Buffer {
                    system_time: std::time::SystemTime::now(),
                    instant: std::time::Instant::now(),
                    first_after_overflow: false,
                    // unsafe: dma_buffer is not null
                    data: BufferData(unsafe { std::ptr::NonNull::new_unchecked(dma_buffer) }),
                    length: 0,
                    capacity: configuration.buffer_length,
                    dma: true,
                });
            }
        }
        let mut transfer_statuses = Vec::new();
        transfer_statuses.reserve_exact(configuration.transfer_queue_length);
        for _ in 0..configuration.transfer_queue_length {
            transfer_statuses.push(TransferStatus::Active);
        }
        let context = std::sync::Arc::new(SharedRingContext {
            on_error: Box::new(on_error),
            on_overflow: Box::new(on_overflow),
            shared: std::sync::Mutex::new(RingContext {
                read: buffers.len() - 1,
                write_range: WriteRange {
                    start: 0,
                    end: configuration.transfer_queue_length,
                    ring_length: configuration.ring_length,
                },
                transfer_statuses,
                buffers,
                freewheel_buffers,
                clutch: Clutch::Disengaged,
            }),
            shared_condvar: std::sync::Condvar::new(),
        });
        let mut transfers: Vec<LibusbTransfer> = Vec::new();
        transfers.reserve_exact(configuration.transfer_queue_length);
        {
            let shared = context
                .shared
                .lock()
                .expect("ring context's lock is not poisoned");
            for index in 0..configuration.transfer_queue_length {
                // unsafe: libusb1_sys wrapper
                let mut transfer = match std::ptr::NonNull::new(unsafe {
                    libusb1_sys::libusb_alloc_transfer(0)
                }) {
                    Some(transfer) => LibusbTransfer(transfer),
                    None => {
                        for transfer in transfers.iter_mut().take(index) {
                            // unsafe: transfer is allocated and user_data is an allocated *mut TransferContext
                            unsafe {
                                let _ = Box::from_raw(
                                    (transfer.as_mut()).user_data as *mut TransferContext,
                                );
                            };
                            // unsafe: transfer is allocated
                            unsafe { libusb1_sys::libusb_free_transfer(transfer.as_ptr()) };
                        }
                        return Err(rusb::Error::NoMem.into());
                    }
                };
                let transfer_context = Box::new(TransferContext {
                    ring: context.clone(),
                    transfer_index: index,
                    clutch: TransferClutch::Disengaged,
                });
                let transfer_context_pointer = Box::into_raw(transfer_context);
                match transfer_type {
                    // unsafe: libusb_alloc_transfer succeeded
                    TransferType::Control(timeout) => unsafe {
                        libusb1_sys::libusb_fill_control_transfer(
                            transfer.as_ptr(),
                            handle.as_raw(),
                            shared.buffers[index].data.as_ptr(),
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
                            transfer.as_ptr(),
                            handle.as_raw(),
                            endpoint,
                            shared.buffers[index].data.as_ptr(),
                            shared.buffers[index].capacity as libc::c_int,
                            packets as libc::c_int,
                            usb_transfer_callback,
                            transfer_context_pointer as *mut libc::c_void,
                            timeout.as_millis() as libc::c_uint,
                        )
                    },
                    // unsafe: libusb_alloc_transfer succeeded
                    TransferType::Bulk { endpoint, timeout } => unsafe {
                        libusb1_sys::libusb_fill_bulk_transfer(
                            transfer.as_ptr(),
                            handle.as_raw(),
                            endpoint,
                            shared.buffers[index].data.as_ptr(),
                            shared.buffers[index].capacity as libc::c_int,
                            usb_transfer_callback,
                            transfer_context_pointer as *mut libc::c_void,
                            timeout.as_millis() as libc::c_uint,
                        )
                    },
                    // unsafe: libusb_alloc_transfer succeeded
                    TransferType::Interrupt { endpoint, timeout } => unsafe {
                        libusb1_sys::libusb_fill_interrupt_transfer(
                            transfer.as_ptr(),
                            handle.as_raw(),
                            endpoint,
                            shared.buffers[index].data.as_ptr(),
                            shared.buffers[index].capacity as libc::c_int,
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
                            transfer.as_ptr(),
                            handle.as_raw(),
                            endpoint,
                            stream_id,
                            shared.buffers[index].data.as_ptr(),
                            shared.buffers[index].capacity as libc::c_int,
                            usb_transfer_callback,
                            transfer_context_pointer as *mut libc::c_void,
                            timeout.as_millis() as libc::c_uint,
                        )
                    },
                }
                // unsafe: libusb_alloc_transfer succeeded
                unsafe {
                    transfer.as_mut().flags = 0; // !LIBUSB_TRANSFER_SHORT_NOT_OK
                                                 // !LIBUSB_TRANSFER_FREE_BUFFER
                                                 // !LIBUSB_TRANSFER_FREE_TRANSFER
                                                 // !LIBUSB_TRANSFER_ADD_ZERO_PACKET
                }
                transfers.push(transfer);
            }
        }
        let result = Self {
            transfers,
            handle,
            active_buffer_view: std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false)),
            event_loop,
            context,
        };
        for (index, transfer) in result.transfers.iter().enumerate() {
            // unsafe: libusb_alloc_transfer succeeded
            match unsafe { libusb1_sys::libusb_submit_transfer(transfer.as_ptr()) } {
                0 => (),
                submit_transfer_status => {
                    {
                        let mut shared = result
                            .context
                            .shared
                            .lock()
                            .expect("ring context's lock is not poisoned");
                        for rest_index in index..result.transfers.len() {
                            // dropping 'result' cancels transfers
                            // mark unscheduled transfers as complete to prevent un-needed cancelling
                            shared.transfer_statuses[rest_index] = TransferStatus::Complete;
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
        Ok(result)
    }
}

pub struct BufferView<'a> {
    pub system_time: std::time::SystemTime,
    pub instant: std::time::Instant,
    pub first_after_overflow: bool,
    pub slice: &'a [u8],
    pub read: usize,
    pub write_range: WriteRange,
    pub clutch: Clutch,
    active: std::sync::Arc<std::sync::atomic::AtomicBool>,
}

impl BufferView<'_> {
    pub fn backlog(&self) -> usize {
        let result = (self.write_range.start + self.write_range.ring_length - 1 - self.read)
            % self.write_range.ring_length;
        if matches!(self.clutch, Clutch::Engaged) && result == 0 {
            self.write_range.ring_length
        } else {
            result
        }
    }

    pub fn delay(&self) -> std::time::Duration {
        self.instant.elapsed()
    }
}

impl Drop for BufferView<'_> {
    fn drop(&mut self) {
        self.active
            .store(false, std::sync::atomic::Ordering::Release);
    }
}

impl Ring {
    pub fn backlog(&self) -> usize {
        let shared = self
            .context
            .shared
            .lock()
            .expect("ring context's lock is not poisoned");
        (shared.write_range.start + shared.buffers.len() - 1 - shared.read) % shared.buffers.len()
    }

    pub fn clutch(&self) -> Clutch {
        let shared = self
            .context
            .shared
            .lock()
            .expect("ring context's lock is not poisoned");
        shared.clutch
    }

    pub fn next_with_timeout(&self, duration: &std::time::Duration) -> Option<BufferView> {
        if self
            .active_buffer_view
            .swap(true, std::sync::atomic::Ordering::AcqRel)
        {
            panic!("the buffer returned by a previous call of next_with_timeout must be dropped before calling next_with_timeout again");
        }
        let (system_time, instant, first_after_overflow, slice, read, write_range, clutch) = {
            let start = std::time::Instant::now();
            let mut shared = self
                .context
                .shared
                .lock()
                .expect("ring context's lock is not poisoned");
            loop {
                shared.read = (shared.read + 1) % shared.buffers.len();
                while (shared.write_range.end + shared.buffers.len() - 1 - shared.read)
                    % shared.buffers.len()
                    < shared.transfer_statuses.len()
                {
                    let ellapsed = std::time::Instant::now() - start;
                    if ellapsed >= *duration {
                        self.active_buffer_view
                            .store(false, std::sync::atomic::Ordering::Release);
                        shared.read =
                            (shared.read + shared.buffers.len() - 1) % shared.buffers.len();
                        return None;
                    }
                    shared = self
                        .context
                        .shared_condvar
                        .wait_timeout(shared, *duration - ellapsed)
                        .expect("shared_condvar used with two different mutexes")
                        .0;
                }
                if shared.buffers[shared.read].length > 0 {
                    break;
                }
            }
            (
                shared.buffers[shared.read].system_time,
                shared.buffers[shared.read].instant,
                shared.buffers[shared.read].first_after_overflow,
                // unsafe: data validity guaranteed by read / write_range in shared
                unsafe {
                    std::slice::from_raw_parts(
                        shared.buffers[shared.read].data.as_ptr(),
                        shared.buffers[shared.read].length,
                    )
                },
                shared.read,
                shared.write_range.clone(),
                shared.clutch,
            )
        };
        Some(BufferView {
            system_time,
            instant,
            first_after_overflow,
            slice,
            read,
            write_range,
            clutch,
            active: self.active_buffer_view.clone(),
        })
    }
}

impl Drop for Ring {
    fn drop(&mut self) {
        let mut dealloc_buffers = true;
        let before_dealloc_transfers = std::time::Instant::now();
        #[cfg(target_os = "macos")]
        {
            let mut shared = self
                .context
                .shared
                .lock()
                .expect("ring context's lock is not poisoned");
            // unsafe: transfer is allocated
            let _ = unsafe { libusb1_sys::libusb_cancel_transfer(self.transfers[0].as_ptr()) };
            for index in 0..self.transfers.len() {
                shared.transfer_statuses[index] = TransferStatus::Cancelling;
            }
        }
        loop {
            let mut deallocated_transfers: usize = 0;
            {
                let mut shared = self
                    .context
                    .shared
                    .lock()
                    .expect("ring context's lock is not poisoned");
                for index in 0..self.transfers.len() {
                    match shared.transfer_statuses[index] {
                        TransferStatus::Active => {
                            let status = unsafe {
                                libusb1_sys::libusb_cancel_transfer(self.transfers[index].as_ptr())
                            };
                            if status == 0 {
                                shared.transfer_statuses[index] = TransferStatus::Cancelling;
                            } else {
                                shared.transfer_statuses[index] = TransferStatus::Complete;
                            }
                        }
                        TransferStatus::Complete => {
                            // unsafe: transfer is allocated and user_data is an allocated *mut TransferContext
                            let _transfer_context = unsafe {
                                Box::from_raw(
                                    (self.transfers[index].as_mut()).user_data
                                        as *mut TransferContext,
                                )
                            };
                            // unsafe: transfer is allocated
                            unsafe {
                                libusb1_sys::libusb_free_transfer(self.transfers[index].as_ptr())
                            };
                            shared.transfer_statuses[index] = TransferStatus::Deallocated;
                            deallocated_transfers += 1;
                        }
                        TransferStatus::Cancelling => (),
                        TransferStatus::Deallocated => {
                            deallocated_transfers += 1;
                        }
                    }
                }
            }
            if deallocated_transfers == self.transfers.len() {
                break;
            }
            // give up if the transfers are not freed after one second (better leak memory that loop forever)
            if std::time::Instant::now() - before_dealloc_transfers
                > std::time::Duration::from_secs(1)
            {
                dealloc_buffers = false;
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
        if dealloc_buffers {
            let shared = self
                .context
                .shared
                .lock()
                .expect("ring context's lock is not poisoned");
            for buffer in shared.buffers.iter() {
                if buffer.dma {
                    // unsafe: buffer was allocated by libusb with 'capacity' bytes
                    unsafe {
                        libusb_dev_mem_free(
                            self.handle.as_raw(),
                            buffer.data.as_ptr() as *mut libc::c_uchar,
                            buffer.capacity as libc::ssize_t,
                        );
                    };
                } else {
                    // unsafe: buffer was allocated by alloc with 'Layout {capacity, 1}'
                    unsafe {
                        std::alloc::dealloc(
                            buffer.data.as_ptr(),
                            std::alloc::Layout::from_size_align_unchecked(buffer.capacity, 1),
                        );
                    }
                }
            }
        }
    }
}
