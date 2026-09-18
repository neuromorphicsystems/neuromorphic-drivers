#[derive(Debug, Clone, Copy)]
pub struct Overflow(pub ());

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Configuration {
    pub buffer_length: usize,
    pub ring_length: usize,
    pub parallel_submissions: usize,
}

#[derive(thiserror::Error, Debug, Clone)]
pub enum ConfigurationError {
    #[error("{0}")]
    Bincode(String),

    #[error("the number of parallel submissions ({parallel_submissions}) must be strictly smaller than the ring length ({ring_length})
    ")]
    LengthMistmatch {
        ring_length: usize,
        parallel_submissions: usize,
    },

    #[error("the buffer length must be strictly larger than zero")]
    ZeroBufferLength,

    #[error("the number of parallel submissions must be strictly larger than zero")]
    ZeroParallelSubmissions,

    #[error("the buffer length ({0}) must be even")]
    OddBufferLength(usize),

    #[error("not enough memory to allocate the requested buffers ({0} B total)")]
    NotEnoughMemory(usize),
}

impl Configuration {
    pub fn validate(&self) -> Result<(), ConfigurationError> {
        if self.buffer_length == 0 {
            return Err(ConfigurationError::ZeroBufferLength);
        }
        if self.parallel_submissions == 0 {
            return Err(ConfigurationError::ZeroParallelSubmissions);
        }
        if !self.buffer_length.is_multiple_of(2) {
            return Err(ConfigurationError::OddBufferLength(self.buffer_length));
        }
        if self.ring_length <= self.parallel_submissions {
            return Err(ConfigurationError::LengthMistmatch {
                ring_length: self.ring_length,
                parallel_submissions: self.parallel_submissions,
            });
        }
        Ok(())
    }

    pub fn deserialize_bincode(data: &[u8]) -> Result<Configuration, ConfigurationError> {
        let configuration: Configuration = bincode::deserialize(data)
            .map_err(|error| ConfigurationError::Bincode(format!("{error}")))?;
        configuration.validate()?;
        Ok(configuration)
    }
}

pub struct Buffer {
    pub system_time: std::time::SystemTime,
    pub instant: std::time::Instant,
    pub first_after_overflow: bool,
    pub data: *mut u8,
    pub capacity: usize,
    pub length: usize,
}

impl Buffer {
    pub fn new(capacity: usize) -> Option<Self> {
        std::ptr::NonNull::new(
            // unsafe: alloc wrapper
            // std::alloc::Layout::from_length_align_unchecked
            // - align must not be zero
            // - align must be a power of two
            // - size, when rounded up to the nearest multiple of align, must not overflow isize
            unsafe {
                std::alloc::alloc(std::alloc::Layout::from_size_align_unchecked(capacity, 1))
            },
        )
        .map(|data| Buffer {
            system_time: std::time::SystemTime::now(),
            instant: std::time::Instant::now(),
            first_after_overflow: false,
            data: data.as_ptr(),
            capacity,
            length: 0,
        })
    }
}

impl Drop for Buffer {
    fn drop(&mut self) {
        unsafe {
            std::alloc::dealloc(
                self.data,
                std::alloc::Layout::from_size_align_unchecked(self.capacity, 1),
            );
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Clutch {
    Disengaged,
    Engaged,
}

#[derive(Debug, Clone, Copy)]
pub enum TransferStatus {
    Active,
    Complete,
    Cancelling,
    Deallocated,
}

pub struct RingData {
    read: usize,
    write_start: usize,
    write_end: usize,
    parallel_submissions: usize,
    transfer_statuses: Vec<TransferStatus>,
    buffers: Vec<Buffer>,
    freewheel_buffers: Vec<Buffer>,
    freewheel_write_start: usize,
    freewheel_write_end: usize,
    read_clutch: Clutch,
    write_clutch: Clutch,
    dropped_packets: u64,
}

// unsafe: ring data is designed to be called from multiple threads
unsafe impl Send for RingData {}
unsafe impl Sync for RingData {}

impl RingData {
    pub fn transfer_status(&self, index: usize) -> TransferStatus {
        self.transfer_statuses[index]
    }

    pub fn update_transfer_status(&mut self, index: usize, status: TransferStatus) {
        self.transfer_statuses[index] = status;
    }

    pub fn transfer_complete_without_next(
        &mut self,
        clutch: Clutch,
        system_time: std::time::SystemTime,
        instant: std::time::Instant,
        length: usize,
    ) {
        match clutch {
            Clutch::Disengaged => {
                let buffer = &mut self.buffers[self.write_start];
                buffer.system_time = system_time;
                buffer.instant = instant;
                buffer.length = length;
                buffer.first_after_overflow = matches!(self.write_clutch, Clutch::Engaged);
                self.write_start = (self.write_start + 1) % self.buffers.len();
            }
            Clutch::Engaged => {
                let buffer = &mut self.freewheel_buffers[self.freewheel_write_start];
                buffer.system_time = system_time;
                buffer.instant = instant;
                buffer.length = length;
                buffer.first_after_overflow = false;
                self.freewheel_write_start =
                    (self.freewheel_write_start + 1) % self.freewheel_buffers.len();
                self.dropped_packets = self.dropped_packets.saturating_add(1);
            }
        }
        self.write_clutch = clutch;
    }

    pub fn transfer_complete(
        &mut self,
        clutch: Clutch,
        system_time: std::time::SystemTime,
        instant: std::time::Instant,
        length: usize,
    ) -> WriteBufferView {
        self.transfer_complete_without_next(clutch, system_time, instant, length);
        if self.write_end == self.read {
            let index = self.freewheel_write_end;
            let write_buffer_view = WriteBufferView::new(
                Clutch::Engaged,
                matches!(self.read_clutch, Clutch::Disengaged),
                &self.freewheel_buffers[index],
            );
            self.freewheel_write_end =
                (self.freewheel_write_end + 1) % self.freewheel_buffers.len();
            self.read_clutch = Clutch::Engaged;
            write_buffer_view
        } else {
            let index = self.write_end;
            let write_buffer_view = WriteBufferView::new(
                Clutch::Disengaged,
                matches!(self.read_clutch, Clutch::Engaged),
                &self.buffers[index],
            );
            self.write_end = (self.write_end + 1) % self.buffers.len();
            self.read_clutch = Clutch::Disengaged;
            write_buffer_view
        }
    }
}

pub struct Ring {
    pub data: std::sync::Mutex<RingData>,
    pub condvar: std::sync::Condvar,
    pub active_buffer_view: std::sync::atomic::AtomicBool,
}

#[derive(Clone)]
pub struct SharedRing(std::sync::Arc<Ring>);

pub struct WriteBufferView {
    pub clutch: Clutch,
    pub clutch_changed: bool,
    pub data: *mut u8,
    pub capacity: usize,
}

impl WriteBufferView {
    fn new(clutch: Clutch, clutch_changed: bool, buffer: &Buffer) -> Self {
        WriteBufferView {
            clutch,
            clutch_changed,
            data: buffer.data,
            capacity: buffer.capacity,
        }
    }
}

impl SharedRing {
    pub fn data(&self) -> std::sync::MutexGuard<'_, RingData> {
        self.0.data.lock().expect("the ring data is not poisoned")
    }

    pub fn notify_one(&self) {
        self.0.condvar.notify_one();
    }

    pub fn new(
        configuration: &Configuration,
    ) -> Result<(Self, Vec<WriteBufferView>), ConfigurationError> {
        configuration.validate()?;
        let mut buffers = Vec::new();
        buffers.reserve_exact(configuration.ring_length);
        let mut freewheel_buffers = Vec::new();
        freewheel_buffers.reserve_exact(configuration.parallel_submissions);
        for index in 0..configuration.ring_length + configuration.parallel_submissions {
            let buffer = Buffer::new(configuration.buffer_length);
            match buffer {
                Some(buffer) => if index < configuration.ring_length {
                    &mut buffers
                } else {
                    &mut freewheel_buffers
                }
                .push(buffer),
                None => {
                    return Err(ConfigurationError::NotEnoughMemory(
                        configuration.buffer_length
                            * (configuration.ring_length + configuration.parallel_submissions),
                    ));
                }
            }
        }
        let ring = Ring {
            data: std::sync::Mutex::new(RingData {
                read: configuration.ring_length - 1,
                write_start: 0,
                write_end: configuration.parallel_submissions,
                parallel_submissions: configuration.parallel_submissions,
                transfer_statuses: vec![TransferStatus::Active; configuration.parallel_submissions],
                buffers,
                freewheel_buffers,
                freewheel_write_start: 0,
                freewheel_write_end: 0,
                read_clutch: Clutch::Disengaged,
                write_clutch: Clutch::Disengaged,
                dropped_packets: 0,
            }),
            condvar: std::sync::Condvar::new(),
            active_buffer_view: std::sync::atomic::AtomicBool::new(false),
        };
        let mut views = Vec::new();
        views.reserve_exact(configuration.parallel_submissions);
        {
            let ring_data = ring.data.lock().expect("the ring data is not poisoned");
            for index in 0..configuration.parallel_submissions {
                views.push(WriteBufferView::new(
                    Clutch::Disengaged,
                    false,
                    &ring_data.buffers[index],
                ));
            }
        }
        Ok((Self(std::sync::Arc::new(ring)), views))
    }

    pub fn dropped_packets(&self) -> u64 {
        self.0
            .data
            .lock()
            .expect("the ring lock is not poisoned")
            .dropped_packets
    }

    pub fn backlog(&self) -> usize {
        let data = self.0.data.lock().expect("the ring lock is not poisoned");
        (data.write_start + data.buffers.len() - 1 - data.read) % data.buffers.len()
    }

    pub fn clutch(&self) -> Clutch {
        let data = self.0.data.lock().expect("the ring lock is not poisoned");
        data.read_clutch
    }

    pub fn next_with_timeout(&self, duration: &std::time::Duration) -> Option<ReadBufferView<'_>> {
        if self
            .0
            .active_buffer_view
            .swap(true, std::sync::atomic::Ordering::AcqRel)
        {
            panic!("the buffer returned by a previous call of next_with_timeout must be dropped before calling next_with_timeout again");
        }
        let (
            system_time,
            instant,
            first_after_overflow,
            slice,
            read,
            write_start,
            ring_length,
            clutch,
        ) = {
            let start = std::time::Instant::now();
            let mut data = self
                .0
                .data
                .lock()
                .expect("ring context's lock is not poisoned");
            loop {
                data.read = (data.read + 1) % data.buffers.len();
                while (data.write_end + data.buffers.len() - 1 - data.read) % data.buffers.len()
                    < data.parallel_submissions
                {
                    let ellapsed = std::time::Instant::now() - start;
                    if ellapsed >= *duration {
                        self.0
                            .active_buffer_view
                            .store(false, std::sync::atomic::Ordering::Release);
                        data.read = (data.read + data.buffers.len() - 1) % data.buffers.len();
                        return None;
                    }
                    data = self
                        .0
                        .condvar
                        .wait_timeout(data, *duration - ellapsed)
                        .expect("shared_condvar used with two different mutexes")
                        .0;
                }
                if data.buffers[data.read].length > 0 {
                    break;
                }
            }
            (
                data.buffers[data.read].system_time,
                data.buffers[data.read].instant,
                data.buffers[data.read].first_after_overflow,
                // unsafe:
                // - the data will not be mutated whilst the slice exists (guaranteed by the ring internal logic)
                // - the data will not be freed whilst the slice exists (buffer view owns an arc to the ring data)
                unsafe {
                    std::slice::from_raw_parts(
                        data.buffers[data.read].data,
                        data.buffers[data.read].length,
                    )
                },
                data.read,
                data.write_start,
                data.buffers.len(),
                data.read_clutch,
            )
        };
        Some(ReadBufferView {
            system_time,
            instant,
            first_after_overflow,
            slice,
            read,
            write_start,
            ring_length,
            clutch,
            shared: self.clone(),
        })
    }
}

pub struct ReadBufferView<'a> {
    pub system_time: std::time::SystemTime,
    pub instant: std::time::Instant,
    pub first_after_overflow: bool,
    pub slice: &'a [u8],
    pub read: usize,
    pub write_start: usize,
    pub ring_length: usize,
    pub clutch: Clutch,
    shared: SharedRing,
}

impl ReadBufferView<'_> {
    pub fn backlog(&self) -> usize {
        let backlog_length =
            (self.write_start + self.ring_length - 1 - self.read) % self.ring_length;
        if backlog_length == 0 && matches!(self.clutch, Clutch::Engaged) {
            self.ring_length
        } else {
            backlog_length
        }
    }

    pub fn delay(&self) -> std::time::Duration {
        self.instant.elapsed()
    }
}

impl Drop for ReadBufferView<'_> {
    fn drop(&mut self) {
        self.shared
            .0
            .active_buffer_view
            .store(false, std::sync::atomic::Ordering::Release);
    }
}
