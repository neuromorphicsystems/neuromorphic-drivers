use crate::devices::inivation_davis346;

const STANDARD_GRAVITY: f32 = 9.80665;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameDataRow {
    Idle,
    Reset(u16),
    Signal(u16),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AccelerometerScale {
    TwoG,
    FourG,
    HeightG,
    SixteenG,
}

impl AccelerometerScale {
    pub fn acceleration_metres_per_second(&self, value: i16) -> f32 {
        (match self {
            AccelerometerScale::TwoG => 2.0,
            AccelerometerScale::FourG => 4.0,
            AccelerometerScale::HeightG => 8.0,
            AccelerometerScale::SixteenG => 16.0,
        }) * STANDARD_GRAVITY
            / 32768.0
            * value as f32
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GyroscopeScale {
    TwoHundredAndFifty,
    FiveHundred,
    OneThousand,
    TwoThousand,
}

impl GyroscopeScale {
    pub fn rotation_radians_per_second(&self, value: i16) -> f32 {
        (match self {
            GyroscopeScale::TwoHundredAndFifty => 250.0,
            GyroscopeScale::FiveHundred => 500.0,
            GyroscopeScale::OneThousand => 1000.0,
            GyroscopeScale::TwoThousand => 2000.0,
        }) * (std::f32::consts::PI / 180.0)
            / 32768.0
            * value as f32
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImuState {
    Default,
    Started {
        t: u64,
    },
    Typed {
        t: u64,
        accelerometer_scale: Option<AccelerometerScale>,
        gyroscope_scale: Option<GyroscopeScale>,
        has_temperature: bool,
        bytes: [u8; 14],
        index: usize,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct State {
    pub t: u64,
    pub t_offset: u64,
    pub row: Option<u16>,
    pub start_t: Option<u64>,
    pub exposure_start_t: Option<u64>,
    pub exposure_end_t: Option<u64>,
    pub frame_reset_column: Option<u16>,
    pub frame_signal_column: Option<u16>,
    pub frame_data_row: FrameDataRow,
    pub imu: ImuState,
}

pub struct Adapter {
    dvs_invert_xy: bool,
    dvs_rows: u16,
    dvs_columns: u16,
    aps_orientation: inivation_davis346::ApsOrientation,
    aps_rows: u16,
    aps_columns: u16,
    imu_orientation: inivation_davis346::ImuOrientation,
    imu_type: inivation_davis346::ImuType,
    pixels: Vec<u16>,
    state: State,
}

#[derive(Debug, Copy, Clone)]
pub struct FrameEvent<'a> {
    pub start_t: u64,
    pub exposure_start_t: Option<u64>,
    pub exposure_end_t: Option<u64>,
    pub t: u64,
    pub pixels: &'a [u16],
}

#[repr(C, packed)]
#[derive(Debug, Copy, Clone)]
pub struct ImuEvent {
    pub t: u64,
    pub accelerometer_x: f32,
    pub accelerometer_y: f32,
    pub accelerometer_z: f32,
    pub gyroscope_x: f32,
    pub gyroscope_y: f32,
    pub gyroscope_z: f32,
    pub temperature: f32,
}

impl ImuEvent {
    pub fn as_bytes(&self) -> &[u8] {
        unsafe {
            std::slice::from_raw_parts(
                (self as *const Self) as *const u8,
                std::mem::size_of::<Self>(),
            )
        }
    }
}

fn acceleration(
    accelerometer_scale: Option<AccelerometerScale>,
    byte0: u8,
    byte1: u8,
    flip: bool,
) -> f32 {
    match accelerometer_scale {
        Some(accelerometer_scale) => accelerometer_scale.acceleration_metres_per_second(
            i16::from_be_bytes([byte0, byte1]) * (if flip { -1 } else { 1 }),
        ),
        None => std::f32::NAN,
    }
}

fn rotation(gyroscope_scale: Option<GyroscopeScale>, byte0: u8, byte1: u8, flip: bool) -> f32 {
    match gyroscope_scale {
        Some(gyroscope_scale) => gyroscope_scale.rotation_radians_per_second(
            i16::from_be_bytes([byte0, byte1]) * (if flip { -1 } else { 1 }),
        ),
        None => std::f32::NAN,
    }
}

fn pixel_index(
    row: u16,
    aps_rows: u16,
    column: u16,
    aps_columns: u16,
    aps_orientation: inivation_davis346::ApsOrientation,
) -> usize {
    let row = if aps_orientation.flip_y {
        aps_rows - 1 - row
    } else {
        row
    };
    let column = if aps_orientation.flip_x {
        aps_columns - 1 - column
    } else {
        column
    };
    if aps_orientation.invert_xy {
        row as usize + column as usize * aps_rows as usize
    } else {
        column as usize + row as usize * aps_columns as usize
    }
}

#[derive(Default)]
pub struct EventsLengths {
    pub on: usize,
    pub off: usize,
    pub imu: usize,
    pub trigger: usize,
}

impl Adapter {
    pub fn new(
        dvs_invert_xy: bool,
        aps_orientation: inivation_davis346::ApsOrientation,
        imu_orientation: inivation_davis346::ImuOrientation,
        imu_type: inivation_davis346::ImuType,
    ) -> Self {
        let (dvs_rows, dvs_columns) = if dvs_invert_xy {
            (
                inivation_davis346::PROPERTIES.width,
                inivation_davis346::PROPERTIES.height,
            )
        } else {
            (
                inivation_davis346::PROPERTIES.height,
                inivation_davis346::PROPERTIES.width,
            )
        };
        let (aps_rows, aps_columns) = if aps_orientation.invert_xy {
            (
                inivation_davis346::PROPERTIES.width,
                inivation_davis346::PROPERTIES.height,
            )
        } else {
            (
                inivation_davis346::PROPERTIES.height,
                inivation_davis346::PROPERTIES.width,
            )
        };
        Self {
            dvs_invert_xy,
            dvs_rows,
            dvs_columns,
            aps_orientation,
            aps_rows,
            aps_columns,
            imu_orientation,
            imu_type,
            pixels: vec![
                0u16;
                inivation_davis346::PROPERTIES.width as usize
                    * inivation_davis346::PROPERTIES.height as usize
            ],
            state: State {
                t: 0,
                t_offset: 0,
                row: None,
                start_t: None,
                exposure_start_t: None,
                exposure_end_t: None,
                frame_reset_column: None,
                frame_signal_column: None,
                frame_data_row: FrameDataRow::Idle,
                imu: ImuState::Default,
            },
        }
    }

    pub fn state(&self) -> &State {
        &self.state
    }

    pub fn current_t(&self) -> u64 {
        self.state.t
    }

    pub fn events_lengths(&self, slice: &[u8]) -> EventsLengths {
        // @TODO
        EventsLengths {
            on: 0,
            off: 0,
            imu: 0,
            trigger: 0,
        }
    }

    pub fn events_lengths_until(
        &mut self,
        slice: &[u8],
        threshold_t: u64,
    ) -> (EventsLengths, usize) {
        // @TODO
        (
            EventsLengths {
                on: 0,
                off: 0,
                imu: 0,
                trigger: 0,
            },
            0,
        )
    }

    pub fn convert<HandlePolarityEvent, HandleImuEvent, HandleTriggerEvent, HandleFrameEvent>(
        &mut self,
        slice: &[u8],
        mut handle_polarity_event: HandlePolarityEvent,
        mut handle_imu_event: HandleImuEvent,
        mut handle_trigger_event: HandleTriggerEvent,
        mut handle_frame_event: HandleFrameEvent,
    ) where
        HandlePolarityEvent: FnMut(neuromorphic_types::PolarityEvent<u64, u16, u16>),
        HandleImuEvent: FnMut(ImuEvent),
        HandleTriggerEvent: FnMut(neuromorphic_types::TriggerEvent<u64, u8>),
        HandleFrameEvent: FnMut(FrameEvent),
    {
        for index in 0..slice.len() / 2 {
            let word = u16::from_le_bytes([slice[index * 2], slice[index * 2 + 1]]);
            if (word & 0x8000) > 0 {
                let candidate_t = self.state.t_offset + (word & 0x7FFF) as u64;
                if candidate_t >= self.state.t {
                    self.state.t = candidate_t;
                }
            } else {
                let message_type = (word & 0x7000) >> 12;
                match message_type {
                    0 => {
                        match word & 0x0FFF {
                            0 => {} // reserved code
                            1 => {} // timestamp reset
                            2 => handle_trigger_event(neuromorphic_types::TriggerEvent {
                                t: self.state.t,
                                id: 0,
                                polarity: neuromorphic_types::TriggerPolarity::Falling,
                            }),
                            3 => handle_trigger_event(neuromorphic_types::TriggerEvent {
                                t: self.state.t,
                                id: 0,
                                polarity: neuromorphic_types::TriggerPolarity::Rising,
                            }),
                            4 => handle_trigger_event(neuromorphic_types::TriggerEvent {
                                t: self.state.t,
                                id: 0,
                                polarity: neuromorphic_types::TriggerPolarity::Pulse,
                            }),
                            5 => {
                                self.state.imu = ImuState::Started { t: self.state.t };
                            }
                            7 => {
                                if let ImuState::Typed {
                                    t,
                                    bytes,
                                    index,
                                    accelerometer_scale,
                                    gyroscope_scale,
                                    has_temperature,
                                } = self.state.imu
                                {
                                    if index == bytes.len() {
                                        handle_imu_event(ImuEvent {
                                            t,
                                            accelerometer_x: acceleration(
                                                accelerometer_scale,
                                                bytes[0],
                                                bytes[1],
                                                self.imu_orientation.flip_x,
                                            ),
                                            accelerometer_y: acceleration(
                                                accelerometer_scale,
                                                bytes[2],
                                                bytes[3],
                                                self.imu_orientation.flip_y,
                                            ),
                                            accelerometer_z: acceleration(
                                                accelerometer_scale,
                                                bytes[4],
                                                bytes[5],
                                                self.imu_orientation.flip_z,
                                            ),
                                            gyroscope_x: rotation(
                                                gyroscope_scale,
                                                bytes[8],
                                                bytes[9],
                                                self.imu_orientation.flip_x,
                                            ),
                                            gyroscope_y: rotation(
                                                gyroscope_scale,
                                                bytes[10],
                                                bytes[11],
                                                self.imu_orientation.flip_y,
                                            ),
                                            gyroscope_z: rotation(
                                                gyroscope_scale,
                                                bytes[12],
                                                bytes[13],
                                                self.imu_orientation.flip_z,
                                            ),
                                            temperature: if has_temperature {
                                                self.imu_type.temperature_celsius(
                                                    i16::from_be_bytes([bytes[6], bytes[7]]),
                                                )
                                            } else {
                                                std::f32::NAN
                                            },
                                        })
                                    }
                                }
                                self.state.imu = ImuState::Default;
                            }
                            8 | 9 => {
                                self.state.start_t = Some(self.state.t);
                                self.state.exposure_start_t = None;
                                self.state.exposure_end_t = None;
                                self.state.frame_reset_column = Some(0);
                                self.state.frame_signal_column = Some(0);
                                self.state.frame_data_row = FrameDataRow::Idle;
                                self.pixels.fill(u16::MAX);
                            }
                            10 => {
                                if let Some(start_t) = self.state.start_t {
                                    for pixel in self.pixels.iter_mut() {
                                        if *pixel == u16::MAX {
                                            *pixel = 0;
                                        }
                                    }
                                    handle_frame_event(FrameEvent {
                                        start_t,
                                        exposure_start_t: self.state.exposure_start_t,
                                        exposure_end_t: self.state.exposure_end_t,
                                        t: self.state.t,
                                        pixels: &self.pixels,
                                    });
                                    self.state.start_t = None;
                                    self.state.exposure_start_t = None;
                                    self.state.exposure_end_t = None;
                                    self.state.frame_reset_column = None;
                                    self.state.frame_signal_column = None;
                                }
                            }
                            11 => {
                                if let Some(frame_reset_column) = self.state.frame_reset_column {
                                    if frame_reset_column < self.aps_columns {
                                        self.state.frame_reset_column =
                                            Some(frame_reset_column + 1);
                                        self.state.frame_data_row = FrameDataRow::Reset(0);
                                    } else {
                                        self.state.frame_reset_column = None;
                                        self.state.frame_data_row = FrameDataRow::Idle;
                                    }
                                }
                            }
                            12 => {
                                if let Some(frame_signal_column) = self.state.frame_signal_column {
                                    if frame_signal_column < self.aps_columns {
                                        self.state.frame_signal_column =
                                            Some(frame_signal_column + 1);
                                        self.state.frame_data_row = FrameDataRow::Signal(0);
                                    } else {
                                        self.state.frame_signal_column = None;
                                        self.state.frame_data_row = FrameDataRow::Idle;
                                    }
                                }
                            }
                            13 => {
                                self.state.frame_data_row = FrameDataRow::Idle;
                            }
                            14 => {
                                self.state.exposure_start_t = Some(self.state.t);
                            }
                            15 => {
                                self.state.exposure_end_t = Some(self.state.t);
                            }
                            16 => {
                                // External generator (falling edge)
                            }
                            17 => {
                                // External generator (rising edge)
                            }
                            _ => {}
                        }
                    }
                    1 => {
                        let candidate_row = word & 0x0FFF;
                        if candidate_row < self.dvs_rows {
                            self.state.row = Some(candidate_row);
                        } else {
                            self.state.row = None;
                        }
                    }
                    2 | 3 => {
                        if let Some(row) = self.state.row {
                            let column = word & 0x0FFF;
                            if column < self.dvs_columns {
                                handle_polarity_event(neuromorphic_types::PolarityEvent {
                                    t: self.state.t,
                                    x: if self.dvs_invert_xy { row } else { column },
                                    y: if self.dvs_invert_xy { column } else { row },
                                    polarity: if message_type == 2 {
                                        neuromorphic_types::Polarity::Off
                                    } else {
                                        neuromorphic_types::Polarity::On
                                    },
                                })
                            }
                        }
                    }
                    4 => {
                        self.state.frame_data_row = match self.state.frame_data_row {
                            FrameDataRow::Idle => FrameDataRow::Idle,
                            FrameDataRow::Reset(row) => {
                                if let Some(column) = self.state.frame_reset_column {
                                    if column > 0 {
                                        self.pixels[pixel_index(
                                            row,
                                            self.aps_rows,
                                            column - 1,
                                            self.aps_columns,
                                            self.aps_orientation,
                                        )] = word & 0x0FFF;
                                        if row < self.aps_rows - 1 {
                                            FrameDataRow::Reset(row + 1)
                                        } else {
                                            FrameDataRow::Idle
                                        }
                                    } else {
                                        FrameDataRow::Reset(row)
                                    }
                                } else {
                                    FrameDataRow::Reset(row)
                                }
                            }
                            FrameDataRow::Signal(row) => {
                                if let Some(column) = self.state.frame_signal_column {
                                    if column > 0 {
                                        let index = pixel_index(
                                            row,
                                            self.aps_rows,
                                            column - 1,
                                            self.aps_columns,
                                            self.aps_orientation,
                                        );
                                        let reset_value = self.pixels[index];
                                        let signal_value = word & 0x0FFF;
                                        self.pixels[index] = if reset_value == u16::MAX {
                                            u16::MAX
                                        } else if reset_value < 384 || signal_value == 0 {
                                            1023 << 6
                                        } else if reset_value < signal_value {
                                            0
                                        } else {
                                            (reset_value - signal_value).min(1023) << 6
                                        };
                                        if row < self.aps_rows - 1 {
                                            FrameDataRow::Signal(row + 1)
                                        } else {
                                            FrameDataRow::Idle
                                        }
                                    } else {
                                        FrameDataRow::Signal(row)
                                    }
                                } else {
                                    FrameDataRow::Signal(row)
                                }
                            }
                        }
                    }
                    5 => {
                        match (word & 0x0F00) >> 8 {
                            0 => {
                                self.state.imu = match self.state.imu {
                                    ImuState::Default => ImuState::Default,
                                    ImuState::Started { t } => ImuState::Started { t },
                                    ImuState::Typed {
                                        t,
                                        accelerometer_scale,
                                        gyroscope_scale,
                                        has_temperature,
                                        mut bytes,
                                        index,
                                    } => {
                                        if index < bytes.len() {
                                            bytes[index] = (word & 0xFF) as u8;
                                        }
                                        ImuState::Typed {
                                            t,
                                            accelerometer_scale,
                                            gyroscope_scale,
                                            has_temperature,
                                            bytes,
                                            index: match index {
                                                5 => {
                                                    if has_temperature {
                                                        index + 1
                                                    } else {
                                                        if gyroscope_scale.is_some() {
                                                            index + 3
                                                        } else {
                                                            index + 9
                                                        }
                                                    }
                                                }
                                                7 => {
                                                    if gyroscope_scale.is_some() {
                                                        index + 1
                                                    } else {
                                                        index + 7
                                                    }
                                                }
                                                _ => index + 1,
                                            },
                                        }
                                    }
                                };
                            }
                            1 => {}
                            2 => {}
                            3 => {
                                self.state.imu = match self.state.imu {
                                    ImuState::Default => ImuState::Default,
                                    ImuState::Started { t } | ImuState::Typed { t, .. } => {
                                        let has_temperature = ((word >> 5) & 1) == 1;
                                        let has_gyroscope = ((word >> 6) & 1) == 1;
                                        let has_accelerometer = ((word >> 7) & 1) == 1;
                                        ImuState::Typed {
                                            t,
                                            accelerometer_scale: if has_accelerometer {
                                                Some(match (word >> 2) & 0b11 {
                                                    0 => AccelerometerScale::TwoG,
                                                    1 => AccelerometerScale::FourG,
                                                    2 => AccelerometerScale::HeightG,
                                                    3 => AccelerometerScale::SixteenG,
                                                    _ => unreachable!(),
                                                })
                                            } else {
                                                None
                                            },
                                            gyroscope_scale: if has_gyroscope {
                                                Some(match word & 0b11 {
                                                    0 => GyroscopeScale::TwoHundredAndFifty,
                                                    1 => GyroscopeScale::FiveHundred,
                                                    2 => GyroscopeScale::OneThousand,
                                                    3 => GyroscopeScale::TwoThousand,
                                                    _ => unreachable!(),
                                                })
                                            } else {
                                                None
                                            },
                                            has_temperature,
                                            bytes: [0u8; 14],
                                            index: if has_accelerometer {
                                                0
                                            } else if has_temperature {
                                                6
                                            } else if has_gyroscope {
                                                8
                                            } else {
                                                14
                                            },
                                        }
                                    }
                                };
                            }
                            _ => {
                                println!("unexpected MISC 8 code {}", (word & 0x0F00) >> 8);
                                // @DEV
                            }
                        }
                    }
                    6 => {
                        match (word & 0x0C00) >> 10 {
                            0 => {
                                // @DEV @TODO auto-exposure feedback
                            }
                            _ => {
                                println!("unexpected MISC 10 code {}", (word & 0x0C00) >> 10);
                                // @DEV
                            }
                        }
                    }
                    7 => {
                        self.state.t_offset += 0x8000;
                    }
                    _ => unreachable!(),
                }
            }
        }
    }
}
