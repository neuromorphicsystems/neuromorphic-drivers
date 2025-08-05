use crate::devices::inivation_dvxplorer;

const STANDARD_GRAVITY: f32 = 9.80665;

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
    TwoThousand,
    OneThousand,
    FiveHundred,
    TwoHundredAndFifty,
    OneHundredAndTwentyFive,
}

impl GyroscopeScale {
    pub fn rotation_radians_per_second(&self, value: i16) -> f32 {
        (match self {
            GyroscopeScale::TwoThousand => 2000.0,
            GyroscopeScale::OneThousand => 1000.0,
            GyroscopeScale::FiveHundred => 500.0,
            GyroscopeScale::TwoHundredAndFifty => 250.0,
            GyroscopeScale::OneHundredAndTwentyFive => 125.0,
        }) * (std::f32::consts::PI / 180.0)
            / 32768.0
            * value as f32
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImuState {
    Default,
    Started,
    Typed {
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
    pub row_group_0: Option<u16>,
    pub row_group_1: Option<u16>,
    pub column: Option<u16>,
    pub imu: ImuState,
}

pub struct Adapter {
    dvs_orientation: inivation_dvxplorer::DvsOrientation,
    imu_orientation: inivation_dvxplorer::ImuOrientation,
    state: State,
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

#[derive(Default)]
pub struct EventsLengths {
    pub on: usize,
    pub off: usize,
    pub imu: usize,
    pub trigger_rising: usize,
    pub trigger_falling: usize,
    pub trigger_pulse: usize,
}

impl Adapter {
    pub fn new(
        dvs_orientation: inivation_dvxplorer::DvsOrientation,
        imu_orientation: inivation_dvxplorer::ImuOrientation,
    ) -> Self {
        Self {
            dvs_orientation,
            imu_orientation,
            state: State {
                t: 0,
                t_offset: 0,
                row_group_0: None,
                row_group_1: None,
                column: None,
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
            trigger_rising: 0,
            trigger_falling: 0,
            trigger_pulse: 0,
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
                trigger_rising: 0,
                trigger_falling: 0,
                trigger_pulse: 0,
            },
            0,
        )
    }

    pub fn convert<HandlePolarityEvent, HandleImuEvent, HandleTriggerEvent>(
        &mut self,
        slice: &[u8],
        mut handle_polarity_event: HandlePolarityEvent,
        mut handle_imu_event: HandleImuEvent,
        mut handle_trigger_event: HandleTriggerEvent,
    ) where
        HandlePolarityEvent: FnMut(neuromorphic_types::PolarityEvent<u64, u16, u16>),
        HandleImuEvent: FnMut(ImuEvent),
        HandleTriggerEvent: FnMut(neuromorphic_types::TriggerEvent<u64, u8>),
    {
        for index in 0..slice.len() / 2 {
            let word = u16::from_le_bytes([slice[index * 2], slice[index * 2 + 1]]);
            if (word >> 15) & 1 == 1 {
                self.state.t = self
                    .state
                    .t
                    .max(self.state.t_offset + (word & 0x7FFF) as u64);
            } else {
                let message_type = (word & 0x7000) >> 12;
                match message_type {
                    0 => match word & 0xFFF {
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
                            self.state.imu = ImuState::Started;
                        }
                        7 => {
                            if let ImuState::Typed {
                                bytes,
                                index,
                                accelerometer_scale,
                                gyroscope_scale,
                                has_temperature,
                            } = self.state.imu
                            {
                                if index == bytes.len() {
                                    handle_imu_event(ImuEvent {
                                        t: self.state.t,
                                        accelerometer_x: acceleration(
                                            accelerometer_scale,
                                            bytes[2],
                                            bytes[3],
                                            self.imu_orientation.flip_x,
                                        ),
                                        accelerometer_y: acceleration(
                                            accelerometer_scale,
                                            bytes[0],
                                            bytes[1],
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
                                            (i16::from_be_bytes([bytes[6], bytes[7]]) as f32
                                                / 512.0)
                                                + 23.0
                                        } else {
                                            std::f32::NAN
                                        },
                                    })
                                }
                            }
                            self.state.imu = ImuState::Default;
                        }
                        _ => {}
                    },
                    1 => {
                        let column = word & 0x03FF;
                        self.state.column = if column < inivation_dvxplorer::PROPERTIES.width {
                            Some(column)
                        } else {
                            None
                        };
                    }
                    2 | 3 => {
                        if let Some(column) = self.state.column {
                            let row_group = if message_type == 2 {
                                self.state.row_group_1
                            } else {
                                self.state.row_group_0
                            };
                            if let Some(row_group) = row_group {
                                let polarity = if (word >> 8) & 1 == 1 {
                                    neuromorphic_types::Polarity::Off
                                } else {
                                    neuromorphic_types::Polarity::On
                                };
                                for bit in 0..8 {
                                    if (word & (1 << bit)) > 0 {
                                        handle_polarity_event(neuromorphic_types::PolarityEvent {
                                            t: self.state.t,
                                            x: column,
                                            y: row_group * 8 + bit,
                                            polarity,
                                        });
                                    }
                                }
                            }
                        }
                    }
                    4 => {
                        let row_group_0 = word & 0b111111;
                        let row_group_1_offset = (word >> 6) & 0b11111;
                        let row_group_1 = if (word >> 11) & 1 == 1 {
                            if row_group_1_offset <= row_group_0 {
                                row_group_0 - row_group_1_offset
                            } else {
                                0xFFFF
                            }
                        } else {
                            row_group_0 + ((word >> 6) & 0b11111)
                        };
                        self.state.row_group_0 =
                            if row_group_0 < inivation_dvxplorer::PROPERTIES.height / 8 {
                                Some(row_group_0)
                            } else {
                                None
                            };
                        self.state.row_group_1 =
                            if row_group_1 < inivation_dvxplorer::PROPERTIES.height / 8 {
                                Some(row_group_1)
                            } else {
                                None
                            };
                    }
                    5 => match (word & 0x0F00) >> 8 {
                        0 => {
                            self.state.imu = match self.state.imu {
                                ImuState::Default => ImuState::Default,
                                ImuState::Started => ImuState::Started,
                                ImuState::Typed {
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
                                ImuState::Started | ImuState::Typed { .. } => {
                                    let has_temperature = ((word >> 5) & 1) == 1;
                                    let has_gyroscope = ((word >> 6) & 1) == 1;
                                    let has_accelerometer = ((word >> 7) & 1) == 1;
                                    ImuState::Typed {
                                        accelerometer_scale: if has_accelerometer {
                                            Some(match (word >> 3) & 0b11 {
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
                                            Some(match word & 0b111 {
                                                0 => GyroscopeScale::TwoThousand,
                                                1 => GyroscopeScale::OneThousand,
                                                2 => GyroscopeScale::FiveHundred,
                                                3 => GyroscopeScale::TwoHundredAndFifty,
                                                4 | 5 | 6 | 7 => {
                                                    GyroscopeScale::OneHundredAndTwentyFive
                                                }
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
                        _ => {}
                    },
                    6 => {}
                    7 => {
                        self.state.t_offset += 0x8000;
                        self.state.t = self.state.t.max(self.state.t_offset);
                    }
                    _ => unreachable!(),
                }
            }
        }
    }
}
