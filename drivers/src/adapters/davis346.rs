#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameDataRow {
    Idle,
    Reset(u16),
    Signal(u16),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct State {
    pub t: u64,
    pub t_offset: u64,
    pub column: Option<u16>,
    pub start_t: Option<u64>,
    pub exposure_start_t: Option<u64>,
    pub exposure_end_t: Option<u64>,
    pub frame_reset_column: Option<u16>,
    pub frame_signal_column: Option<u16>,
    pub frame_data_row: FrameDataRow,
}

pub struct Adapter {
    pixels: Vec<u16>,
    state: State, // @DEV move state props here
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

#[derive(Default)]
pub struct EventsLengths {
    pub on: usize,
    pub off: usize,
    pub imu: usize,
    pub trigger: usize,
}

impl Adapter {
    pub fn new() -> Self {
        Self {
            pixels: vec![0u16; 346 * 260],
            state: State {
                t: 0,
                t_offset: 0,
                column: None,
                start_t: None,
                exposure_start_t: None,
                exposure_end_t: None,
                frame_reset_column: None,
                frame_signal_column: None,
                frame_data_row: FrameDataRow::Idle,
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

    pub fn convert<HandleDvsEvent, HandleImuEvent, HandleTriggerEvent, HandleFrameEvent>(
        &mut self,
        slice: &[u8],
        mut handle_dvs_event: HandleDvsEvent,
        mut handle_imu_event: HandleImuEvent,
        mut handle_trigger_event: HandleTriggerEvent,
        mut handle_frame_event: HandleFrameEvent,
    ) where
        HandleDvsEvent: FnMut(neuromorphic_types::PolarityEvent<u64, u16, u16>),
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
                                // IMU start
                            }
                            7 => {
                                // IMU end
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
                                    if frame_reset_column < 260 {
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
                                    if frame_signal_column < 260 {
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
                        let candidate_column = word & 0x0FFF;
                        if candidate_column < 346 {
                            self.state.column = Some(candidate_column);
                        } else {
                            self.state.column = None;
                        }
                    }
                    2 | 3 => {
                        if let Some(column) = self.state.column {
                            let row = word & 0x0FFF;
                            if row < 260 {
                                handle_dvs_event(neuromorphic_types::PolarityEvent {
                                    t: self.state.t,
                                    x: column,
                                    y: row,
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
                                        let x = 345 - row;
                                        let y = column - 1;
                                        self.pixels[x as usize + y as usize * 346] = word & 0x0FFF;
                                        if row < 345 {
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
                                        let x = 345 - row;
                                        let y = column - 1;
                                        let reset_value =
                                            self.pixels[x as usize + y as usize * 346];
                                        let signal_value = word & 0x0FFF;
                                        self.pixels[x as usize + y as usize * 346] =
                                            if reset_value == u16::MAX {
                                                u16::MAX
                                            } else if reset_value < 384 || signal_value == 0 {
                                                1023 << 6
                                            } else if reset_value < signal_value {
                                                0
                                            } else {
                                                (reset_value - signal_value).min(1023) << 6
                                            };
                                        if row < 345 {
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
                        // misc 8 bit (IMU data, APS ROI)
                    }
                    6 => {
                        // misc 10  bit (frame exposure), related to auto exposure?
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
