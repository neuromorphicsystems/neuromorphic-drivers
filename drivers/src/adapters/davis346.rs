enum FrameState {}

pub struct Adapter {
    t: u64,
    t_offset: u64,
    column: Option<u16>,
    frame: Vec<u8>,
    time_ref: std::time::Instant, // @DEV
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
    pub dvs: usize,
    pub imu: usize,
    pub trigger: usize,
}

impl Adapter {
    pub fn new() -> Self {
        Self {
            t: 0,
            t_offset: 0,
            column: None,
            frame: vec![0u8; 346 * 260],
            time_ref: std::time::Instant::now(), // @DEV
        }
    }

    pub fn events_lengths(&self, slice: &[u8]) -> EventsLengths {
        EventsLengths {
            dvs: 0,
            imu: 0,
            trigger: 0,
        } // @TODO
    }

    pub fn current_t(&self) -> u64 {
        self.t
    }

    pub fn convert<HandleDvsEvent, HandleImu, HandleTrigger, HandleFrame>(
        &mut self,
        slice: &[u8],
        mut handle_dvs_event: HandleDvsEvent,
        mut handle_imu: HandleImu,
        mut handle_trigger: HandleTrigger,
        mut handle_frame: HandleFrame,
    ) where
        HandleDvsEvent: FnMut(neuromorphic_types::DvsEvent<u64, u16, u16>),
        HandleImu: FnMut(ImuEvent),
        HandleTrigger: FnMut(neuromorphic_types::TriggerEvent<u64, u8>),
        HandleFrame: FnMut(&[u8]),
    {
        for index in 0..slice.len() / 2 {
            let word = u16::from_le_bytes([slice[index * 2], slice[index * 2 + 1]]);
            if (word & 0x8000) > 0 {
                let candidate_t = self.t_offset + (word & 0x7FFF) as u64;
                if candidate_t >= self.t {
                    self.t = candidate_t;
                }
            } else {
                let message_type = (word & 0x7000) >> 12;
                match message_type {
                    0 => {
                        match word & 0x0FFF {
                            0 => {} // reserved code
                            1 => {} // timestamp reset
                            2 => handle_trigger(neuromorphic_types::TriggerEvent {
                                t: self.t,
                                id: 0,
                                polarity: neuromorphic_types::TriggerPolarity::Falling,
                            }),

                            3 => handle_trigger(neuromorphic_types::TriggerEvent {
                                t: self.t,
                                id: 0,
                                polarity: neuromorphic_types::TriggerPolarity::Rising,
                            }),
                            4 => handle_trigger(neuromorphic_types::TriggerEvent {
                                t: self.t,
                                id: 0,
                                polarity: neuromorphic_types::TriggerPolarity::Rising, // @TODO replace with pulse after updating neuromorphic_types
                            }),
                            5 => {
                                // IMU start
                            }
                            7 => {
                                // IMU end
                            }
                            8 => {
                                // APS Global Shutter Frame Start
                                println!("\n{} frame start", self.time_ref.elapsed().as_micros());
                            }
                            9 => {
                                // APS Rolling Shutter Frame Start
                                println!("\n{} frame start (rolling)", self.time_ref.elapsed().as_micros());
                            }
                            10 => {
                                // APS frame end
                                println!("\n{} frame end", self.time_ref.elapsed().as_micros());
                            }
                            11 => {
                                // APS Reset Column Start
                                println!("\n{} reset column start", self.time_ref.elapsed().as_micros());
                            }
                            12 => {
                                // APS Signal Column Start
                                println!("\n{} signal column start", self.time_ref.elapsed().as_micros());
                            }
                            13 => {
                                // APS Column End
                                println!("\n{} column end", self.time_ref.elapsed().as_micros());
                            }
                            14 => {
                                // APS Exposure Start
                                println!("\n{} exposure start", self.time_ref.elapsed().as_micros());
                            }
                            15 => {
                                // APS Exposure End
                                println!("\n{} exposure end", self.time_ref.elapsed().as_micros());
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
                            self.column = Some(candidate_column);
                        } else {
                            self.column = None;
                        }
                    }
                    2 | 3 => {
                        if let Some(column) = self.column {
                            let row = word & 0x0FFF;
                            if row < 260 {
                                handle_dvs_event(neuromorphic_types::DvsEvent {
                                    t: self.t,
                                    x: column,
                                    y: row,
                                    polarity: if message_type == 2 {
                                        neuromorphic_types::DvsPolarity::Off
                                    } else {
                                        neuromorphic_types::DvsPolarity::On
                                    },
                                })
                            }
                        }
                    }
                    4 => {
                        // frame data
                        print!(" d");
                    }
                    5 => {
                        // misc 8 bit (IMU data, APS ROI)
                    }
                    6 => {
                        // misc 10  bit (frame exposure), related to auto exposure?
                    }
                    7 => {
                        self.t_offset += 0x8000;
                    }
                    _ => unreachable!(),
                }
            }
        }
    }
}
