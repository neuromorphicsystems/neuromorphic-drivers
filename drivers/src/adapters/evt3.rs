#[derive(Clone, Copy)]
pub struct State {
    pub t: u64,
    pub overflows: u32,
    pub previous_msb_t: u16,
    pub previous_lsb_t: u16,
    pub x: u16,
    pub y: u16,
    pub polarity: neuromorphic_types::DvsPolarity,
}

pub struct Adapter {
    width: u16,
    height: u16,
    state: State,
}

#[derive(Default)]
pub struct EventsLengths {
    pub on: usize,
    pub off: usize,
    pub trigger_rising: usize,
    pub trigger_falling: usize,
}

impl Adapter {
    pub fn from_dimensions(width: u16, height: u16) -> Self {
        Self {
            width,
            height,
            state: State {
                t: 0,
                overflows: 0,
                previous_msb_t: 0,
                previous_lsb_t: 0,
                x: 0,
                y: 0,
                polarity: neuromorphic_types::DvsPolarity::Off,
            },
        }
    }

    pub fn from_dimensions_and_state(width: u16, height: u16, state: State) -> Self {
        Self {
            width,
            height,
            state,
        }
    }

    pub fn state(&self) -> &State {
        &self.state
    }

    pub fn events_lengths(&self, slice: &[u8]) -> EventsLengths {
        let mut lengths = EventsLengths::default();
        let mut x = self.state.x;
        let mut y = self.state.y;
        let mut polarity = self.state.polarity;
        for index in 0..slice.len() / 2 {
            let word = u16::from_le_bytes([slice[index * 2], slice[index * 2 + 1]]);
            match word >> 12 {
                0b0000 => {
                    y = word & 0b11111111111;
                }
                0b0001 => (),
                0b0010 => {
                    x = word & 0b11111111111;
                    polarity = if (word & (1 << 11)) > 0 {
                        neuromorphic_types::DvsPolarity::On
                    } else {
                        neuromorphic_types::DvsPolarity::Off
                    };
                    if x < self.width && y < self.height {
                        match polarity {
                            neuromorphic_types::DvsPolarity::On => {
                                lengths.on += 1;
                            }
                            neuromorphic_types::DvsPolarity::Off => {
                                lengths.off += 1;
                            }
                        }
                    }
                }
                0b0011 => {
                    x = word & 0b11111111111;
                    polarity = if (word & (1 << 11)) > 0 {
                        neuromorphic_types::DvsPolarity::On
                    } else {
                        neuromorphic_types::DvsPolarity::Off
                    };
                }
                0b0100 => {
                    if x < self.width && y < self.height {
                        match polarity {
                            neuromorphic_types::DvsPolarity::On => {
                                lengths.on +=
                                    (word & ((1 << std::cmp::min(12, self.width - x)) - 1))
                                        .count_ones() as usize;
                            }
                            neuromorphic_types::DvsPolarity::Off => {
                                lengths.off +=
                                    (word & ((1 << std::cmp::min(12, self.width - x)) - 1))
                                        .count_ones() as usize;
                            }
                        }
                        x = x.overflowing_add(12).0;
                    }
                }
                0b0101 => {
                    if x < self.width && y < self.height {
                        match polarity {
                            neuromorphic_types::DvsPolarity::On => {
                                lengths.on += (word & ((1 << std::cmp::min(8, self.width - x)) - 1))
                                    .count_ones()
                                    as usize;
                            }
                            neuromorphic_types::DvsPolarity::Off => {
                                lengths.off +=
                                    (word & ((1 << std::cmp::min(8, self.width - x)) - 1))
                                        .count_ones() as usize;
                            }
                        }
                        x = x.overflowing_add(8).0;
                    }
                }
                0b1010 => {
                    if (word & 1) > 0 {
                        lengths.trigger_rising += 1;
                    } else {
                        lengths.trigger_falling += 1;
                    }
                }
                _ => (),
            }
        }
        lengths
    }

    pub fn events_lengths_until(
        &mut self,
        slice: &[u8],
        threshold_t: u64,
    ) -> (EventsLengths, usize) {
        let mut lengths = EventsLengths::default();
        let mut index = 0;
        while index < slice.len() / 2 {
            let word = u16::from_le_bytes([slice[index * 2], slice[index * 2 + 1]]);
            index += 2;
            match word >> 12 {
                0b0000 => {
                    self.state.y = word & 0b11111111111;
                }
                0b0001 => (),
                0b0010 => {
                    self.state.x = word & 0b11111111111;
                    self.state.polarity = if (word & (1 << 11)) > 0 {
                        neuromorphic_types::DvsPolarity::On
                    } else {
                        neuromorphic_types::DvsPolarity::Off
                    };
                    if self.state.x < self.width && self.state.y < self.height {
                        match self.state.polarity {
                            neuromorphic_types::DvsPolarity::On => {
                                lengths.on += 1;
                            }
                            neuromorphic_types::DvsPolarity::Off => {
                                lengths.off += 1;
                            }
                        }
                    }
                }
                0b0011 => {
                    self.state.x = word & 0b11111111111;
                    self.state.polarity = if (word & (1 << 11)) > 0 {
                        neuromorphic_types::DvsPolarity::On
                    } else {
                        neuromorphic_types::DvsPolarity::Off
                    };
                }
                0b0100 => {
                    if self.state.x < self.width && self.state.y < self.height {
                        match self.state.polarity {
                            neuromorphic_types::DvsPolarity::On => {
                                lengths.on += (word
                                    & ((1 << std::cmp::min(12, self.width - self.state.x)) - 1))
                                    .count_ones()
                                    as usize;
                            }
                            neuromorphic_types::DvsPolarity::Off => {
                                lengths.off += (word
                                    & ((1 << std::cmp::min(12, self.width - self.state.x)) - 1))
                                    .count_ones()
                                    as usize;
                            }
                        }
                        self.state.x = self.state.x.overflowing_add(12).0;
                    }
                }
                0b0101 => {
                    if self.state.x < self.width && self.state.y < self.height {
                        match self.state.polarity {
                            neuromorphic_types::DvsPolarity::On => {
                                lengths.on += (word
                                    & ((1 << std::cmp::min(8, self.width - self.state.x)) - 1))
                                    .count_ones()
                                    as usize;
                            }
                            neuromorphic_types::DvsPolarity::Off => {
                                lengths.off += (word
                                    & ((1 << std::cmp::min(8, self.width - self.state.x)) - 1))
                                    .count_ones()
                                    as usize;
                            }
                        }
                        self.state.x = self.state.x.overflowing_add(8).0;
                    }
                }
                0b0110 => {
                    let lsb_t = word & 0b111111111111;
                    if lsb_t != self.state.previous_lsb_t {
                        self.state.previous_lsb_t = lsb_t;
                        let t = (((self.state.previous_lsb_t as u32)
                            | ((self.state.previous_msb_t as u32) << 12))
                            as u64)
                            | ((self.state.overflows as u64) << 24);
                        if t >= self.state.t {
                            self.state.t = t;
                            if self.state.t >= threshold_t {
                                break;
                            }
                        }
                    }
                }
                0b0111 => (),
                0b1000 => {
                    let msb_t = word & 0b111111111111;
                    if msb_t != self.state.previous_msb_t {
                        if msb_t > self.state.previous_msb_t {
                            if (msb_t - self.state.previous_msb_t) < (1 << 11) {
                                self.state.previous_lsb_t = 0;
                                self.state.previous_msb_t = msb_t;
                            }
                        } else if (self.state.previous_msb_t - msb_t) > (1 << 11) {
                            self.state.overflows += 1;
                            self.state.previous_lsb_t = 0;
                            self.state.previous_msb_t = msb_t;
                        }
                        let t = (((self.state.previous_lsb_t as u32)
                            | ((self.state.previous_msb_t as u32) << 12))
                            as u64)
                            | ((self.state.overflows as u64) << 24);
                        if t >= self.state.t {
                            self.state.t = t;
                            if self.state.t >= threshold_t {
                                break;
                            }
                        }
                    }
                }
                0b1001 => (),
                0b1010 => {
                    if (word & 1) > 0 {
                        lengths.trigger_rising += 1;
                    } else {
                        lengths.trigger_falling += 1;
                    }
                }
                #[allow(clippy::manual_range_patterns)]
                0b1011 | 0b1100 | 0b1101 | 0b1110 | 0b1111 => (),
                _ => (),
            }
        }
        (lengths, index * 2)
    }

    pub fn convert<HandleDvsEvent, HandleTriggerEvent>(
        &mut self,
        slice: &[u8],
        mut handle_dvs_event: HandleDvsEvent,
        mut handle_trigger_event: HandleTriggerEvent,
    ) where
        HandleDvsEvent: FnMut(neuromorphic_types::DvsEvent<u64, u16, u16>),
        HandleTriggerEvent: FnMut(neuromorphic_types::TriggerEvent<u64, u8>),
    {
        for index in 0..slice.len() / 2 {
            let word = u16::from_le_bytes([slice[index * 2], slice[index * 2 + 1]]);
            match word >> 12 {
                0b0000 => {
                    self.state.y = word & 0b11111111111;
                }
                0b0001 => (),
                0b0010 => {
                    self.state.x = word & 0b11111111111;
                    self.state.polarity = if (word & (1 << 11)) > 0 {
                        neuromorphic_types::DvsPolarity::On
                    } else {
                        neuromorphic_types::DvsPolarity::Off
                    };
                    if self.state.x < self.width && self.state.y < self.height {
                        handle_dvs_event(neuromorphic_types::DvsEvent {
                            t: self.state.t,
                            x: self.state.x,
                            y: self.state.y,
                            polarity: self.state.polarity,
                        });
                    }
                }
                0b0011 => {
                    self.state.x = word & 0b11111111111;
                    self.state.polarity = if (word & (1 << 11)) > 0 {
                        neuromorphic_types::DvsPolarity::On
                    } else {
                        neuromorphic_types::DvsPolarity::Off
                    };
                }
                0b0100 => {
                    if self.state.x < self.width && self.state.y < self.height {
                        let set = word & ((1 << std::cmp::min(12, self.width - self.state.x)) - 1);
                        for bit in 0..12 {
                            if (set & (1 << bit)) > 0 {
                                handle_dvs_event(neuromorphic_types::DvsEvent {
                                    t: self.state.t,
                                    x: self.state.x + bit,
                                    y: self.state.y,
                                    polarity: self.state.polarity,
                                });
                            }
                        }
                        self.state.x = self.state.x.overflowing_add(12).0;
                    }
                }
                0b0101 => {
                    if self.state.x < self.width && self.state.y < self.height {
                        let set = word & ((1 << std::cmp::min(8, self.width - self.state.x)) - 1);
                        for bit in 0..8 {
                            if (set & (1 << bit)) > 0 {
                                handle_dvs_event(neuromorphic_types::DvsEvent {
                                    t: self.state.t,
                                    x: self.state.x + bit,
                                    y: self.state.y,
                                    polarity: self.state.polarity,
                                });
                            }
                        }
                        self.state.x = self.state.x.overflowing_add(8).0;
                    }
                }
                0b0110 => {
                    let lsb_t = word & 0b111111111111;
                    if lsb_t != self.state.previous_lsb_t {
                        self.state.previous_lsb_t = lsb_t;
                        let t = (((self.state.previous_lsb_t as u32)
                            | ((self.state.previous_msb_t as u32) << 12))
                            as u64)
                            | ((self.state.overflows as u64) << 24);
                        if t >= self.state.t {
                            self.state.t = t;
                        }
                    }
                }
                0b0111 => (),
                0b1000 => {
                    let msb_t = word & 0b111111111111;
                    if msb_t != self.state.previous_msb_t {
                        if msb_t > self.state.previous_msb_t {
                            if (msb_t - self.state.previous_msb_t) < (1 << 11) {
                                self.state.previous_lsb_t = 0;
                                self.state.previous_msb_t = msb_t;
                            }
                        } else if (self.state.previous_msb_t - msb_t) > (1 << 11) {
                            self.state.overflows += 1;
                            self.state.previous_lsb_t = 0;
                            self.state.previous_msb_t = msb_t;
                        }
                        let t = (((self.state.previous_lsb_t as u32)
                            | ((self.state.previous_msb_t as u32) << 12))
                            as u64)
                            | ((self.state.overflows as u64) << 24);
                        if t >= self.state.t {
                            self.state.t = t;
                        }
                    }
                }
                0b1001 => (),
                0b1010 => handle_trigger_event(neuromorphic_types::TriggerEvent {
                    t: self.state.t,
                    id: ((word >> 8) & 0b1111) as u8,
                    polarity: if (word & 1) > 0 {
                        neuromorphic_types::TriggerPolarity::Rising
                    } else {
                        neuromorphic_types::TriggerPolarity::Falling
                    },
                }),
                #[allow(clippy::manual_range_patterns)]
                0b1011 | 0b1100 | 0b1101 | 0b1110 | 0b1111 => (),
                _ => (),
            }
        }
    }
}
