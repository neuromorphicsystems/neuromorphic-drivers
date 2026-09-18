#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct State {
    pub t: u64,
    pub overflows: u32,
    pub previous_msb_t: Option<u16>,
    pub previous_lsb_t: u16,
    pub x: u16,
    pub y: u16,
    pub polarity: neuromorphic_types::Polarity,
}

impl State {
    fn t_from_words(&self) -> Option<u64> {
        self.previous_msb_t.map(|previous_msb_t| {
            (((self.previous_lsb_t as u32) | ((previous_msb_t as u32) << 12)) as u64)
                | ((self.overflows as u64) << 24)
        })
    }
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

const EVT_ADDR_Y: u16 = 0b0000;
const EVT_ADDR_X: u16 = 0b0010;
const VECT_BASE_X: u16 = 0b0011;
const VECT_12: u16 = 0b0100;
const VECT_8: u16 = 0b0101;
const EVT_TIME_LOW: u16 = 0b0110;
const CONTINUED_4: u16 = 0b0111;
const EVT_TIME_HIGH: u16 = 0b1000;
const EXT_TRIGGER: u16 = 0b1010;
const OTHERS: u16 = 0b1110;
const CONTINUED_12: u16 = 0b1111;

impl Adapter {
    pub fn from_dimensions(width: u16, height: u16) -> Self {
        Self {
            width,
            height,
            state: State {
                t: 0,
                overflows: 0,
                previous_msb_t: None,
                previous_lsb_t: 0,
                x: 0,
                y: 0,
                polarity: neuromorphic_types::Polarity::Off,
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

    pub fn width(&self) -> u16 {
        self.width
    }

    pub fn height(&self) -> u16 {
        self.height
    }

    pub fn state(&self) -> &State {
        &self.state
    }

    pub fn current_t(&self) -> u64 {
        self.state.t
    }

    pub fn events_lengths(&self, slice: &[u8]) -> EventsLengths {
        let mut lengths = EventsLengths::default();
        let mut x = self.state.x;
        let mut y = self.state.y;
        let mut polarity = self.state.polarity;
        let mut has_msb_t = self.state.previous_msb_t.is_some();
        for index in 0..slice.len() / 2 {
            let word = u16::from_le_bytes([slice[index * 2], slice[index * 2 + 1]]);
            match word >> 12 {
                EVT_ADDR_Y => {
                    y = word & 0b11111111111;
                }
                EVT_ADDR_X => {
                    x = word & 0b11111111111;
                    polarity = if (word & (1 << 11)) > 0 {
                        neuromorphic_types::Polarity::On
                    } else {
                        neuromorphic_types::Polarity::Off
                    };
                    if has_msb_t && x < self.width && y < self.height {
                        match polarity {
                            neuromorphic_types::Polarity::On => {
                                lengths.on += 1;
                            }
                            neuromorphic_types::Polarity::Off => {
                                lengths.off += 1;
                            }
                        }
                    }
                }
                VECT_BASE_X => {
                    x = word & 0b11111111111;
                    polarity = if (word & (1 << 11)) > 0 {
                        neuromorphic_types::Polarity::On
                    } else {
                        neuromorphic_types::Polarity::Off
                    };
                }
                VECT_12 => {
                    if x < self.width && y < self.height {
                        if has_msb_t {
                            match polarity {
                                neuromorphic_types::Polarity::On => {
                                    lengths.on += (word
                                        & ((1 << std::cmp::min(12, self.width - x)) - 1))
                                        .count_ones()
                                        as usize;
                                }
                                neuromorphic_types::Polarity::Off => {
                                    lengths.off += (word
                                        & ((1 << std::cmp::min(12, self.width - x)) - 1))
                                        .count_ones()
                                        as usize;
                                }
                            }
                        }
                        x = x.overflowing_add(12).0;
                    }
                }
                VECT_8 => {
                    if x < self.width && y < self.height {
                        if has_msb_t {
                            match polarity {
                                neuromorphic_types::Polarity::On => {
                                    lengths.on += (word
                                        & ((1 << std::cmp::min(8, self.width - x)) - 1))
                                        .count_ones()
                                        as usize;
                                }
                                neuromorphic_types::Polarity::Off => {
                                    lengths.off += (word
                                        & ((1 << std::cmp::min(8, self.width - x)) - 1))
                                        .count_ones()
                                        as usize;
                                }
                            }
                        }
                        x = x.overflowing_add(8).0;
                    }
                }
                EVT_TIME_LOW => (),
                EVT_TIME_HIGH => {
                    has_msb_t = true;
                }
                EXT_TRIGGER => {
                    if has_msb_t {
                        if (word & 1) > 0 {
                            lengths.trigger_rising += 1;
                        } else {
                            lengths.trigger_falling += 1;
                        }
                    }
                }
                OTHERS => (),
                CONTINUED_12 => (),
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
            index += 1;
            match word >> 12 {
                EVT_ADDR_Y => {
                    self.state.y = word & 0b11111111111;
                }
                EVT_ADDR_X => {
                    self.state.x = word & 0b11111111111;
                    self.state.polarity = if (word & (1 << 11)) > 0 {
                        neuromorphic_types::Polarity::On
                    } else {
                        neuromorphic_types::Polarity::Off
                    };
                    if self.state.previous_msb_t.is_some()
                        && self.state.x < self.width
                        && self.state.y < self.height
                    {
                        match self.state.polarity {
                            neuromorphic_types::Polarity::On => {
                                lengths.on += 1;
                            }
                            neuromorphic_types::Polarity::Off => {
                                lengths.off += 1;
                            }
                        }
                    }
                }
                VECT_BASE_X => {
                    self.state.x = word & 0b11111111111;
                    self.state.polarity = if (word & (1 << 11)) > 0 {
                        neuromorphic_types::Polarity::On
                    } else {
                        neuromorphic_types::Polarity::Off
                    };
                }
                VECT_12 => {
                    if self.state.x < self.width && self.state.y < self.height {
                        if self.state.previous_msb_t.is_some() {
                            match self.state.polarity {
                                neuromorphic_types::Polarity::On => {
                                    lengths.on += (word
                                        & ((1 << std::cmp::min(12, self.width - self.state.x)) - 1))
                                        .count_ones()
                                        as usize;
                                }
                                neuromorphic_types::Polarity::Off => {
                                    lengths.off += (word
                                        & ((1 << std::cmp::min(12, self.width - self.state.x)) - 1))
                                        .count_ones()
                                        as usize;
                                }
                            }
                        }
                        self.state.x = self.state.x.overflowing_add(12).0;
                    }
                }
                VECT_8 => {
                    if self.state.x < self.width && self.state.y < self.height {
                        if self.state.previous_msb_t.is_some() {
                            match self.state.polarity {
                                neuromorphic_types::Polarity::On => {
                                    lengths.on += (word
                                        & ((1 << std::cmp::min(8, self.width - self.state.x)) - 1))
                                        .count_ones()
                                        as usize;
                                }
                                neuromorphic_types::Polarity::Off => {
                                    lengths.off += (word
                                        & ((1 << std::cmp::min(8, self.width - self.state.x)) - 1))
                                        .count_ones()
                                        as usize;
                                }
                            }
                        }
                        self.state.x = self.state.x.overflowing_add(8).0;
                    }
                }
                EVT_TIME_LOW => {
                    let lsb_t = word & 0b111111111111;
                    if self.state.previous_msb_t.is_some() && self.state.previous_lsb_t != lsb_t {
                        self.state.previous_lsb_t = lsb_t;
                        if let Some(t) = self.state.t_from_words() {
                            if t >= self.state.t {
                                self.state.t = t;
                                if self.state.t >= threshold_t {
                                    break;
                                }
                            }
                        }
                    }
                }
                CONTINUED_4 => (),
                EVT_TIME_HIGH => {
                    let msb_t = word & 0b111111111111;
                    if self.state.previous_msb_t != Some(msb_t) {
                        match self.state.previous_msb_t {
                            None => {
                                self.state.previous_msb_t = Some(msb_t);
                                self.state.previous_lsb_t = 0;
                            }
                            Some(previous_msb_t) => {
                                if msb_t > previous_msb_t {
                                    if (msb_t - previous_msb_t) < (1 << 11) {
                                        self.state.previous_msb_t = Some(msb_t);
                                        self.state.previous_lsb_t = 0;
                                    }
                                } else if (previous_msb_t - msb_t) > (1 << 11) {
                                    self.state.overflows += 1;
                                    self.state.previous_msb_t = Some(msb_t);
                                    self.state.previous_lsb_t = 0;
                                }
                            }
                        }
                        if let Some(t) = self.state.t_from_words() {
                            if t >= self.state.t {
                                self.state.t = t;
                                if self.state.t >= threshold_t {
                                    break;
                                }
                            }
                        }
                    }
                }
                EXT_TRIGGER => {
                    if self.state.previous_msb_t.is_some() {
                        if (word & 1) > 0 {
                            lengths.trigger_rising += 1;
                        } else {
                            lengths.trigger_falling += 1;
                        }
                    }
                }
                OTHERS => (),
                CONTINUED_12 => (),
                _ => (),
            }
        }
        (lengths, index * 2)
    }

    pub fn convert<HandlePolarityEvent, HandleTriggerEvent>(
        &mut self,
        slice: &[u8],
        mut handle_polarity_event: HandlePolarityEvent,
        mut handle_trigger_event: HandleTriggerEvent,
    ) where
        HandlePolarityEvent: FnMut(neuromorphic_types::PolarityEvent<u64, u16, u16>),
        HandleTriggerEvent: FnMut(neuromorphic_types::TriggerEvent<u64, u8>),
    {
        for index in 0..slice.len() / 2 {
            let word = u16::from_le_bytes([slice[index * 2], slice[index * 2 + 1]]);
            match word >> 12 {
                EVT_ADDR_Y => {
                    self.state.y = word & 0b11111111111;
                }
                EVT_ADDR_X => {
                    self.state.x = word & 0b11111111111;
                    self.state.polarity = if (word & (1 << 11)) > 0 {
                        neuromorphic_types::Polarity::On
                    } else {
                        neuromorphic_types::Polarity::Off
                    };
                    if self.state.previous_msb_t.is_some()
                        && self.state.x < self.width
                        && self.state.y < self.height
                    {
                        handle_polarity_event(neuromorphic_types::PolarityEvent {
                            t: self.state.t,
                            x: self.state.x,
                            y: self.state.y,
                            polarity: self.state.polarity,
                        });
                    }
                }
                VECT_BASE_X => {
                    self.state.x = word & 0b11111111111;
                    self.state.polarity = if (word & (1 << 11)) > 0 {
                        neuromorphic_types::Polarity::On
                    } else {
                        neuromorphic_types::Polarity::Off
                    };
                }
                VECT_12 => {
                    if self.state.x < self.width && self.state.y < self.height {
                        if self.state.previous_msb_t.is_some() {
                            let set =
                                word & ((1 << std::cmp::min(12, self.width - self.state.x)) - 1);
                            for bit in 0..12 {
                                if (set & (1 << bit)) > 0 {
                                    handle_polarity_event(neuromorphic_types::PolarityEvent {
                                        t: self.state.t,
                                        x: self.state.x + bit,
                                        y: self.state.y,
                                        polarity: self.state.polarity,
                                    });
                                }
                            }
                        }
                        self.state.x = self.state.x.overflowing_add(12).0;
                    }
                }
                VECT_8 => {
                    if self.state.x < self.width && self.state.y < self.height {
                        if self.state.previous_msb_t.is_some() {
                            let set =
                                word & ((1 << std::cmp::min(8, self.width - self.state.x)) - 1);
                            for bit in 0..8 {
                                if (set & (1 << bit)) > 0 {
                                    handle_polarity_event(neuromorphic_types::PolarityEvent {
                                        t: self.state.t,
                                        x: self.state.x + bit,
                                        y: self.state.y,
                                        polarity: self.state.polarity,
                                    });
                                }
                            }
                        }
                        self.state.x = self.state.x.overflowing_add(8).0;
                    }
                }
                EVT_TIME_LOW => {
                    let lsb_t = word & 0b111111111111;
                    if self.state.previous_msb_t.is_some() && self.state.previous_lsb_t != lsb_t {
                        self.state.previous_lsb_t = lsb_t;
                        if let Some(t) = self.state.t_from_words() {
                            if t >= self.state.t {
                                self.state.t = t;
                            }
                        }
                    }
                }
                CONTINUED_4 => (),
                EVT_TIME_HIGH => {
                    let msb_t = word & 0b111111111111;
                    if self.state.previous_msb_t != Some(msb_t) {
                        match self.state.previous_msb_t {
                            None => {
                                self.state.previous_msb_t = Some(msb_t);
                                self.state.previous_lsb_t = 0;
                            }
                            Some(previous_msb_t) => {
                                if msb_t > previous_msb_t {
                                    if (msb_t - previous_msb_t) < (1 << 11) {
                                        self.state.previous_msb_t = Some(msb_t);
                                        self.state.previous_lsb_t = 0;
                                    }
                                } else if (previous_msb_t - msb_t) > (1 << 11) {
                                    self.state.overflows += 1;
                                    self.state.previous_msb_t = Some(msb_t);
                                    self.state.previous_lsb_t = 0;
                                }
                            }
                        }
                        if let Some(t) = self.state.t_from_words() {
                            if t >= self.state.t {
                                self.state.t = t;
                            }
                        }
                    }
                }
                EXT_TRIGGER => {
                    if self.state.previous_msb_t.is_some() {
                        handle_trigger_event(neuromorphic_types::TriggerEvent {
                            t: self.state.t,
                            id: ((word >> 8) & 0b1111) as u8,
                            polarity: if (word & 1) > 0 {
                                neuromorphic_types::TriggerPolarity::Rising
                            } else {
                                neuromorphic_types::TriggerPolarity::Falling
                            },
                        });
                    }
                }
                OTHERS => (),
                CONTINUED_12 => (),
                _ => (),
            }
        }
    }
}
