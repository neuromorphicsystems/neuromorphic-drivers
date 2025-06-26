pub trait SliceView {
    fn as_bytes(&self) -> &[u8]
    where
        Self: Sized,
    {
        unsafe {
            std::slice::from_raw_parts(
                (self as *const Self) as *const u8,
                std::mem::size_of::<Self>(),
            )
        }
    }
}

#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum DvsPolarity {
    Off = 0,
    On = 1,
}
impl SliceView for DvsPolarity {}

#[repr(C, packed)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub struct DvsEvent<Timestamp, X, Y> {
    pub t: Timestamp,
    pub x: X,
    pub y: Y,
    pub polarity: DvsPolarity,
}
impl<Timestamp, X, Y> SliceView for DvsEvent<Timestamp, X, Y> {}

#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum AtisPolarity {
    Off = 0,
    On = 1,
    ExposureStart = 2,
    ExposureEnd = 3,
}
impl SliceView for AtisPolarity {}

#[repr(C, packed)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub struct AtisEvent<Timestamp, X, Y> {
    pub t: Timestamp,
    pub x: X,
    pub y: Y,
    pub polarity: AtisPolarity,
}
impl<Timestamp, X, Y> SliceView for AtisEvent<Timestamp, X, Y> {}

#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub enum TriggerPolarity {
    Falling = 0,
    Rising = 1,
}
impl SliceView for TriggerPolarity {}

#[repr(C, packed)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub struct TriggerEvent<Timestamp, Id> {
    pub t: Timestamp,
    pub id: Id,
    pub polarity: TriggerPolarity,
}
impl<Timestamp, Id> SliceView for TriggerEvent<Timestamp, Id> {}
