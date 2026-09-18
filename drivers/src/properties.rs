#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct Camera<Configuration> {
    pub name: &'static str,
    pub width: u16,
    pub height: u16,
    pub default_configuration: Configuration,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct Bounds<Value> {
    pub minimum: Value,
    pub maximum: Value,
}

impl<Value> Bounds<Value> {
    pub const fn new(minimum: Value, maximum: Value) -> Self {
        Self { minimum, maximum }
    }
}

impl Bounds<i16> {
    pub const fn offsets_from(chip_firmware_value: u8) -> Self {
        Self::new(
            -(chip_firmware_value as i16),
            u8::MAX as i16 - chip_firmware_value as i16,
        )
    }
}
