pub struct Bytes(Option<Vec<u8>>);

impl Bytes {
    pub fn new() -> Self {
        Self(None)
    }

    pub fn length(&self) -> usize {
        self.0.as_ref().map_or(0, Vec::len)
    }

    pub fn extend_from_slice(&mut self, slice: &[u8]) {
        self.0.get_or_insert_default().extend_from_slice(slice);
    }

    pub fn take<'p>(&mut self, python: pyo3::Python<'p>) -> Option<pyo3::Bound<'p, pyo3::PyAny>> {
        self.0
            .take()
            .map(|bytes| pyo3::types::PyBytes::new(python, &bytes).into_any())
    }
}
