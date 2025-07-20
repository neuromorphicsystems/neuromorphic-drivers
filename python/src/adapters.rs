use pyo3::prelude::*;
use pyo3::types::PyListMethods;

use neuromorphic_drivers::types::SliceView;
use numpy::IntoPyArray;

use crate::structured_array;

pub struct InternalFrame {
    start_t: u64,
    exposure_start_t: Option<u64>,
    exposure_end_t: Option<u64>,
    t: u64,
    pixels: numpy::ndarray::Array2<u16>,
}

#[pyclass]
pub struct Frame {
    #[pyo3(get)]
    start_t: u64,
    #[pyo3(get)]
    exposure_start_t: Option<u64>,
    #[pyo3(get)]
    exposure_end_t: Option<u64>,
    #[pyo3(get)]
    t: u64,
    #[pyo3(get)]
    pixels: PyObject,
}

#[pyclass]
pub struct Davis346Packet {
    #[pyo3(get)]
    polarity_events: Option<PyObject>,
    #[pyo3(get)]
    polarity_events_overflow_indices: Option<PyObject>,
    #[pyo3(get)]
    frames: pyo3::Py<pyo3::types::PyList>,
}

#[pyclass]
pub struct Evt3Packet {
    #[pyo3(get)]
    polarity_events: Option<PyObject>,
    #[pyo3(get)]
    polarity_events_overflow_indices: Option<PyObject>,
    #[pyo3(get)]
    trigger_events: Option<PyObject>,
    #[pyo3(get)]
    trigger_events_overflow_indices: Option<PyObject>,
}

#[pymethods]
impl Frame {
    fn __repr__(&self) -> String {
        Python::with_gil(|python| -> String {
            format!(
                "neuromorphic_drivers.Frame(start_t={}, exposure_start_t={}, exposure_end_t={}, t={}, pixels={})",
                self.start_t,
                self.exposure_start_t.map_or("None".to_owned(), |value| format!("{value}")),
                self.exposure_end_t.map_or("None".to_owned(), |value| format!("{value}")),
                self.t,
                self.pixels.bind(python).repr().map_or_else(
                    |error| error.to_string(),
                    |representation| representation.to_string()
                ),
            )
        })
    }
}

pub enum Adapter {
    Davis346 {
        inner: neuromorphic_drivers_rs::adapters::davis346::Adapter,
        polarity_events: Vec<u8>,
        //imu_events: Vec<u8>,
        //trigger_events: Vec<u8>,
        frames: Vec<InternalFrame>,
        polarity_events_overflow_indices: Vec<usize>,
        //imu_events_overflow_indices: Vec<usize>,
        //trigger_events_overflow_indices: Vec<usize>,
    },
    Evt3 {
        inner: neuromorphic_drivers_rs::adapters::evt3::Adapter,
        polarity_events: Vec<u8>,
        trigger_events: Vec<u8>,
        polarity_events_overflow_indices: Vec<usize>,
        trigger_events_overflow_indices: Vec<usize>,
    },
}

impl Adapter {
    pub fn current_t(&self) -> u64 {
        match self {
            Adapter::Davis346 { inner, .. } => inner.state().t,
            Adapter::Evt3 { inner, .. } => inner.state().t,
        }
    }

    pub fn consume(&mut self, slice: &[u8]) {
        match self {
            Adapter::Davis346 { inner, .. } => inner.convert(slice, |_| {}, |_| {}, |_| {}, |_| {}),
            Adapter::Evt3 { inner, .. } => inner.convert(slice, |_| {}, |_| {}),
        }
    }

    pub fn push(&mut self, first_after_overflow: bool, slice: &[u8]) {
        match self {
            Adapter::Davis346 {
                inner,
                polarity_events,
                frames,
                polarity_events_overflow_indices,
            } => {
                if first_after_overflow {
                    polarity_events_overflow_indices
                        .push(polarity_events.len() / structured_array::DVS_EVENTS_DTYPE.size());
                }
                let events_lengths = inner.events_lengths(slice);
                polarity_events.reserve_exact(events_lengths.on + events_lengths.off);
                inner.convert(
                    slice,
                    |polarity_event| {
                        polarity_events.extend_from_slice(polarity_event.as_bytes());
                    },
                    |imu_event| {},
                    |trigger_event| {},
                    |frame_event| {
                        let mut array = numpy::ndarray::Array2::<u16>::zeros((260, 346));
                        array
                            .as_slice_mut()
                            .expect("the array is contiguous")
                            .copy_from_slice(frame_event.pixels);
                        frames.push(InternalFrame {
                            start_t: frame_event.start_t,
                            exposure_start_t: frame_event.exposure_start_t,
                            exposure_end_t: frame_event.exposure_end_t,
                            t: frame_event.t,
                            pixels: array,
                        });
                    },
                );
            }
            Adapter::Evt3 {
                inner,
                polarity_events,
                trigger_events,
                polarity_events_overflow_indices,
                trigger_events_overflow_indices,
            } => {
                if first_after_overflow {
                    polarity_events_overflow_indices
                        .push(polarity_events.len() / structured_array::DVS_EVENTS_DTYPE.size());
                    trigger_events_overflow_indices.push(
                        polarity_events.len() / structured_array::TRIGGER_EVENTS_DTYPE.size(),
                    );
                }
                let events_lengths = inner.events_lengths(slice);
                polarity_events.reserve_exact(events_lengths.on + events_lengths.off);
                trigger_events
                    .reserve_exact(events_lengths.trigger_rising + events_lengths.trigger_falling);
                inner.convert(
                    slice,
                    |polarity_event| {
                        polarity_events.extend_from_slice(polarity_event.as_bytes());
                    },
                    |trigger_event| {
                        trigger_events.extend_from_slice(trigger_event.as_bytes());
                    },
                );
            }
        }
    }

    pub fn take_into_packet(&mut self, python: pyo3::Python) -> pyo3::PyResult<pyo3::PyObject> {
        match self {
            Adapter::Davis346 {
                inner: _,
                polarity_events,
                frames,
                polarity_events_overflow_indices,
            } => {
                let mut packet_polarity_events = None;
                let mut packet_polarity_events_overflow_indices = None;
                let packet_frames = pyo3::types::PyList::empty(python);
                if !polarity_events.is_empty() {
                    let polarity_events_array = {
                        let mut taken_polarity_events = Vec::new();
                        std::mem::swap(polarity_events, &mut taken_polarity_events);
                        taken_polarity_events.into_pyarray(python)
                    };
                    let description =
                        structured_array::DVS_EVENTS_DTYPE.as_array_description(python);
                    use numpy::prelude::PyUntypedArrayMethods;
                    {
                        let polarity_events_array_pointer = polarity_events_array.as_array_ptr();
                        unsafe {
                            *(*polarity_events_array_pointer).dimensions /=
                                structured_array::DVS_EVENTS_DTYPE.size() as isize;
                            *(*polarity_events_array_pointer).strides =
                                structured_array::DVS_EVENTS_DTYPE.size() as isize;
                            let previous_description = (*polarity_events_array_pointer).descr;
                            (*polarity_events_array_pointer).descr = description;
                            pyo3::ffi::Py_DECREF(previous_description as *mut pyo3::ffi::PyObject);
                        }
                    }
                    packet_polarity_events = Some(polarity_events_array.unbind().into_any());
                    if !polarity_events_overflow_indices.is_empty() {
                        let polarity_events_overflow_indices_array = {
                            let mut taken_polarity_events_overflow_indices = Vec::new();
                            std::mem::swap(
                                polarity_events_overflow_indices,
                                &mut taken_polarity_events_overflow_indices,
                            );
                            taken_polarity_events_overflow_indices.into_pyarray(python)
                        };
                        packet_polarity_events_overflow_indices =
                            Some(polarity_events_overflow_indices_array.unbind().into_any());
                    }
                }
                for frame in frames.iter() {
                    packet_frames.append(Frame {
                        start_t: frame.start_t,
                        exposure_start_t: frame.exposure_start_t,
                        exposure_end_t: frame.exposure_end_t,
                        t: frame.t,
                        pixels: frame.pixels.clone().into_pyarray(python).into(),
                    })?;
                }
                frames.clear();
                Davis346Packet {
                    polarity_events: packet_polarity_events,
                    polarity_events_overflow_indices: packet_polarity_events_overflow_indices,
                    frames: packet_frames.unbind(),
                }
                .into_pyobject(python)
                .map(|object| object.unbind().into_any())
            }
            Adapter::Evt3 {
                inner: _,
                polarity_events,
                trigger_events,
                polarity_events_overflow_indices,
                trigger_events_overflow_indices,
            } => {
                let mut packet = Evt3Packet {
                    polarity_events: None,
                    polarity_events_overflow_indices: None,
                    trigger_events: None,
                    trigger_events_overflow_indices: None,
                };
                if !polarity_events.is_empty() {
                    let polarity_events_array = {
                        let mut taken_polarity_events = Vec::new();
                        std::mem::swap(polarity_events, &mut taken_polarity_events);
                        taken_polarity_events.into_pyarray(python)
                    };
                    let description =
                        structured_array::DVS_EVENTS_DTYPE.as_array_description(python);
                    use numpy::prelude::PyUntypedArrayMethods;
                    {
                        let polarity_events_array_pointer = polarity_events_array.as_array_ptr();
                        unsafe {
                            *(*polarity_events_array_pointer).dimensions /=
                                structured_array::DVS_EVENTS_DTYPE.size() as isize;
                            *(*polarity_events_array_pointer).strides =
                                structured_array::DVS_EVENTS_DTYPE.size() as isize;
                            let previous_description = (*polarity_events_array_pointer).descr;
                            (*polarity_events_array_pointer).descr = description;
                            pyo3::ffi::Py_DECREF(previous_description as *mut pyo3::ffi::PyObject);
                        }
                    }
                    packet.polarity_events = Some(polarity_events_array.unbind().into_any());
                    if !polarity_events_overflow_indices.is_empty() {
                        let polarity_events_overflow_indices_array = {
                            let mut taken_polarity_events_overflow_indices = Vec::new();
                            std::mem::swap(
                                polarity_events_overflow_indices,
                                &mut taken_polarity_events_overflow_indices,
                            );
                            taken_polarity_events_overflow_indices.into_pyarray(python)
                        };
                        packet.polarity_events_overflow_indices =
                            Some(polarity_events_overflow_indices_array.unbind().into_any());
                    }
                }
                if !trigger_events.is_empty() {
                    let trigger_events_array = {
                        let mut taken_trigger_events = Vec::new();
                        std::mem::swap(trigger_events, &mut taken_trigger_events);
                        taken_trigger_events.into_pyarray(python)
                    };
                    let description =
                        structured_array::TRIGGER_EVENTS_DTYPE.as_array_description(python);
                    use numpy::prelude::PyUntypedArrayMethods;
                    {
                        let trigger_events_array_pointer = trigger_events_array.as_array_ptr();
                        unsafe {
                            *(*trigger_events_array_pointer).dimensions /=
                                structured_array::TRIGGER_EVENTS_DTYPE.size() as isize;
                            *(*trigger_events_array_pointer).strides =
                                structured_array::TRIGGER_EVENTS_DTYPE.size() as isize;
                            let previous_description = (*trigger_events_array_pointer).descr;
                            (*trigger_events_array_pointer).descr = description;
                            pyo3::ffi::Py_DECREF(previous_description as *mut pyo3::ffi::PyObject);
                        }
                    }
                    packet.trigger_events = Some(trigger_events_array.unbind().into_any());
                    if !trigger_events_overflow_indices.is_empty() {
                        let trigger_events_overflow_indices_array = {
                            let mut taken_trigger_events_overflow_indices = Vec::new();
                            std::mem::swap(
                                trigger_events_overflow_indices,
                                &mut taken_trigger_events_overflow_indices,
                            );
                            taken_trigger_events_overflow_indices.into_pyarray(python)
                        };
                        packet.trigger_events_overflow_indices =
                            Some(trigger_events_overflow_indices_array.unbind().into_any());
                    }
                }
                packet
                    .into_pyobject(python)
                    .map(|object| object.unbind().into_any())
            }
        }
    }
}

impl From<neuromorphic_drivers::Adapter> for Adapter {
    fn from(adapter: neuromorphic_drivers::Adapter) -> Self {
        match adapter {
            neuromorphic_drivers_rs::Adapter::Davis346(inner) => Adapter::Davis346 {
                inner,
                polarity_events: Vec::new(),
                frames: Vec::new(),
                polarity_events_overflow_indices: Vec::new(),
            },
            neuromorphic_drivers::Adapter::Evt3(inner) => Adapter::Evt3 {
                inner,
                polarity_events: Vec::new(),
                trigger_events: Vec::new(),
                polarity_events_overflow_indices: Vec::new(),
                trigger_events_overflow_indices: Vec::new(),
            },
        }
    }
}
