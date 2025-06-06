use pyo3::prelude::*;

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
        dvs_events: Vec<u8>,
        //imu_events: Vec<u8>,
        //trigger_events: Vec<u8>,
        frames: Vec<InternalFrame>,
        dvs_events_overflow_indices: Vec<usize>,
        //trigger_events_overflow_indices: Vec<usize>,
        frame_write_index: usize, // @DEV
    },
    Evt3 {
        inner: neuromorphic_drivers_rs::adapters::evt3::Adapter,
        dvs_events: Vec<u8>,
        trigger_events: Vec<u8>,
        dvs_events_overflow_indices: Vec<usize>,
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
            Adapter::Davis346 { inner, .. } => {} // @TODO
            Adapter::Evt3 { inner, .. } => inner.convert(slice, |_| {}, |_| {}),
        }
    }

    pub fn push(&mut self, first_after_overflow: bool, slice: &[u8]) {
        match self {
            Adapter::Davis346 {
                inner,
                dvs_events,
                frames,
                dvs_events_overflow_indices,
                ref mut frame_write_index, // @DEV
            } => {
                if first_after_overflow {
                    dvs_events_overflow_indices
                        .push(dvs_events.len() / structured_array::DVS_EVENTS_DTYPE.size());
                }
                let events_lengths = inner.events_lengths(slice);
                dvs_events.reserve_exact(events_lengths.on + events_lengths.off);
                inner.convert(
                    slice,
                    |dvs_event| {
                        dvs_events.extend_from_slice(dvs_event.as_bytes());
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
                dvs_events,
                trigger_events,
                dvs_events_overflow_indices,
                trigger_events_overflow_indices,
            } => {
                if first_after_overflow {
                    dvs_events_overflow_indices
                        .push(dvs_events.len() / structured_array::DVS_EVENTS_DTYPE.size());
                    trigger_events_overflow_indices
                        .push(dvs_events.len() / structured_array::TRIGGER_EVENTS_DTYPE.size());
                }
                let events_lengths = inner.events_lengths(slice);
                dvs_events.reserve_exact(events_lengths.on + events_lengths.off);
                trigger_events
                    .reserve_exact(events_lengths.trigger_rising + events_lengths.trigger_falling);
                inner.convert(
                    slice,
                    |dvs_event| {
                        dvs_events.extend_from_slice(dvs_event.as_bytes());
                    },
                    |trigger_event| {
                        trigger_events.extend_from_slice(trigger_event.as_bytes());
                    },
                );
            }
        }
    }

    pub fn take_into_dict(&mut self, python: pyo3::Python) -> pyo3::PyResult<pyo3::PyObject> {
        match self {
            Adapter::Davis346 {
                inner: _,
                dvs_events,
                frames,
                dvs_events_overflow_indices,
                frame_write_index, // @DEV
            } => {
                let dict = pyo3::types::PyDict::new(python);
                if !dvs_events.is_empty() {
                    let dvs_events_array = {
                        let mut taken_dvs_events = Vec::new();
                        std::mem::swap(dvs_events, &mut taken_dvs_events);
                        taken_dvs_events.into_pyarray(python)
                    };
                    let description =
                        structured_array::DVS_EVENTS_DTYPE.as_array_description(python);
                    use numpy::prelude::PyUntypedArrayMethods;
                    {
                        let dvs_events_array_pointer = dvs_events_array.as_array_ptr();
                        unsafe {
                            *(*dvs_events_array_pointer).dimensions /=
                                structured_array::DVS_EVENTS_DTYPE.size() as isize;
                            *(*dvs_events_array_pointer).strides =
                                structured_array::DVS_EVENTS_DTYPE.size() as isize;
                            let previous_description = (*dvs_events_array_pointer).descr;
                            (*dvs_events_array_pointer).descr = description;
                            pyo3::ffi::Py_DECREF(previous_description as *mut pyo3::ffi::PyObject);
                        }
                    }
                    dict.set_item("dvs_events", dvs_events_array)?;
                    if !dvs_events_overflow_indices.is_empty() {
                        let dvs_events_overflow_indices_array = {
                            let mut taken_dvs_events_overflow_indices = Vec::new();
                            std::mem::swap(
                                dvs_events_overflow_indices,
                                &mut taken_dvs_events_overflow_indices,
                            );
                            taken_dvs_events_overflow_indices.into_pyarray(python)
                        };
                        dict.set_item(
                            "dvs_events_overflow_indices",
                            dvs_events_overflow_indices_array,
                        )?;
                    }
                }
                if !frames.is_empty() {
                    let frames_array = {
                        let mut taken_frames = Vec::new();
                        std::mem::swap(frames, &mut taken_frames);
                        taken_frames
                            .into_iter()
                            .map(|frame| Frame {
                                start_t: frame.start_t,
                                exposure_start_t: frame.exposure_start_t,
                                exposure_end_t: frame.exposure_end_t,
                                t: frame.t,
                                pixels: frame.pixels.into_pyarray(python).into(),
                            })
                            .collect::<Vec<Frame>>()
                            .into_pyobject(python)?
                    };
                    dict.set_item("frames", frames_array)?;
                }
                Ok(dict.into())
            }
            Adapter::Evt3 {
                inner: _,
                dvs_events,
                trigger_events,
                dvs_events_overflow_indices,
                trigger_events_overflow_indices,
            } => {
                let dict = pyo3::types::PyDict::new(python);
                if !dvs_events.is_empty() {
                    let dvs_events_array = {
                        let mut taken_dvs_events = Vec::new();
                        std::mem::swap(dvs_events, &mut taken_dvs_events);
                        taken_dvs_events.into_pyarray(python)
                    };
                    let description =
                        structured_array::DVS_EVENTS_DTYPE.as_array_description(python);
                    use numpy::prelude::PyUntypedArrayMethods;
                    {
                        let dvs_events_array_pointer = dvs_events_array.as_array_ptr();
                        unsafe {
                            *(*dvs_events_array_pointer).dimensions /=
                                structured_array::DVS_EVENTS_DTYPE.size() as isize;
                            *(*dvs_events_array_pointer).strides =
                                structured_array::DVS_EVENTS_DTYPE.size() as isize;
                            let previous_description = (*dvs_events_array_pointer).descr;
                            (*dvs_events_array_pointer).descr = description;
                            pyo3::ffi::Py_DECREF(previous_description as *mut pyo3::ffi::PyObject);
                        }
                    }
                    dict.set_item("dvs_events", dvs_events_array)?;
                    if !dvs_events_overflow_indices.is_empty() {
                        let dvs_events_overflow_indices_array = {
                            let mut taken_dvs_events_overflow_indices = Vec::new();
                            std::mem::swap(
                                dvs_events_overflow_indices,
                                &mut taken_dvs_events_overflow_indices,
                            );
                            taken_dvs_events_overflow_indices.into_pyarray(python)
                        };
                        dict.set_item(
                            "dvs_events_overflow_indices",
                            dvs_events_overflow_indices_array,
                        )?;
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

                    dict.set_item("trigger_events", trigger_events_array)?;
                    if !trigger_events_overflow_indices.is_empty() {
                        let trigger_events_overflow_indices_array = {
                            let mut taken_trigger_events_overflow_indices = Vec::new();
                            std::mem::swap(
                                trigger_events_overflow_indices,
                                &mut taken_trigger_events_overflow_indices,
                            );
                            taken_trigger_events_overflow_indices.into_pyarray(python)
                        };
                        dict.set_item(
                            "trigger_events_overflow_indices",
                            trigger_events_overflow_indices_array,
                        )?;
                    }
                }
                Ok(dict.into())
            }
        }
    }
}

impl From<neuromorphic_drivers::Adapter> for Adapter {
    fn from(adapter: neuromorphic_drivers::Adapter) -> Self {
        match adapter {
            neuromorphic_drivers_rs::Adapter::Davis346(inner) => Adapter::Davis346 {
                inner,
                dvs_events: Vec::new(),
                frames: Vec::new(),
                dvs_events_overflow_indices: Vec::new(),
                frame_write_index: 0, // @DEV
            },
            neuromorphic_drivers::Adapter::Evt3(inner) => Adapter::Evt3 {
                inner,
                dvs_events: Vec::new(),
                trigger_events: Vec::new(),
                dvs_events_overflow_indices: Vec::new(),
                trigger_events_overflow_indices: Vec::new(),
            },
        }
    }
}
