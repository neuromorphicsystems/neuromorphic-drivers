use pyo3::prelude::*;

extern crate neuromorphic_drivers as neuromorphic_drivers_rs;

mod adapters;
mod bytes;
mod structured_array;

type ListedDevice = (String, String, Option<String>, Option<String>);

#[pyo3::pyfunction]
fn list_devices() -> pyo3::PyResult<Vec<ListedDevice>> {
    Ok(neuromorphic_drivers_rs::list_devices()
        .map_err(|error| pyo3::exceptions::PyRuntimeError::new_err(format!("{error}")))?
        .into_iter()
        .map(|listed_device| {
            let (serial, error) = match listed_device.serial {
                Ok(serial) => (Some(serial), None),
                Err(error) => (None, Some(format!("{error}"))),
            };
            (
                listed_device.device_type.name().to_owned(),
                listed_device.speed.to_string(),
                serial,
                error,
            )
        })
        .collect())
}

#[pyo3::pyclass]
struct Davis346Orientation {
    #[pyo3(get)]
    dvs_invert_xy: bool,
    #[pyo3(get)]
    aps_flip_x: bool,
    #[pyo3(get)]
    aps_flip_y: bool,
    #[pyo3(get)]
    aps_invert_xy: bool,
    #[pyo3(get)]
    imu_flip_x: bool,
    #[pyo3(get)]
    imu_flip_y: bool,
    #[pyo3(get)]
    imu_flip_z: bool,
}

#[pymethods]
impl Davis346Orientation {
    fn __repr__(&self) -> String {
        format!(
            "neuromorphic_drivers.Davis346Orientation(dvs_invert_xy={}, aps_flip_x={}, aps_flip_y={}, aps_invert_xy={}, imu_flip_x={}, imu_flip_y={}, imu_flip_z={})",
            if self.dvs_invert_xy { "True" } else { "False" },
            if self.aps_flip_x { "True" } else { "False" },
            if self.aps_flip_y { "True" } else { "False" },
            if self.aps_invert_xy { "True" } else { "False" },
            if self.imu_flip_x { "True" } else { "False" },
            if self.imu_flip_y { "True" } else { "False" },
            if self.imu_flip_z { "True" } else { "False" },
        )
    }
}

#[pyo3::pyclass]
struct DVXplorerOrientation {
    #[pyo3(get)]
    dvs_flip_x: bool,
    #[pyo3(get)]
    dvs_flip_y: bool,
    #[pyo3(get)]
    dvs_invert_xy: bool,
    #[pyo3(get)]
    imu_flip_x: bool,
    #[pyo3(get)]
    imu_flip_y: bool,
    #[pyo3(get)]
    imu_flip_z: bool,
}

#[pymethods]
impl DVXplorerOrientation {
    fn __repr__(&self) -> String {
        format!(
            "neuromorphic_drivers.DVXplorerOrientation(dvs_flip_x={}, dvs_flip_y={}, dvs_invert_xy={}, imu_flip_x={}, imu_flip_y={}, imu_flip_z={})",
            if self.dvs_flip_x { "True" } else { "False" },
            if self.dvs_flip_y { "True" } else { "False" },
            if self.dvs_invert_xy { "True" } else { "False" },
            if self.imu_flip_x { "True" } else { "False" },
            if self.imu_flip_y { "True" } else { "False" },
            if self.imu_flip_z { "True" } else { "False" },
        )
    }
}

#[derive(IntoPyObject)]
enum Orientation {
    Davis346Orientation(Davis346Orientation),
    DVXplorerOrientation(DVXplorerOrientation),
}

#[pyo3::pyclass(subclass)]
struct Device {
    device: Option<neuromorphic_drivers_rs::Device>,
    adapter: Option<std::sync::Mutex<adapters::Adapter>>,
    iterator_timeout: Option<std::time::Duration>,
    iterator_maximum_raw_packets: usize,
    flag: neuromorphic_drivers_rs::Flag<
        neuromorphic_drivers_rs::Error,
        neuromorphic_drivers_rs::UsbOverflow,
    >,
}

// unsafe workaround until auto traits are stabilized
// see https://docs.rs/pyo3/0.19.0/pyo3/marker/index.html
struct DeviceReference<'a>(pub &'a neuromorphic_drivers_rs::Device);
unsafe impl Send for DeviceReference<'_> {}
unsafe impl Sync for DeviceReference<'_> {}
enum Buffer<'a> {
    Adapter(std::sync::MutexGuard<'a, adapters::Adapter>),
    Bytes(bytes::Bytes),
}
unsafe impl Send for Buffer<'_> {}

struct Status {
    instant: std::time::Instant,
    backlog: usize,
    raw_packets: usize,
    clutch_engaged: bool,
    overflow_indices: Vec<usize>,
    current_t: Option<u64>,
}

#[pyo3::pymethods]
impl Device {
    #[new]
    fn new(
        raw: bool,
        iterator_maximum_raw_packets: usize,
        device_type: Option<String>,
        configuration: Option<Vec<u8>>,
        serial: Option<String>,
        usb_configuration: Option<Vec<u8>>,
        iterator_timeout: Option<f64>,
    ) -> pyo3::PyResult<Self> {
        let (flag, event_loop) = neuromorphic_drivers_rs::flag_and_event_loop()
            .map_err(|error| pyo3::exceptions::PyRuntimeError::new_err(format!("{error}")))?;
        let device = neuromorphic_drivers_rs::open(
            match serial.as_ref() {
                Some(serial) => {
                    neuromorphic_drivers_rs::SerialOrBusNumberAndAddress::Serial(serial.as_str())
                }
                None => neuromorphic_drivers_rs::SerialOrBusNumberAndAddress::None,
            },
            if let Some(device_type) = device_type {
                if let Some(configuration) = configuration {
                    Some(
                        neuromorphic_drivers_rs::Configuration::deserialize_bincode(
                            device_type.parse().map_err(|error| {
                                pyo3::exceptions::PyRuntimeError::new_err(format!("{error}"))
                            })?,
                            &configuration,
                        )
                        .map_err(|error| {
                            pyo3::exceptions::PyRuntimeError::new_err(format!("{error}"))
                        })?,
                    )
                } else {
                    None
                }
            } else {
                None
            },
            if let Some(usb_configuration) = usb_configuration {
                Some(
                    neuromorphic_drivers_rs::UsbConfiguration::deserialize_bincode(
                        &usb_configuration,
                    )
                    .map_err(|error| {
                        pyo3::exceptions::PyRuntimeError::new_err(format!("{error}"))
                    })?,
                )
            } else {
                None
            },
            event_loop,
            flag.clone(),
        )
        .map_err(|error| pyo3::exceptions::PyRuntimeError::new_err(format!("{error}")))?;
        let adapter = if raw {
            None
        } else {
            Some(std::sync::Mutex::new(device.create_adapter().into()))
        };
        Ok(Self {
            device: Some(device),
            adapter,
            iterator_timeout: match iterator_timeout {
                Some(seconds) => {
                    if seconds < 0.0 {
                        return Err(pyo3::exceptions::PyValueError::new_err(
                            "iterator_timeout must larger than or equal to 0",
                        ));
                    } else {
                        Some(std::time::Duration::from_secs_f64(seconds))
                    }
                }
                None => None,
            },
            iterator_maximum_raw_packets,
            flag,
        })
    }

    fn __enter__(slf: pyo3::Py<Self>) -> pyo3::Py<Self> {
        slf
    }

    fn __exit__(
        &mut self,
        _exception_type: Option<&pyo3::Bound<'_, pyo3::types::PyType>>,
        _value: Option<&pyo3::Bound<'_, pyo3::types::PyAny>>,
        _traceback: Option<&pyo3::Bound<'_, pyo3::types::PyAny>>,
    ) {
        self.device = None;
    }

    fn __iter__(slf: pyo3::Py<Self>) -> pyo3::Py<Self> {
        slf
    }

    fn __next__(
        slf: pyo3::PyRef<Self>,
        python: pyo3::Python,
    ) -> pyo3::PyResult<Option<pyo3::PyObject>> {
        let start = std::time::Instant::now();
        let flag = slf.flag.clone();
        let iterator_timeout = slf.iterator_timeout;
        let iterator_maximum_raw_packets = slf.iterator_maximum_raw_packets;
        let device = DeviceReference(slf.device.as_ref().ok_or(
            pyo3::exceptions::PyRuntimeError::new_err("__next__ called after __exit__"),
        )?);
        let mut buffer = match slf.adapter.as_ref() {
            Some(adapter) => Some(adapter.try_lock().map_err(|_| {
                pyo3::exceptions::PyRuntimeError::new_err(
                    "__next__ called while device is used by a different thread",
                )
            })?),
            None => None,
        }
        .map_or_else(|| Buffer::Bytes(bytes::Bytes::new()), Buffer::Adapter);
        python.allow_threads(|| -> pyo3::PyResult<Option<pyo3::PyObject>> {
            let mut status: Option<Status> = None;
            let mut available_raw_packets = None;
            let buffer_timeout = iterator_timeout.unwrap_or(std::time::Duration::from_millis(100));
            loop {
                flag.load_error().map_err(|error| {
                    pyo3::exceptions::PyRuntimeError::new_err(format!("{error:?}"))
                })?;
                if let Some(buffer_view) = device.0.next_with_timeout(&buffer_timeout) {
                    if let Some(status) = status.as_mut() {
                        status.raw_packets += 1;
                        status.backlog = buffer_view.backlog();
                        status.clutch_engaged = matches!(
                            buffer_view.clutch,
                            neuromorphic_drivers_rs::usb::Clutch::Engaged
                        );
                    } else {
                        status = Some(Status {
                            instant: buffer_view.instant,
                            backlog: buffer_view.backlog(),
                            raw_packets: 1,
                            clutch_engaged: matches!(
                                buffer_view.clutch,
                                neuromorphic_drivers_rs::usb::Clutch::Engaged
                            ),
                            overflow_indices: Vec::new(),
                            current_t: None,
                        });
                    }
                    let _ = available_raw_packets.get_or_insert_with(|| {
                        let available_now = buffer_view.backlog() + 1;
                        if iterator_maximum_raw_packets == 0 {
                            available_now
                        } else {
                            iterator_maximum_raw_packets.min(available_now)
                        }
                    });
                    match &mut buffer {
                        Buffer::Adapter(adapter) => {
                            adapter.push(buffer_view.first_after_overflow, buffer_view.slice);
                            if let Some(status) = status.as_mut() {
                                status.current_t = Some(adapter.current_t());
                            }
                        }
                        Buffer::Bytes(bytes) => pyo3::Python::with_gil(|python| {
                            if buffer_view.first_after_overflow {
                                status
                                    .as_mut()
                                    .expect("status is always Some here")
                                    .overflow_indices
                                    .push(bytes.length());
                            }
                            bytes.extend_from_slice(python, buffer_view.slice);
                        }),
                    }
                }
                if iterator_timeout.is_some_and(|timeout| start.elapsed() >= timeout)
                    || available_raw_packets.is_some_and(|available_raw_packets| {
                        status
                            .as_ref()
                            .is_some_and(|status| status.raw_packets >= available_raw_packets)
                    })
                {
                    return pyo3::Python::with_gil(|python| {
                        let packet = match &mut buffer {
                            Buffer::Adapter(adapter) => Some(adapter.take_into_packet(python)?),
                            Buffer::Bytes(bytes) => pyo3::Python::with_gil(|python| {
                                bytes.take(python).map(|bytes| bytes.into())
                            }),
                        };
                        let duration_since_epoch = std::time::SystemTime::now()
                            .duration_since(std::time::SystemTime::UNIX_EPOCH)
                            .unwrap_or(std::time::Duration::new(0, 0));
                        Ok((
                            duration_since_epoch.as_secs_f64(),
                            status.map(|status| {
                                (
                                    (duration_since_epoch
                                        .checked_sub(status.instant.elapsed())
                                        .unwrap_or(std::time::Duration::new(0, 0)))
                                    .as_secs_f64(),
                                    status.backlog,
                                    status.raw_packets,
                                    status.clutch_engaged,
                                    status.overflow_indices,
                                    status.current_t,
                                )
                            }),
                            packet,
                        )
                            .into_pyobject(python)?
                            .unbind()
                            .into_any())
                    })
                    .map(Some);
                }
            }
        })
    }

    fn backlog(slf: pyo3::PyRef<Self>) -> pyo3::PyResult<usize> {
        Ok(slf
            .device
            .as_ref()
            .ok_or(pyo3::exceptions::PyRuntimeError::new_err(
                "backlog called after __exit__",
            ))?
            .backlog())
    }

    fn clear_backlog(
        slf: pyo3::PyRef<Self>,
        python: pyo3::Python,
        until: usize,
    ) -> pyo3::PyResult<()> {
        let flag = slf.flag.clone();
        let device = DeviceReference(slf.device.as_ref().ok_or(
            pyo3::exceptions::PyRuntimeError::new_err("__next__ called after __exit__"),
        )?);
        let mut buffer = match slf.adapter.as_ref() {
            Some(adapter) => Some(adapter.try_lock().map_err(|_| {
                pyo3::exceptions::PyRuntimeError::new_err(
                    "__next__ called while device is used by a different thread",
                )
            })?),
            None => None,
        }
        .map_or_else(|| Buffer::Bytes(bytes::Bytes::new()), Buffer::Adapter);
        python.allow_threads(|| loop {
            flag.load_error()
                .map_err(|error| pyo3::exceptions::PyRuntimeError::new_err(format!("{error:?}")))?;
            if let Some(buffer_view) = device
                .0
                .next_with_timeout(&std::time::Duration::from_millis(0))
            {
                if buffer_view.backlog() < until {
                    return Ok(());
                }
                if let Buffer::Adapter(adapter) = &mut buffer {
                    adapter.consume(buffer_view.slice);
                }
            } else {
                return Ok(());
            }
        })
    }

    fn overflow(slf: pyo3::PyRef<Self>) -> bool {
        slf.flag.load_warning().is_some()
    }

    fn name(slf: pyo3::PyRef<Self>) -> pyo3::PyResult<String> {
        Ok(slf
            .device
            .as_ref()
            .ok_or(pyo3::exceptions::PyRuntimeError::new_err(
                "name called after __exit__",
            ))?
            .name()
            .to_owned())
    }

    fn serial(slf: pyo3::PyRef<Self>) -> pyo3::PyResult<String> {
        Ok(slf
            .device
            .as_ref()
            .ok_or(pyo3::exceptions::PyRuntimeError::new_err(
                "serial called after __exit__",
            ))?
            .serial())
    }

    fn chip_firmware_configuration(
        slf: pyo3::PyRef<Self>,
        python: pyo3::Python,
    ) -> pyo3::PyResult<pyo3::PyObject> {
        Ok(pyo3::types::PyBytes::new(
            python,
            &slf.device
                .as_ref()
                .ok_or(pyo3::exceptions::PyRuntimeError::new_err(
                    "serial called after __exit__",
                ))?
                .chip_firmware_configuration()
                .serialize_bincode()
                .map_err(|error| pyo3::exceptions::PyRuntimeError::new_err(format!("{error}")))?,
        )
        .into_pyobject(python)?
        .unbind()
        .into_any())
    }

    fn speed(slf: pyo3::PyRef<Self>) -> pyo3::PyResult<String> {
        Ok(slf
            .device
            .as_ref()
            .ok_or(pyo3::exceptions::PyRuntimeError::new_err(
                "speed called after __exit__",
            ))?
            .speed()
            .to_string())
    }

    fn temperature_celsius(slf: pyo3::PyRef<Self>) -> pyo3::PyResult<f32> {
        match slf
            .device
            .as_ref()
            .ok_or(pyo3::exceptions::PyRuntimeError::new_err(
                "temperature_celsius called after __exit__",
            ))? {
            neuromorphic_drivers_rs::Device::InivationDavis346(_) => Err(
                pyo3::exceptions::PyRuntimeError::new_err(format!("temperature_celsius is not implemented for the DAVIS346 (temperature samples are stored with IMU samples and can be accessed in the packet loop)")),
            ),
            neuromorphic_drivers_rs::Device::PropheseeEvk4(device) =>
                device
                .temperature_celsius()
                .map(|temperature| temperature.0)
                .map_err(|error| pyo3::exceptions::PyRuntimeError::new_err(format!("{error}"))),
            device => Err(pyo3::exceptions::PyRuntimeError::new_err(format!(
                "temperature_celsius is not implemented for the {}",
                device.name()
            ))),
        }
    }

    fn illuminance(slf: pyo3::PyRef<Self>) -> pyo3::PyResult<u32> {
        match slf
            .device
            .as_ref()
            .ok_or(pyo3::exceptions::PyRuntimeError::new_err(
                "illuminance called after __exit__",
            ))? {
            neuromorphic_drivers_rs::Device::PropheseeEvk4(device) => device
                .illuminance()
                .map_err(|error| pyo3::exceptions::PyRuntimeError::new_err(format!("{error}"))),
            device => Err(pyo3::exceptions::PyRuntimeError::new_err(format!(
                "illuminance is not implemented for the {}",
                device.name()
            ))),
        }
    }

    fn orientation(slf: pyo3::PyRef<Self>) -> pyo3::PyResult<Orientation> {
        match slf
            .device
            .as_ref()
            .ok_or(pyo3::exceptions::PyRuntimeError::new_err(
                "orientation called after __exit__",
            ))? {
            neuromorphic_drivers_rs::Device::InivationDavis346(device) => {
                let aps_orientation = device.aps_orientation();
                let imu_orientation = device.imu_orientation();
                Ok(Orientation::Davis346Orientation(Davis346Orientation {
                    dvs_invert_xy: device.dvs_invert_xy(),
                    aps_flip_x: aps_orientation.flip_x,
                    aps_flip_y: aps_orientation.flip_y,
                    aps_invert_xy: aps_orientation.invert_xy,
                    imu_flip_x: imu_orientation.flip_x,
                    imu_flip_y: imu_orientation.flip_y,
                    imu_flip_z: imu_orientation.flip_z,
                }))
            }
            neuromorphic_drivers_rs::Device::InivationDvxplorer(device) => {
                let dvs_orientation = device.dvs_orientation();
                let imu_orientation = device.imu_orientation();
                Ok(Orientation::DVXplorerOrientation(DVXplorerOrientation {
                    dvs_flip_x: dvs_orientation.flip_x,
                    dvs_flip_y: dvs_orientation.flip_y,
                    dvs_invert_xy: dvs_orientation.invert_xy,
                    imu_flip_x: imu_orientation.flip_x,
                    imu_flip_y: imu_orientation.flip_y,
                    imu_flip_z: imu_orientation.flip_z,
                }))
            }
            device => Err(pyo3::exceptions::PyRuntimeError::new_err(format!(
                "orientation is not implemented for the {}",
                device.name()
            ))),
        }
    }

    fn imu_type(slf: pyo3::PyRef<Self>) -> pyo3::PyResult<Option<String>> {
        match slf
            .device
            .as_ref()
            .ok_or(pyo3::exceptions::PyRuntimeError::new_err(
                "imu_type called after __exit__",
            ))? {
            neuromorphic_drivers_rs::Device::InivationDavis346(device) => match device.imu_type() {
                neuromorphic_drivers_rs::inivation_davis346::ImuType::None => Ok(None),
                neuromorphic_drivers_rs::inivation_davis346::ImuType::InvenSense6050Or6150 => {
                    Ok(Some("InvenSense6050Or6150".to_owned()))
                }
                neuromorphic_drivers_rs::inivation_davis346::ImuType::InvenSense9250 => {
                    Ok(Some("InvenSense9250".to_owned()))
                }
            },
            device => Err(pyo3::exceptions::PyRuntimeError::new_err(format!(
                "imu_type is not implemented for the {}",
                device.name()
            ))),
        }
    }

    fn update_configuration(
        slf: pyo3::PyRef<Self>,
        device_type: &str,
        configuration: &[u8],
    ) -> pyo3::PyResult<()> {
        let configuration = neuromorphic_drivers_rs::Configuration::deserialize_bincode(
            device_type
                .parse()
                .map_err(|error| pyo3::exceptions::PyRuntimeError::new_err(format!("{error}")))?,
            configuration,
        )
        .map_err(|error| pyo3::exceptions::PyRuntimeError::new_err(format!("{error}")))?;
        slf.device
            .as_ref()
            .ok_or(pyo3::exceptions::PyRuntimeError::new_err(
                "__next__ called after __exit__",
            ))?
            .update_configuration(configuration)
            .map_err(|_| {
                pyo3::exceptions::PyRuntimeError::new_err(
                    "update_configuration called while device is used by a different thread",
                )
            })
    }
}

#[pyo3::pymodule]
fn neuromorphic_drivers(module: &pyo3::Bound<'_, pyo3::types::PyModule>) -> pyo3::PyResult<()> {
    module.add_class::<Device>()?;
    module.add_function(pyo3::wrap_pyfunction!(list_devices, module)?)?;
    Ok(())
}
