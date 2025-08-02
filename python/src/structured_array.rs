use numpy::Element;

macro_rules! dtype_base {
    ($($type:ident),+) => {
        paste::paste! {
            #[derive(Debug)]
            pub enum DtypeBase {
                $(
                    #[allow(dead_code)]
                    [<$type:camel>],
                )+
            }

            impl DtypeBase {
                const fn size(&self) -> usize {
                    match self {
                        $(
                            Self::[<$type:camel>] => std::mem::size_of::<$type>(),
                        )+
                    }
                }

                pub fn get_type_num(&self, python: pyo3::Python) -> core::ffi::c_int {
                    use numpy::prelude::PyArrayDescrMethods;
                    match self {
                        $(
                            Self::[<$type:camel>] => $type::get_dtype(python).num(),
                        )+
                    }
                }
            }
        }
    }
}

dtype_base![bool, i8, i16, i32, i64, isize, u8, u16, u32, u64, usize, f32, f64];

#[derive(Debug)]
pub struct DtypeField {
    pub name: &'static str,
    pub base: DtypeBase,
}

impl DtypeField {
    const fn new(name: &'static str, base: DtypeBase) -> Self {
        Self { name, base }
    }

    const fn size(&self) -> usize {
        self.base.size()
    }
}

#[derive(Debug)]
pub struct Dtype<const N: usize>([DtypeField; N]);

impl<const N: usize> Dtype<N> {
    pub const fn size(&self) -> usize {
        let mut total = 0;
        let mut index = 0;
        while index < N {
            total += self.0[index].size();
            index += 1;
        }
        total
    }

    pub fn as_array_description(&self, python: pyo3::Python) -> *mut numpy::npyffi::PyArray_Descr {
        let dtype_description = unsafe { pyo3::ffi::PyList_New(N as pyo3::ffi::Py_ssize_t) };
        for (index, field) in self.0.iter().enumerate() {
            let tuple = unsafe { pyo3::ffi::PyTuple_New(2) };
            assert!(
                unsafe {
                    pyo3::ffi::PyTuple_SetItem(
                        tuple,
                        0 as pyo3::ffi::Py_ssize_t,
                        pyo3::ffi::PyUnicode_FromStringAndSize(
                            field.name.as_ptr() as *const core::ffi::c_char,
                            field.name.len() as pyo3::ffi::Py_ssize_t,
                        ),
                    )
                } == 0,
                "PyTuple_SetItem 0 failed"
            );
            assert!(
                unsafe {
                    pyo3::ffi::PyTuple_SetItem(
                        tuple,
                        1 as pyo3::ffi::Py_ssize_t,
                        numpy::PY_ARRAY_API
                            .PyArray_TypeObjectFromType(python, field.base.get_type_num(python)),
                    )
                } == 0,
                "PyTuple_SetItem 1 failed"
            );
            assert!(
                unsafe {
                    pyo3::ffi::PyList_SetItem(
                        dtype_description,
                        index as pyo3::ffi::Py_ssize_t,
                        tuple,
                    )
                } == 0,
                "PyList_SetItem failed"
            );
        }
        let mut dtype: *mut numpy::npyffi::PyArray_Descr = std::ptr::null_mut();
        assert!(
            unsafe {
                numpy::PY_ARRAY_API.PyArray_DescrConverter(python, dtype_description, &mut dtype)
            } != 0, // numpy uses 0 for error and 1 for success
            "PyArray_DescrConverter failed"
        );
        dtype
    }
}

pub const POLARITY_EVENTS_DTYPE: Dtype<4> = Dtype([
    DtypeField::new("t", DtypeBase::U64),
    DtypeField::new("x", DtypeBase::U16),
    DtypeField::new("y", DtypeBase::U16),
    DtypeField::new("on", DtypeBase::Bool),
]);

pub const EVT3_TRIGGER_EVENTS_DTYPE: Dtype<3> = Dtype([
    DtypeField::new("t", DtypeBase::U64),
    DtypeField::new("id", DtypeBase::U8),
    DtypeField::new("rising", DtypeBase::Bool),
]);

pub const DAVIS346_IMU_EVENTS_DTYPE: Dtype<8> = Dtype([
    DtypeField::new("t", DtypeBase::U64),
    DtypeField::new("accelerometer_x", DtypeBase::F32),
    DtypeField::new("accelerometer_y", DtypeBase::F32),
    DtypeField::new("accelerometer_z", DtypeBase::F32),
    DtypeField::new("gyroscope_x", DtypeBase::F32),
    DtypeField::new("gyroscope_y", DtypeBase::F32),
    DtypeField::new("gyroscope_z", DtypeBase::F32),
    DtypeField::new("temperature", DtypeBase::F32),
]);

pub const DAVIS346_TRIGGER_EVENTS_DTYPE: Dtype<3> = Dtype([
    DtypeField::new("t", DtypeBase::U64),
    DtypeField::new("id", DtypeBase::U8),
    DtypeField::new("polarity", DtypeBase::U8),
]);
