// Copyright (c) Facebook, Inc. and its affiliates
// SPDX-License-Identifier: MIT OR Apache-2.0

use crate::{
    error::{Error, Result},
    format::*,
    trace::{Samples, Tracer},
    value::Value,
};
use serde::{ser, Serialize};

// Serialize a single value.
// The lifetime 'a is set by the serialization call site and the `&'a mut`
// references used to return tracing results and serialization samples.
pub(crate) struct Serializer<'a> {
    tracer: &'a mut Tracer,
    samples: &'a mut Samples,
}

impl<'a> Serializer<'a> {
    pub(crate) fn new(tracer: &'a mut Tracer, samples: &'a mut Samples) -> Self {
        Self { tracer, samples }
    }
}

impl<'a> ser::Serializer for Serializer<'a> {
    type Ok = (Format, Value);
    type Error = Error;
    type SerializeSeq = SeqSerializer<'a>;
    type SerializeTuple = TupleSerializer<'a>;
    type SerializeTupleStruct = TupleStructSerializer<'a>;
    type SerializeTupleVariant = TupleVariantSerializer<'a>;
    type SerializeMap = MapSerializer<'a>;
    type SerializeStruct = StructSerializer<'a>;
    type SerializeStructVariant = StructVariantSerializer<'a>;

    fn serialize_bool(self, content: bool) -> Result<(Format, Value)> {
        Ok((Format::Bool, Value::Bool(content)))
    }

    fn serialize_i8(self, content: i8) -> Result<(Format, Value)> {
        Ok((Format::I8, Value::I8(content)))
    }

    fn serialize_i16(self, content: i16) -> Result<(Format, Value)> {
        Ok((Format::I16, Value::I16(content)))
    }

    fn serialize_i32(self, content: i32) -> Result<(Format, Value)> {
        Ok((Format::I32, Value::I32(content)))
    }

    fn serialize_i64(self, content: i64) -> Result<(Format, Value)> {
        Ok((Format::I64, Value::I64(content)))
    }

    fn serialize_i128(self, content: i128) -> Result<(Format, Value)> {
        Ok((Format::I128, Value::I128(content)))
    }

    fn serialize_u8(self, content: u8) -> Result<(Format, Value)> {
        Ok((Format::U8, Value::U8(content)))
    }

    fn serialize_u16(self, content: u16) -> Result<(Format, Value)> {
        Ok((Format::U16, Value::U16(content)))
    }

    fn serialize_u32(self, content: u32) -> Result<(Format, Value)> {
        Ok((Format::U32, Value::U32(content)))
    }

    fn serialize_u64(self, content: u64) -> Result<(Format, Value)> {
        Ok((Format::U64, Value::U64(content)))
    }

    fn serialize_u128(self, content: u128) -> Result<(Format, Value)> {
        Ok((Format::U128, Value::U128(content)))
    }

    fn serialize_f32(self, content: f32) -> Result<(Format, Value)> {
        Ok((Format::F32, Value::F32(content)))
    }

    fn serialize_f64(self, content: f64) -> Result<(Format, Value)> {
        Ok((Format::F64, Value::F64(content)))
    }

    fn serialize_char(self, content: char) -> Result<(Format, Value)> {
        Ok((Format::Char, Value::Char(content)))
    }

    fn serialize_str(self, content: &str) -> Result<(Format, Value)> {
        Ok((Format::Str, Value::Str(content.into())))
    }

    fn serialize_bytes(self, content: &[u8]) -> Result<(Format, Value)> {
        Ok((Format::Bytes, Value::Bytes(content.into())))
    }

    fn serialize_none(self) -> Result<(Format, Value)> {
        Ok((Format::unknown(), Value::Option(None)))
    }

    fn serialize_some<T>(self, content: &T) -> Result<(Format, Value)>
    where
        T: ?Sized + Serialize,
    {
        let (format, value) = content.serialize(self)?;
        Ok((
            Format::Option(Box::new(format)),
            Value::Option(Some(Box::new(value))),
        ))
    }

    fn serialize_unit(self) -> Result<(Format, Value)> {
        Ok((Format::Unit, Value::Unit))
    }

    fn serialize_unit_struct(self, name: &'static str) -> Result<(Format, Value)> {
        self.tracer.record_container(
            self.samples,
            name,
            ContainerFormat::UnitStruct,
            Value::Unit,
            false,
        )
    }

    fn serialize_unit_variant(
        self,
        name: &'static str,
        variant_index: u32,
        variant_name: &'static str,
    ) -> Result<(Format, Value)> {
        self.tracer.record_variant(
            self.samples,
            name,
            variant_index,
            variant_name,
            VariantFormat::Unit,
            Value::Unit,
        )
    }

    fn serialize_newtype_struct<T>(self, name: &'static str, content: &T) -> Result<(Format, Value)>
    where
        T: ?Sized + Serialize,
    {
        let (format, value) = content.serialize(Serializer::new(self.tracer, self.samples))?;
        self.tracer.record_container(
            self.samples,
            name,
            ContainerFormat::NewTypeStruct(Box::new(format)),
            value,
            self.tracer.config.record_samples_for_newtype_structs,
        )
    }

    fn serialize_newtype_variant<T>(
        self,
        name: &'static str,
        variant_index: u32,
        variant_name: &'static str,
        content: &T,
    ) -> Result<(Format, Value)>
    where
        T: ?Sized + Serialize,
    {
        let (format, value) = content.serialize(Serializer::new(self.tracer, self.samples))?;
        self.tracer.record_variant(
            self.samples,
            name,
            variant_index,
            variant_name,
            VariantFormat::NewType(Box::new(format)),
            value,
        )
    }

    fn serialize_seq(self, _len: Option<usize>) -> Result<Self::SerializeSeq> {
        Ok(SeqSerializer {
            tracer: self.tracer,
            samples: self.samples,
            format: Format::unknown(),
            values: Vec::new(),
        })
    }

    fn serialize_tuple(self, _len: usize) -> Result<Self::SerializeTuple> {
        Ok(TupleSerializer {
            tracer: self.tracer,
            samples: self.samples,
            formats: Vec::new(),
            values: Vec::new(),
        })
    }

    fn serialize_tuple_struct(
        self,
        name: &'static str,
        _len: usize,
    ) -> Result<Self::SerializeTupleStruct> {
        Ok(TupleStructSerializer {
            tracer: self.tracer,
            samples: self.samples,
            name,
            formats: Vec::new(),
            values: Vec::new(),
        })
    }

    fn serialize_tuple_variant(
        self,
        name: &'static str,
        variant_index: u32,
        variant_name: &'static str,
        _len: usize,
    ) -> Result<Self::SerializeTupleVariant> {
        Ok(TupleVariantSerializer {
            tracer: self.tracer,
            samples: self.samples,
            name,
            variant_index,
            variant_name,
            formats: Vec::new(),
            values: Vec::new(),
        })
    }

    fn serialize_map(self, _len: Option<usize>) -> Result<Self::SerializeMap> {
        Ok(MapSerializer {
            tracer: self.tracer,
            samples: self.samples,
            key_format: Format::unknown(),
            value_format: Format::unknown(),
            values: Vec::new(),
        })
    }

    fn serialize_struct(self, name: &'static str, _len: usize) -> Result<Self::SerializeStruct> {
        Ok(StructSerializer {
            tracer: self.tracer,
            samples: self.samples,
            name,
            fields: Vec::new(),
            values: Vec::new(),
        })
    }

    fn serialize_struct_variant(
        self,
        name: &'static str,
        variant_index: u32,
        variant_name: &'static str,
        _len: usize,
    ) -> Result<Self::SerializeStructVariant> {
        Ok(StructVariantSerializer {
            tracer: self.tracer,
            samples: self.samples,
            name,
            variant_index,
            variant_name,
            fields: Vec::new(),
            values: Vec::new(),
        })
    }

    fn is_human_readable(&self) -> bool {
        self.tracer.config.is_human_readable
    }
}

pub struct SeqSerializer<'a> {
    tracer: &'a mut Tracer,
    samples: &'a mut Samples,

    format: Format,
    values: Vec<Value>,
}

impl ser::SerializeSeq for SeqSerializer<'_> {
    type Ok = (Format, Value);
    type Error = Error;

    fn serialize_element<T>(&mut self, content: &T) -> Result<()>
    where
        T: ?Sized + Serialize,
    {
        let (format, value) = content.serialize(Serializer::new(self.tracer, self.samples))?;
        self.format.unify(format)?;
        self.values.push(value);
        Ok(())
    }

    fn end(self) -> Result<(Format, Value)> {
        Ok((Format::Seq(Box::new(self.format)), Value::Seq(self.values)))
    }
}

pub struct TupleSerializer<'a> {
    tracer: &'a mut Tracer,
    samples: &'a mut Samples,

    formats: Vec<Format>,
    values: Vec<Value>,
}

impl ser::SerializeTuple for TupleSerializer<'_> {
    type Ok = (Format, Value);
    type Error = Error;

    fn serialize_element<T>(&mut self, content: &T) -> Result<()>
    where
        T: ?Sized + Serialize,
    {
        let (format, value) = content.serialize(Serializer::new(self.tracer, self.samples))?;
        self.formats.push(format);
        self.values.push(value);
        Ok(())
    }

    fn end(self) -> Result<(Format, Value)> {
        Ok((Format::Tuple(self.formats), Value::Seq(self.values)))
    }
}

pub struct TupleStructSerializer<'a> {
    tracer: &'a mut Tracer,
    samples: &'a mut Samples,

    name: &'static str,
    formats: Vec<Format>,
    values: Vec<Value>,
}

impl ser::SerializeTupleStruct for TupleStructSerializer<'_> {
    type Ok = (Format, Value);
    type Error = Error;

    fn serialize_field<T>(&mut self, content: &T) -> Result<()>
    where
        T: ?Sized + Serialize,
    {
        let (format, value) = content.serialize(Serializer::new(self.tracer, self.samples))?;
        self.formats.push(format);
        self.values.push(value);
        Ok(())
    }

    fn end(self) -> Result<(Format, Value)> {
        let format = ContainerFormat::TupleStruct(self.formats);
        let value = Value::Seq(self.values);
        self.tracer.record_container(
            self.samples,
            self.name,
            format,
            value,
            self.tracer.config.record_samples_for_tuple_structs,
        )
    }
}

pub struct TupleVariantSerializer<'a> {
    tracer: &'a mut Tracer,
    samples: &'a mut Samples,

    name: &'static str,
    variant_index: u32,
    variant_name: &'static str,
    formats: Vec<Format>,
    values: Vec<Value>,
}

impl ser::SerializeTupleVariant for TupleVariantSerializer<'_> {
    type Ok = (Format, Value);
    type Error = Error;

    fn serialize_field<T>(&mut self, content: &T) -> Result<()>
    where
        T: ?Sized + Serialize,
    {
        let (format, value) = content.serialize(Serializer::new(self.tracer, self.samples))?;
        self.formats.push(format);
        self.values.push(value);
        Ok(())
    }

    fn end(self) -> Result<(Format, Value)> {
        let variant = VariantFormat::Tuple(self.formats);
        let value = Value::Seq(self.values);
        self.tracer.record_variant(
            self.samples,
            self.name,
            self.variant_index,
            self.variant_name,
            variant,
            value,
        )
    }
}

pub struct MapSerializer<'a> {
    tracer: &'a mut Tracer,
    samples: &'a mut Samples,

    key_format: Format,
    value_format: Format,
    values: Vec<Value>,
}

impl ser::SerializeMap for MapSerializer<'_> {
    type Ok = (Format, Value);
    type Error = Error;

    fn serialize_key<T>(&mut self, key: &T) -> Result<()>
    where
        T: ?Sized + Serialize,
    {
        let (format, value) = key.serialize(Serializer::new(self.tracer, self.samples))?;
        self.key_format.unify(format)?;
        self.values.push(value);
        Ok(())
    }

    fn serialize_value<T>(&mut self, content: &T) -> Result<()>
    where
        T: ?Sized + Serialize,
    {
        let (format, value) = content.serialize(Serializer::new(self.tracer, self.samples))?;
        self.value_format.unify(format)?;
        self.values.push(value);
        Ok(())
    }

    fn end(self) -> Result<(Format, Value)> {
        let format = Format::Map {
            key: Box::new(self.key_format),
            value: Box::new(self.value_format),
        };
        let value = Value::Seq(self.values);
        Ok((format, value))
    }
}

pub struct StructSerializer<'a> {
    tracer: &'a mut Tracer,
    samples: &'a mut Samples,

    name: &'static str,
    fields: Vec<Named<Format>>,
    values: Vec<Value>,
}

impl ser::SerializeStruct for StructSerializer<'_> {
    type Ok = (Format, Value);
    type Error = Error;

    fn serialize_field<T>(&mut self, name: &'static str, content: &T) -> Result<()>
    where
        T: ?Sized + Serialize,
    {
        let (format, value) = content.serialize(Serializer::new(self.tracer, self.samples))?;
        self.fields.push(Named {
            name: name.into(),
            value: format,
        });
        self.values.push(value);
        Ok(())
    }

    fn end(self) -> Result<(Format, Value)> {
        let format = ContainerFormat::Struct(self.fields);
        let value = Value::Seq(self.values);
        self.tracer.record_container(
            self.samples,
            self.name,
            format,
            value,
            self.tracer.config.record_samples_for_structs,
        )
    }
}

pub struct StructVariantSerializer<'a> {
    tracer: &'a mut Tracer,
    samples: &'a mut Samples,
    name: &'static str,
    variant_index: u32,
    variant_name: &'static str,
    fields: Vec<Named<Format>>,
    values: Vec<Value>,
}

impl ser::SerializeStructVariant for StructVariantSerializer<'_> {
    type Ok = (Format, Value);
    type Error = Error;

    fn serialize_field<T>(&mut self, name: &'static str, content: &T) -> Result<()>
    where
        T: ?Sized + Serialize,
    {
        let (format, value) = content.serialize(Serializer::new(self.tracer, self.samples))?;
        self.fields.push(Named {
            name: name.into(),
            value: format,
        });
        self.values.push(value);
        Ok(())
    }

    fn end(self) -> Result<(Format, Value)> {
        let variant = VariantFormat::Struct(self.fields);
        let value = Value::Seq(self.values);
        self.tracer.record_variant(
            self.samples,
            self.name,
            self.variant_index,
            self.variant_name,
            variant,
            value,
        )
    }
}
