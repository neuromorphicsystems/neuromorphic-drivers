// Copyright (c) Facebook, Inc. and its affiliates
// SPDX-License-Identifier: MIT OR Apache-2.0

use crate::{
    error::{Error, Result},
    format::{ContainerFormat, ContainerFormatEntry, Format, FormatHolder, Named, VariantFormat},
    trace::{Samples, Tracer},
    value::IntoSeqDeserializer,
};
use serde::de::{self, DeserializeSeed, IntoDeserializer, Visitor};
use std::collections::BTreeMap;

// Deserialize a single value.
// * The lifetime 'a is set by the deserialization call site and the
//   `&'a mut` references used to return tracing results.
// * The lifetime 'de is fixed and the `&'de` reference meant to let us
//   borrow values from previous serialization runs.
pub(crate) struct Deserializer<'de, 'a> {
    tracer: &'a mut Tracer,
    samples: &'de Samples,
    format: &'a mut Format,
}

impl<'de, 'a> Deserializer<'de, 'a> {
    pub(crate) fn new(
        tracer: &'a mut Tracer,
        samples: &'de Samples,
        format: &'a mut Format,
    ) -> Self {
        Deserializer {
            tracer,
            samples,
            format,
        }
    }
}

impl<'de> de::Deserializer<'de> for Deserializer<'de, '_> {
    type Error = Error;

    fn deserialize_any<V>(self, _visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        Err(Error::NotSupported("deserialize_any"))
    }

    fn deserialize_bool<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.format.unify(Format::Bool)?;
        visitor.visit_bool(false)
    }

    fn deserialize_i8<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.format.unify(Format::I8)?;
        visitor.visit_i8(0)
    }

    fn deserialize_i16<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.format.unify(Format::I16)?;
        visitor.visit_i16(0)
    }

    fn deserialize_i32<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.format.unify(Format::I32)?;
        visitor.visit_i32(0)
    }

    fn deserialize_i64<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.format.unify(Format::I64)?;
        visitor.visit_i64(0)
    }

    fn deserialize_i128<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.format.unify(Format::I128)?;
        visitor.visit_i128(0)
    }

    fn deserialize_u8<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.format.unify(Format::U8)?;
        visitor.visit_u8(0)
    }

    fn deserialize_u16<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.format.unify(Format::U16)?;
        visitor.visit_u16(0)
    }

    fn deserialize_u32<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.format.unify(Format::U32)?;
        visitor.visit_u32(0)
    }

    fn deserialize_u64<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.format.unify(Format::U64)?;
        visitor.visit_u64(0)
    }

    fn deserialize_u128<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.format.unify(Format::U128)?;
        visitor.visit_u128(0)
    }

    fn deserialize_f32<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.format.unify(Format::F32)?;
        visitor.visit_f32(0.0)
    }

    fn deserialize_f64<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.format.unify(Format::F64)?;
        visitor.visit_f64(0.0)
    }

    fn deserialize_char<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.format.unify(Format::Char)?;
        visitor.visit_char('A')
    }

    fn deserialize_str<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.format.unify(Format::Str)?;
        visitor.visit_borrowed_str("")
    }

    fn deserialize_string<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.format.unify(Format::Str)?;
        visitor.visit_string(String::new())
    }

    fn deserialize_bytes<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.format.unify(Format::Bytes)?;
        visitor.visit_borrowed_bytes(b"")
    }

    fn deserialize_byte_buf<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.format.unify(Format::Bytes)?;
        visitor.visit_byte_buf(Vec::new())
    }

    fn deserialize_option<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        let mut format = Format::unknown();
        self.format
            .unify(Format::Option(Box::new(format.clone())))?;
        if format.is_unknown() {
            let inner = Deserializer::new(self.tracer, self.samples, &mut format);
            visitor.visit_some(inner)
        } else {
            // Cut exploration.
            visitor.visit_none()
        }
    }

    fn deserialize_unit<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.format.unify(Format::Unit)?;
        visitor.visit_unit()
    }

    fn deserialize_unit_struct<V>(self, name: &'static str, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.format.unify(Format::TypeName(name.into()))?;
        self.tracer
            .registry
            .entry(name.to_string())
            .unify(ContainerFormat::UnitStruct)?;
        visitor.visit_unit()
    }

    fn deserialize_newtype_struct<V>(self, name: &'static str, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.format.unify(Format::TypeName(name.into()))?;
        if self.tracer.config.record_samples_for_newtype_structs {
            // If a value was recorded during serialization, use it.
            if let Some((format, sample)) = self.tracer.get_sample(self.samples, name) {
                return visitor
                    .visit_newtype_struct(sample.into_deserializer())
                    .map_err(|err| match err {
                        Error::DeserializationError(msg) => {
                            let mut format = format.clone();
                            format.reduce();
                            Error::UnexpectedDeserializationFormat(name, format, msg)
                        }
                        _ => err,
                    });
            }
        }
        // Pre-update the registry.
        let mut format = Format::unknown();
        self.tracer
            .registry
            .entry(name.to_string())
            .unify(ContainerFormat::NewTypeStruct(Box::new(format.clone())))?;
        // Compute the format.
        let inner = Deserializer::new(self.tracer, self.samples, &mut format);
        visitor.visit_newtype_struct(inner)
    }

    fn deserialize_seq<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        let mut format = Format::unknown();
        self.format.unify(Format::Seq(Box::new(format.clone())))?;
        if format.is_unknown() {
            // Simulate vector of size 1.
            let inner =
                SeqDeserializer::new(self.tracer, self.samples, std::iter::once(&mut format));
            visitor.visit_seq(inner)
        } else {
            // Cut exploration with a vector of size 0.
            let inner = SeqDeserializer::new(self.tracer, self.samples, std::iter::empty());
            visitor.visit_seq(inner)
        }
    }

    fn deserialize_tuple<V>(self, len: usize, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        let mut formats: Vec<_> = std::iter::repeat_with(Format::unknown).take(len).collect();
        self.format.unify(Format::Tuple(formats.clone()))?;
        let inner = SeqDeserializer::new(self.tracer, self.samples, formats.iter_mut());
        visitor.visit_seq(inner)
    }

    fn deserialize_tuple_struct<V>(
        self,
        name: &'static str,
        len: usize,
        visitor: V,
    ) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.format.unify(Format::TypeName(name.into()))?;
        if self.tracer.config.record_samples_for_tuple_structs {
            // If a value was recorded during serialization, use it.
            if let Some((format, sample)) = self.tracer.get_sample(self.samples, name) {
                let result = || visitor.visit_seq(sample.seq_values()?.into_seq_deserializer());
                return result().map_err(|err| match err {
                    Error::DeserializationError(msg) => {
                        let mut format = format.clone();
                        format.reduce();
                        Error::UnexpectedDeserializationFormat(name, format, msg)
                    }
                    _ => err,
                });
            }
        }
        // Pre-update the registry.
        let mut formats: Vec<_> = std::iter::repeat_with(Format::unknown).take(len).collect();
        self.tracer
            .registry
            .entry(name.to_string())
            .unify(ContainerFormat::TupleStruct(formats.clone()))?;
        // Compute the formats.
        let inner = SeqDeserializer::new(self.tracer, self.samples, formats.iter_mut());
        visitor.visit_seq(inner)
    }

    fn deserialize_map<V>(self, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        let mut key_format = Format::unknown();
        let mut value_format = Format::unknown();
        self.format.unify(Format::Map {
            key: Box::new(key_format.clone()),
            value: Box::new(value_format.clone()),
        })?;
        if key_format.is_unknown() || value_format.is_unknown() {
            // Simulate a map with one entry.
            let inner = SeqDeserializer::new(
                self.tracer,
                self.samples,
                vec![&mut key_format, &mut value_format].into_iter(),
            );
            visitor.visit_map(inner)
        } else {
            // Stop exploration.
            let inner = SeqDeserializer::new(self.tracer, self.samples, std::iter::empty());
            visitor.visit_map(inner)
        }
    }

    fn deserialize_struct<V>(
        self,
        name: &'static str,
        fields: &'static [&'static str],
        visitor: V,
    ) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.format.unify(Format::TypeName(name.into()))?;
        if self.tracer.config.record_samples_for_structs {
            // If a value was recorded during serialization, use it.
            if let Some((format, sample)) = self.tracer.get_sample(self.samples, name) {
                let result = || visitor.visit_seq(sample.seq_values()?.into_seq_deserializer());
                return result().map_err(|err| match err {
                    Error::DeserializationError(msg) => {
                        let mut format = format.clone();
                        format.reduce();
                        Error::UnexpectedDeserializationFormat(name, format, msg)
                    }
                    _ => err,
                });
            }
        }
        // Pre-update the registry.
        let mut formats: Vec<_> = fields
            .iter()
            .map(|&name| Named {
                name: name.into(),
                value: Format::unknown(),
            })
            .collect();
        self.tracer
            .registry
            .entry(name.to_string())
            .unify(ContainerFormat::Struct(formats.clone()))?;
        // Compute the formats.
        let inner = SeqDeserializer::new(
            self.tracer,
            self.samples,
            formats.iter_mut().map(|named| &mut named.value),
        );
        visitor.visit_seq(inner)
    }

    // Assumption: The first variant(s) should be "base cases", i.e. not cause infinite recursion
    // while constructing sample values.
    fn deserialize_enum<V>(
        self,
        name: &'static str,
        variants: &'static [&'static str],
        visitor: V,
    ) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        self.format.unify(Format::TypeName(name.into()))?;
        // Pre-update the registry.
        self.tracer
            .registry
            .entry(name.to_string())
            .unify(ContainerFormat::Enum(BTreeMap::new()))?;
        let known_variants = match self.tracer.registry.get_mut(name) {
            Some(ContainerFormat::Enum(x)) => x,
            _ => unreachable!(),
        };
        // If we have found all the variants OR if the enum is marked as
        // incomplete already, pick the first index.
        let index = if known_variants.len() == variants.len()
            || self.tracer.incomplete_enums.contains(name)
        {
            0
        } else {
            let mut index = known_variants.len() as u32;
            // Scan the range 0..=known_variants.len() downwards to find the next
            // variant index to explore.
            while known_variants.contains_key(&index) {
                index -= 1;
            }
            index
        };
        let variant = known_variants.entry(index).or_insert_with(|| Named {
            name: (*variants
                .get(index as usize)
                .expect("variant indexes must be a non-empty range 0..variants.len()"))
            .to_string(),
            value: VariantFormat::unknown(),
        });
        let mut value = variant.value.clone();
        // Mark the enum as incomplete if this was not the last variant to explore.
        if known_variants.len() != variants.len() {
            self.tracer.incomplete_enums.insert(name.into());
        }
        // Compute the format for this variant.
        let inner = EnumDeserializer::new(self.tracer, self.samples, index, &mut value);
        visitor.visit_enum(inner)
    }

    fn deserialize_identifier<V>(self, _visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        Err(Error::NotSupported("deserialize_identifier"))
    }

    fn deserialize_ignored_any<V>(self, _visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        Err(Error::NotSupported("deserialize_ignored_any"))
    }

    fn is_human_readable(&self) -> bool {
        self.tracer.config.is_human_readable
    }
}

struct SeqDeserializer<'de, 'a, I> {
    tracer: &'a mut Tracer,
    samples: &'de Samples,
    formats: I,
}

impl<'de, 'a, I> SeqDeserializer<'de, 'a, I> {
    fn new(tracer: &'a mut Tracer, samples: &'de Samples, formats: I) -> Self {
        Self {
            tracer,
            samples,
            formats,
        }
    }
}

impl<'de, 'a, I> de::SeqAccess<'de> for SeqDeserializer<'de, 'a, I>
where
    I: Iterator<Item = &'a mut Format>,
{
    type Error = Error;

    fn next_element_seed<T>(&mut self, seed: T) -> Result<Option<T::Value>>
    where
        T: DeserializeSeed<'de>,
    {
        let format = match self.formats.next() {
            Some(x) => x,
            None => return Ok(None),
        };
        let inner = Deserializer::new(self.tracer, self.samples, format);
        seed.deserialize(inner).map(Some)
    }

    fn size_hint(&self) -> Option<usize> {
        self.formats.size_hint().1
    }
}

impl<'de, 'a, I> de::MapAccess<'de> for SeqDeserializer<'de, 'a, I>
where
    // Must have an even number of elements
    I: Iterator<Item = &'a mut Format>,
{
    type Error = Error;

    fn next_key_seed<K>(&mut self, seed: K) -> Result<Option<K::Value>>
    where
        K: DeserializeSeed<'de>,
    {
        let format = match self.formats.next() {
            Some(x) => x,
            None => return Ok(None),
        };
        let inner = Deserializer::new(self.tracer, self.samples, format);
        seed.deserialize(inner).map(Some)
    }

    fn next_value_seed<V>(&mut self, seed: V) -> Result<V::Value>
    where
        V: DeserializeSeed<'de>,
    {
        let format = match self.formats.next() {
            Some(x) => x,
            None => unreachable!(),
        };
        let inner = Deserializer::new(self.tracer, self.samples, format);
        seed.deserialize(inner)
    }

    fn size_hint(&self) -> Option<usize> {
        self.formats.size_hint().1.map(|x| x / 2)
    }
}

struct EnumDeserializer<'de, 'a> {
    tracer: &'a mut Tracer,
    samples: &'de Samples,
    index: u32,
    format: &'a mut VariantFormat,
}

impl<'de, 'a> EnumDeserializer<'de, 'a> {
    fn new(
        tracer: &'a mut Tracer,
        samples: &'de Samples,
        index: u32,
        format: &'a mut VariantFormat,
    ) -> Self {
        Self {
            tracer,
            samples,
            index,
            format,
        }
    }
}

impl<'de> de::EnumAccess<'de> for EnumDeserializer<'de, '_> {
    type Error = Error;
    type Variant = Self;

    fn variant_seed<V>(self, seed: V) -> Result<(V::Value, Self::Variant)>
    where
        V: DeserializeSeed<'de>,
    {
        let index = self.index;
        let value = seed.deserialize(index.into_deserializer())?;
        Ok((value, self))
    }
}

impl<'de> de::VariantAccess<'de> for EnumDeserializer<'de, '_> {
    type Error = Error;

    fn unit_variant(self) -> Result<()> {
        self.format.unify(VariantFormat::Unit)
    }

    fn newtype_variant_seed<T>(self, seed: T) -> Result<T::Value>
    where
        T: DeserializeSeed<'de>,
    {
        let mut format = Format::unknown();
        self.format
            .unify(VariantFormat::NewType(Box::new(format.clone())))?;
        let inner = Deserializer::new(self.tracer, self.samples, &mut format);
        seed.deserialize(inner)
    }

    fn tuple_variant<V>(self, len: usize, visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        let mut formats: Vec<_> = std::iter::repeat_with(Format::unknown).take(len).collect();
        self.format.unify(VariantFormat::Tuple(formats.clone()))?;
        let inner = SeqDeserializer::new(self.tracer, self.samples, formats.iter_mut());
        visitor.visit_seq(inner)
    }

    fn struct_variant<V>(self, fields: &'static [&'static str], visitor: V) -> Result<V::Value>
    where
        V: Visitor<'de>,
    {
        let mut formats: Vec<_> = fields
            .iter()
            .map(|&name| Named {
                name: name.into(),
                value: Format::unknown(),
            })
            .collect();
        self.format.unify(VariantFormat::Struct(formats.clone()))?;

        let inner = SeqDeserializer::new(
            self.tracer,
            self.samples,
            formats.iter_mut().map(|named| &mut named.value),
        );
        visitor.visit_seq(inner)
    }
}
