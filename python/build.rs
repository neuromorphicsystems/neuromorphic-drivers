use neuromorphic_drivers::device::Usb;
use std::io::Write;

fn quote_type(
    format: &reflect::Format,
    name_to_new_name: &std::collections::HashMap<String, String>,
) -> String {
    match format {
        reflect::Format::TypeName(name) => name_to_new_name.get(name).unwrap_or(name).clone(),
        reflect::Format::Unit => "serde.type.unit".into(),
        reflect::Format::Bool => "bool".into(),
        reflect::Format::I8 => "serde.type.int8".into(),
        reflect::Format::I16 => "serde.type.int16".into(),
        reflect::Format::I32 => "serde.type.int32".into(),
        reflect::Format::I64 => "serde.type.int64".into(),
        reflect::Format::I128 => "serde.type.int128".into(),
        reflect::Format::U8 => "serde.type.uint8".into(),
        reflect::Format::U16 => "serde.type.uint16".into(),
        reflect::Format::U32 => "serde.type.uint32".into(),
        reflect::Format::U64 => "serde.type.uint64".into(),
        reflect::Format::U128 => "serde.type.uint128".into(),
        reflect::Format::F32 => "serde.type.float32".into(),
        reflect::Format::F64 => "serde.type.float64".into(),
        reflect::Format::Char => "serde.type.char".into(),
        reflect::Format::Str => "str".into(),
        reflect::Format::Bytes => "bytes".into(),
        reflect::Format::Option(format) => {
            format!("typing.Optional[{}]", quote_type(format, name_to_new_name))
        }
        reflect::Format::Seq(format) => {
            format!("list[{}]", quote_type(format, name_to_new_name))
        }
        reflect::Format::Map { key, value } => {
            format!(
                "dict[{}, {}]",
                quote_type(key, name_to_new_name),
                quote_type(value, name_to_new_name)
            )
        }
        reflect::Format::Tuple(formats) => {
            if formats.is_empty() {
                "tuple[()]".into()
            } else {
                format!("tuple[{}]", quote_types(formats, name_to_new_name))
            }
        }
        reflect::Format::TupleArray { content, size } => format!(
            "tuple[\n        {}\n    ]",
            quote_types(&vec![content.as_ref().clone(); *size], name_to_new_name)
        ),
        reflect::Format::Variable(_) => panic!("unexpected value"),
    }
}

fn quote_types(
    formats: &[reflect::Format],
    name_to_new_name: &std::collections::HashMap<String, String>,
) -> String {
    format!(
        "{},",
        formats
            .iter()
            .map(|format| quote_type(format, name_to_new_name))
            .collect::<Vec<_>>()
            .join(",\n        ")
    )
}

fn value_to_string(value: &reflect::Value) -> String {
    match value {
        reflect::Value::Unit => "()".to_owned(),
        reflect::Value::Bool(value) => (if *value { "True" } else { "False" }).to_owned(),
        reflect::Value::I8(value) => value.to_string(),
        reflect::Value::I16(value) => value.to_string(),
        reflect::Value::I32(value) => value.to_string(),
        reflect::Value::I64(value) => value.to_string(),
        reflect::Value::I128(value) => value.to_string(),
        reflect::Value::U8(value) => format!("{:#04X}", value),
        reflect::Value::U16(value) => value.to_string(),
        reflect::Value::U32(value) => value.to_string(),
        reflect::Value::U64(value) => value.to_string(),
        reflect::Value::U128(value) => value.to_string(),
        reflect::Value::F32(value) => value.to_string(),
        reflect::Value::F64(value) => value.to_string(),
        reflect::Value::Char(value) => value.to_string(),
        reflect::Value::Str(value) => format!("\"{}\"", value.replace('"', "\\\"")),
        reflect::Value::Bytes(value) => format!(
            "[{}]",
            value
                .iter()
                .map(|value| format!("{:#04x}", value))
                .collect::<Vec<String>>()
                .join(", ")
        ),
        reflect::Value::Option(value) => value
            .as_ref()
            .map_or_else(|| "None".to_owned(), |value| value_to_string(value)),
        reflect::Value::Variant(_, value) => value_to_string(value),
        reflect::Value::Seq(value) => format!(
            "({})",
            value
                .iter()
                .map(value_to_string)
                .collect::<Vec<String>>()
                .join(", ")
        ),
    }
}

fn camel_case_to_screaming_case(string: &str) -> String {
    let mut result = String::new();
    for (index, character) in string.char_indices() {
        if index > 0 && character.is_ascii_uppercase() {
            result.push('_');
        }
        result.push(character.to_ascii_uppercase());
    }
    result
}

fn snake_case_to_camel_case(string: &str) -> String {
    let mut result = String::new();
    let mut capitalize = true;
    for character in string.chars() {
        if character == '_' {
            capitalize = true;
        } else if capitalize {
            result.push(character.to_ascii_uppercase());
            capitalize = false;
        } else {
            result.push(character);
        }
    }
    result
}

enum NodeClass {
    Dataclass {
        children: std::collections::HashSet<String>,
        fields: Vec<reflect::Named<reflect::Format>>,
    },
    Enum {
        id_to_field: std::collections::BTreeMap<u32, reflect::Named<reflect::VariantFormat>>,
    },
}

struct Node {
    name: String,
    class: NodeClass,
    required: bool,
}

struct DataclassParameters {
    frozen: bool,
    serializable: bool,
    module_name: Option<String>,
    skip_fields: std::collections::HashSet<String>,
    name_to_new_name: std::collections::HashMap<String, String>,
    new_root_name: Option<String>,
}

fn generate_dataclasses<Writer: std::io::Write, Structure>(
    writer: &mut Writer,
    default_structure: &Structure,
    mut parameters: DataclassParameters,
) where
    Structure: serde::Serialize + serde::de::Deserialize<'static>,
{
    let mut samples = reflect::Samples::new();
    let mut tracer =
        reflect::Tracer::new(reflect::TracerConfig::default().record_samples_for_structs(true));
    let root_name = match tracer
        .recursive_trace(&mut samples, default_structure)
        .unwrap()
        .0
    {
        reflect::Format::TypeName(name) => name,
        _ => panic!("the root format is not a type name"),
    };
    if let Some(new_root_name) = parameters.new_root_name {
        parameters
            .name_to_new_name
            .insert(root_name.clone(), new_root_name);
    }
    let registry = tracer.registry().unwrap();
    let mut nodes: Vec<Node> = Vec::new();
    for (name, format) in registry.iter() {
        if nodes.iter().any(|node| node.name == *name) {
            panic!("multiple nodes have the same name \"{}\"", name);
        }
        let mut class = match format {
            reflect::ContainerFormat::UnitStruct => NodeClass::Dataclass {
                children: std::collections::HashSet::new(),
                fields: Vec::new(),
            },
            reflect::ContainerFormat::NewTypeStruct(format) => NodeClass::Dataclass {
                children: std::collections::HashSet::new(),
                fields: vec![reflect::Named {
                    name: "value".to_string(),
                    value: format.as_ref().clone(),
                }],
            },
            reflect::ContainerFormat::TupleStruct(formats) => NodeClass::Dataclass {
                children: std::collections::HashSet::new(),
                fields: vec![reflect::Named {
                    name: "value".to_string(),
                    value: reflect::Format::Tuple(formats.clone()),
                }],
            },
            reflect::ContainerFormat::Struct(fields) => NodeClass::Dataclass {
                children: std::collections::HashSet::new(),
                fields: fields.clone(),
            },
            reflect::ContainerFormat::Enum(id_to_field) => NodeClass::Enum {
                id_to_field: id_to_field.clone(),
            },
        };
        if let NodeClass::Dataclass { children, fields } = &mut class {
            for field in fields.iter() {
                if !parameters.skip_fields.contains(&field.name) {
                    match &field.value {
                        reflect::Format::Option(format) => {
                            if let reflect::Format::TypeName(name) = format.as_ref() {
                                children.insert(name.to_owned());
                            }
                        }
                        reflect::Format::TypeName(name) => {
                            children.insert(name.to_owned());
                        }
                        _ => (),
                    }
                }
            }
        }
        nodes.push(Node {
            name: name.clone(),
            class,
            required: false,
        });
    }
    {
        let mut nodes_names_updated = std::collections::HashSet::new();
        let mut nodes_names_to_update = std::collections::HashSet::from([root_name.clone()]);
        while !nodes_names_to_update.is_empty() {
            let mut new_nodes_names_to_update = std::collections::HashSet::new();
            for node_name in nodes_names_to_update {
                let node = nodes
                    .iter_mut()
                    .find(|node| node.name == node_name)
                    .unwrap();
                node.required = true;
                if let NodeClass::Dataclass {
                    children,
                    fields: _,
                } = &node.class
                {
                    new_nodes_names_to_update.extend(
                        children
                            .iter()
                            .filter(|child| !nodes_names_updated.contains(*child))
                            .cloned(),
                    );
                }
                nodes_names_updated.insert(node.name.clone());
            }
            nodes_names_to_update = new_nodes_names_to_update;
        }
    }
    {
        let mut generated_nodes = std::collections::HashSet::new();
        loop {
            let mut nodes_to_generate = Vec::new();
            for node in nodes.iter() {
                if node.required
                    && !generated_nodes.contains(&node.name)
                    && match &node.class {
                        NodeClass::Dataclass {
                            children,
                            fields: _,
                        } => children.iter().all(|child| generated_nodes.contains(child)),
                        _ => true,
                    }
                {
                    nodes_to_generate.push(node);
                }
            }
            if nodes_to_generate.is_empty() {
                if nodes
                    .iter()
                    .any(|node| node.required && !generated_nodes.contains(&node.name))
                {
                    panic!("circular dependency in {root_name}");
                }
                break;
            } else {
                nodes_to_generate.sort_by(|a, b| a.name.cmp(&b.name));
                for node in nodes_to_generate.iter() {
                    let name = parameters
                        .name_to_new_name
                        .get(&node.name)
                        .unwrap_or(&node.name)
                        .clone();
                    match &node.class {
                        NodeClass::Dataclass {
                            children: _,
                            fields,
                        } => {
                            let values = samples.value(&node.name).map(|value| match value {
                                reflect::Value::Seq(values) => values,
                                _ => panic!("{} is not a sequence or dictionary", node.name),
                            });
                            writeln!(
                                writer,
                                concat!("\n", "\n", "@dataclasses.dataclass{}\n", "class {}:",),
                                if parameters.frozen {
                                    "(frozen=True)"
                                } else {
                                    ""
                                },
                                name,
                            )
                            .unwrap();
                            for (index, field) in fields.iter().enumerate() {
                                if !parameters.skip_fields.contains(&field.name) {
                                    writeln!(
                                        writer,
                                        "    {}: {}{}",
                                        field.name,
                                        quote_type(&field.value, &parameters.name_to_new_name),
                                        match &field.value {
                                            reflect::Format::TypeName(name) => {
                                                let node = nodes
                                                    .iter()
                                                    .find(|node| node.name == *name)
                                                    .unwrap();
                                                match &node.class {
                                                    NodeClass::Dataclass { children: _, fields: _ } => {
                                                        format!(" = dataclasses.field(default_factory={name})")
                                                    },
                                                    NodeClass::Enum { id_to_field } => {
                                                        format!(" = {}.{}", name, camel_case_to_screaming_case(&id_to_field.get(&0).unwrap().name))
                                                    },
                                                }
                                            },
                                            _ => match values {
                                                Some(values) => format!(" = {}", value_to_string(&values[index])),
                                                None => "".to_owned(),
                                            },
                                        },
                                    )
                                    .unwrap();
                                }
                            }
                        }
                        NodeClass::Enum { id_to_field } => {
                            writeln!(writer, concat!("\n", "\n", "class {}(enum.Enum):",), name,)
                                .unwrap();
                            for (id, field) in id_to_field {
                                match field.value {
                                    reflect::VariantFormat::Unit => (),
                                    _ => panic!(
                                        "unsupported non-unit field {} in {}",
                                        field.name, name
                                    ),
                                }
                                writeln!(
                                    writer,
                                    "    {} = {}",
                                    camel_case_to_screaming_case(&field.name),
                                    id
                                )
                                .unwrap();
                            }
                        }
                    }
                    if parameters.serializable {
                        writeln!(
                            writer,
                            concat!(
                                "\n",
                                "    def serialize(self) -> bytes:\n",
                                "        return serde.bincode.serialize(self, {})",
                            ),
                            name
                        )
                        .unwrap();
                    }
                    if name == root_name {
                        if let Some(module_name) = parameters.module_name.as_ref() {
                            writeln!(
                                writer,
                                concat!(
                                    "\n",
                                    "    @staticmethod\n",
                                    "    def type() -> str:\n",
                                    "        return \"{}\"",
                                ),
                                module_name
                            )
                            .unwrap();
                        }
                    }
                    generated_nodes.insert(node.name.clone());
                }
            }
        }
    }
}

macro_rules! generate {
    ($(($module:ident, $packet:ident)),+) => {
        let python_generated_directory = std::path::Path::new("python/neuromorphic_drivers/generated");
        let _ = std::fs::remove_dir_all(python_generated_directory);
        std::fs::create_dir(python_generated_directory).unwrap();
        let devices_directory = python_generated_directory.join("devices");
        std::fs::create_dir(&devices_directory).unwrap();
        paste::paste! {
            let mut writer = std::io::BufWriter::new(
                std::fs::File::create(python_generated_directory.join("enums.py")).unwrap(),
            );
            writeln!(writer, concat!(
                "import enum\n",
                "\n",
                "\n",
                "class Speed(enum.Enum):"
            )).unwrap();
            {
                let mut tracer = reflect::Tracer::new(reflect::TracerConfig::default());
                let (_, samples) = tracer.trace_simple_type::<neuromorphic_drivers::usb::Speed>().unwrap();
                let registry = tracer.registry().unwrap();
                for (name, format) in registry {
                    match format {
                        reflect::ContainerFormat::Enum(variants) => {
                            for (variant, sample) in variants.iter().zip(samples.iter()) {
                                writeln!(
                                    writer,
                                    "    {} = \"{}\"",
                                    camel_case_to_screaming_case(&variant.1.name),
                                    sample.to_string(),
                                ).unwrap();
                            }
                        }
                        _ => {
                            panic!("{name} uses an unsupported root type");
                        }
                    }
                }
            }
            writeln!(writer, "\n\nclass Name(enum.Enum):").unwrap();
            $(
                writeln!(
                    writer,
                    "    {} = \"{}\"",
                    stringify!([<$module:upper>]),
                    neuromorphic_drivers::devices::$module::Device::PROPERTIES.name,
                ).unwrap();
            )+
        }
        $(
            paste::paste! {
                let mut writer = std::io::BufWriter::new(
                    std::fs::File::create(devices_directory.join(format!("{}.py", stringify!($module)))).unwrap(),
                );
                writeln!(writer, concat!(
                    "import dataclasses\n",
                    "import enum\n",
                    "import types\n",
                    "import typing\n",
                    "\n",
                    "import numpy\n",
                    "\n",
                    "from .. import enums\n",
                    "from ... import orientation\n",
                    "from ... import packet\n",
                    "from ... import serde\n",
                    "from ... import status",
                )).unwrap();
                generate_dataclasses(
                    &mut writer,
                    &neuromorphic_drivers::devices::$module::Device::PROPERTIES.default_configuration,
                    DataclassParameters {
                        frozen: false,
                        serializable: true,
                        module_name: Some(stringify!($module).to_owned()),
                        skip_fields: std::collections::HashSet::new(),
                        name_to_new_name: std::collections::HashMap::new(),
                        new_root_name: None,
                    },
                );
                generate_dataclasses(
                    &mut writer,
                    &neuromorphic_drivers::devices::$module::Device::DEFAULT_USB_CONFIGURATION,
                    DataclassParameters {
                        frozen: false,
                        serializable: true,
                        module_name: None,
                        skip_fields: std::collections::HashSet::new(),
                        name_to_new_name: std::collections::HashMap::new(),
                        new_root_name: Some("UsbConfiguration".to_owned()),
                    },
                );
                generate_dataclasses(
                    &mut writer,
                    &neuromorphic_drivers::devices::$module::Device::PROPERTIES,
                    DataclassParameters {
                        frozen: true,
                        serializable: false,
                        module_name: None,
                        skip_fields: std::collections::HashSet::from([
                            "name".to_owned(),
                            "default_configuration".to_owned(),
                        ]),
                        name_to_new_name: std::collections::HashMap::new(),
                        new_root_name: Some("Properties".into()),
                    },
                );
                for (class_name_suffix, iter_data_left_prefix, iter_data_right) in [
                    ("Device",  "status.", concat!("packet.", stringify!($packet))),
                    ("DeviceRaw",  "status.Raw", "bytes"),
                ] {
                    for (class_suffix, iter_data_left, iter_data_right_prefix, iter_data_right_suffix) in [
                        ("", "StatusNonOptional", "", ""),
                        ("Optional", "Status", "typing.Optional[", "]"),
                    ] {
                        let class_name = format!("{}{}", snake_case_to_camel_case(stringify!($module)), class_name_suffix);
                        writeln!(
                            writer,
                            concat!(
                                "\n",
                                "\n",
                                "class {}{}(typing.Protocol):\n",
                                "    def __enter__(self) -> \"{}{}\": ...\n",
                                "\n",
                                "    def __exit__(\n",
                                "        self,\n",
                                "        exception_type: typing.Optional[typing.Type[BaseException]],\n",
                                "        value: typing.Optional[BaseException],\n",
                                "        traceback: typing.Optional[types.TracebackType],\n",
                                "    ) -> bool:\n",
                                "        ...\n",
                                "\n",
                                "    def __iter__(self) -> \"{}{}\": ...\n",
                                "\n",
                                "    def __next__(self) -> tuple[{}{}, {}{}{}]: ...\n",
                                "\n",
                                "    def backlog(self) -> int: ...\n",
                                "\n",
                                "    def clear_backlog(self, until: int): ...\n",
                                "\n",
                                "    def overflow(self) -> bool: ...\n",
                                "\n",
                                "    def name(self) -> typing.Literal[enums.Name.{}]: ...\n",
                                "\n",
                                "    def properties(self) -> Properties: ...\n",
                                "\n",
                                "    def serial(self) -> str: ...\n",
                                "\n",
                                "    def chip_firmware_configuration(self) -> Configuration: ...\n",
                                "\n",
                                "    def speed(self) -> enums.Speed: ...\n",
                                "\n",
                                "    def update_configuration(self, configuration: Configuration): ...",
                            ),
                            class_name,
                            class_suffix,
                            class_name,
                            class_suffix,
                            class_name,
                            class_suffix,
                            iter_data_left_prefix,
                            iter_data_left,
                            iter_data_right_prefix,
                            iter_data_right,
                            iter_data_right_suffix,
                            stringify!([<$module:upper>]),
                        ).unwrap();
                        if stringify!($module) == "inivation_davis346" {
                            writeln!(
                                writer,
                                concat!(
                                    "\n",
                                    "    def orientation(self) -> orientation.Davis346Orientation: ...\n",
                                    "\n",
                                    "    def imu_type(self) -> typing.Literal[None, \"InvenSense6050Or6150\", \"InvenSense9250\"]: ...",
                                )
                            ).unwrap();
                        } else if stringify!($module) == "inivation_dvxplorer" {
                            writeln!(
                                writer,
                                concat!(
                                    "\n",
                                    "    def orientation(self) -> orientation.DvxplorerOrientation: ...\n",
                                )
                            ).unwrap();
                        } else if stringify!($module) == "prophesee_evk4" {
                            writeln!(
                                writer,
                                concat!(
                                    "\n",
                                    "    def temperature_celsius(self) -> float: ...\n",
                                    "\n",
                                    "    def illuminance(self) -> int: ...",
                                )
                            ).unwrap();
                        }
                    }
                }
            }
        )+
        paste::paste! {
            let mut writer = std::io::BufWriter::new(
                std::fs::File::create(python_generated_directory.join("unions.py")).unwrap(),
            );
            writeln!(
                writer,
                concat!(
                    "import typing\n",
                    "\n",
                    "from . import enums\n",
                    "from .. import serde",
                ),
            ).unwrap();
            $(
                writeln!(writer,  "from .devices import {}", stringify!($module)).unwrap();
            )+
            writeln!(writer, "\n\nProperties = typing.Union[").unwrap();
            $(
                writeln!(writer,  "    {}.Properties,", stringify!($module)).unwrap();
            )+
            writeln!(writer, "]").unwrap();
            writeln!(writer, "\nConfiguration = typing.Union[").unwrap();
            $(
                writeln!(writer,  "    {}.Configuration,", stringify!($module)).unwrap();
            )+
            writeln!(writer, "]").unwrap();
            writeln!(writer, "\nUsbConfiguration = typing.Union[").unwrap();
            $(
                writeln!(writer,  "    {}.UsbConfiguration,", stringify!($module)).unwrap();
            )+
            writeln!(writer, "]").unwrap();
            writeln!(
                writer,
                concat!(
                    "\n",
                    "\n",
                    "def name_to_properties(name: enums.Name) -> Properties:",
                ),
            ).unwrap();
            $(
                writeln!(
                    writer,
                    concat!(
                        "    if name == enums.Name.{}:\n",
                        "        return {}.Properties()",
                    ),
                    stringify!([<$module:upper>]),
                    stringify!($module),
                ).unwrap();
            )+
            writeln!(
                writer,
                "    raise Exception(f\"unknown name {{name}}\")",
            ).unwrap();
            writeln!(
                writer,
                concat!(
                    "\n",
                    "\n",
                    "def deserialize_configuration(name: enums.Name, data: bytes) -> Configuration:",
                ),
            ).unwrap();
            $(
                writeln!(
                    writer,
                    concat!(
                        "    if name == enums.Name.{}:\n",
                        "        return serde.bincode.deserialize(data, {}.Configuration)[0]",
                    ),
                    stringify!([<$module:upper>]),
                    stringify!($module),
                ).unwrap();
            )+
            writeln!(
                writer,
                "    raise Exception(f\"unknown name {{name}}\")",
            ).unwrap();
        }
        {
            let mut writer = std::io::BufWriter::new(
                std::fs::File::create(python_generated_directory.join("devices_types.py")).unwrap(),
            );
            writeln!(
                writer,
                concat!(
                    "# pyright: reportOverlappingOverload=false\n",
                    "\n",
                    "import types\n",
                    "import typing\n",
                    "\n",
                    "import numpy\n",
                    "\n",
                    "from .. import device\n",
                    "from .. import packet\n",
                    "from .. import status",
                ),
            ).unwrap();
            $(
                writeln!(writer,  "from .devices import {} as {}", stringify!($module), stringify!($module)).unwrap();
            )+
            writeln!(
                writer,
                concat!(
                    "from .enums import *\n",
                    "from .unions import *\n",
                ),
            ).unwrap();
            for (class_name, iter_data_left_prefix, iter_data_right) in [
                ("GenericDevice", "status.", "typing.Union[packet.Davis346Packet, packet.DvxplorerPacket, packet.Evt3Packet]"),
                ("GenericDeviceRaw", "status.Raw", "bytes"),
            ] {
                for (class_suffix, iter_data_left, iter_data_right_prefix, iter_data_right_suffix) in [
                    ("", "StatusNonOptional", "", ""),
                    ("Optional", "Status", "typing.Optional[", "]"),
                ] {
                    writeln!(
                        writer,
                        concat!(
                            "\n",
                            "\n",
                            "class {}{}(typing.Protocol):\n",
                            "    def __enter__(self) -> \"{}{}\": ...\n",
                            "\n",
                            "    def __exit__(\n",
                            "        self,\n",
                            "        exception_type: typing.Optional[typing.Type[BaseException]],\n",
                            "        value: typing.Optional[BaseException],\n",
                            "        traceback: typing.Optional[types.TracebackType],\n",
                            "    ) -> bool:\n",
                            "        ...\n",
                            "\n",
                            "    def __iter__(self) -> \"{}{}\": ...\n",
                            "\n",
                            "    def __next__(self) -> tuple[{}{}, {}{}{}]: ...\n",
                            "\n",
                            "    def backlog(self) -> int: ...\n",
                            "\n",
                            "    def clear_backlog(self, until: int): ...\n",
                            "\n",
                            "    def overflow(self) -> bool: ...\n",
                            "\n",
                            "    def name(self) -> Name: ...\n",
                            "\n",
                            "    def properties(self) -> Properties: ...\n",
                            "\n",
                            "    def serial(self) -> str: ...\n",
                            "\n",
                            "    def chip_firmware_configuration(self) -> Configuration: ...\n",
                            "\n",
                            "    def speed(self) -> Speed: ...\n",
                            "\n",
                            "    def update_configuration(self, configuration: Configuration): ...\n",
                        ),
                        class_name,
                        class_suffix,
                        class_name,
                        class_suffix,
                        class_name,
                        class_suffix,
                        iter_data_left_prefix,
                        iter_data_left,
                        iter_data_right_prefix,
                        iter_data_right,
                        iter_data_right_suffix,
                    ).unwrap();
                }
            }
            $(
                for (class_name_suffix, raw) in [
                    ("Device", "False"),
                    ("DeviceRaw", "True"),
                ] {
                    let class_name = format!("{}{}", snake_case_to_camel_case(stringify!($module)), class_name_suffix);
                    for (class_suffix, iterator_timeout) in [
                        ("", "typing.Literal[None] = None"),
                        ("Optional", "float"),
                    ] {
                        writeln!(
                            writer,
                            concat!(
                                "\n",
                                "\n",
                                "@typing.overload\n",
                                "def open(\n",
                                "    configuration: {}.Configuration,\n",
                                "    iterator_timeout: {},\n",
                                "    raw: typing.Literal[{}] = {},\n",
                                "    serial: typing.Optional[str] = None,\n",
                                "    usb_configuration: typing.Optional[UsbConfiguration] = None,\n",
                                "    iterator_maximum_raw_packets: int = 64,\n",
                                ") -> {}.{}{}:\n",
                                "    ...",
                            ),
                            stringify!($module),
                            iterator_timeout,
                            raw,
                            raw,
                            stringify!($module),
                            class_name,
                            class_suffix,
                        ).unwrap();
                    }
                }
            )+
            for (class_name, raw) in [
                ("GenericDevice", "False"),
                ("GenericDeviceRaw", "True"),
            ] {
                for (class_suffix, iterator_timeout) in [
                    ("", "typing.Literal[None]"),
                    ("Optional", "typing.Optional[float]"),
                ] {
                    writeln!(
                        writer,
                        concat!(
                            "\n",
                            "\n",
                            "@typing.overload\n",
                            "def open(\n",
                            "    configuration: typing.Optional[Configuration] = None,\n",
                            "    iterator_timeout: {} = None,\n",
                            "    raw: typing.Literal[{}] = {},\n",
                            "    serial: typing.Optional[str] = None,\n",
                            "    usb_configuration: typing.Optional[UsbConfiguration] = None,\n",
                            "    iterator_maximum_raw_packets: int = 64,\n",
                            ") -> {}{}:\n",
                            "    ...",
                        ),
                        iterator_timeout,
                        raw,
                        raw,
                        class_name,
                        class_suffix,
                    ).unwrap();
                }
            }
            writeln!(
                writer,
                concat!(
                    "\n",
                    "\n",
                    "def open(\n",
                    "    configuration: typing.Optional[Configuration] = None,\n",
                    "    iterator_timeout: typing.Optional[float] = None,\n",
                    "    raw: bool = False,\n",
                    "    serial: typing.Optional[str] = None,\n",
                    "    usb_configuration: typing.Optional[UsbConfiguration] = None,\n",
                    "    iterator_maximum_raw_packets: int = 64,\n",
                    ") -> typing.Any:\n",
                    "    return device.Device.__new__(\n",
                    "        device.Device,\n",
                    "        raw,\n",
                    "        iterator_maximum_raw_packets,\n",
                    "        None if configuration is None else configuration.type(),\n",
                    "        None if configuration is None else configuration.serialize(),\n",
                    "        serial,\n",
                    "        None if usb_configuration is None else usb_configuration.serialize(),\n",
                    "        iterator_timeout,\n",
                    "    )",
                ),
            ).unwrap();
        }
    };
}

fn macos_link_search_path() -> Option<String> {
    let output = cc::Build::new()
        .get_compiler()
        .to_command()
        .arg("--print-search-dirs")
        .output()
        .unwrap();
    for line in String::from_utf8_lossy(&output.stdout).lines() {
        if line.contains("libraries: =") {
            let path = line.split('=').nth(1).unwrap();
            if !path.is_empty() {
                return Some(format!("{}/lib/darwin", path));
            }
        }
    }
    None
}

fn main() {
    if std::env::var("TARGET").unwrap().contains("apple") {
        if let Some(path) = macos_link_search_path() {
            println!("cargo:rustc-link-lib=clang_rt.osx");
            println!("cargo:rustc-link-search={}", path);
        }
    }
    generate!(
        (inivation_davis346, Davis346Packet),
        (inivation_dvxplorer, DvxplorerPacket),
        (prophesee_evk3_hd, Evt3Packet),
        (prophesee_evk4, Evt3Packet)
    );
}
