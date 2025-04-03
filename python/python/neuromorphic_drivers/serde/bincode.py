import io
import struct
import typing

import numpy

from . import binary, type

MAX_LENGTH = (1 << 31) - 1


class BincodeSerializer(binary.Serializer):
    def __init__(self):
        super().__init__(output=io.BytesIO(), container_depth_budget=None)

    def serialize_f32(self, value: type.float32):
        self.output.write(struct.pack("<f", value))

    def serialize_f64(self, value: type.float64):
        self.output.write(struct.pack("<d", value))

    def serialize_len(self, value: int):
        if value > MAX_LENGTH:
            raise type.SerializationError("Length exceeds the maximum supported value.")
        self.output.write(int(value).to_bytes(8, "little", signed=False))

    def serialize_variant_index(self, value: int):
        self.output.write(int(value).to_bytes(4, "little", signed=False))

    def sort_map_entries(self, offsets: typing.List[int]):
        pass


class BincodeDeserializer(binary.Deserializer):
    def __init__(self, content):
        super().__init__(input=io.BytesIO(content), container_depth_budget=None)

    def deserialize_f32(self) -> type.float32:
        (value,) = struct.unpack("<f", self.read(4))
        return numpy.float32(value)

    def deserialize_f64(self) -> type.float64:
        (value,) = struct.unpack("<d", self.read(8))
        return numpy.float64(value)

    def deserialize_len(self) -> int:
        value = int.from_bytes(self.read(8), byteorder="little", signed=False)
        if value > MAX_LENGTH:
            raise type.DeserializationError(
                "Length exceeds the maximum supported value."
            )
        return value

    def deserialize_variant_index(self) -> int:
        return int.from_bytes(self.read(4), byteorder="little", signed=False)

    def check_that_key_slices_are_increasing(
        self,
        slice1: tuple[int, int],
        slice2: tuple[int, int],
    ):
        pass


def serialize(obj: typing.Any, obj_type) -> bytes:
    serializer = BincodeSerializer()
    serializer.serialize_any(obj, obj_type)
    return serializer.get_buffer()


def deserialize(content: bytes, obj_type) -> tuple[typing.Any, bytes]:
    deserializer = BincodeDeserializer(content)
    value = deserializer.deserialize_any(obj_type)
    return value, deserializer.get_remaining_buffer()
