import dataclasses
import typing

import numpy


class SerializationError(ValueError):
    """Error raised during Serialization"""

    pass


class DeserializationError(ValueError):
    """Error raised during Deserialization"""

    pass


@dataclasses.dataclass(init=False)
class uint128:
    high: numpy.uint64
    low: numpy.uint64

    def __init__(self, num):
        self.high = numpy.uint64(num >> 64)
        self.low = numpy.uint64(num & 0xFFFFFFFFFFFFFFFF)

    def __int__(self):
        return (int(self.high) << 64) | int(self.low)


@dataclasses.dataclass(init=False)
class int128:
    high: numpy.int64
    low: numpy.uint64

    def __init__(self, num):
        self.high = numpy.int64(num >> 64)
        self.low = numpy.uint64(num & 0xFFFFFFFFFFFFFFFF)

    def __int__(self):
        return (int(self.high) << 64) | int(self.low)


@dataclasses.dataclass(init=False)
class char:
    value: str

    def __init__(self, s):
        if len(s) != 1:
            raise ValueError("`char` expects a single unicode character")
        self.value = s

    def __str__(self):
        return self.value


unit = type(None)
int8 = typing.Union[numpy.int8, int]
int16 = typing.Union[numpy.int16, int]
int32 = typing.Union[numpy.int32, int]
int64 = typing.Union[numpy.int64, int]
uint8 = typing.Union[numpy.uint8, int]
uint16 = typing.Union[numpy.uint16, int]
uint32 = typing.Union[numpy.uint32, int]
uint64 = typing.Union[numpy.uint64, int]
float32 = typing.Union[numpy.float32, int]
float64 = typing.Union[numpy.float64, int]
