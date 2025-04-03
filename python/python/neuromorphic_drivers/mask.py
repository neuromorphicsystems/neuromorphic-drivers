import pathlib
import struct
import typing
import zlib

import numpy

from . import serde


def write_mask_as_png(
    pixels: numpy.ndarray, path: typing.Union[str, bytes, pathlib.Path]
):
    def pack(tag: bytes, data: bytes):
        content = tag + data
        return b"".join(
            (
                struct.pack(">I", len(data)),
                content,
                struct.pack(">I", 0xFFFFFFFF & zlib.crc32(content)),
            )
        )

    with open(path, "wb") as file:
        file.write(
            b"".join(
                [
                    b"\x89PNG\r\n\x1a\n",
                    pack(
                        b"IHDR",
                        struct.pack(
                            ">IIBBBBB",
                            pixels.shape[1],
                            pixels.shape[0],
                            1,
                            0,
                            0,
                            0,
                            0,
                        ),
                    ),
                    pack(
                        b"IDAT",
                        zlib.compress(
                            numpy.hstack(
                                (
                                    numpy.zeros(
                                        (pixels.shape[0], 1), dtype=numpy.uint8
                                    ),
                                    numpy.packbits(pixels).reshape(
                                        (pixels.shape[0], pixels.shape[1] // 8)
                                    ),
                                )
                            ).tobytes(),
                            9,
                        ),
                    ),
                    pack(b"IEND", b""),
                ]
            )
        )


class RowColumnMask:
    def __init__(self, width: serde.type.uint16, height: serde.type.uint16, set: bool):
        self.x_unpacked = numpy.full(width, fill_value=set, dtype=bool)
        self.y_unpacked = numpy.full(height, fill_value=set, dtype=bool)

    def clear_x_range(
        self,
        start: serde.type.uint16,
        stop: serde.type.uint16,
        step: serde.type.uint16,
    ):
        for index in range(start, stop, step):
            self.x_unpacked[index] = False

    def set_x_range(
        self,
        start: serde.type.uint16,
        stop: serde.type.uint16,
        step: serde.type.uint16,
    ):
        for index in range(start, stop, step):
            self.x_unpacked[index] = True

    def clear_y_range(
        self,
        start: serde.type.uint16,
        stop: serde.type.uint16,
        step: serde.type.uint16,
    ):
        for index in range(start, stop, step):
            self.y_unpacked[index] = False

    def set_y_range(
        self,
        start: serde.type.uint16,
        stop: serde.type.uint16,
        step: serde.type.uint16,
    ):
        for index in range(start, stop, step):
            self.x_unpacked[index] = True

    def clear_rectangle(
        self,
        x: serde.type.uint16,
        y: serde.type.uint16,
        width: serde.type.uint16,
        height: serde.type.uint16,
    ):
        self.clear_x_range(start=x, stop=x + width, step=1)
        self.clear_y_range(start=y, stop=y + height, step=1)

    def x_mask(self) -> tuple[serde.type.uint64, ...]:
        uint8 = numpy.packbits(self.x_unpacked, bitorder="little")
        if len(uint8) % 8 != 0:
            uint8 = numpy.append(
                uint8, numpy.zeros(8 - len(uint8) % 8, dtype=numpy.uint8)
            )
        return tuple(uint8.view(dtype="=u8").tolist())  # type: ignore

    def y_mask(self) -> tuple[serde.type.uint64, ...]:
        uint8 = numpy.packbits(self.y_unpacked, bitorder="little")
        if len(uint8) % 8 != 0:
            uint8 = numpy.append(
                uint8, numpy.zeros(8 - len(uint8) % 8, dtype=numpy.uint8)
            )
        return tuple(uint8.view(dtype="=u8").tolist())  # type: ignore

    def pixels(self) -> numpy.ndarray[typing.Any, numpy.dtype[numpy.bool_]]:
        result = numpy.full(
            (len(self.y_unpacked), len(self.x_unpacked)), fill_value=False, dtype=bool
        )
        result[:, self.x_unpacked] = True
        result[numpy.flip(self.y_unpacked), :] = True
        return numpy.bitwise_not(result)


class PropheseeEvk4PixelMask:
    def __init__(self):
        self.pixel_masks = numpy.zeros(
            64,
            dtype=[("x", numpy.uint16), ("y", numpy.uint16), ("set", numpy.bool_)],
        )

    def set(self, index: int, x: int, y: int):
        assert index >= 0 and index < 64
        assert x >= 0 and x < 1280
        assert y >= 0 and y < 720
        self.pixel_masks["x"][index] = x
        self.pixel_masks["y"][index] = y
        self.pixel_masks["set"][index] = True

    def clear(self, index: int):
        assert index < 64
        self.pixel_masks["x"][index] = 0
        self.pixel_masks["y"][index] = 0
        self.pixel_masks["set"][index] = False

    def pixels(self) -> numpy.ndarray[typing.Any, numpy.dtype[numpy.bool_]]:
        result = numpy.full(
            (720, 1280),
            fill_value=False,
            dtype=bool,
        )
        selected_pixel_masks = self.pixel_masks[self.pixel_masks["set"]]
        result[720 - 1 - selected_pixel_masks["y"], selected_pixel_masks["x"]] = True
        return result

    def pixel_mask(
        self,
    ) -> tuple[
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
        serde.type.uint64,
    ]:
        codes = (
            self.pixel_masks["x"].astype(numpy.uint64)
            + self.pixel_masks["y"].astype(numpy.uint64) * 1280
            + 1
        )
        codes[numpy.logical_not(self.pixel_masks["set"])] = 0
        result = numpy.zeros(21, dtype=numpy.uint64)
        for index in range(0, 63):
            result[index // 3] |= numpy.left_shift(
                codes[index], numpy.uint64((index % 3) * 21)
            )
        for bit in range(0, 21):
            result[bit] |= numpy.left_shift(
                numpy.bitwise_and(
                    numpy.right_shift(codes[63], numpy.uint64(bit)),
                    numpy.uint64(1),
                ),
                numpy.uint64(63),
            )
        return tuple(result.tolist())  # type: ignore
