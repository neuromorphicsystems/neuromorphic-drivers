import dataclasses
import sys
import typing

import numpy
import PySide6.QtCore
import PySide6.QtGraphs
import PySide6.QtGui
import PySide6.QtOpenGL
import PySide6.QtQml
import PySide6.QtQuick

EventStyle = typing.Literal["exponential", "linear", "window"]
FrameMode = typing.Literal["L", "RGB", "RGBA", "P"]
FrameDtype = typing.Literal["u1", "u2", "f4"]

VERTEX_SHADER = """
#version 330 core

in vec2 vertices;
out vec2 coordinates;

void main() {
    gl_Position = vec4(vertices.x * 2.0 - 1.0, 1.0 - vertices.y * 2.0, 0.0, 1.0);
    coordinates = vertices;
}
"""

EVENT_DISPLAY_FRAGMENT_SHADER = """
#version 330 core

in vec2 coordinates;
out vec4 color;
uniform sampler2D t_and_on_sampler;
uniform sampler1D colormap_sampler;
uniform float colormap_split;
uniform float current_t;
uniform int style;
uniform float tau;

void main() {
    float t_and_on = texture(t_and_on_sampler, coordinates).r;
    float t = abs(t_and_on);
    bool on = t_and_on >= 0.0f;
    float lambda = 0.0f;
    if (style == 0) {
        lambda = exp(-float(current_t - t) / tau);
    } else if (style == 1) {
        lambda = (current_t - t) < tau ? 1.0f - (current_t - t) / tau : 0.0f;
    } else {
        lambda = (current_t - t) < tau ? 1.0f : 0.0f;
    }
    color = texture(colormap_sampler, colormap_split * (1.0f - lambda) + (on ? lambda : 0.0f));
}
"""


frame_display_mode_and_dtype_to_fragment_shader: dict[
    tuple[FrameMode, FrameDtype], str
] = {}
for mode in ("L", "RGB", "RGBA", "P"):
    if mode == "L" or mode == "P":
        r = "r"
        g = "r"
        b = "r"
        a = None
    elif mode == "RGB":
        r = "r"
        g = "g"
        b = "b"
        a = None
    elif mode == "RGBA":
        r = "r"
        g = "g"
        b = "b"
        a = "a"
    else:
        raise Exception(f"unknown mode {mode}")
    for dtype in ("u1", "u2", "f4"):
        if dtype == "u1":
            sampler = "usampler2D"
            sample_type = "uvec4"
            r_value = f"float(sample.{r}) / 255.0f"
            g_value = f"float(sample.{g}) / 255.0f"
            b_value = f"float(sample.{b}) / 255.0f"
            a_value = "1.0f" if a is None else f"float(sample.{a}) / 255.0f"
        elif dtype == "u2":
            sampler = "usampler2D"
            sample_type = "uvec4"
            r_value = f"float(sample.{r}) / 65535.0f"
            g_value = f"float(sample.{g}) / 65535.0f"
            b_value = f"float(sample.{b}) / 65535.0f"
            a_value = "1.0f" if a is None else f"float(sample.{a}) / 65535.0f"
        elif dtype == "f4":
            sampler = "sampler2D"
            sample_type = "vec4"
            r_value = f"sample.{r}"
            g_value = f"sample.{g}"
            b_value = f"sample.{b}"
            a_value = "1.0f" if a is None else f"sample.{a}"
        else:
            raise Exception(f"unknown dtype {dtype}")
        if mode == "P":
            extra_sampler = "uniform sampler1D colormap_sampler;"
            color = f"texture(colormap_sampler, {r_value})"
        else:
            extra_sampler = ""
            color = f"vec4({r_value}, {g_value}, {b_value}, {a_value})"
        frame_display_mode_and_dtype_to_fragment_shader[
            (mode, dtype)
        ] = f"""
#version 330 core

in vec2 coordinates;
out vec4 color;
uniform {sampler} frame_sampler;
{extra_sampler}

void main() {{
    {sample_type} sample = texture(frame_sampler, coordinates);
    color = {color};
}}
"""

MAXIMUM_DELTA: int = 3600000000

GL_TRIANGLE_STRIP: int = 0x0005
GL_SRC_ALPHA: int = 0x0302
GL_ONE_MINUS_SRC_ALPHA: int = 0x0303
GL_DEPTH_TEST: int = 0x0B71
GL_BLEND: int = 0x0BE2
GL_SCISSOR_TEST: int = 0x0C11
GL_FLOAT: int = 0x1406
GL_COLOR_BUFFER_BIT: int = 0x4000
DEFAULT_TAU: float = 200000.0
DEFAULT_ON_COLORMAP: list[PySide6.QtGui.QColor] = [
    PySide6.QtGui.QColor(25, 25, 25, a=255),
    PySide6.QtGui.QColor(26, 26, 25, a=255),
    PySide6.QtGui.QColor(28, 26, 24, a=255),
    PySide6.QtGui.QColor(29, 27, 24, a=255),
    PySide6.QtGui.QColor(30, 28, 24, a=255),
    PySide6.QtGui.QColor(32, 28, 23, a=255),
    PySide6.QtGui.QColor(33, 29, 23, a=255),
    PySide6.QtGui.QColor(34, 30, 22, a=255),
    PySide6.QtGui.QColor(35, 30, 22, a=255),
    PySide6.QtGui.QColor(37, 31, 22, a=255),
    PySide6.QtGui.QColor(38, 32, 21, a=255),
    PySide6.QtGui.QColor(39, 32, 21, a=255),
    PySide6.QtGui.QColor(40, 33, 21, a=255),
    PySide6.QtGui.QColor(41, 34, 20, a=255),
    PySide6.QtGui.QColor(42, 34, 20, a=255),
    PySide6.QtGui.QColor(44, 35, 20, a=255),
    PySide6.QtGui.QColor(45, 36, 19, a=255),
    PySide6.QtGui.QColor(46, 36, 19, a=255),
    PySide6.QtGui.QColor(47, 37, 18, a=255),
    PySide6.QtGui.QColor(48, 38, 18, a=255),
    PySide6.QtGui.QColor(49, 38, 18, a=255),
    PySide6.QtGui.QColor(50, 39, 17, a=255),
    PySide6.QtGui.QColor(52, 40, 17, a=255),
    PySide6.QtGui.QColor(53, 40, 16, a=255),
    PySide6.QtGui.QColor(54, 41, 16, a=255),
    PySide6.QtGui.QColor(55, 42, 16, a=255),
    PySide6.QtGui.QColor(56, 43, 15, a=255),
    PySide6.QtGui.QColor(57, 43, 15, a=255),
    PySide6.QtGui.QColor(58, 44, 15, a=255),
    PySide6.QtGui.QColor(59, 45, 14, a=255),
    PySide6.QtGui.QColor(60, 45, 14, a=255),
    PySide6.QtGui.QColor(61, 46, 13, a=255),
    PySide6.QtGui.QColor(63, 47, 13, a=255),
    PySide6.QtGui.QColor(64, 48, 13, a=255),
    PySide6.QtGui.QColor(65, 48, 12, a=255),
    PySide6.QtGui.QColor(66, 49, 12, a=255),
    PySide6.QtGui.QColor(67, 50, 11, a=255),
    PySide6.QtGui.QColor(68, 50, 11, a=255),
    PySide6.QtGui.QColor(69, 51, 10, a=255),
    PySide6.QtGui.QColor(70, 52, 10, a=255),
    PySide6.QtGui.QColor(71, 53, 10, a=255),
    PySide6.QtGui.QColor(72, 53, 9, a=255),
    PySide6.QtGui.QColor(73, 54, 9, a=255),
    PySide6.QtGui.QColor(74, 55, 8, a=255),
    PySide6.QtGui.QColor(75, 56, 8, a=255),
    PySide6.QtGui.QColor(77, 56, 7, a=255),
    PySide6.QtGui.QColor(78, 57, 7, a=255),
    PySide6.QtGui.QColor(79, 58, 7, a=255),
    PySide6.QtGui.QColor(80, 59, 6, a=255),
    PySide6.QtGui.QColor(81, 59, 6, a=255),
    PySide6.QtGui.QColor(82, 60, 5, a=255),
    PySide6.QtGui.QColor(83, 61, 5, a=255),
    PySide6.QtGui.QColor(84, 62, 5, a=255),
    PySide6.QtGui.QColor(85, 63, 4, a=255),
    PySide6.QtGui.QColor(86, 63, 4, a=255),
    PySide6.QtGui.QColor(87, 64, 4, a=255),
    PySide6.QtGui.QColor(88, 65, 3, a=255),
    PySide6.QtGui.QColor(89, 66, 3, a=255),
    PySide6.QtGui.QColor(90, 66, 2, a=255),
    PySide6.QtGui.QColor(91, 67, 2, a=255),
    PySide6.QtGui.QColor(93, 68, 2, a=255),
    PySide6.QtGui.QColor(94, 69, 1, a=255),
    PySide6.QtGui.QColor(95, 70, 1, a=255),
    PySide6.QtGui.QColor(96, 70, 1, a=255),
    PySide6.QtGui.QColor(97, 71, 0, a=255),
    PySide6.QtGui.QColor(98, 72, 0, a=255),
    PySide6.QtGui.QColor(99, 73, 0, a=255),
    PySide6.QtGui.QColor(100, 74, 0, a=255),
    PySide6.QtGui.QColor(101, 74, 0, a=255),
    PySide6.QtGui.QColor(102, 75, 0, a=255),
    PySide6.QtGui.QColor(103, 76, 0, a=255),
    PySide6.QtGui.QColor(104, 77, 0, a=255),
    PySide6.QtGui.QColor(105, 78, 0, a=255),
    PySide6.QtGui.QColor(106, 78, 0, a=255),
    PySide6.QtGui.QColor(107, 79, 0, a=255),
    PySide6.QtGui.QColor(108, 80, 0, a=255),
    PySide6.QtGui.QColor(109, 81, 0, a=255),
    PySide6.QtGui.QColor(111, 82, 0, a=255),
    PySide6.QtGui.QColor(112, 83, 0, a=255),
    PySide6.QtGui.QColor(113, 83, 0, a=255),
    PySide6.QtGui.QColor(114, 84, 0, a=255),
    PySide6.QtGui.QColor(115, 85, 0, a=255),
    PySide6.QtGui.QColor(116, 86, 0, a=255),
    PySide6.QtGui.QColor(117, 87, 0, a=255),
    PySide6.QtGui.QColor(118, 88, 0, a=255),
    PySide6.QtGui.QColor(119, 88, 0, a=255),
    PySide6.QtGui.QColor(120, 89, 0, a=255),
    PySide6.QtGui.QColor(121, 90, 0, a=255),
    PySide6.QtGui.QColor(122, 91, 0, a=255),
    PySide6.QtGui.QColor(123, 92, 0, a=255),
    PySide6.QtGui.QColor(124, 93, 0, a=255),
    PySide6.QtGui.QColor(125, 93, 0, a=255),
    PySide6.QtGui.QColor(126, 94, 0, a=255),
    PySide6.QtGui.QColor(127, 95, 0, a=255),
    PySide6.QtGui.QColor(128, 96, 0, a=255),
    PySide6.QtGui.QColor(129, 97, 0, a=255),
    PySide6.QtGui.QColor(130, 98, 0, a=255),
    PySide6.QtGui.QColor(132, 99, 0, a=255),
    PySide6.QtGui.QColor(133, 99, 0, a=255),
    PySide6.QtGui.QColor(134, 100, 0, a=255),
    PySide6.QtGui.QColor(135, 101, 0, a=255),
    PySide6.QtGui.QColor(136, 102, 0, a=255),
    PySide6.QtGui.QColor(137, 103, 0, a=255),
    PySide6.QtGui.QColor(138, 104, 0, a=255),
    PySide6.QtGui.QColor(139, 105, 0, a=255),
    PySide6.QtGui.QColor(140, 106, 0, a=255),
    PySide6.QtGui.QColor(141, 106, 0, a=255),
    PySide6.QtGui.QColor(142, 107, 1, a=255),
    PySide6.QtGui.QColor(143, 108, 1, a=255),
    PySide6.QtGui.QColor(144, 109, 2, a=255),
    PySide6.QtGui.QColor(145, 110, 2, a=255),
    PySide6.QtGui.QColor(146, 111, 3, a=255),
    PySide6.QtGui.QColor(147, 112, 4, a=255),
    PySide6.QtGui.QColor(148, 113, 5, a=255),
    PySide6.QtGui.QColor(149, 114, 6, a=255),
    PySide6.QtGui.QColor(150, 114, 7, a=255),
    PySide6.QtGui.QColor(151, 115, 7, a=255),
    PySide6.QtGui.QColor(152, 116, 9, a=255),
    PySide6.QtGui.QColor(153, 117, 10, a=255),
    PySide6.QtGui.QColor(154, 118, 11, a=255),
    PySide6.QtGui.QColor(155, 119, 12, a=255),
    PySide6.QtGui.QColor(156, 120, 13, a=255),
    PySide6.QtGui.QColor(157, 121, 14, a=255),
    PySide6.QtGui.QColor(158, 122, 15, a=255),
    PySide6.QtGui.QColor(159, 123, 16, a=255),
    PySide6.QtGui.QColor(160, 124, 17, a=255),
    PySide6.QtGui.QColor(161, 124, 18, a=255),
    PySide6.QtGui.QColor(162, 125, 20, a=255),
    PySide6.QtGui.QColor(163, 126, 21, a=255),
    PySide6.QtGui.QColor(164, 127, 22, a=255),
    PySide6.QtGui.QColor(165, 128, 23, a=255),
    PySide6.QtGui.QColor(166, 129, 24, a=255),
    PySide6.QtGui.QColor(167, 130, 25, a=255),
    PySide6.QtGui.QColor(168, 131, 26, a=255),
    PySide6.QtGui.QColor(169, 132, 27, a=255),
    PySide6.QtGui.QColor(170, 133, 28, a=255),
    PySide6.QtGui.QColor(171, 134, 29, a=255),
    PySide6.QtGui.QColor(172, 135, 31, a=255),
    PySide6.QtGui.QColor(173, 136, 32, a=255),
    PySide6.QtGui.QColor(174, 137, 33, a=255),
    PySide6.QtGui.QColor(175, 138, 34, a=255),
    PySide6.QtGui.QColor(176, 138, 35, a=255),
    PySide6.QtGui.QColor(177, 139, 36, a=255),
    PySide6.QtGui.QColor(178, 140, 37, a=255),
    PySide6.QtGui.QColor(179, 141, 39, a=255),
    PySide6.QtGui.QColor(180, 142, 40, a=255),
    PySide6.QtGui.QColor(181, 143, 41, a=255),
    PySide6.QtGui.QColor(182, 144, 42, a=255),
    PySide6.QtGui.QColor(182, 145, 43, a=255),
    PySide6.QtGui.QColor(183, 146, 44, a=255),
    PySide6.QtGui.QColor(184, 147, 46, a=255),
    PySide6.QtGui.QColor(185, 148, 47, a=255),
    PySide6.QtGui.QColor(186, 149, 48, a=255),
    PySide6.QtGui.QColor(187, 150, 49, a=255),
    PySide6.QtGui.QColor(188, 151, 51, a=255),
    PySide6.QtGui.QColor(189, 152, 52, a=255),
    PySide6.QtGui.QColor(190, 153, 53, a=255),
    PySide6.QtGui.QColor(191, 154, 54, a=255),
    PySide6.QtGui.QColor(192, 155, 56, a=255),
    PySide6.QtGui.QColor(193, 156, 57, a=255),
    PySide6.QtGui.QColor(194, 157, 58, a=255),
    PySide6.QtGui.QColor(194, 158, 59, a=255),
    PySide6.QtGui.QColor(195, 159, 61, a=255),
    PySide6.QtGui.QColor(196, 160, 62, a=255),
    PySide6.QtGui.QColor(197, 161, 63, a=255),
    PySide6.QtGui.QColor(198, 162, 65, a=255),
    PySide6.QtGui.QColor(199, 163, 66, a=255),
    PySide6.QtGui.QColor(200, 164, 67, a=255),
    PySide6.QtGui.QColor(201, 165, 69, a=255),
    PySide6.QtGui.QColor(201, 166, 70, a=255),
    PySide6.QtGui.QColor(202, 167, 71, a=255),
    PySide6.QtGui.QColor(203, 168, 73, a=255),
    PySide6.QtGui.QColor(204, 169, 74, a=255),
    PySide6.QtGui.QColor(205, 170, 76, a=255),
    PySide6.QtGui.QColor(206, 171, 77, a=255),
    PySide6.QtGui.QColor(207, 172, 79, a=255),
    PySide6.QtGui.QColor(207, 173, 80, a=255),
    PySide6.QtGui.QColor(208, 174, 81, a=255),
    PySide6.QtGui.QColor(209, 175, 83, a=255),
    PySide6.QtGui.QColor(210, 176, 84, a=255),
    PySide6.QtGui.QColor(211, 177, 86, a=255),
    PySide6.QtGui.QColor(212, 178, 87, a=255),
    PySide6.QtGui.QColor(212, 179, 89, a=255),
    PySide6.QtGui.QColor(213, 180, 90, a=255),
    PySide6.QtGui.QColor(214, 181, 92, a=255),
    PySide6.QtGui.QColor(215, 182, 93, a=255),
    PySide6.QtGui.QColor(215, 183, 95, a=255),
    PySide6.QtGui.QColor(216, 184, 96, a=255),
    PySide6.QtGui.QColor(217, 185, 98, a=255),
    PySide6.QtGui.QColor(218, 186, 100, a=255),
    PySide6.QtGui.QColor(219, 187, 101, a=255),
    PySide6.QtGui.QColor(219, 188, 103, a=255),
    PySide6.QtGui.QColor(220, 189, 104, a=255),
    PySide6.QtGui.QColor(221, 190, 106, a=255),
    PySide6.QtGui.QColor(222, 191, 108, a=255),
    PySide6.QtGui.QColor(222, 192, 109, a=255),
    PySide6.QtGui.QColor(223, 193, 111, a=255),
    PySide6.QtGui.QColor(224, 195, 113, a=255),
    PySide6.QtGui.QColor(224, 196, 114, a=255),
    PySide6.QtGui.QColor(225, 197, 116, a=255),
    PySide6.QtGui.QColor(226, 198, 118, a=255),
    PySide6.QtGui.QColor(226, 199, 119, a=255),
    PySide6.QtGui.QColor(227, 200, 121, a=255),
    PySide6.QtGui.QColor(228, 201, 123, a=255),
    PySide6.QtGui.QColor(228, 202, 125, a=255),
    PySide6.QtGui.QColor(229, 203, 126, a=255),
    PySide6.QtGui.QColor(230, 204, 128, a=255),
    PySide6.QtGui.QColor(230, 205, 130, a=255),
    PySide6.QtGui.QColor(231, 206, 132, a=255),
    PySide6.QtGui.QColor(232, 207, 134, a=255),
    PySide6.QtGui.QColor(232, 208, 135, a=255),
    PySide6.QtGui.QColor(233, 209, 137, a=255),
    PySide6.QtGui.QColor(233, 211, 139, a=255),
    PySide6.QtGui.QColor(234, 212, 141, a=255),
    PySide6.QtGui.QColor(235, 213, 143, a=255),
    PySide6.QtGui.QColor(235, 214, 145, a=255),
    PySide6.QtGui.QColor(236, 215, 147, a=255),
    PySide6.QtGui.QColor(236, 216, 149, a=255),
    PySide6.QtGui.QColor(237, 217, 150, a=255),
    PySide6.QtGui.QColor(237, 218, 152, a=255),
    PySide6.QtGui.QColor(238, 219, 154, a=255),
    PySide6.QtGui.QColor(238, 220, 156, a=255),
    PySide6.QtGui.QColor(239, 222, 158, a=255),
    PySide6.QtGui.QColor(239, 223, 160, a=255),
    PySide6.QtGui.QColor(240, 224, 162, a=255),
    PySide6.QtGui.QColor(240, 225, 164, a=255),
    PySide6.QtGui.QColor(241, 226, 166, a=255),
    PySide6.QtGui.QColor(241, 227, 168, a=255),
    PySide6.QtGui.QColor(242, 228, 170, a=255),
    PySide6.QtGui.QColor(242, 229, 172, a=255),
    PySide6.QtGui.QColor(243, 230, 175, a=255),
    PySide6.QtGui.QColor(243, 232, 177, a=255),
    PySide6.QtGui.QColor(243, 233, 179, a=255),
    PySide6.QtGui.QColor(244, 234, 181, a=255),
    PySide6.QtGui.QColor(244, 235, 183, a=255),
    PySide6.QtGui.QColor(245, 236, 185, a=255),
    PySide6.QtGui.QColor(245, 237, 187, a=255),
    PySide6.QtGui.QColor(245, 238, 190, a=255),
    PySide6.QtGui.QColor(245, 239, 192, a=255),
    PySide6.QtGui.QColor(246, 241, 194, a=255),
    PySide6.QtGui.QColor(246, 242, 196, a=255),
    PySide6.QtGui.QColor(246, 243, 198, a=255),
    PySide6.QtGui.QColor(247, 244, 201, a=255),
    PySide6.QtGui.QColor(247, 245, 203, a=255),
    PySide6.QtGui.QColor(247, 246, 205, a=255),
    PySide6.QtGui.QColor(247, 247, 207, a=255),
    PySide6.QtGui.QColor(248, 249, 210, a=255),
    PySide6.QtGui.QColor(248, 250, 212, a=255),
    PySide6.QtGui.QColor(248, 251, 214, a=255),
    PySide6.QtGui.QColor(248, 252, 217, a=255),
    PySide6.QtGui.QColor(248, 253, 219, a=255),
    PySide6.QtGui.QColor(248, 254, 221, a=255),
    PySide6.QtGui.QColor(248, 255, 224, a=255),
    PySide6.QtGui.QColor(249, 255, 226, a=255),
    PySide6.QtGui.QColor(249, 255, 229, a=255),
    PySide6.QtGui.QColor(249, 255, 231, a=255),
]
DEFAULT_OFF_COLORMAP: list[PySide6.QtGui.QColor] = [
    PySide6.QtGui.QColor(25, 25, 25, a=255),
    PySide6.QtGui.QColor(25, 25, 26, a=255),
    PySide6.QtGui.QColor(26, 24, 27, a=255),
    PySide6.QtGui.QColor(26, 24, 28, a=255),
    PySide6.QtGui.QColor(26, 24, 29, a=255),
    PySide6.QtGui.QColor(27, 23, 29, a=255),
    PySide6.QtGui.QColor(27, 23, 30, a=255),
    PySide6.QtGui.QColor(27, 23, 31, a=255),
    PySide6.QtGui.QColor(27, 22, 32, a=255),
    PySide6.QtGui.QColor(27, 22, 33, a=255),
    PySide6.QtGui.QColor(28, 21, 34, a=255),
    PySide6.QtGui.QColor(28, 21, 34, a=255),
    PySide6.QtGui.QColor(28, 21, 35, a=255),
    PySide6.QtGui.QColor(28, 20, 36, a=255),
    PySide6.QtGui.QColor(28, 20, 37, a=255),
    PySide6.QtGui.QColor(28, 20, 38, a=255),
    PySide6.QtGui.QColor(29, 19, 39, a=255),
    PySide6.QtGui.QColor(29, 19, 39, a=255),
    PySide6.QtGui.QColor(29, 19, 40, a=255),
    PySide6.QtGui.QColor(29, 18, 41, a=255),
    PySide6.QtGui.QColor(29, 18, 42, a=255),
    PySide6.QtGui.QColor(29, 18, 43, a=255),
    PySide6.QtGui.QColor(29, 17, 43, a=255),
    PySide6.QtGui.QColor(29, 17, 44, a=255),
    PySide6.QtGui.QColor(29, 17, 45, a=255),
    PySide6.QtGui.QColor(29, 16, 46, a=255),
    PySide6.QtGui.QColor(29, 16, 47, a=255),
    PySide6.QtGui.QColor(29, 16, 47, a=255),
    PySide6.QtGui.QColor(29, 15, 48, a=255),
    PySide6.QtGui.QColor(29, 15, 49, a=255),
    PySide6.QtGui.QColor(29, 15, 50, a=255),
    PySide6.QtGui.QColor(29, 14, 51, a=255),
    PySide6.QtGui.QColor(29, 14, 51, a=255),
    PySide6.QtGui.QColor(29, 14, 52, a=255),
    PySide6.QtGui.QColor(29, 14, 53, a=255),
    PySide6.QtGui.QColor(29, 13, 54, a=255),
    PySide6.QtGui.QColor(29, 13, 55, a=255),
    PySide6.QtGui.QColor(29, 13, 55, a=255),
    PySide6.QtGui.QColor(29, 12, 56, a=255),
    PySide6.QtGui.QColor(29, 12, 57, a=255),
    PySide6.QtGui.QColor(29, 12, 58, a=255),
    PySide6.QtGui.QColor(29, 12, 58, a=255),
    PySide6.QtGui.QColor(29, 11, 59, a=255),
    PySide6.QtGui.QColor(28, 11, 60, a=255),
    PySide6.QtGui.QColor(28, 11, 61, a=255),
    PySide6.QtGui.QColor(28, 11, 62, a=255),
    PySide6.QtGui.QColor(28, 10, 62, a=255),
    PySide6.QtGui.QColor(28, 10, 63, a=255),
    PySide6.QtGui.QColor(28, 10, 64, a=255),
    PySide6.QtGui.QColor(28, 10, 65, a=255),
    PySide6.QtGui.QColor(27, 9, 65, a=255),
    PySide6.QtGui.QColor(27, 9, 66, a=255),
    PySide6.QtGui.QColor(27, 9, 67, a=255),
    PySide6.QtGui.QColor(27, 9, 68, a=255),
    PySide6.QtGui.QColor(27, 9, 68, a=255),
    PySide6.QtGui.QColor(26, 9, 69, a=255),
    PySide6.QtGui.QColor(26, 8, 70, a=255),
    PySide6.QtGui.QColor(26, 8, 71, a=255),
    PySide6.QtGui.QColor(26, 8, 71, a=255),
    PySide6.QtGui.QColor(25, 8, 72, a=255),
    PySide6.QtGui.QColor(25, 8, 73, a=255),
    PySide6.QtGui.QColor(25, 8, 74, a=255),
    PySide6.QtGui.QColor(24, 8, 74, a=255),
    PySide6.QtGui.QColor(24, 8, 75, a=255),
    PySide6.QtGui.QColor(24, 7, 76, a=255),
    PySide6.QtGui.QColor(23, 7, 77, a=255),
    PySide6.QtGui.QColor(23, 7, 77, a=255),
    PySide6.QtGui.QColor(23, 7, 78, a=255),
    PySide6.QtGui.QColor(22, 7, 79, a=255),
    PySide6.QtGui.QColor(22, 7, 79, a=255),
    PySide6.QtGui.QColor(22, 7, 80, a=255),
    PySide6.QtGui.QColor(21, 7, 81, a=255),
    PySide6.QtGui.QColor(21, 7, 82, a=255),
    PySide6.QtGui.QColor(20, 7, 82, a=255),
    PySide6.QtGui.QColor(20, 7, 83, a=255),
    PySide6.QtGui.QColor(19, 7, 84, a=255),
    PySide6.QtGui.QColor(19, 7, 84, a=255),
    PySide6.QtGui.QColor(18, 7, 85, a=255),
    PySide6.QtGui.QColor(18, 7, 86, a=255),
    PySide6.QtGui.QColor(17, 7, 87, a=255),
    PySide6.QtGui.QColor(17, 7, 87, a=255),
    PySide6.QtGui.QColor(16, 8, 88, a=255),
    PySide6.QtGui.QColor(15, 8, 89, a=255),
    PySide6.QtGui.QColor(15, 8, 89, a=255),
    PySide6.QtGui.QColor(14, 8, 90, a=255),
    PySide6.QtGui.QColor(13, 8, 91, a=255),
    PySide6.QtGui.QColor(12, 8, 91, a=255),
    PySide6.QtGui.QColor(12, 8, 92, a=255),
    PySide6.QtGui.QColor(11, 9, 93, a=255),
    PySide6.QtGui.QColor(10, 9, 94, a=255),
    PySide6.QtGui.QColor(9, 9, 94, a=255),
    PySide6.QtGui.QColor(8, 9, 95, a=255),
    PySide6.QtGui.QColor(7, 9, 96, a=255),
    PySide6.QtGui.QColor(6, 10, 96, a=255),
    PySide6.QtGui.QColor(5, 10, 97, a=255),
    PySide6.QtGui.QColor(4, 10, 98, a=255),
    PySide6.QtGui.QColor(3, 10, 98, a=255),
    PySide6.QtGui.QColor(2, 11, 99, a=255),
    PySide6.QtGui.QColor(1, 11, 100, a=255),
    PySide6.QtGui.QColor(0, 11, 100, a=255),
    PySide6.QtGui.QColor(0, 12, 101, a=255),
    PySide6.QtGui.QColor(0, 12, 102, a=255),
    PySide6.QtGui.QColor(0, 12, 102, a=255),
    PySide6.QtGui.QColor(0, 13, 103, a=255),
    PySide6.QtGui.QColor(0, 13, 104, a=255),
    PySide6.QtGui.QColor(0, 13, 104, a=255),
    PySide6.QtGui.QColor(0, 14, 105, a=255),
    PySide6.QtGui.QColor(0, 14, 106, a=255),
    PySide6.QtGui.QColor(0, 15, 106, a=255),
    PySide6.QtGui.QColor(0, 15, 107, a=255),
    PySide6.QtGui.QColor(0, 15, 107, a=255),
    PySide6.QtGui.QColor(0, 16, 108, a=255),
    PySide6.QtGui.QColor(0, 16, 109, a=255),
    PySide6.QtGui.QColor(0, 17, 109, a=255),
    PySide6.QtGui.QColor(0, 17, 110, a=255),
    PySide6.QtGui.QColor(0, 17, 111, a=255),
    PySide6.QtGui.QColor(0, 18, 111, a=255),
    PySide6.QtGui.QColor(0, 18, 112, a=255),
    PySide6.QtGui.QColor(0, 19, 112, a=255),
    PySide6.QtGui.QColor(0, 19, 113, a=255),
    PySide6.QtGui.QColor(0, 20, 114, a=255),
    PySide6.QtGui.QColor(0, 20, 114, a=255),
    PySide6.QtGui.QColor(0, 21, 115, a=255),
    PySide6.QtGui.QColor(0, 21, 116, a=255),
    PySide6.QtGui.QColor(0, 22, 116, a=255),
    PySide6.QtGui.QColor(0, 22, 117, a=255),
    PySide6.QtGui.QColor(0, 23, 117, a=255),
    PySide6.QtGui.QColor(0, 23, 118, a=255),
    PySide6.QtGui.QColor(0, 24, 119, a=255),
    PySide6.QtGui.QColor(0, 24, 119, a=255),
    PySide6.QtGui.QColor(0, 25, 120, a=255),
    PySide6.QtGui.QColor(0, 25, 120, a=255),
    PySide6.QtGui.QColor(0, 26, 121, a=255),
    PySide6.QtGui.QColor(0, 26, 121, a=255),
    PySide6.QtGui.QColor(0, 27, 122, a=255),
    PySide6.QtGui.QColor(0, 27, 123, a=255),
    PySide6.QtGui.QColor(0, 28, 123, a=255),
    PySide6.QtGui.QColor(0, 28, 124, a=255),
    PySide6.QtGui.QColor(0, 29, 124, a=255),
    PySide6.QtGui.QColor(0, 29, 125, a=255),
    PySide6.QtGui.QColor(0, 30, 125, a=255),
    PySide6.QtGui.QColor(0, 31, 126, a=255),
    PySide6.QtGui.QColor(0, 31, 127, a=255),
    PySide6.QtGui.QColor(0, 32, 127, a=255),
    PySide6.QtGui.QColor(0, 32, 128, a=255),
    PySide6.QtGui.QColor(0, 33, 128, a=255),
    PySide6.QtGui.QColor(0, 33, 129, a=255),
    PySide6.QtGui.QColor(0, 34, 129, a=255),
    PySide6.QtGui.QColor(0, 35, 130, a=255),
    PySide6.QtGui.QColor(0, 35, 130, a=255),
    PySide6.QtGui.QColor(0, 36, 131, a=255),
    PySide6.QtGui.QColor(0, 36, 131, a=255),
    PySide6.QtGui.QColor(0, 37, 132, a=255),
    PySide6.QtGui.QColor(0, 38, 133, a=255),
    PySide6.QtGui.QColor(0, 38, 133, a=255),
    PySide6.QtGui.QColor(0, 39, 134, a=255),
    PySide6.QtGui.QColor(0, 40, 134, a=255),
    PySide6.QtGui.QColor(0, 40, 135, a=255),
    PySide6.QtGui.QColor(0, 41, 135, a=255),
    PySide6.QtGui.QColor(0, 42, 136, a=255),
    PySide6.QtGui.QColor(0, 42, 136, a=255),
    PySide6.QtGui.QColor(0, 43, 137, a=255),
    PySide6.QtGui.QColor(0, 44, 137, a=255),
    PySide6.QtGui.QColor(0, 44, 138, a=255),
    PySide6.QtGui.QColor(0, 45, 138, a=255),
    PySide6.QtGui.QColor(0, 46, 139, a=255),
    PySide6.QtGui.QColor(0, 46, 139, a=255),
    PySide6.QtGui.QColor(0, 47, 140, a=255),
    PySide6.QtGui.QColor(0, 48, 140, a=255),
    PySide6.QtGui.QColor(0, 48, 141, a=255),
    PySide6.QtGui.QColor(0, 49, 141, a=255),
    PySide6.QtGui.QColor(0, 50, 142, a=255),
    PySide6.QtGui.QColor(0, 51, 142, a=255),
    PySide6.QtGui.QColor(0, 51, 142, a=255),
    PySide6.QtGui.QColor(0, 52, 143, a=255),
    PySide6.QtGui.QColor(0, 53, 143, a=255),
    PySide6.QtGui.QColor(0, 54, 144, a=255),
    PySide6.QtGui.QColor(0, 54, 144, a=255),
    PySide6.QtGui.QColor(0, 55, 145, a=255),
    PySide6.QtGui.QColor(0, 56, 145, a=255),
    PySide6.QtGui.QColor(0, 57, 146, a=255),
    PySide6.QtGui.QColor(0, 57, 146, a=255),
    PySide6.QtGui.QColor(0, 58, 147, a=255),
    PySide6.QtGui.QColor(0, 59, 147, a=255),
    PySide6.QtGui.QColor(0, 60, 147, a=255),
    PySide6.QtGui.QColor(0, 60, 148, a=255),
    PySide6.QtGui.QColor(0, 61, 148, a=255),
    PySide6.QtGui.QColor(0, 62, 149, a=255),
    PySide6.QtGui.QColor(0, 63, 149, a=255),
    PySide6.QtGui.QColor(0, 64, 150, a=255),
    PySide6.QtGui.QColor(0, 64, 150, a=255),
    PySide6.QtGui.QColor(0, 65, 150, a=255),
    PySide6.QtGui.QColor(0, 66, 151, a=255),
    PySide6.QtGui.QColor(0, 67, 151, a=255),
    PySide6.QtGui.QColor(0, 68, 152, a=255),
    PySide6.QtGui.QColor(0, 69, 152, a=255),
    PySide6.QtGui.QColor(0, 69, 152, a=255),
    PySide6.QtGui.QColor(0, 70, 153, a=255),
    PySide6.QtGui.QColor(0, 71, 153, a=255),
    PySide6.QtGui.QColor(0, 72, 153, a=255),
    PySide6.QtGui.QColor(0, 73, 154, a=255),
    PySide6.QtGui.QColor(0, 74, 154, a=255),
    PySide6.QtGui.QColor(0, 75, 155, a=255),
    PySide6.QtGui.QColor(0, 76, 155, a=255),
    PySide6.QtGui.QColor(0, 76, 155, a=255),
    PySide6.QtGui.QColor(0, 77, 156, a=255),
    PySide6.QtGui.QColor(0, 78, 156, a=255),
    PySide6.QtGui.QColor(0, 79, 156, a=255),
    PySide6.QtGui.QColor(0, 80, 157, a=255),
    PySide6.QtGui.QColor(0, 81, 157, a=255),
    PySide6.QtGui.QColor(0, 82, 157, a=255),
    PySide6.QtGui.QColor(0, 83, 158, a=255),
    PySide6.QtGui.QColor(0, 84, 158, a=255),
    PySide6.QtGui.QColor(0, 85, 158, a=255),
    PySide6.QtGui.QColor(0, 86, 159, a=255),
    PySide6.QtGui.QColor(0, 87, 159, a=255),
    PySide6.QtGui.QColor(0, 88, 159, a=255),
    PySide6.QtGui.QColor(0, 89, 160, a=255),
    PySide6.QtGui.QColor(0, 90, 160, a=255),
    PySide6.QtGui.QColor(0, 91, 160, a=255),
    PySide6.QtGui.QColor(0, 92, 161, a=255),
    PySide6.QtGui.QColor(0, 93, 161, a=255),
    PySide6.QtGui.QColor(0, 94, 161, a=255),
    PySide6.QtGui.QColor(0, 95, 162, a=255),
    PySide6.QtGui.QColor(0, 96, 162, a=255),
    PySide6.QtGui.QColor(0, 97, 162, a=255),
    PySide6.QtGui.QColor(0, 98, 162, a=255),
    PySide6.QtGui.QColor(0, 99, 163, a=255),
    PySide6.QtGui.QColor(0, 100, 163, a=255),
    PySide6.QtGui.QColor(0, 101, 163, a=255),
    PySide6.QtGui.QColor(0, 102, 163, a=255),
    PySide6.QtGui.QColor(0, 103, 164, a=255),
    PySide6.QtGui.QColor(0, 104, 164, a=255),
    PySide6.QtGui.QColor(0, 105, 164, a=255),
    PySide6.QtGui.QColor(0, 106, 165, a=255),
    PySide6.QtGui.QColor(0, 107, 165, a=255),
    PySide6.QtGui.QColor(0, 108, 165, a=255),
    PySide6.QtGui.QColor(0, 109, 165, a=255),
    PySide6.QtGui.QColor(0, 110, 165, a=255),
    PySide6.QtGui.QColor(0, 112, 166, a=255),
    PySide6.QtGui.QColor(0, 113, 166, a=255),
    PySide6.QtGui.QColor(0, 114, 166, a=255),
    PySide6.QtGui.QColor(0, 115, 166, a=255),
    PySide6.QtGui.QColor(0, 116, 167, a=255),
    PySide6.QtGui.QColor(0, 117, 167, a=255),
    PySide6.QtGui.QColor(0, 118, 167, a=255),
    PySide6.QtGui.QColor(0, 119, 167, a=255),
    PySide6.QtGui.QColor(0, 121, 167, a=255),
    PySide6.QtGui.QColor(0, 122, 168, a=255),
    PySide6.QtGui.QColor(0, 123, 168, a=255),
    PySide6.QtGui.QColor(0, 124, 168, a=255),
    PySide6.QtGui.QColor(0, 125, 168, a=255),
    PySide6.QtGui.QColor(0, 127, 168, a=255),
    PySide6.QtGui.QColor(0, 128, 168, a=255),
    PySide6.QtGui.QColor(0, 129, 169, a=255),
    PySide6.QtGui.QColor(0, 130, 169, a=255),
]
DEFAULT_FRAME_COLORMAP: list[PySide6.QtGui.QColor] = [
    PySide6.QtGui.QColor(1, 25, 89, a=255),
    PySide6.QtGui.QColor(2, 27, 89, a=255),
    PySide6.QtGui.QColor(3, 28, 90, a=255),
    PySide6.QtGui.QColor(4, 30, 90, a=255),
    PySide6.QtGui.QColor(5, 31, 90, a=255),
    PySide6.QtGui.QColor(6, 33, 91, a=255),
    PySide6.QtGui.QColor(7, 34, 91, a=255),
    PySide6.QtGui.QColor(7, 36, 91, a=255),
    PySide6.QtGui.QColor(8, 37, 91, a=255),
    PySide6.QtGui.QColor(9, 39, 92, a=255),
    PySide6.QtGui.QColor(10, 40, 92, a=255),
    PySide6.QtGui.QColor(10, 42, 92, a=255),
    PySide6.QtGui.QColor(11, 43, 92, a=255),
    PySide6.QtGui.QColor(11, 45, 93, a=255),
    PySide6.QtGui.QColor(12, 46, 93, a=255),
    PySide6.QtGui.QColor(12, 47, 93, a=255),
    PySide6.QtGui.QColor(13, 49, 93, a=255),
    PySide6.QtGui.QColor(13, 50, 94, a=255),
    PySide6.QtGui.QColor(13, 51, 94, a=255),
    PySide6.QtGui.QColor(14, 53, 94, a=255),
    PySide6.QtGui.QColor(14, 54, 94, a=255),
    PySide6.QtGui.QColor(14, 55, 94, a=255),
    PySide6.QtGui.QColor(15, 56, 95, a=255),
    PySide6.QtGui.QColor(15, 57, 95, a=255),
    PySide6.QtGui.QColor(15, 59, 95, a=255),
    PySide6.QtGui.QColor(15, 60, 95, a=255),
    PySide6.QtGui.QColor(16, 61, 95, a=255),
    PySide6.QtGui.QColor(16, 62, 95, a=255),
    PySide6.QtGui.QColor(16, 63, 96, a=255),
    PySide6.QtGui.QColor(16, 64, 96, a=255),
    PySide6.QtGui.QColor(17, 65, 96, a=255),
    PySide6.QtGui.QColor(17, 66, 96, a=255),
    PySide6.QtGui.QColor(17, 67, 96, a=255),
    PySide6.QtGui.QColor(17, 68, 96, a=255),
    PySide6.QtGui.QColor(18, 69, 97, a=255),
    PySide6.QtGui.QColor(18, 70, 97, a=255),
    PySide6.QtGui.QColor(18, 71, 97, a=255),
    PySide6.QtGui.QColor(18, 72, 97, a=255),
    PySide6.QtGui.QColor(19, 73, 97, a=255),
    PySide6.QtGui.QColor(19, 74, 97, a=255),
    PySide6.QtGui.QColor(19, 75, 97, a=255),
    PySide6.QtGui.QColor(20, 76, 98, a=255),
    PySide6.QtGui.QColor(20, 77, 98, a=255),
    PySide6.QtGui.QColor(20, 78, 98, a=255),
    PySide6.QtGui.QColor(21, 79, 98, a=255),
    PySide6.QtGui.QColor(21, 79, 98, a=255),
    PySide6.QtGui.QColor(22, 80, 98, a=255),
    PySide6.QtGui.QColor(22, 81, 98, a=255),
    PySide6.QtGui.QColor(23, 82, 98, a=255),
    PySide6.QtGui.QColor(23, 83, 98, a=255),
    PySide6.QtGui.QColor(24, 84, 98, a=255),
    PySide6.QtGui.QColor(24, 85, 98, a=255),
    PySide6.QtGui.QColor(25, 86, 98, a=255),
    PySide6.QtGui.QColor(25, 87, 98, a=255),
    PySide6.QtGui.QColor(26, 87, 98, a=255),
    PySide6.QtGui.QColor(27, 88, 98, a=255),
    PySide6.QtGui.QColor(27, 89, 98, a=255),
    PySide6.QtGui.QColor(28, 90, 98, a=255),
    PySide6.QtGui.QColor(29, 91, 98, a=255),
    PySide6.QtGui.QColor(30, 92, 98, a=255),
    PySide6.QtGui.QColor(30, 93, 98, a=255),
    PySide6.QtGui.QColor(31, 93, 97, a=255),
    PySide6.QtGui.QColor(32, 94, 97, a=255),
    PySide6.QtGui.QColor(33, 95, 97, a=255),
    PySide6.QtGui.QColor(34, 96, 97, a=255),
    PySide6.QtGui.QColor(35, 96, 96, a=255),
    PySide6.QtGui.QColor(36, 97, 96, a=255),
    PySide6.QtGui.QColor(37, 98, 96, a=255),
    PySide6.QtGui.QColor(38, 99, 95, a=255),
    PySide6.QtGui.QColor(39, 99, 95, a=255),
    PySide6.QtGui.QColor(40, 100, 95, a=255),
    PySide6.QtGui.QColor(42, 101, 94, a=255),
    PySide6.QtGui.QColor(43, 101, 94, a=255),
    PySide6.QtGui.QColor(44, 102, 93, a=255),
    PySide6.QtGui.QColor(45, 103, 93, a=255),
    PySide6.QtGui.QColor(47, 103, 92, a=255),
    PySide6.QtGui.QColor(48, 104, 92, a=255),
    PySide6.QtGui.QColor(49, 105, 91, a=255),
    PySide6.QtGui.QColor(51, 105, 90, a=255),
    PySide6.QtGui.QColor(52, 106, 90, a=255),
    PySide6.QtGui.QColor(53, 106, 89, a=255),
    PySide6.QtGui.QColor(55, 107, 88, a=255),
    PySide6.QtGui.QColor(56, 108, 88, a=255),
    PySide6.QtGui.QColor(58, 108, 87, a=255),
    PySide6.QtGui.QColor(59, 109, 86, a=255),
    PySide6.QtGui.QColor(60, 109, 86, a=255),
    PySide6.QtGui.QColor(62, 110, 85, a=255),
    PySide6.QtGui.QColor(63, 110, 84, a=255),
    PySide6.QtGui.QColor(65, 111, 83, a=255),
    PySide6.QtGui.QColor(66, 111, 82, a=255),
    PySide6.QtGui.QColor(68, 112, 82, a=255),
    PySide6.QtGui.QColor(69, 112, 81, a=255),
    PySide6.QtGui.QColor(71, 113, 80, a=255),
    PySide6.QtGui.QColor(72, 113, 79, a=255),
    PySide6.QtGui.QColor(74, 114, 78, a=255),
    PySide6.QtGui.QColor(76, 114, 77, a=255),
    PySide6.QtGui.QColor(77, 115, 77, a=255),
    PySide6.QtGui.QColor(79, 115, 76, a=255),
    PySide6.QtGui.QColor(80, 116, 75, a=255),
    PySide6.QtGui.QColor(82, 116, 74, a=255),
    PySide6.QtGui.QColor(83, 117, 73, a=255),
    PySide6.QtGui.QColor(85, 117, 72, a=255),
    PySide6.QtGui.QColor(87, 118, 71, a=255),
    PySide6.QtGui.QColor(88, 118, 70, a=255),
    PySide6.QtGui.QColor(90, 119, 69, a=255),
    PySide6.QtGui.QColor(91, 119, 69, a=255),
    PySide6.QtGui.QColor(93, 120, 68, a=255),
    PySide6.QtGui.QColor(95, 120, 67, a=255),
    PySide6.QtGui.QColor(96, 121, 66, a=255),
    PySide6.QtGui.QColor(98, 121, 65, a=255),
    PySide6.QtGui.QColor(99, 122, 64, a=255),
    PySide6.QtGui.QColor(101, 122, 63, a=255),
    PySide6.QtGui.QColor(103, 123, 62, a=255),
    PySide6.QtGui.QColor(104, 123, 62, a=255),
    PySide6.QtGui.QColor(106, 123, 61, a=255),
    PySide6.QtGui.QColor(108, 124, 60, a=255),
    PySide6.QtGui.QColor(109, 124, 59, a=255),
    PySide6.QtGui.QColor(111, 125, 58, a=255),
    PySide6.QtGui.QColor(113, 125, 57, a=255),
    PySide6.QtGui.QColor(115, 126, 56, a=255),
    PySide6.QtGui.QColor(116, 126, 56, a=255),
    PySide6.QtGui.QColor(118, 127, 55, a=255),
    PySide6.QtGui.QColor(120, 127, 54, a=255),
    PySide6.QtGui.QColor(121, 128, 53, a=255),
    PySide6.QtGui.QColor(123, 128, 52, a=255),
    PySide6.QtGui.QColor(125, 129, 52, a=255),
    PySide6.QtGui.QColor(127, 129, 51, a=255),
    PySide6.QtGui.QColor(129, 130, 50, a=255),
    PySide6.QtGui.QColor(130, 130, 49, a=255),
    PySide6.QtGui.QColor(132, 131, 49, a=255),
    PySide6.QtGui.QColor(134, 131, 48, a=255),
    PySide6.QtGui.QColor(136, 132, 47, a=255),
    PySide6.QtGui.QColor(138, 132, 47, a=255),
    PySide6.QtGui.QColor(140, 133, 46, a=255),
    PySide6.QtGui.QColor(142, 133, 46, a=255),
    PySide6.QtGui.QColor(143, 134, 45, a=255),
    PySide6.QtGui.QColor(145, 134, 45, a=255),
    PySide6.QtGui.QColor(147, 135, 44, a=255),
    PySide6.QtGui.QColor(149, 135, 44, a=255),
    PySide6.QtGui.QColor(151, 136, 44, a=255),
    PySide6.QtGui.QColor(153, 136, 44, a=255),
    PySide6.QtGui.QColor(155, 137, 43, a=255),
    PySide6.QtGui.QColor(157, 137, 43, a=255),
    PySide6.QtGui.QColor(159, 137, 43, a=255),
    PySide6.QtGui.QColor(161, 138, 43, a=255),
    PySide6.QtGui.QColor(163, 138, 44, a=255),
    PySide6.QtGui.QColor(165, 139, 44, a=255),
    PySide6.QtGui.QColor(167, 139, 44, a=255),
    PySide6.QtGui.QColor(169, 140, 44, a=255),
    PySide6.QtGui.QColor(171, 140, 45, a=255),
    PySide6.QtGui.QColor(173, 140, 45, a=255),
    PySide6.QtGui.QColor(175, 141, 46, a=255),
    PySide6.QtGui.QColor(177, 141, 47, a=255),
    PySide6.QtGui.QColor(179, 142, 47, a=255),
    PySide6.QtGui.QColor(181, 142, 48, a=255),
    PySide6.QtGui.QColor(183, 142, 49, a=255),
    PySide6.QtGui.QColor(185, 143, 50, a=255),
    PySide6.QtGui.QColor(187, 143, 51, a=255),
    PySide6.QtGui.QColor(189, 143, 52, a=255),
    PySide6.QtGui.QColor(190, 144, 53, a=255),
    PySide6.QtGui.QColor(192, 144, 54, a=255),
    PySide6.QtGui.QColor(194, 144, 55, a=255),
    PySide6.QtGui.QColor(196, 145, 56, a=255),
    PySide6.QtGui.QColor(198, 145, 58, a=255),
    PySide6.QtGui.QColor(200, 145, 59, a=255),
    PySide6.QtGui.QColor(202, 146, 60, a=255),
    PySide6.QtGui.QColor(203, 146, 62, a=255),
    PySide6.QtGui.QColor(205, 146, 63, a=255),
    PySide6.QtGui.QColor(207, 147, 64, a=255),
    PySide6.QtGui.QColor(209, 147, 66, a=255),
    PySide6.QtGui.QColor(210, 147, 67, a=255),
    PySide6.QtGui.QColor(212, 148, 69, a=255),
    PySide6.QtGui.QColor(214, 148, 70, a=255),
    PySide6.QtGui.QColor(216, 148, 72, a=255),
    PySide6.QtGui.QColor(217, 149, 74, a=255),
    PySide6.QtGui.QColor(219, 149, 75, a=255),
    PySide6.QtGui.QColor(221, 149, 77, a=255),
    PySide6.QtGui.QColor(222, 150, 79, a=255),
    PySide6.QtGui.QColor(224, 150, 81, a=255),
    PySide6.QtGui.QColor(225, 151, 82, a=255),
    PySide6.QtGui.QColor(227, 151, 84, a=255),
    PySide6.QtGui.QColor(228, 151, 86, a=255),
    PySide6.QtGui.QColor(230, 152, 88, a=255),
    PySide6.QtGui.QColor(231, 152, 90, a=255),
    PySide6.QtGui.QColor(233, 153, 92, a=255),
    PySide6.QtGui.QColor(234, 153, 94, a=255),
    PySide6.QtGui.QColor(235, 154, 96, a=255),
    PySide6.QtGui.QColor(237, 154, 98, a=255),
    PySide6.QtGui.QColor(238, 155, 100, a=255),
    PySide6.QtGui.QColor(239, 155, 103, a=255),
    PySide6.QtGui.QColor(240, 156, 105, a=255),
    PySide6.QtGui.QColor(241, 157, 107, a=255),
    PySide6.QtGui.QColor(242, 157, 109, a=255),
    PySide6.QtGui.QColor(243, 158, 112, a=255),
    PySide6.QtGui.QColor(244, 159, 114, a=255),
    PySide6.QtGui.QColor(245, 159, 116, a=255),
    PySide6.QtGui.QColor(246, 160, 119, a=255),
    PySide6.QtGui.QColor(247, 161, 121, a=255),
    PySide6.QtGui.QColor(248, 161, 123, a=255),
    PySide6.QtGui.QColor(248, 162, 126, a=255),
    PySide6.QtGui.QColor(249, 163, 128, a=255),
    PySide6.QtGui.QColor(249, 163, 130, a=255),
    PySide6.QtGui.QColor(250, 164, 133, a=255),
    PySide6.QtGui.QColor(250, 165, 135, a=255),
    PySide6.QtGui.QColor(251, 166, 137, a=255),
    PySide6.QtGui.QColor(251, 166, 140, a=255),
    PySide6.QtGui.QColor(252, 167, 142, a=255),
    PySide6.QtGui.QColor(252, 168, 144, a=255),
    PySide6.QtGui.QColor(252, 169, 147, a=255),
    PySide6.QtGui.QColor(252, 169, 149, a=255),
    PySide6.QtGui.QColor(253, 170, 151, a=255),
    PySide6.QtGui.QColor(253, 171, 154, a=255),
    PySide6.QtGui.QColor(253, 172, 156, a=255),
    PySide6.QtGui.QColor(253, 172, 158, a=255),
    PySide6.QtGui.QColor(253, 173, 160, a=255),
    PySide6.QtGui.QColor(253, 174, 162, a=255),
    PySide6.QtGui.QColor(253, 175, 165, a=255),
    PySide6.QtGui.QColor(253, 175, 167, a=255),
    PySide6.QtGui.QColor(253, 176, 169, a=255),
    PySide6.QtGui.QColor(253, 177, 171, a=255),
    PySide6.QtGui.QColor(253, 178, 173, a=255),
    PySide6.QtGui.QColor(253, 178, 175, a=255),
    PySide6.QtGui.QColor(253, 179, 177, a=255),
    PySide6.QtGui.QColor(253, 180, 180, a=255),
    PySide6.QtGui.QColor(253, 180, 182, a=255),
    PySide6.QtGui.QColor(253, 181, 184, a=255),
    PySide6.QtGui.QColor(253, 182, 186, a=255),
    PySide6.QtGui.QColor(253, 183, 188, a=255),
    PySide6.QtGui.QColor(253, 183, 190, a=255),
    PySide6.QtGui.QColor(253, 184, 192, a=255),
    PySide6.QtGui.QColor(253, 185, 194, a=255),
    PySide6.QtGui.QColor(253, 186, 196, a=255),
    PySide6.QtGui.QColor(253, 186, 199, a=255),
    PySide6.QtGui.QColor(253, 187, 201, a=255),
    PySide6.QtGui.QColor(253, 188, 203, a=255),
    PySide6.QtGui.QColor(253, 188, 205, a=255),
    PySide6.QtGui.QColor(252, 189, 207, a=255),
    PySide6.QtGui.QColor(252, 190, 209, a=255),
    PySide6.QtGui.QColor(252, 191, 211, a=255),
    PySide6.QtGui.QColor(252, 191, 214, a=255),
    PySide6.QtGui.QColor(252, 192, 216, a=255),
    PySide6.QtGui.QColor(252, 193, 218, a=255),
    PySide6.QtGui.QColor(252, 194, 220, a=255),
    PySide6.QtGui.QColor(252, 195, 223, a=255),
    PySide6.QtGui.QColor(252, 195, 225, a=255),
    PySide6.QtGui.QColor(252, 196, 227, a=255),
    PySide6.QtGui.QColor(252, 197, 229, a=255),
    PySide6.QtGui.QColor(251, 198, 232, a=255),
    PySide6.QtGui.QColor(251, 198, 234, a=255),
    PySide6.QtGui.QColor(251, 199, 236, a=255),
    PySide6.QtGui.QColor(251, 200, 239, a=255),
    PySide6.QtGui.QColor(251, 201, 241, a=255),
    PySide6.QtGui.QColor(251, 202, 243, a=255),
    PySide6.QtGui.QColor(251, 202, 246, a=255),
    PySide6.QtGui.QColor(250, 203, 248, a=255),
    PySide6.QtGui.QColor(250, 204, 250, a=255),
]
CYCLIC_COLORMAP: list[PySide6.QtGui.QColor] = [
    PySide6.QtGui.QColor(115, 57, 87, a=255),
    PySide6.QtGui.QColor(116, 57, 86, a=255),
    PySide6.QtGui.QColor(117, 57, 84, a=255),
    PySide6.QtGui.QColor(117, 56, 83, a=255),
    PySide6.QtGui.QColor(118, 56, 81, a=255),
    PySide6.QtGui.QColor(119, 56, 80, a=255),
    PySide6.QtGui.QColor(119, 56, 79, a=255),
    PySide6.QtGui.QColor(120, 56, 77, a=255),
    PySide6.QtGui.QColor(121, 56, 76, a=255),
    PySide6.QtGui.QColor(121, 56, 75, a=255),
    PySide6.QtGui.QColor(122, 56, 73, a=255),
    PySide6.QtGui.QColor(123, 56, 72, a=255),
    PySide6.QtGui.QColor(124, 56, 71, a=255),
    PySide6.QtGui.QColor(124, 57, 70, a=255),
    PySide6.QtGui.QColor(125, 57, 69, a=255),
    PySide6.QtGui.QColor(126, 57, 67, a=255),
    PySide6.QtGui.QColor(126, 57, 66, a=255),
    PySide6.QtGui.QColor(127, 58, 65, a=255),
    PySide6.QtGui.QColor(128, 58, 64, a=255),
    PySide6.QtGui.QColor(129, 59, 63, a=255),
    PySide6.QtGui.QColor(129, 59, 62, a=255),
    PySide6.QtGui.QColor(130, 60, 61, a=255),
    PySide6.QtGui.QColor(131, 60, 60, a=255),
    PySide6.QtGui.QColor(132, 61, 59, a=255),
    PySide6.QtGui.QColor(132, 61, 58, a=255),
    PySide6.QtGui.QColor(133, 62, 57, a=255),
    PySide6.QtGui.QColor(134, 63, 56, a=255),
    PySide6.QtGui.QColor(135, 64, 55, a=255),
    PySide6.QtGui.QColor(135, 64, 55, a=255),
    PySide6.QtGui.QColor(136, 65, 54, a=255),
    PySide6.QtGui.QColor(137, 66, 53, a=255),
    PySide6.QtGui.QColor(138, 67, 52, a=255),
    PySide6.QtGui.QColor(139, 68, 51, a=255),
    PySide6.QtGui.QColor(140, 69, 51, a=255),
    PySide6.QtGui.QColor(140, 70, 50, a=255),
    PySide6.QtGui.QColor(141, 71, 49, a=255),
    PySide6.QtGui.QColor(142, 72, 49, a=255),
    PySide6.QtGui.QColor(143, 73, 48, a=255),
    PySide6.QtGui.QColor(144, 74, 48, a=255),
    PySide6.QtGui.QColor(145, 76, 47, a=255),
    PySide6.QtGui.QColor(146, 77, 47, a=255),
    PySide6.QtGui.QColor(147, 78, 46, a=255),
    PySide6.QtGui.QColor(148, 80, 46, a=255),
    PySide6.QtGui.QColor(148, 81, 45, a=255),
    PySide6.QtGui.QColor(149, 82, 45, a=255),
    PySide6.QtGui.QColor(150, 84, 45, a=255),
    PySide6.QtGui.QColor(151, 85, 44, a=255),
    PySide6.QtGui.QColor(152, 87, 44, a=255),
    PySide6.QtGui.QColor(153, 88, 44, a=255),
    PySide6.QtGui.QColor(154, 90, 44, a=255),
    PySide6.QtGui.QColor(155, 91, 44, a=255),
    PySide6.QtGui.QColor(156, 93, 43, a=255),
    PySide6.QtGui.QColor(157, 95, 43, a=255),
    PySide6.QtGui.QColor(158, 96, 43, a=255),
    PySide6.QtGui.QColor(159, 98, 43, a=255),
    PySide6.QtGui.QColor(160, 100, 44, a=255),
    PySide6.QtGui.QColor(162, 102, 44, a=255),
    PySide6.QtGui.QColor(163, 103, 44, a=255),
    PySide6.QtGui.QColor(164, 105, 44, a=255),
    PySide6.QtGui.QColor(165, 107, 45, a=255),
    PySide6.QtGui.QColor(166, 109, 45, a=255),
    PySide6.QtGui.QColor(167, 111, 45, a=255),
    PySide6.QtGui.QColor(168, 113, 46, a=255),
    PySide6.QtGui.QColor(169, 115, 46, a=255),
    PySide6.QtGui.QColor(170, 117, 47, a=255),
    PySide6.QtGui.QColor(171, 119, 48, a=255),
    PySide6.QtGui.QColor(173, 121, 48, a=255),
    PySide6.QtGui.QColor(174, 123, 49, a=255),
    PySide6.QtGui.QColor(175, 125, 50, a=255),
    PySide6.QtGui.QColor(176, 127, 51, a=255),
    PySide6.QtGui.QColor(177, 129, 52, a=255),
    PySide6.QtGui.QColor(178, 131, 53, a=255),
    PySide6.QtGui.QColor(180, 134, 54, a=255),
    PySide6.QtGui.QColor(181, 136, 55, a=255),
    PySide6.QtGui.QColor(182, 138, 57, a=255),
    PySide6.QtGui.QColor(183, 140, 58, a=255),
    PySide6.QtGui.QColor(184, 142, 59, a=255),
    PySide6.QtGui.QColor(185, 145, 61, a=255),
    PySide6.QtGui.QColor(187, 147, 63, a=255),
    PySide6.QtGui.QColor(188, 149, 64, a=255),
    PySide6.QtGui.QColor(189, 152, 66, a=255),
    PySide6.QtGui.QColor(190, 154, 68, a=255),
    PySide6.QtGui.QColor(191, 156, 70, a=255),
    PySide6.QtGui.QColor(193, 159, 71, a=255),
    PySide6.QtGui.QColor(194, 161, 73, a=255),
    PySide6.QtGui.QColor(195, 163, 75, a=255),
    PySide6.QtGui.QColor(196, 165, 78, a=255),
    PySide6.QtGui.QColor(197, 168, 80, a=255),
    PySide6.QtGui.QColor(198, 170, 82, a=255),
    PySide6.QtGui.QColor(200, 172, 84, a=255),
    PySide6.QtGui.QColor(201, 175, 87, a=255),
    PySide6.QtGui.QColor(202, 177, 89, a=255),
    PySide6.QtGui.QColor(203, 179, 92, a=255),
    PySide6.QtGui.QColor(204, 181, 94, a=255),
    PySide6.QtGui.QColor(205, 183, 97, a=255),
    PySide6.QtGui.QColor(206, 186, 99, a=255),
    PySide6.QtGui.QColor(207, 188, 102, a=255),
    PySide6.QtGui.QColor(207, 190, 104, a=255),
    PySide6.QtGui.QColor(208, 192, 107, a=255),
    PySide6.QtGui.QColor(209, 194, 110, a=255),
    PySide6.QtGui.QColor(210, 196, 112, a=255),
    PySide6.QtGui.QColor(210, 198, 115, a=255),
    PySide6.QtGui.QColor(211, 200, 118, a=255),
    PySide6.QtGui.QColor(212, 201, 120, a=255),
    PySide6.QtGui.QColor(212, 203, 123, a=255),
    PySide6.QtGui.QColor(212, 205, 126, a=255),
    PySide6.QtGui.QColor(213, 206, 129, a=255),
    PySide6.QtGui.QColor(213, 208, 131, a=255),
    PySide6.QtGui.QColor(213, 209, 134, a=255),
    PySide6.QtGui.QColor(214, 211, 136, a=255),
    PySide6.QtGui.QColor(214, 212, 139, a=255),
    PySide6.QtGui.QColor(214, 213, 142, a=255),
    PySide6.QtGui.QColor(214, 215, 144, a=255),
    PySide6.QtGui.QColor(214, 216, 147, a=255),
    PySide6.QtGui.QColor(213, 217, 149, a=255),
    PySide6.QtGui.QColor(213, 218, 152, a=255),
    PySide6.QtGui.QColor(213, 219, 154, a=255),
    PySide6.QtGui.QColor(212, 220, 156, a=255),
    PySide6.QtGui.QColor(212, 221, 159, a=255),
    PySide6.QtGui.QColor(211, 221, 161, a=255),
    PySide6.QtGui.QColor(211, 222, 163, a=255),
    PySide6.QtGui.QColor(210, 223, 165, a=255),
    PySide6.QtGui.QColor(209, 223, 167, a=255),
    PySide6.QtGui.QColor(208, 224, 169, a=255),
    PySide6.QtGui.QColor(207, 224, 171, a=255),
    PySide6.QtGui.QColor(206, 224, 173, a=255),
    PySide6.QtGui.QColor(205, 225, 175, a=255),
    PySide6.QtGui.QColor(204, 225, 177, a=255),
    PySide6.QtGui.QColor(203, 225, 179, a=255),
    PySide6.QtGui.QColor(202, 225, 181, a=255),
    PySide6.QtGui.QColor(200, 225, 182, a=255),
    PySide6.QtGui.QColor(199, 225, 184, a=255),
    PySide6.QtGui.QColor(197, 225, 185, a=255),
    PySide6.QtGui.QColor(196, 225, 187, a=255),
    PySide6.QtGui.QColor(194, 225, 188, a=255),
    PySide6.QtGui.QColor(193, 225, 190, a=255),
    PySide6.QtGui.QColor(191, 225, 191, a=255),
    PySide6.QtGui.QColor(189, 224, 192, a=255),
    PySide6.QtGui.QColor(187, 224, 194, a=255),
    PySide6.QtGui.QColor(185, 223, 195, a=255),
    PySide6.QtGui.QColor(184, 223, 196, a=255),
    PySide6.QtGui.QColor(182, 222, 197, a=255),
    PySide6.QtGui.QColor(180, 222, 198, a=255),
    PySide6.QtGui.QColor(177, 221, 199, a=255),
    PySide6.QtGui.QColor(175, 220, 200, a=255),
    PySide6.QtGui.QColor(173, 220, 200, a=255),
    PySide6.QtGui.QColor(171, 219, 201, a=255),
    PySide6.QtGui.QColor(169, 218, 202, a=255),
    PySide6.QtGui.QColor(167, 217, 203, a=255),
    PySide6.QtGui.QColor(164, 216, 203, a=255),
    PySide6.QtGui.QColor(162, 215, 204, a=255),
    PySide6.QtGui.QColor(160, 214, 204, a=255),
    PySide6.QtGui.QColor(157, 213, 205, a=255),
    PySide6.QtGui.QColor(155, 212, 205, a=255),
    PySide6.QtGui.QColor(153, 211, 206, a=255),
    PySide6.QtGui.QColor(150, 209, 206, a=255),
    PySide6.QtGui.QColor(148, 208, 206, a=255),
    PySide6.QtGui.QColor(146, 207, 206, a=255),
    PySide6.QtGui.QColor(143, 206, 207, a=255),
    PySide6.QtGui.QColor(141, 204, 207, a=255),
    PySide6.QtGui.QColor(139, 203, 207, a=255),
    PySide6.QtGui.QColor(136, 201, 207, a=255),
    PySide6.QtGui.QColor(134, 200, 207, a=255),
    PySide6.QtGui.QColor(132, 198, 207, a=255),
    PySide6.QtGui.QColor(129, 197, 207, a=255),
    PySide6.QtGui.QColor(127, 195, 207, a=255),
    PySide6.QtGui.QColor(125, 194, 206, a=255),
    PySide6.QtGui.QColor(123, 192, 206, a=255),
    PySide6.QtGui.QColor(120, 190, 206, a=255),
    PySide6.QtGui.QColor(118, 189, 206, a=255),
    PySide6.QtGui.QColor(116, 187, 205, a=255),
    PySide6.QtGui.QColor(114, 185, 205, a=255),
    PySide6.QtGui.QColor(112, 184, 205, a=255),
    PySide6.QtGui.QColor(110, 182, 204, a=255),
    PySide6.QtGui.QColor(108, 180, 204, a=255),
    PySide6.QtGui.QColor(106, 178, 203, a=255),
    PySide6.QtGui.QColor(104, 177, 203, a=255),
    PySide6.QtGui.QColor(103, 175, 202, a=255),
    PySide6.QtGui.QColor(101, 173, 202, a=255),
    PySide6.QtGui.QColor(99, 171, 201, a=255),
    PySide6.QtGui.QColor(98, 169, 201, a=255),
    PySide6.QtGui.QColor(96, 168, 200, a=255),
    PySide6.QtGui.QColor(95, 166, 199, a=255),
    PySide6.QtGui.QColor(93, 164, 199, a=255),
    PySide6.QtGui.QColor(92, 162, 198, a=255),
    PySide6.QtGui.QColor(90, 160, 197, a=255),
    PySide6.QtGui.QColor(89, 158, 196, a=255),
    PySide6.QtGui.QColor(88, 156, 196, a=255),
    PySide6.QtGui.QColor(87, 155, 195, a=255),
    PySide6.QtGui.QColor(86, 153, 194, a=255),
    PySide6.QtGui.QColor(85, 151, 193, a=255),
    PySide6.QtGui.QColor(84, 149, 192, a=255),
    PySide6.QtGui.QColor(83, 147, 191, a=255),
    PySide6.QtGui.QColor(82, 145, 190, a=255),
    PySide6.QtGui.QColor(82, 143, 189, a=255),
    PySide6.QtGui.QColor(81, 141, 188, a=255),
    PySide6.QtGui.QColor(80, 139, 187, a=255),
    PySide6.QtGui.QColor(80, 138, 186, a=255),
    PySide6.QtGui.QColor(79, 136, 185, a=255),
    PySide6.QtGui.QColor(79, 134, 184, a=255),
    PySide6.QtGui.QColor(79, 132, 183, a=255),
    PySide6.QtGui.QColor(79, 130, 182, a=255),
    PySide6.QtGui.QColor(78, 128, 180, a=255),
    PySide6.QtGui.QColor(78, 126, 179, a=255),
    PySide6.QtGui.QColor(78, 124, 178, a=255),
    PySide6.QtGui.QColor(78, 122, 176, a=255),
    PySide6.QtGui.QColor(79, 120, 175, a=255),
    PySide6.QtGui.QColor(79, 118, 174, a=255),
    PySide6.QtGui.QColor(79, 117, 172, a=255),
    PySide6.QtGui.QColor(79, 115, 171, a=255),
    PySide6.QtGui.QColor(80, 113, 169, a=255),
    PySide6.QtGui.QColor(80, 111, 168, a=255),
    PySide6.QtGui.QColor(81, 109, 166, a=255),
    PySide6.QtGui.QColor(81, 107, 164, a=255),
    PySide6.QtGui.QColor(82, 105, 163, a=255),
    PySide6.QtGui.QColor(82, 103, 161, a=255),
    PySide6.QtGui.QColor(83, 102, 159, a=255),
    PySide6.QtGui.QColor(84, 100, 158, a=255),
    PySide6.QtGui.QColor(84, 98, 156, a=255),
    PySide6.QtGui.QColor(85, 96, 154, a=255),
    PySide6.QtGui.QColor(86, 95, 152, a=255),
    PySide6.QtGui.QColor(87, 93, 150, a=255),
    PySide6.QtGui.QColor(87, 91, 148, a=255),
    PySide6.QtGui.QColor(88, 89, 147, a=255),
    PySide6.QtGui.QColor(89, 88, 145, a=255),
    PySide6.QtGui.QColor(90, 86, 143, a=255),
    PySide6.QtGui.QColor(91, 85, 141, a=255),
    PySide6.QtGui.QColor(92, 83, 139, a=255),
    PySide6.QtGui.QColor(92, 82, 137, a=255),
    PySide6.QtGui.QColor(93, 80, 135, a=255),
    PySide6.QtGui.QColor(94, 79, 133, a=255),
    PySide6.QtGui.QColor(95, 77, 131, a=255),
    PySide6.QtGui.QColor(96, 76, 129, a=255),
    PySide6.QtGui.QColor(97, 75, 127, a=255),
    PySide6.QtGui.QColor(98, 73, 125, a=255),
    PySide6.QtGui.QColor(99, 72, 123, a=255),
    PySide6.QtGui.QColor(99, 71, 121, a=255),
    PySide6.QtGui.QColor(100, 70, 119, a=255),
    PySide6.QtGui.QColor(101, 69, 118, a=255),
    PySide6.QtGui.QColor(102, 68, 116, a=255),
    PySide6.QtGui.QColor(103, 67, 114, a=255),
    PySide6.QtGui.QColor(104, 66, 112, a=255),
    PySide6.QtGui.QColor(104, 65, 110, a=255),
    PySide6.QtGui.QColor(105, 64, 108, a=255),
    PySide6.QtGui.QColor(106, 63, 107, a=255),
    PySide6.QtGui.QColor(107, 63, 105, a=255),
    PySide6.QtGui.QColor(108, 62, 103, a=255),
    PySide6.QtGui.QColor(108, 61, 101, a=255),
    PySide6.QtGui.QColor(109, 61, 100, a=255),
    PySide6.QtGui.QColor(110, 60, 98, a=255),
    PySide6.QtGui.QColor(111, 59, 96, a=255),
    PySide6.QtGui.QColor(111, 59, 95, a=255),
    PySide6.QtGui.QColor(112, 58, 93, a=255),
    PySide6.QtGui.QColor(113, 58, 92, a=255),
    PySide6.QtGui.QColor(114, 58, 90, a=255),
    PySide6.QtGui.QColor(114, 57, 89, a=255),
]


def style_to_integer(style: EventStyle) -> int:
    if style == "exponential":
        return 0
    if style == "linear":
        return 1
    if style == "window":
        return 2
    raise Exception(f"unknown {style=}")


def event_colormap_to_texture(
    on_colormap: list[PySide6.QtGui.QColor],
    off_colormap: list[PySide6.QtGui.QColor],
) -> tuple[PySide6.QtOpenGL.QOpenGLTexture, float]:
    colormap_texture = PySide6.QtOpenGL.QOpenGLTexture(
        PySide6.QtOpenGL.QOpenGLTexture.Target.Target1D
    )
    colormap_texture.setWrapMode(PySide6.QtOpenGL.QOpenGLTexture.WrapMode.ClampToEdge)
    colormap_texture.setMinMagFilters(
        PySide6.QtOpenGL.QOpenGLTexture.Filter.Linear,
        PySide6.QtOpenGL.QOpenGLTexture.Filter.Linear,
    )
    colormap_texture.setFormat(PySide6.QtOpenGL.QOpenGLTexture.TextureFormat.RGBA32F)
    length = len(on_colormap) + len(off_colormap)
    colormap_texture.setSize(length, height=1, depth=1)
    colormap_texture.allocateStorage()
    colormap_data = numpy.zeros(length * 4, dtype=numpy.float32)
    index = 0
    for color in reversed(off_colormap):
        colormap_data[index] = color.redF()
        colormap_data[index + 1] = color.greenF()
        colormap_data[index + 2] = color.blueF()
        colormap_data[index + 3] = color.alphaF()
        index += 4
    for color in on_colormap:
        colormap_data[index] = color.redF()
        colormap_data[index + 1] = color.greenF()
        colormap_data[index + 2] = color.blueF()
        colormap_data[index + 3] = color.alphaF()
        index += 4
    colormap_texture.setData(
        PySide6.QtOpenGL.QOpenGLTexture.PixelFormat.RGBA,
        PySide6.QtOpenGL.QOpenGLTexture.PixelType.Float32,
        colormap_data,  # type: ignore
    )
    return (
        colormap_texture,
        0.0 if length == 0 else float(len(off_colormap)) / float(length),
    )


def frame_colormap_to_texture(
    colormap: list[PySide6.QtGui.QColor],
) -> PySide6.QtOpenGL.QOpenGLTexture:
    colormap_texture = PySide6.QtOpenGL.QOpenGLTexture(
        PySide6.QtOpenGL.QOpenGLTexture.Target.Target1D
    )
    colormap_texture.setWrapMode(PySide6.QtOpenGL.QOpenGLTexture.WrapMode.ClampToEdge)
    colormap_texture.setMinMagFilters(
        PySide6.QtOpenGL.QOpenGLTexture.Filter.Linear,
        PySide6.QtOpenGL.QOpenGLTexture.Filter.Linear,
    )
    colormap_texture.setFormat(PySide6.QtOpenGL.QOpenGLTexture.TextureFormat.RGBA32F)
    length = len(colormap)
    colormap_texture.setSize(length, height=1, depth=1)
    colormap_texture.allocateStorage()
    colormap_data = numpy.zeros(length * 4, dtype=numpy.float32)
    for index, color in enumerate(colormap):
        colormap_data[index * 4] = color.redF()
        colormap_data[index * 4 + 1] = color.greenF()
        colormap_data[index * 4 + 2] = color.blueF()
        colormap_data[index * 4 + 3] = color.alphaF()
    colormap_texture.setData(
        PySide6.QtOpenGL.QOpenGLTexture.PixelFormat.RGBA,
        PySide6.QtOpenGL.QOpenGLTexture.PixelType.Float32,
        colormap_data,  # type: ignore
    )
    return colormap_texture


class EventDisplayRenderer(PySide6.QtGui.QOpenGLFunctions):
    @dataclasses.dataclass
    class Program:
        inner: PySide6.QtOpenGL.QOpenGLShaderProgram
        vertices_buffer: PySide6.QtOpenGL.QOpenGLBuffer
        vertex_array_object: PySide6.QtOpenGL.QOpenGLVertexArrayObject
        ts_and_ons_texture: PySide6.QtOpenGL.QOpenGLTexture
        colormap_texture: PySide6.QtOpenGL.QOpenGLTexture
        colormap_split: float

        def cleanup(self):
            self.vertices_buffer.destroy()
            self.ts_and_ons_texture.destroy()
            self.colormap_texture.destroy()

    def __init__(
        self,
        window: PySide6.QtQuick.QQuickWindow,
        visible: bool,
        sensor_size: PySide6.QtCore.QSize,
        style: EventStyle,
        tau: float,
        on_colormap: list[PySide6.QtGui.QColor],
        off_colormap: list[PySide6.QtGui.QColor],
        padding_color: PySide6.QtGui.QColor,
        clear_background: bool,
    ):
        super().__init__()
        self.window = window
        self.visible = visible
        self.sensor_size = sensor_size
        self.style = style_to_integer(style=style)
        self.tau = tau
        self.on_colormap = on_colormap
        self.off_colormap = off_colormap
        self.padding_color = padding_color
        self.clear_background = clear_background
        self.ts_and_ons = numpy.zeros(
            sensor_size.width() * sensor_size.height(),
            dtype=numpy.float32,
        )
        self.current_t: float = 0.0
        self.offset_t: int = 0
        self.colormaps_changed = False
        self.clear_area = PySide6.QtCore.QRect()
        self.draw_area = PySide6.QtCore.QRect()
        self.program: typing.Optional[EventDisplayRenderer.Program] = None
        self.lock = PySide6.QtCore.QMutex()

    def push(self, events: numpy.ndarray, current_t: int):
        if len(events) > 0:
            assert current_t >= int(events["t"][-1])
        with PySide6.QtCore.QMutexLocker(self.lock):
            while current_t - self.offset_t > MAXIMUM_DELTA:
                self.offset_t += MAXIMUM_DELTA // 2
                recent = numpy.abs(self.ts_and_ons) > MAXIMUM_DELTA // 2
                on = self.ts_and_ons > 0
                self.ts_and_ons[numpy.logical_and(recent, on)] -= MAXIMUM_DELTA // 2
                self.ts_and_ons[numpy.logical_and(recent, numpy.logical_not(on))] += (
                    MAXIMUM_DELTA // 2
                )
                self.ts_and_ons[numpy.logical_not(recent)] = 0.0
            if len(events) > 0:
                t_and_on = (events["t"] - self.offset_t).astype(numpy.float32)
                t_and_on[numpy.logical_not(events["on"])] *= -1.0
                self.ts_and_ons[
                    events["x"].astype(numpy.uint32)
                    + events["y"].astype(numpy.uint32) * self.sensor_size.width()
                ] = t_and_on
            self.current_t = float(current_t - self.offset_t)

    def set_visible(self, visible: bool):
        with PySide6.QtCore.QMutexLocker(self.lock):
            self.visible = visible

    def set_style(self, style: EventStyle):
        with PySide6.QtCore.QMutexLocker(self.lock):
            self.style = style_to_integer(style=style)

    def set_tau(self, tau: float):
        with PySide6.QtCore.QMutexLocker(self.lock):
            self.tau = tau

    def set_on_colormap(self, on_colormap: list[PySide6.QtGui.QColor]):
        with PySide6.QtCore.QMutexLocker(self.lock):
            self.on_colormap = on_colormap
            self.colormaps_changed = True

    def set_off_colormap(self, off_colormap: list[PySide6.QtGui.QColor]):
        with PySide6.QtCore.QMutexLocker(self.lock):
            self.off_colormap = off_colormap
            self.colormaps_changed = True

    def set_padding_color(self, padding_color: PySide6.QtGui.QColor):
        with PySide6.QtCore.QMutexLocker(self.lock):
            self.padding_color = padding_color

    def set_clear_background(self, clear_background: bool):
        with PySide6.QtCore.QMutexLocker(self.lock):
            self.clear_background = clear_background

    def set_clear_and_draw_areas(
        self,
        clear_area: PySide6.QtCore.QRectF,
        draw_area: PySide6.QtCore.QRectF,
    ):
        with PySide6.QtCore.QMutexLocker(self.lock):
            self.clear_area = clear_area
            self.draw_area = draw_area

    @PySide6.QtCore.Slot()
    def init(self):
        with PySide6.QtCore.QMutexLocker(self.lock):
            if self.program is not None:
                return
            assert (
                self.window.rendererInterface().graphicsApi()
                == PySide6.QtQuick.QSGRendererInterface.GraphicsApi.OpenGL
            )
            self.initializeOpenGLFunctions()
            program = PySide6.QtOpenGL.QOpenGLShaderProgram()
            assert program.addShaderFromSourceCode(
                PySide6.QtOpenGL.QOpenGLShader.ShaderTypeBit.Vertex,
                VERTEX_SHADER,
            )
            assert program.addShaderFromSourceCode(
                PySide6.QtOpenGL.QOpenGLShader.ShaderTypeBit.Fragment,
                EVENT_DISPLAY_FRAGMENT_SHADER,
            )
            assert program.link()
            assert program.bind()
            vertex_array_object = PySide6.QtOpenGL.QOpenGLVertexArrayObject()
            assert vertex_array_object.create()
            vertex_array_object.bind()
            vertices_buffer = PySide6.QtOpenGL.QOpenGLBuffer()
            assert vertices_buffer.create()
            vertices_buffer.bind()
            vertices = numpy.array(
                [-1.0, -1.0, 1.0, -1.0, -1.0, 1.0, 1.0, 1.0],
                dtype=numpy.float32,
            )
            vertices_buffer.allocate(vertices.tobytes(), vertices.nbytes)
            ts_and_ons_texture = PySide6.QtOpenGL.QOpenGLTexture(
                PySide6.QtOpenGL.QOpenGLTexture.Target.Target2D
            )
            ts_and_ons_texture.setWrapMode(
                PySide6.QtOpenGL.QOpenGLTexture.WrapMode.ClampToBorder
            )
            ts_and_ons_texture.setMinMagFilters(
                PySide6.QtOpenGL.QOpenGLTexture.Filter.Nearest,
                PySide6.QtOpenGL.QOpenGLTexture.Filter.Nearest,
            )
            ts_and_ons_texture.setFormat(
                PySide6.QtOpenGL.QOpenGLTexture.TextureFormat.R32F
            )
            ts_and_ons_texture.setSize(
                self.sensor_size.width(),
                height=self.sensor_size.height(),
                depth=1,
            )
            ts_and_ons_texture.allocateStorage()
            ts_and_ons_texture.setData(
                PySide6.QtOpenGL.QOpenGLTexture.PixelFormat.Red,
                PySide6.QtOpenGL.QOpenGLTexture.PixelType.Float32,
                self.ts_and_ons,  # type: ignore
            )
            colormap_texture, colormap_split = event_colormap_to_texture(
                on_colormap=self.on_colormap,
                off_colormap=self.off_colormap,
            )
            vertices_location = program.attributeLocation("vertices")
            program.enableAttributeArray(vertices_location)
            program.setAttributeBuffer(vertices_location, GL_FLOAT, 0, 2, 0)
            program.release()
            vertices_buffer.release()
            vertex_array_object.release()
            self.program = EventDisplayRenderer.Program(
                inner=program,
                vertices_buffer=vertices_buffer,
                vertex_array_object=vertex_array_object,
                ts_and_ons_texture=ts_and_ons_texture,
                colormap_texture=colormap_texture,
                colormap_split=colormap_split,
            )

    @PySide6.QtCore.Slot()
    def paint(self):
        with PySide6.QtCore.QMutexLocker(self.lock):
            if self.program is None or not self.visible:
                return
            self.window.beginExternalCommands()
            self.program.inner.bind()
            self.program.inner.setUniformValue1f(
                "current_t",  # type: ignore
                self.current_t,
            )
            self.program.inner.setUniformValue1i(
                "style",  # type: ignore
                self.style,
            )
            self.program.inner.setUniformValue1f(
                "tau",  # type: ignore
                self.tau,
            )
            self.program.inner.setUniformValue1i(
                self.program.inner.uniformLocation("t_and_on_sampler"), 0
            )
            self.program.inner.setUniformValue1i(
                self.program.inner.uniformLocation("colormap_sampler"), 1
            )
            if self.clear_background:
                self.glEnable(GL_SCISSOR_TEST)
                self.glScissor(
                    round(self.clear_area.left()),
                    round(
                        self.window.height() * self.window.devicePixelRatio()
                        - self.clear_area.bottom()
                    ),
                    round(self.clear_area.width()),
                    round(self.clear_area.height()),
                )
                self.glClearColor(
                    self.padding_color.redF(),
                    self.padding_color.greenF(),
                    self.padding_color.blueF(),
                    self.padding_color.alphaF(),
                )
                self.glClear(GL_COLOR_BUFFER_BIT)
                self.glDisable(GL_SCISSOR_TEST)
            self.glViewport(
                round(self.draw_area.left()),
                round(
                    self.window.height() * self.window.devicePixelRatio()
                    - self.draw_area.bottom()
                ),
                round(self.draw_area.width()),
                round(self.draw_area.height()),
            )
            self.glDisable(GL_DEPTH_TEST)
            self.glEnable(GL_BLEND)
            self.glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
            self.program.ts_and_ons_texture.bind(0)
            self.program.ts_and_ons_texture.setData(
                PySide6.QtOpenGL.QOpenGLTexture.PixelFormat.Red,
                PySide6.QtOpenGL.QOpenGLTexture.PixelType.Float32,
                self.ts_and_ons,  # type: ignore
            )
            if self.colormaps_changed:
                self.program.colormap_texture, self.program.colormap_split = (
                    event_colormap_to_texture(
                        on_colormap=self.on_colormap,
                        off_colormap=self.off_colormap,
                    )
                )
                self.colormaps_changed = False
            self.program.inner.setUniformValue1f(
                "colormap_split",  # type: ignore
                self.program.colormap_split,
            )
            self.program.colormap_texture.bind(1)
            self.program.vertex_array_object.bind()
            self.glDrawArrays(GL_TRIANGLE_STRIP, 0, 4)
            self.program.colormap_texture.release()
            self.program.ts_and_ons_texture.release()
            self.program.vertex_array_object.release()
            self.program.inner.release()
            self.window.endExternalCommands()

    def cleanup(self):
        with PySide6.QtCore.QMutexLocker(self.lock):
            if self.program is not None:
                self.program.cleanup()
                self.program = None


class EventDisplay(PySide6.QtQuick.QQuickItem):

    def __init__(self, parent: typing.Optional[PySide6.QtQuick.QQuickItem] = None):
        super().__init__(parent)
        self._window: typing.Optional[PySide6.QtQuick.QQuickWindow] = None
        self._visible: bool = True
        self._renderer: typing.Optional[EventDisplayRenderer] = None
        self._clear_area: typing.Optional[PySide6.QtCore.QRectF] = None
        self._draw_area: typing.Optional[PySide6.QtCore.QRectF] = None
        self._sensor_size: typing.Optional[PySide6.QtCore.QSize] = None
        self._style: EventStyle = "exponential"
        self._tau: float = DEFAULT_TAU
        self._on_colormap = DEFAULT_ON_COLORMAP
        self._off_colormap = DEFAULT_OFF_COLORMAP
        self._padding_color = PySide6.QtGui.QColor(0x19, 0x19, 0x19)
        self._clear_background = True
        self._timer = PySide6.QtCore.QTimer(interval=16)
        self._timer.timeout.connect(self.trigger_draw)
        self._timer.start()
        self.windowChanged.connect(self.handleWindowChanged)
        self.visibleChanged.connect(self.handleVisibleChanged)

    def push(self, events: numpy.ndarray, current_t: int):
        if hasattr(self, "_renderer") and self._renderer is not None:
            self._renderer.push(events=events, current_t=current_t)

    def set_sensor_size(self, sensor_size: PySide6.QtCore.QSize):
        assert sensor_size.width() > 0 and sensor_size.height() > 0
        if self._sensor_size is not None:
            raise Exception(f"sensor size may only be set once")
        self._sensor_size = sensor_size

    sensor_size = PySide6.QtCore.Property(
        PySide6.QtCore.QSize,
        None,
        set_sensor_size,
        None,
        "sensor size in pixels",
    )

    def get_style(self) -> EventStyle:
        return self._style

    def set_style(self, style: EventStyle):
        assert style in {"exponential", "linear", "window"}
        self._style = style
        if self._renderer is not None:
            self._renderer.set_style(style=style)

    style = PySide6.QtCore.Property(
        str,
        get_style,
        set_style,
        None,
        "decay function",
    )

    def get_tau(self) -> float:
        return self._tau

    def set_tau(self, tau: float):
        assert tau > 0.0
        self._tau = tau
        if self._renderer is not None:
            self._renderer.set_tau(tau=tau)

    tau = PySide6.QtCore.Property(
        float,
        get_tau,
        set_tau,
        None,
        "decay time constant",
    )

    def get_on_colormap(self) -> list[PySide6.QtGui.QColor]:
        return self._on_colormap

    def set_on_colormap(self, on_colormap: list[PySide6.QtGui.QColor]):
        on_colormap = [PySide6.QtGui.QColor(color) for color in on_colormap]
        self._on_colormap = on_colormap
        if self._renderer is not None:
            self._renderer.set_on_colormap(on_colormap=on_colormap)

    on_colormap = PySide6.QtCore.Property(
        list,
        get_on_colormap,
        set_on_colormap,
        None,
        "colormap for ON events (polarity 1)",
    )

    def get_off_colormap(self) -> list[PySide6.QtGui.QColor]:
        return self._off_colormap

    def set_off_colormap(self, off_colormap: list[PySide6.QtGui.QColor]):
        off_colormap = [PySide6.QtGui.QColor(color) for color in off_colormap]
        self._off_colormap = off_colormap
        if self._renderer is not None:
            self._renderer.set_off_colormap(off_colormap=off_colormap)

    off_colormap = PySide6.QtCore.Property(
        list,
        get_off_colormap,
        set_off_colormap,
        None,
        "colormap for OFF events (polarity 0)",
    )

    def get_padding_color(self) -> PySide6.QtGui.QColor:
        return self._padding_color

    def set_padding_color(self, padding_color: PySide6.QtGui.QColor):
        self._padding_color = padding_color
        if self._renderer is not None:
            self._renderer.set_padding_color(padding_color=padding_color)

    padding_color = PySide6.QtCore.Property(
        PySide6.QtCore.QObject,
        get_padding_color,
        set_padding_color,
        None,
        "background color to pad a ratio mismatch between the container and the event display",
    )

    def get_clear_background(self) -> bool:
        return self._clear_background

    def set_clear_background(self, clear_background: bool):
        self._clear_background = clear_background
        if self._renderer is not None:
            self._renderer.set_clear_background(clear_background=clear_background)

    clear_background = PySide6.QtCore.Property(
        bool,
        get_clear_background,
        set_clear_background,
        None,
        "whether to clear the display's background with padding color",
    )

    @PySide6.QtCore.Slot()
    def trigger_draw(self):
        if self._window is not None:
            self._window.update()

    @PySide6.QtCore.Slot(PySide6.QtQuick.QQuickWindow)
    def handleWindowChanged(
        self, window: typing.Optional[PySide6.QtQuick.QQuickWindow]
    ):
        self._window = window
        if window is not None:
            window.beforeSynchronizing.connect(
                self.sync, type=PySide6.QtCore.Qt.ConnectionType.DirectConnection
            )
            window.sceneGraphInvalidated.connect(
                self.cleanup, type=PySide6.QtCore.Qt.ConnectionType.DirectConnection
            )
            self.sync()

    @PySide6.QtCore.Slot(bool)
    def handleVisibleChanged(self):
        self._visible = self.isVisible()
        if self._renderer is not None:
            self._renderer.set_visible(visible=self._visible)

    @PySide6.QtCore.Slot()
    def cleanup(self):
        if self._renderer is not None:
            self._renderer.cleanup()
            del self._renderer
            self._renderer = None

    @PySide6.QtCore.Slot()
    def sync(self):
        window = self.window()
        if window is None:
            return
        if self._sensor_size is None:
            raise Exception(
                'the sensor size must be set in QML (for example, EventDisplay {sensor_size: "1280x720"})'
            )
        pixel_ratio = self.window().devicePixelRatio()
        if self._renderer is None:
            self._renderer = EventDisplayRenderer(
                window=window,
                visible=self._visible,
                sensor_size=self._sensor_size,
                style=self._style,
                tau=self._tau,
                on_colormap=self._on_colormap,
                off_colormap=self._off_colormap,
                padding_color=self._padding_color,
                clear_background=self._clear_background,
            )
            window.beforeRendering.connect(
                self._renderer.init, PySide6.QtCore.Qt.ConnectionType.DirectConnection
            )
            window.beforeRenderPassRecording.connect(
                self._renderer.paint, PySide6.QtCore.Qt.ConnectionType.DirectConnection
            )
        clear_area = PySide6.QtCore.QRectF(
            0,
            0,
            self.width() * self.window().devicePixelRatio(),
            self.height() * self.window().devicePixelRatio(),
        )
        item = self
        while item is not None:
            clear_area.moveLeft(clear_area.left() + item.x() * pixel_ratio)
            clear_area.moveTop(clear_area.top() + item.y() * pixel_ratio)
            item = item.parentItem()
        if self._clear_area != clear_area:
            self._clear_area = clear_area
            self._draw_area = PySide6.QtCore.QRectF()

            if (
                clear_area.width() * self._sensor_size.height()
                > clear_area.height() * self._sensor_size.width()
            ):
                self._draw_area.setWidth(
                    clear_area.height()
                    * self._sensor_size.width()
                    / self._sensor_size.height()
                )
                self._draw_area.setHeight(clear_area.height())
                self._draw_area.moveLeft(
                    clear_area.left()
                    + (clear_area.width() - self._draw_area.width()) / 2
                )
                self._draw_area.moveTop(clear_area.top())
            else:
                self._draw_area.setWidth(clear_area.width())
                self._draw_area.setHeight(
                    clear_area.width()
                    * self._sensor_size.height()
                    / self._sensor_size.width()
                )
                self._draw_area.moveLeft(clear_area.left())
                self._draw_area.moveTop(
                    clear_area.top()
                    + (clear_area.height() - self._draw_area.height()) / 2
                )

            self._renderer.set_clear_and_draw_areas(
                clear_area=self._clear_area, draw_area=self._draw_area
            )
            # @TODO emit signal for paint area change


class FrameDisplayRenderer(PySide6.QtGui.QOpenGLFunctions):
    @dataclasses.dataclass
    class Program:
        inner: PySide6.QtOpenGL.QOpenGLShaderProgram
        vertices_buffer: PySide6.QtOpenGL.QOpenGLBuffer
        vertex_array_object: PySide6.QtOpenGL.QOpenGLVertexArrayObject
        frame_texture: PySide6.QtOpenGL.QOpenGLTexture
        colormap_texture: typing.Optional[PySide6.QtOpenGL.QOpenGLTexture]

        def cleanup(self):
            self.vertices_buffer.destroy()
            self.frame_texture.destroy()
            if self.colormap_texture is not None:
                self.colormap_texture.destroy()

    def __init__(
        self,
        window: PySide6.QtQuick.QQuickWindow,
        visible: bool,
        sensor_size: PySide6.QtCore.QSize,
        mode: FrameMode,
        dtype: FrameDtype,
        colormap: list[PySide6.QtGui.QColor],
        padding_color: PySide6.QtGui.QColor,
        clear_background: bool,
    ):
        super().__init__()
        self.window = window
        self.visible = visible
        self.sensor_size = sensor_size
        self.mode: FrameMode = mode
        self.colormap = colormap
        if dtype == "u1":
            self.pixel_type = PySide6.QtOpenGL.QOpenGLTexture.PixelType.UInt8
            pixel_format_suffix = "_Integer"
            texture_format = "8U"
            byte_width = 1
        elif dtype == "u2":
            self.pixel_type = PySide6.QtOpenGL.QOpenGLTexture.PixelType.UInt16
            pixel_format_suffix = "_Integer"
            texture_format = "16U"
            byte_width = 2
        elif dtype == "f4":
            self.pixel_type = PySide6.QtOpenGL.QOpenGLTexture.PixelType.Float32
            pixel_format_suffix = ""
            texture_format = "32F"
            byte_width = 4
        else:
            raise Exception(f"unsupported dtype {self.dtype}")
        if mode == "L" or mode == "P":
            self.depth = 1
            self.pixel_format = PySide6.QtOpenGL.QOpenGLTexture.PixelFormat[
                f"Red{pixel_format_suffix}"
            ]
            self.texture_format = PySide6.QtOpenGL.QOpenGLTexture.TextureFormat[
                f"R{texture_format}"
            ]
            byte_width *= 1
        elif mode == "RGB":
            self.depth = 3
            self.pixel_format = PySide6.QtOpenGL.QOpenGLTexture.PixelFormat[
                f"RGB{pixel_format_suffix}"
            ]
            self.texture_format = PySide6.QtOpenGL.QOpenGLTexture.TextureFormat[
                f"RGB{texture_format}"
            ]
            byte_width *= 3
        elif mode == "RGBA":
            self.depth = 4
            self.pixel_format = PySide6.QtOpenGL.QOpenGLTexture.PixelFormat[
                f"RGBA{pixel_format_suffix}"
            ]
            self.texture_format = PySide6.QtOpenGL.QOpenGLTexture.TextureFormat[
                f"RGBA{texture_format}"
            ]
            byte_width *= 4
        else:
            raise Exception(f"unsupported mode {mode}")
        byte_width *= sensor_size.width()
        self.transfer_options = PySide6.QtOpenGL.QOpenGLPixelTransferOptions()
        if byte_width % 4 == 0:
            self.transfer_options.setAlignment(4)
        elif byte_width % 2 == 0:
            self.transfer_options.setAlignment(2)
        else:
            self.transfer_options.setAlignment(0)
        self.dtype: FrameDtype = dtype
        self.padding_color = padding_color
        self.clear_background = clear_background
        self.frame = numpy.zeros(
            sensor_size.width() * sensor_size.height() * self.depth,
            dtype=self.dtype,
        )
        self.colormap_changed = False
        self.clear_area = PySide6.QtCore.QRect()
        self.draw_area = PySide6.QtCore.QRect()
        self.program: typing.Optional[FrameDisplayRenderer.Program] = None
        self.lock = PySide6.QtCore.QMutex()

    def push(self, frame: numpy.ndarray):
        assert frame.dtype == numpy.dtype(self.dtype)
        if self.depth == 1:
            assert frame.shape == (self.sensor_size.height(), self.sensor_size.width())
        else:
            assert frame.shape == (
                self.sensor_size.height(),
                self.sensor_size.width(),
                self.depth,
            )
        with PySide6.QtCore.QMutexLocker(self.lock):
            numpy.copyto(self.frame, frame.flatten())

    def set_visible(self, visible: bool):
        with PySide6.QtCore.QMutexLocker(self.lock):
            self.visible = visible

    def set_colormap(self, colormap: list[PySide6.QtGui.QColor]):
        with PySide6.QtCore.QMutexLocker(self.lock):
            self.colormap = colormap
            self.colormap_changed = True

    def set_padding_color(self, padding_color: PySide6.QtGui.QColor):
        with PySide6.QtCore.QMutexLocker(self.lock):
            self.padding_color = padding_color

    def set_clear_background(self, clear_background: bool):
        with PySide6.QtCore.QMutexLocker(self.lock):
            self.clear_background = clear_background

    def set_clear_and_draw_areas(
        self,
        clear_area: PySide6.QtCore.QRectF,
        draw_area: PySide6.QtCore.QRectF,
    ):
        with PySide6.QtCore.QMutexLocker(self.lock):
            self.clear_area = clear_area
            self.draw_area = draw_area

    @PySide6.QtCore.Slot()
    def init(self):
        with PySide6.QtCore.QMutexLocker(self.lock):
            if self.program is not None:
                return
            assert (
                self.window.rendererInterface().graphicsApi()
                == PySide6.QtQuick.QSGRendererInterface.GraphicsApi.OpenGL
            )
            self.initializeOpenGLFunctions()
            program = PySide6.QtOpenGL.QOpenGLShaderProgram()
            assert program.addShaderFromSourceCode(
                PySide6.QtOpenGL.QOpenGLShader.ShaderTypeBit.Vertex,
                VERTEX_SHADER,
            )
            assert program.addShaderFromSourceCode(
                PySide6.QtOpenGL.QOpenGLShader.ShaderTypeBit.Fragment,
                frame_display_mode_and_dtype_to_fragment_shader[
                    (self.mode, self.dtype)
                ],
            )
            assert program.link()
            assert program.bind()
            vertex_array_object = PySide6.QtOpenGL.QOpenGLVertexArrayObject()
            assert vertex_array_object.create()
            vertex_array_object.bind()
            vertices_buffer = PySide6.QtOpenGL.QOpenGLBuffer()
            assert vertices_buffer.create()
            vertices_buffer.bind()
            vertices = numpy.array(
                [-1.0, -1.0, 1.0, -1.0, -1.0, 1.0, 1.0, 1.0],
                dtype=numpy.float32,
            )
            vertices_buffer.allocate(vertices.tobytes(), vertices.nbytes)

            frame_texture = PySide6.QtOpenGL.QOpenGLTexture(
                PySide6.QtOpenGL.QOpenGLTexture.Target.Target2D
            )
            frame_texture.setWrapMode(
                PySide6.QtOpenGL.QOpenGLTexture.WrapMode.ClampToBorder
            )
            frame_texture.setMinMagFilters(
                PySide6.QtOpenGL.QOpenGLTexture.Filter.Nearest,
                PySide6.QtOpenGL.QOpenGLTexture.Filter.Nearest,
            )
            frame_texture.setFormat(self.texture_format)
            frame_texture.setSize(
                self.sensor_size.width(),
                height=self.sensor_size.height(),
                depth=self.depth,
            )
            frame_texture.allocateStorage()
            frame_texture.setData(
                self.pixel_format,
                self.pixel_type,
                self.frame,  # type: ignore
                self.transfer_options,
            )
            colormap_texture = frame_colormap_to_texture(self.colormap)

            vertices_location = program.attributeLocation("vertices")
            program.enableAttributeArray(vertices_location)
            program.setAttributeBuffer(vertices_location, GL_FLOAT, 0, 2, 0)
            program.release()
            vertices_buffer.release()
            vertex_array_object.release()
            self.program = FrameDisplayRenderer.Program(
                inner=program,
                vertices_buffer=vertices_buffer,
                vertex_array_object=vertex_array_object,
                frame_texture=frame_texture,
                colormap_texture=colormap_texture,
            )

    @PySide6.QtCore.Slot()
    def paint(self):
        with PySide6.QtCore.QMutexLocker(self.lock):
            if self.program is None or not self.visible:
                return
            self.window.beginExternalCommands()
            self.program.inner.bind()
            if self.clear_background:
                self.glEnable(GL_SCISSOR_TEST)
                self.glScissor(
                    round(self.clear_area.left()),
                    round(
                        self.window.height() * self.window.devicePixelRatio()
                        - self.clear_area.bottom()
                    ),
                    round(self.clear_area.width()),
                    round(self.clear_area.height()),
                )
                self.glClearColor(
                    self.padding_color.redF(),
                    self.padding_color.greenF(),
                    self.padding_color.blueF(),
                    self.padding_color.alphaF(),
                )
                self.glClear(GL_COLOR_BUFFER_BIT)
                self.glDisable(GL_SCISSOR_TEST)
            self.glViewport(
                round(self.draw_area.left()),
                round(
                    self.window.height() * self.window.devicePixelRatio()
                    - self.draw_area.bottom()
                ),
                round(self.draw_area.width()),
                round(self.draw_area.height()),
            )
            self.glDisable(GL_DEPTH_TEST)
            self.glEnable(GL_BLEND)
            self.glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
            self.program.inner.setUniformValue1i(
                self.program.inner.uniformLocation("frame_sampler"), 0
            )
            if self.mode == "P":
                self.program.inner.setUniformValue1i(
                    self.program.inner.uniformLocation("colormap_sampler"), 1
                )
            self.program.frame_texture.bind(0)
            self.program.frame_texture.setData(
                self.pixel_format,
                self.pixel_type,
                self.frame,  # type: ignore
                self.transfer_options,
            )
            if self.colormap_changed and self.program.colormap_texture is not None:
                self.program.colormap_texture = frame_colormap_to_texture(
                    colormap=self.colormap
                )
                self.colormap_changed = False
            if self.program.colormap_texture is not None:
                self.program.colormap_texture.bind(1)
            self.program.vertex_array_object.bind()
            self.glDrawArrays(GL_TRIANGLE_STRIP, 0, 4)
            self.program.vertex_array_object.release()
            self.program.inner.release()
            self.window.endExternalCommands()

    def cleanup(self):
        with PySide6.QtCore.QMutexLocker(self.lock):
            if self.program is not None:
                self.program.cleanup()
                self.program = None


class FrameDisplay(PySide6.QtQuick.QQuickItem):

    def __init__(self, parent: typing.Optional[PySide6.QtQuick.QQuickItem] = None):
        super().__init__(parent)
        self._window: typing.Optional[PySide6.QtQuick.QQuickWindow] = None
        self._visible: bool = True
        self._renderer: typing.Optional[FrameDisplayRenderer] = None
        self._clear_area: typing.Optional[PySide6.QtCore.QRectF] = None
        self._draw_area: typing.Optional[PySide6.QtCore.QRectF] = None
        self._sensor_size: typing.Optional[PySide6.QtCore.QSize] = None
        self._mode: typing.Optional[FrameMode] = None
        self._dtype: typing.Optional[FrameDtype] = None
        self._colormap = DEFAULT_FRAME_COLORMAP
        self._padding_color = PySide6.QtGui.QColor(0x19, 0x19, 0x19)
        self._clear_background = True
        self._timer = PySide6.QtCore.QTimer(interval=16)
        self._timer.timeout.connect(self.trigger_draw)
        self._timer.start()
        self.windowChanged.connect(self.handleWindowChanged)
        self.visibleChanged.connect(self.handleVisibleChanged)

    def push(self, frame: numpy.ndarray):
        if hasattr(self, "_renderer") and self._renderer is not None:
            self._renderer.push(frame=frame)

    def set_sensor_size(self, sensor_size: PySide6.QtCore.QSize):
        if self._sensor_size is not None:
            raise Exception(f"sensor size may only be set once")
        self._sensor_size = sensor_size

    sensor_size = PySide6.QtCore.Property(
        PySide6.QtCore.QSize,
        None,
        set_sensor_size,
        None,
        "sensor size in pixels",
    )

    def set_mode(self, mode: FrameMode):
        assert mode in {"L", "RGB", "RGBA", "P"}
        if self._mode is not None:
            raise Exception(f"mode may only be set once")
        self._mode = mode

    mode = PySide6.QtCore.Property(
        str,
        None,
        set_mode,
        None,
        "input frame depth",
    )

    def set_dtype(self, dtype: FrameDtype):
        assert dtype in {"u1", "u2", "f4"}
        if self._dtype is not None:
            raise Exception(f"dtype may only be set once")
        self._dtype = dtype

    dtype = PySide6.QtCore.Property(
        str,
        None,
        set_dtype,
        None,
        "input frame pixel type",
    )

    def get_colormap(self) -> list[PySide6.QtGui.QColor]:
        return self._colormap

    def set_colormap(self, colormap: list[PySide6.QtGui.QColor]):
        colormap = [PySide6.QtGui.QColor(color) for color in colormap]
        self._colormap = colormap
        if self._renderer is not None:
            self._renderer.set_colormap(colormap=colormap)

    colormap = PySide6.QtCore.Property(
        list,
        get_colormap,
        set_colormap,
        None,
        "colormap for frame values (mode P only)",
    )

    def get_padding_color(self) -> PySide6.QtGui.QColor:
        return self._padding_color

    def set_padding_color(self, padding_color: PySide6.QtGui.QColor):
        self._padding_color = padding_color
        if self._renderer is not None:
            self._renderer.set_padding_color(padding_color=padding_color)

    padding_color = PySide6.QtCore.Property(
        PySide6.QtGui.QColor,
        get_padding_color,
        set_padding_color,
        None,
        "background color to pad a ratio mismatch between the container and the frame display",
    )

    def get_clear_background(self) -> bool:
        return self._clear_background

    def set_clear_background(self, clear_background: bool):
        self._clear_background = clear_background
        if self._renderer is not None:
            self._renderer.set_clear_background(clear_background=clear_background)

    clear_background = PySide6.QtCore.Property(
        bool,
        get_clear_background,
        set_clear_background,
        None,
        "whether to clear the display's background with padding color",
    )

    @PySide6.QtCore.Slot()
    def trigger_draw(self):
        if self._window is not None:
            self._window.update()

    @PySide6.QtCore.Slot(PySide6.QtQuick.QQuickWindow)
    def handleWindowChanged(
        self, window: typing.Optional[PySide6.QtQuick.QQuickWindow]
    ):
        self._window = window
        if window is not None:
            window.beforeSynchronizing.connect(
                self.sync, type=PySide6.QtCore.Qt.ConnectionType.DirectConnection
            )
            window.sceneGraphInvalidated.connect(
                self.cleanup, type=PySide6.QtCore.Qt.ConnectionType.DirectConnection
            )
            self.sync()

    @PySide6.QtCore.Slot(bool)
    def handleVisibleChanged(self):
        self._visible = self.isVisible()
        if self._renderer is not None:
            self._renderer.set_visible(visible=self._visible)

    @PySide6.QtCore.Slot()
    def cleanup(self):
        if self._renderer is not None:
            self._renderer.cleanup()
            self._renderer = None

    @PySide6.QtCore.Slot()
    def sync(self):
        window = self.window()
        if window is None:
            return
        if self._sensor_size is None:
            raise Exception(
                'the sensor size must be set in QML (for example, FrameDisplay {sensor_size: "1280x720"})'
            )
        if self._mode is None:
            raise Exception(
                'the mode must be set in QML (for example, FrameDisplay {mode: "RGB"})'
            )
        if self._dtype is None:
            raise Exception(
                'the dtype must be set in QML (for example, FrameDisplay {dtype: "u1"})'
            )
        pixel_ratio = self.window().devicePixelRatio()
        if self._renderer is None:
            self._renderer = FrameDisplayRenderer(
                window=window,
                visible=self._visible,
                sensor_size=self._sensor_size,
                mode=self._mode,
                dtype=self._dtype,
                colormap=self._colormap,
                padding_color=self._padding_color,
                clear_background=self._clear_background,
            )
            window.beforeRendering.connect(
                self._renderer.init, PySide6.QtCore.Qt.ConnectionType.DirectConnection
            )
            window.beforeRenderPassRecording.connect(
                self._renderer.paint, PySide6.QtCore.Qt.ConnectionType.DirectConnection
            )
        clear_area = PySide6.QtCore.QRectF(
            0,
            0,
            self.width() * self.window().devicePixelRatio(),
            self.height() * self.window().devicePixelRatio(),
        )
        item = self
        while item is not None:
            clear_area.moveLeft(clear_area.left() + item.x() * pixel_ratio)
            clear_area.moveTop(clear_area.top() + item.y() * pixel_ratio)
            item = item.parentItem()
        if self._clear_area != clear_area:
            self._clear_area = clear_area
            self._draw_area = PySide6.QtCore.QRectF()
            if (
                clear_area.width() * self._sensor_size.height()
                > clear_area.height() * self._sensor_size.width()
            ):
                self._draw_area.setWidth(
                    clear_area.height()
                    * self._sensor_size.width()
                    / self._sensor_size.height()
                )
                self._draw_area.setHeight(clear_area.height())
                self._draw_area.moveLeft(
                    clear_area.left()
                    + (clear_area.width() - self._draw_area.width()) / 2
                )
                self._draw_area.moveTop(clear_area.top())
            else:
                self._draw_area.setWidth(clear_area.width())
                self._draw_area.setHeight(
                    clear_area.width()
                    * self._sensor_size.height()
                    / self._sensor_size.width()
                )
                self._draw_area.moveLeft(clear_area.left())
                self._draw_area.moveTop(
                    clear_area.top()
                    + (clear_area.height() - self._draw_area.height()) / 2
                )
            self._renderer.set_clear_and_draw_areas(
                clear_area=self._clear_area, draw_area=self._draw_area
            )
            # @TODO emit signal for paint area change


class App:
    def __init__(
        self,
        qml: str,
        from_python_defaults: dict[str, typing.Any] = {},
        to_python: typing.Optional[typing.Callable[[str, typing.Any], None]] = None,
        argv: list[str] = sys.argv,
    ):
        PySide6.QtQml.qmlRegisterType(
            EventDisplay,
            "NeuromorphicDrivers",
            1,
            0,
            "EventDisplay",  # type: ignore
        )
        PySide6.QtQml.qmlRegisterType(
            FrameDisplay,
            "NeuromorphicDrivers",
            1,
            0,
            "FrameDisplay",  # type: ignore
        )
        format = PySide6.QtGui.QSurfaceFormat()
        format.setVersion(3, 3)
        format.setDepthBufferSize(24)
        format.setStencilBufferSize(8)
        format.setProfile(PySide6.QtGui.QSurfaceFormat.OpenGLContextProfile.CoreProfile)
        PySide6.QtGui.QSurfaceFormat.setDefaultFormat(format)
        self.from_python = PySide6.QtQml.QQmlPropertyMap()
        for key, value in from_python_defaults.items():
            self.from_python.setProperty(key, value)
        self.to_python = PySide6.QtQml.QQmlPropertyMap()
        if to_python is not None:
            self.to_python.valueChanged.connect(to_python)
        self.app = PySide6.QtGui.QGuiApplication(argv)
        PySide6.QtQuick.QQuickWindow.setGraphicsApi(
            PySide6.QtQuick.QSGRendererInterface.GraphicsApi.OpenGL
        )
        self.engine = PySide6.QtQml.QQmlApplicationEngine()
        self.engine.rootContext().setContextProperty("from_python", self.from_python)
        self.engine.rootContext().setContextProperty("to_python", self.to_python)
        self.engine.loadData(qml.encode())
        if not self.engine.rootObjects()[0].isWindowType():
            raise Exception("the QML root component must be a Window")
        self.window: PySide6.QtQuick.QQuickWindow = self.engine.rootObjects()[0]  # type: ignore

    def event_display(self, object_name: typing.Optional[str] = None) -> EventDisplay:
        if object_name is None:
            child = self.window.findChild(EventDisplay)
        else:
            child = self.window.findChild(EventDisplay, name=object_name)
        if child is None:
            if object_name is None:
                raise Exception(f"no EventDisplay found in the QML tree")
            else:
                raise Exception(
                    f'no EventDisplay with name: "{object_name}" found in the QML tree'
                )
        return child

    def frame_display(self, object_name: typing.Optional[str] = None) -> FrameDisplay:
        if object_name is None:
            child = self.window.findChild(FrameDisplay)
        else:
            child = self.window.findChild(FrameDisplay, name=object_name)
        if child is None:
            if object_name is None:
                raise Exception(f"no FrameDisplay found in the QML tree")
            else:
                raise Exception(
                    f'no FrameDisplay with name: "{object_name}" found in the QML tree'
                )
        return child

    def line_series(
        self, object_name: typing.Optional[str] = None
    ) -> PySide6.QtGraphs.QLineSeries:
        if object_name is None:
            child = self.window.findChild(PySide6.QtGraphs.QLineSeries)
        else:
            child = self.window.findChild(
                PySide6.QtGraphs.QLineSeries, name=object_name
            )
        if child is None:
            if object_name is None:
                raise Exception(
                    f"no PySide6.QtGraphs.QLineSeries found in the QML tree"
                )
            else:
                raise Exception(
                    f'no PySide6.QtGraphs.QLineSeries with name: "{object_name}" found in the QML tree'
                )
        return child

    def run(self) -> int:
        self.window.show()
        return self.app.exec()
