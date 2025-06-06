import dataclasses
import sys
import typing

import numpy
import PySide6.QtCore
import PySide6.QtGui
import PySide6.QtOpenGL
import PySide6.QtQml
import PySide6.QtQuick

EventStyle = typing.Literal["exponential", "linear", "window"]
FrameMode = typing.Literal["L", "RGB", "RGBA"]
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
for mode in ("L", "RGB", "RGBA"):
    if mode == "L":
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
        frame_display_mode_and_dtype_to_fragment_shader[
            (mode, dtype)
        ] = f"""
#version 330 core

in vec2 coordinates;
out vec4 color;
uniform {sampler} frame_sampler;

void main() {{
    {sample_type} sample = texture(frame_sampler, coordinates);
    color = vec4({r_value}, {g_value}, {b_value}, {a_value});
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


def style_to_integer(style: EventStyle) -> int:
    if style == "exponential":
        return 0
    if style == "linear":
        return 1
    if style == "window":
        return 2
    raise Exception(f"unknown {style=}")


def colormap_to_texture(
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


class EventDisplayRenderer(PySide6.QtGui.QOpenGLFunctions):
    @dataclasses.dataclass
    class Program:
        inner: PySide6.QtOpenGL.QOpenGLShaderProgram
        vertices_buffer: PySide6.QtOpenGL.QOpenGLBuffer
        vertex_array_object: PySide6.QtOpenGL.QOpenGLVertexArrayObject
        ts_and_ons_texture: PySide6.QtOpenGL.QOpenGLTexture
        colormap_texture: PySide6.QtOpenGL.QOpenGLTexture
        colormap_split: float

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
                self.ts_and_ons[numpy.logical_and(recent, numpy.logical_not(on))] += MAXIMUM_DELTA // 2
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
            colormap_texture, colormap_split = colormap_to_texture(
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
                    colormap_to_texture(
                        on_colormap=self.on_colormap,
                        off_colormap=self.off_colormap,
                    )
                )
            self.program.inner.setUniformValue1f(
                "colormap_split",  # type: ignore
                self.program.colormap_split,
            )
            self.program.colormap_texture.bind(1)

            # @DEV {
            colormap_data = numpy.zeros(
                (len(self.on_colormap) + len(self.off_colormap)) * 4,
                dtype=numpy.float32,
            )
            index = 0
            for color in reversed(self.off_colormap):
                colormap_data[index] = color.redF()
                colormap_data[index + 1] = color.greenF()
                colormap_data[index + 2] = color.blueF()
                colormap_data[index + 3] = color.alphaF()
                index += 4
            for color in self.on_colormap:
                colormap_data[index] = color.redF()
                colormap_data[index + 1] = color.greenF()
                colormap_data[index + 2] = color.blueF()
                colormap_data[index + 3] = color.alphaF()
                index += 4
            self.program.colormap_texture.setData(
                PySide6.QtOpenGL.QOpenGLTexture.PixelFormat.RGBA,
                PySide6.QtOpenGL.QOpenGLTexture.PixelType.Float32,
                colormap_data,  # type: ignore
            )
            # }

            self.program.vertex_array_object.bind()
            self.glDrawArrays(GL_TRIANGLE_STRIP, 0, 4)
            self.program.colormap_texture.release()
            self.program.ts_and_ons_texture.release()
            self.program.vertex_array_object.release()
            self.program.inner.release()
            self.window.endExternalCommands()

    def cleanup(self):
        return
        """
        if self.vbo:
            self.vbo.destroy()
            self.vbo = None

        if self.vao:
            self.vao.destroy()
            self.vao = None

        if self.program:
            self.program = None
        """


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
        self._tau: float = 200000.0
        self._on_colormap: list[PySide6.QtGui.QColor] = [
            PySide6.QtGui.QColor(0x19, 0x19, 0x19, a=0xFF),
            PySide6.QtGui.QColor(0xFB, 0xBC, 0x05, a=0xFF),
        ]
        self._off_colormap: list[PySide6.QtGui.QColor] = [
            PySide6.QtGui.QColor(0x19, 0x19, 0x19, a=0xFF),
            PySide6.QtGui.QColor(0x42, 0x85, 0xF4, a=0xFF),
        ]
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
        "colormap for OFF events (polarity 0)",
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

    def __init__(
        self,
        window: PySide6.QtQuick.QQuickWindow,
        visible: bool,
        sensor_size: PySide6.QtCore.QSize,
        mode: typing.Literal["L", "RGB", "RGBA"],
        dtype: typing.Literal["u1", "u2", "f4"],
        padding_color: PySide6.QtGui.QColor,
        clear_background: bool,
    ):
        super().__init__()
        self.window = window
        self.visible = visible
        self.sensor_size = sensor_size
        self.mode: FrameMode = mode
        if dtype == "u1":
            self.pixel_type = PySide6.QtOpenGL.QOpenGLTexture.PixelType.UInt8
            pixel_format_suffix = "_Integer"
            texture_format = "8U"
        elif dtype == "u2":
            self.pixel_type = PySide6.QtOpenGL.QOpenGLTexture.PixelType.UInt16
            pixel_format_suffix = "_Integer"
            texture_format = "16U"
        elif dtype == "f4":
            self.pixel_type = PySide6.QtOpenGL.QOpenGLTexture.PixelType.Float32
            pixel_format_suffix = ""
            texture_format = "32F"
        else:
            raise Exception(f"unsupported dtype {self.dtype}")
        if mode == "L":
            self.depth = 1
            self.pixel_format = PySide6.QtOpenGL.QOpenGLTexture.PixelFormat[
                f"Red{pixel_format_suffix}"
            ]
            self.texture_format = PySide6.QtOpenGL.QOpenGLTexture.TextureFormat[
                f"R{texture_format}"
            ]
        elif mode == "RGB":
            self.depth = 3
            self.pixel_format = PySide6.QtOpenGL.QOpenGLTexture.PixelFormat[
                f"RGB{pixel_format_suffix}"
            ]
            self.texture_format = PySide6.QtOpenGL.QOpenGLTexture.TextureFormat[
                f"RGB{texture_format}"
            ]
        elif mode == "RGBA":
            self.depth = 4
            self.pixel_format = PySide6.QtOpenGL.QOpenGLTexture.PixelFormat[
                f"RGBA{pixel_format_suffix}"
            ]
            self.texture_format = PySide6.QtOpenGL.QOpenGLTexture.TextureFormat[
                f"RGBA{texture_format}"
            ]
        else:
            raise Exception(f"unsupported mode {mode}")
        self.dtype: FrameDtype = dtype
        self.padding_color = padding_color
        self.clear_background = clear_background
        self.frame = numpy.zeros(
            sensor_size.width() * sensor_size.height() * self.depth,
            dtype=self.dtype,
        )
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
            )
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
            self.program.frame_texture.bind(0)
            self.program.frame_texture.setData(
                self.pixel_format,
                self.pixel_type,
                self.frame,  # type: ignore
            )
            self.program.vertex_array_object.bind()
            self.glDrawArrays(GL_TRIANGLE_STRIP, 0, 4)
            self.program.vertex_array_object.release()
            self.program.inner.release()
            self.window.endExternalCommands()

    def cleanup(self):
        return
        """
        if self.vbo:
            self.vbo.destroy()
            self.vbo = None

        if self.vao:
            self.vao.destroy()
            self.vao = None

        if self.program:
            self.program = None
        """


class FrameDisplay(PySide6.QtQuick.QQuickItem):

    def __init__(self, parent: typing.Optional[PySide6.QtQuick.QQuickItem] = None):
        super().__init__(parent)
        self._window: typing.Optional[PySide6.QtQuick.QQuickWindow] = None
        self._visible: bool = True
        self._renderer: typing.Optional[FrameDisplayRenderer] = None
        self._clear_area: typing.Optional[PySide6.QtCore.QRectF] = None
        self._draw_area: typing.Optional[PySide6.QtCore.QRectF] = None
        self._sensor_size: typing.Optional[PySide6.QtCore.QSize] = None
        self._mode: typing.Optional[typing.Literal["L", "RGB", "RGBA"]] = None
        self._dtype: typing.Optional[typing.Literal["u1", "u2", "f4"]] = None
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
        assert mode in {"L", "RGB", "RGBA"}
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
            del self._renderer
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
    def __init__(self, qml: str, argv: list[str] = sys.argv):
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
        self.app = PySide6.QtGui.QGuiApplication(sys.argv)
        PySide6.QtQuick.QQuickWindow.setGraphicsApi(
            PySide6.QtQuick.QSGRendererInterface.GraphicsApi.OpenGL
        )
        self.engine = PySide6.QtQml.QQmlApplicationEngine()
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
                    f'no EventDisplay with objectName: "{object_name}" found in the QML tree'
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
                    f'no FrameDisplay with objectName: "{object_name}" found in the QML tree'
                )
        return child

    def run(self) -> int:
        self.window.show()
        return self.app.exec()
