import neuromorphic_drivers as nd
import numpy as np
import vispy.app
import vispy.gloo
import vispy.util.transforms

vispy.app.use_app(backend_name="glfw")

FRAME_DURATION: float = 1.0 / 60.0
ON_COLORMAP: list[str] = ["#F4C20D", "#191919"]
OFF_COLORMAP: list[str] = ["#1E88E5", "#191919"]

VERTEX_SHADER: str = """
uniform mat4 u_projection;
attribute vec2 a_position;
attribute vec2 a_texcoord;
varying vec2 v_texcoord;
void main (void) {
    v_texcoord = a_texcoord;
    gl_Position = u_projection * vec4(a_position, 0.0, 1.0);
}
"""


def color_to_vec4(color: str) -> str:
    return f"vec4({int(color[1:3], 16) / 255.0}, {int(color[3:5], 16) / 255.0}, {int(color[5:7], 16) / 255.0}, 1.0)"


FRAGMENT_SHADER: str = f"""
uniform sampler2D u_texture;
uniform float u_t;
uniform float u_tau;
const float on_color_table_scale = {len(ON_COLORMAP) - 1};
const vec4 on_color_table[2] = vec4[]({','.join(color_to_vec4(color) for color in ON_COLORMAP)});
const float off_color_table_scale = {len(OFF_COLORMAP) - 1};
const vec4 off_color_table[2] = vec4[]({','.join(color_to_vec4(color) for color in OFF_COLORMAP)});
varying vec2 v_texcoord;
void main() {{
    float t_and_on = texture2D(u_texture, v_texcoord).r;
    float lambda = 1.0f - exp(-float(u_t - abs(t_and_on)) / u_tau);
    float scaled_lambda = lambda * (t_and_on > 0.0 ? on_color_table_scale : off_color_table_scale);
    gl_FragColor = t_and_on == 0.0 ? on_color_table[int(on_color_table_scale)] : mix(
        (t_and_on > 0.0 ? on_color_table : off_color_table)[int(scaled_lambda)],
        (t_and_on > 0.0 ? on_color_table : off_color_table)[int(scaled_lambda) + 1],
        scaled_lambda - float(int(scaled_lambda))
    );
}}
"""


class Canvas(vispy.app.Canvas):
    def __init__(
        self, sensor_width: int, sensor_height: int, device: nd.GenericDeviceOptional
    ):
        self.program = None
        vispy.app.Canvas.__init__(
            self,
            keys="interactive",
            size=(sensor_width, sensor_height),
            vsync=True,
            title=device.name().value,
        )
        self.sensor_width = sensor_width
        self.sensor_height = sensor_height
        self.device = device
        self.program = vispy.gloo.Program(VERTEX_SHADER, FRAGMENT_SHADER)
        self.ts_and_ons = np.zeros(
            (self.sensor_width, self.sensor_height),
            dtype=np.float32,
        )
        self.current_t = 0
        self.offset_t = 0
        self.texture = vispy.gloo.texture.Texture2D(
            data=self.ts_and_ons,
            format="luminance",
            internalformat="r32f",
        )
        self.projection = vispy.util.transforms.ortho(
            0, sensor_width, sensor_height, 0, -1, 1
        )
        self.program["u_projection"] = self.projection
        self.program["u_texture"] = self.texture
        self.program["u_t"] = 0
        self.program["u_tau"] = 100000
        self.coordinates = np.zeros(
            4,
            dtype=[("a_position", np.float32, 2), ("a_texcoord", np.float32, 2)],
        )
        self.coordinates["a_position"] = np.array(
            [
                [0, 0],
                [sensor_width, 0],
                [0, sensor_height],
                [sensor_width, sensor_height],
            ]
        )
        self.coordinates["a_texcoord"] = np.array([[0, 0], [0, 1], [1, 0], [1, 1]])
        self.program.bind(vispy.gloo.VertexBuffer(self.coordinates))
        self.projection = vispy.util.transforms.ortho(
            0,
            self.sensor_width,
            self.sensor_height,
            0,
            -1,
            1,
        )
        vispy.gloo.set_clear_color("#292929")
        self.timer = vispy.app.Timer(FRAME_DURATION, connect=self.update, start=True)  # type: ignore
        self.show()

    def on_resize(self, event):
        if self.program is not None:
            width, height = event.physical_size
            vispy.gloo.set_viewport(0, 0, width, height)
            self.projection = vispy.util.transforms.ortho(0, width, height, 0, -1, 1)
            self.program["u_projection"] = self.projection
            ratio = width / float(height)
            sensor_ratio = self.sensor_width / float(self.sensor_height)
            if ratio < sensor_ratio:
                w, h = width, width / sensor_ratio
                x, y = 0, int((height - h) / 2)
            else:
                w, h = height * sensor_ratio, height
                x, y = int((width - w) / 2), 0
            self.coordinates["a_position"] = np.array(
                [[x, y], [x + w, y], [x, y + h], [x + w, y + h]]
            )
            self.program.bind(vispy.gloo.VertexBuffer(self.coordinates))

    def on_draw(self, event):
        assert self.program is not None
        vispy.gloo.clear(color=True, depth=True)
        status, packet = next(device)
        if packet is not None:
            if "dvs_events" in packet:
                assert status.ring is not None and status.ring.current_t is not None
                self.program["u_t"] = np.float32(status.ring.current_t)
                self.ts_and_ons[
                    packet["dvs_events"]["x"], packet["dvs_events"]["y"]
                ] = packet["dvs_events"]["t"].astype(np.float32) * (
                    packet["dvs_events"]["on"].astype(np.float32) * 2.0 - 1.0
                )
            elif status.ring is not None and status.ring.current_t is not None:
                self.program["u_t"] = np.float32(status.ring.current_t)
        self.texture.set_data(self.ts_and_ons)
        self.program.draw("triangle_strip")


if __name__ == "__main__":
    nd.print_device_list()
    with nd.open(
        configuration=nd.inivation_davis346.Configuration(),
        iterator_timeout=FRAME_DURATION,
    ) as device:
        print(device.serial(), device.properties())
        canvas = Canvas(
            sensor_width=int(device.properties().width),
            sensor_height=int(device.properties().height),
            device=device,
        )
        vispy.app.run()
