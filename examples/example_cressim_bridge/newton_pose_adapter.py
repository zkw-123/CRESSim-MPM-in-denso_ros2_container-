import warp as wp


@wp.kernel
def extract_body_translation_kernel(
    body_q: wp.array(dtype=wp.transform),
    body_id: int,
    out_pos: wp.array(dtype=wp.vec3),
):
    tf = body_q[body_id]
    out_pos[0] = wp.transform_get_translation(tf)


class NewtonPoseAdapter:
    def __init__(self, device="cuda:0"):
        self.device = device
        self._out_pos = wp.zeros(
            1,
            dtype=wp.vec3,
            device=self.device,
        )

    def get_body_position(self, state, body_id):
        """
        Read one rigid body position from Newton state.body_q.

        state:
            Newton State object.
            It must contain state.body_q.

        body_id:
            Newton body index of cutter / end-effector.
        """
        wp.launch(
            kernel=extract_body_translation_kernel,
            dim=1,
            inputs=[
                state.body_q,
                int(body_id),
                self._out_pos,
            ],
            device=self.device,
        )

        pos_np = self._out_pos.numpy()[0]

        return (
            float(pos_np[0]),
            float(pos_np[1]),
            float(pos_np[2]),
        )
