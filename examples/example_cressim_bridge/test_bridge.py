import ctypes
import os


LIB_PATH = "/workspace/CRESSim-MPM/build/bin/libcressim_bridge.so"


def main():
    os.environ["LD_LIBRARY_PATH"] = (
        "/workspace/CRESSim-MPM/build/bin:"
        + os.environ.get("LD_LIBRARY_PATH", "")
    )

    lib = ctypes.CDLL(LIB_PATH)

    lib.cressim_create.restype = ctypes.c_void_p

    lib.cressim_set_cutter_pose.argtypes = [
        ctypes.c_void_p,
        ctypes.c_float,
        ctypes.c_float,
        ctypes.c_float,
    ]

    lib.cressim_step.argtypes = [
        ctypes.c_void_p,
        ctypes.c_float,
    ]

    lib.cressim_get_target_centroid.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_float),
    ]

    lib.cressim_get_target_particle_count.argtypes = [
        ctypes.c_void_p,
    ]
    lib.cressim_get_target_particle_count.restype = ctypes.c_int

    lib.cressim_destroy.argtypes = [
        ctypes.c_void_p,
    ]

    sim = lib.cressim_create()

    if not sim:
        raise RuntimeError("Failed to create CRESSim bridge sim.")

    try:
        target_count = lib.cressim_get_target_particle_count(sim)
        print(f"Target particle count = {target_count}")

        dt = 0.02
        num_steps = 300

        for step_id in range(num_steps):
            cutter_y = 1.2

            if step_id > 10:
                cutter_y -= float(step_id - 10) * 0.01

            lib.cressim_set_cutter_pose(
                sim,
                ctypes.c_float(0.0),
                ctypes.c_float(cutter_y),
                ctypes.c_float(0.0),
            )

            lib.cressim_step(
                sim,
                ctypes.c_float(dt),
            )

            if step_id % 10 == 0:
                centroid = (ctypes.c_float * 3)()
                lib.cressim_get_target_centroid(sim, centroid)

                print(
                    f"step {step_id}, "
                    f"cutter y = {cutter_y:.6f}, "
                    f"target centroid = "
                    f"{centroid[0]:.8f}, "
                    f"{centroid[1]:.8f}, "
                    f"{centroid[2]:.8f}"
                )

    finally:
        lib.cressim_destroy(sim)


if __name__ == "__main__":
    main()
