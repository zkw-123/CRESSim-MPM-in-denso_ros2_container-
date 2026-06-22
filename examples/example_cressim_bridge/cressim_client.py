import ctypes
from pathlib import Path


class CressimBridge:
    def __init__(self, build_bin="/workspace/CRESSim-MPM/build/bin"):
        build_bin = Path(build_bin)

        self.engine_path = build_bin / "libcrmpm_engine.so"
        self.bridge_path = build_bin / "libcressim_bridge.so"

        # Load engine first so dependent symbols are visible.
        self.engine = ctypes.CDLL(
            str(self.engine_path),
            mode=ctypes.RTLD_GLOBAL,
        )

        self.lib = ctypes.CDLL(
            str(self.bridge_path),
            mode=ctypes.RTLD_GLOBAL,
        )

        self.lib.cressim_create.restype = ctypes.c_void_p

        self.lib.cressim_set_cutter_pose.argtypes = [
            ctypes.c_void_p,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
        ]

        self.lib.cressim_step.argtypes = [
            ctypes.c_void_p,
            ctypes.c_float,
        ]

        self.lib.cressim_get_target_centroid.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_float),
        ]

        self.lib.cressim_get_target_particle_count.argtypes = [
            ctypes.c_void_p,
        ]
        self.lib.cressim_get_target_particle_count.restype = ctypes.c_int

        self.lib.cressim_destroy.argtypes = [
            ctypes.c_void_p,
        ]

        self.handle = self.lib.cressim_create()

        if not self.handle:
            raise RuntimeError("Failed to create CRESSim bridge simulation.")

    def set_cutter_pose(self, x, y, z):
        self.lib.cressim_set_cutter_pose(
            self.handle,
            ctypes.c_float(x),
            ctypes.c_float(y),
            ctypes.c_float(z),
        )

    def step(self, dt):
        self.lib.cressim_step(
            self.handle,
            ctypes.c_float(dt),
        )

    def get_target_centroid(self):
        centroid = (ctypes.c_float * 3)()
        self.lib.cressim_get_target_centroid(
            self.handle,
            centroid,
        )

        return float(centroid[0]), float(centroid[1]), float(centroid[2])

    def get_target_particle_count(self):
        return int(
            self.lib.cressim_get_target_particle_count(self.handle)
        )

    def close(self):
        if self.handle:
            self.lib.cressim_destroy(self.handle)
            self.handle = None

    def __del__(self):
        self.close()
