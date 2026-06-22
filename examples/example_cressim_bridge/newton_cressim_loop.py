import csv
from pathlib import Path

from cressim_client import CressimBridge


def get_cutter_pose_from_newton_side_loop(step_id, dt):
    """
    This function represents the Newton-side control output.

    Later, replace this function with:
        cutter pose = Newton robot end-effector pose

    Current test:
        cutter starts at y = 1.2
        after step 10, cutter moves downward by 0.01 per step
    """
    cutter_x = 0.0
    cutter_y = 1.2
    cutter_z = 0.0

    if step_id > 10:
        cutter_y -= float(step_id - 10) * 0.01

    return cutter_x, cutter_y, cutter_z


def main():
    dt = 0.02
    num_steps = 300

    output_path = Path(
        "/workspace/CRESSim-MPM/build/bin/newton_cressim_loop_log.csv"
    )

    sim = CressimBridge()

    try:
        target_count = sim.get_target_particle_count()
        print(f"Target particle count = {target_count}")

        with output_path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "step",
                "time",
                "cutter_x",
                "cutter_y",
                "cutter_z",
                "centroid_x",
                "centroid_y",
                "centroid_z",
            ])

            for step_id in range(num_steps):
                time = step_id * dt

                cutter_x, cutter_y, cutter_z = get_cutter_pose_from_newton_side_loop(
                    step_id,
                    dt,
                )

                # Newton-side loop sends cutter pose to CRESSim.
                sim.set_cutter_pose(
                    cutter_x,
                    cutter_y,
                    cutter_z,
                )

                # CRESSim advances one step.
                sim.step(dt)

                # Newton-side loop reads target centroid from CRESSim.
                centroid_x, centroid_y, centroid_z = sim.get_target_centroid()

                writer.writerow([
                    step_id,
                    time,
                    cutter_x,
                    cutter_y,
                    cutter_z,
                    centroid_x,
                    centroid_y,
                    centroid_z,
                ])

                if step_id % 10 == 0:
                    print(
                        f"step {step_id}, "
                        f"cutter = "
                        f"{cutter_x:.6f}, "
                        f"{cutter_y:.6f}, "
                        f"{cutter_z:.6f}, "
                        f"target centroid = "
                        f"{centroid_x:.8f}, "
                        f"{centroid_y:.8f}, "
                        f"{centroid_z:.8f}"
                    )

        print(f"Saved log to: {output_path}")

    finally:
        sim.close()


if __name__ == "__main__":
    main()
