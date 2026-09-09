#!/usr/bin/env python3
"""Open the STEP-derived mobile platform in the mj_kdl_wrapper simulate viewer.

Run from the workspace venv that carries the wrapper's Python bindings:
    source .venv-ros/bin/activate && python visualize_mobile_platform.py
"""

from pathlib import Path

import mj_kdl_wrapper as mjk


def main() -> int:
    spec = mjk.SceneSpec()
    spec.timestep = 0.002
    # Attaching a robot keeps only its bodies, so the scene supplies the floor.
    spec.add_floor = True
    spec.add_skybox = True
    robot = mjk.RobotSpec()
    robot.path = str(Path(__file__).with_name("mobile_platform.xml"))
    spec.robots = [robot]

    scene = mjk.Scene.build(spec)
    viewer = mjk.SimulateViewer.open(scene, "eddie mobile platform")
    try:
        viewer.set_free_camera(azimuth=135.0, elevation=-15.0, distance=1.8,
                               lookat=(0.0, 0.0, 0.12))
        while viewer.is_running():
            viewer.step()
            viewer.pace()
    finally:
        viewer.close()
        scene.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
