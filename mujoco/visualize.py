#!/usr/bin/env python3
"""Open the STEP-derived Eddie mobile platform."""

from pathlib import Path

import mujoco
import mujoco.viewer
import time


def main() -> int:
    model = mujoco.MjModel.from_xml_path(str(Path(__file__).with_name("mobile_platform.xml")))
    data = mujoco.MjData(model)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = (0.0, 0.0, 0.15)
        viewer.cam.distance = 1.4
        viewer.cam.azimuth = 135.0
        viewer.cam.elevation = -25.0
        while viewer.is_running():
            start = time.time()
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0, model.opt.timestep - (time.time() - start)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
