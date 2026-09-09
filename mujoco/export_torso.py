#!/usr/bin/env python3
"""Export the Eddie torso from the "Torso with arms" STEP into MuJoCo STLs.

The arms are left out; this is the tower up to and including the arm mounting
plates, written as torso.stl and torso_mounts.stl centred on the tower's bbox.

Requires cadquery-ocp: uv run --with cadquery python export_torso.py STEP_FILE OUTPUT_DIR
"""

from collections import defaultdict
import json
from pathlib import Path
import sys

from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.XCAFPrs import XCAFPrs, XCAFPrs_IndexedDataMapOfShapeStyle

from export_step_meshes import (MAX_TRIANGLES, area, color_groups, compound, dominant_rgb,
                                hex_rgb, sub_shapes, write_stl)

ARM_LABELS = (5, 6)
# The mounting brackets sit inside the arm components; the bracket stays within
# the tower's half width where the arm's own base reaches well past it.
MOUNT_MAX_X = 140.0   # [mm] tower half width plus a margin
MOUNT_MIN_Y = 280.0   # [mm] the brackets sit high on the tower
# dark anodised on the robot, not the tower's light grey
MOUNT_PAINT = (0.42, 0.42, 0.43)
DEFLECTION = 5.0


def main(step_file, output_dir):
    app = XCAFApp_Application.GetApplication_s()
    document = TDocStd_Document(TCollection_ExtendedString("step"))
    app.NewDocument(TCollection_ExtendedString("MDTV-XCAF"), document)
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(True)
    if reader.ReadFile(str(step_file)) != 1 or not reader.Transfer(document):
        raise RuntimeError(f"Could not import {step_file}")

    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())
    roots = TDF_LabelSequence()
    shape_tool.GetFreeShapes(roots)
    styles = XCAFPrs_IndexedDataMapOfShapeStyle()
    XCAFPrs.CollectStyleSettings_s(roots.Value(1), TopLoc_Location(), styles)
    components = TDF_LabelSequence()
    shape_tool.GetComponents_s(roots.Value(1), components)

    groups = defaultdict(list)
    mount_groups = defaultdict(list)
    box = Bnd_Box()
    for i in range(1, components.Length() + 1):
        shape = shape_tool.GetShape_s(components.Value(i))
        target = groups
        if i in ARM_LABELS:
            mounts = []
            for solid in sub_shapes(shape, TopAbs_SOLID):
                bounds = Bnd_Box()
                BRepBndLib.Add_s(solid, bounds, False)
                xmin, ymin, _, xmax, ymax, _ = bounds.Get()
                if max(abs(xmin), abs(xmax)) <= MOUNT_MAX_X and ymax >= MOUNT_MIN_Y:
                    mounts.append(solid)
            if not mounts:
                continue
            # the filter also catches the cable gland beside it
            bracket = max(mounts, key=area)
            print(f"  component {i}: {len(mounts)} solid(s) at the mount, kept the "
                  f"bracket at {area(bracket) / 1e2:.1f} cm2")
            shape = compound([bracket])
            target = mount_groups
        BRepBndLib.Add_s(shape, box, False)
        for rgb, faces in color_groups(styles, shape).items():
            target[rgb].extend(faces)

    xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
    # both meshes share an origin so they take the same pos
    origin = ((xmin + xmax) / 2.0, (ymin + ymax) / 2.0, (zmin + zmax) / 2.0)

    output_dir.mkdir(parents=True, exist_ok=True)
    manifest = []
    for name, collected, paint in (("torso", groups, None),
                                   ("torso_mounts", mount_groups, MOUNT_PAINT)):
        rgb = paint or dominant_rgb(collected)
        faces = [face for group in collected.values() for face in group]
        count = write_stl(faces, origin, output_dir / f"{name}.stl", DEFLECTION)
        if count > MAX_TRIANGLES:
            raise RuntimeError(f"{name}.stl has {count} triangles, over MuJoCo's mesh limit")
        manifest.append({"part": name, "file": f"{name}.stl", "rgb": [round(c, 4) for c in rgb],
                         "hex": hex_rgb(rgb), "triangles": count})
        print(f"{name:13s} {hex_rgb(rgb)}  faces={len(faces):5d}  triangles={count}")

    (output_dir / "torso_colors.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"  bbox {tuple(round(v, 1) for v in box.Get())} mm\n"
          f"  centred on {tuple(round(v, 1) for v in origin)} mm")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit(f"usage: {Path(sys.argv[0]).name} STEP_FILE OUTPUT_DIR")
    main(Path(sys.argv[1]), Path(sys.argv[2]))
