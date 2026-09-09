#!/usr/bin/env python3
"""Export the Eddie mobile-platform STEP assembly into MuJoCo STL assets.

One mesh per physical part, not per CAD colour: the platform is a casing, a top
cover, the laser scanners, and one drive unit that the assembly instances four
times, so the drive body, its two wheels and their tyre ring are exported once in
their own frames and placed by the MJCF.  Each mesh gets the colour that dominates
its surface in the CAD, except where the built robot is painted differently.
Writes <part>.stl plus a colors.json manifest.

Requires cadquery-ocp: uv run --with cadquery python export_step_meshes.py STEP_FILE assets
"""

from collections import defaultdict
import json
from pathlib import Path
import sys

from OCP.BRep import BRep_Builder, BRep_Tool
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.GProp import GProp_GProps
from OCP.Quantity import Quantity_TOC_sRGB
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS, TopoDS_Compound
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.XCAFPrs import XCAFPrs, XCAFPrs_IndexedDataMapOfShapeStyle
from OCP.gp import gp_Trsf, gp_Vec

# STEP coordinates are millimetres, y up.
BASE_ORIGIN = (272.0, 398.7, 444.0)
# The unit's slew-bearing axis, 10.00 mm behind the wheel axle: that offset is
# the castor offset.  Both wheel origins sit on the tyres' common revolution axis
# (74.479, 234.777), read from the STEP's toroidal faces: a tenth of a millimetre
# off that axis makes an eccentric wheel that lifts its partner once per turn.
PIVOT_ORIGIN = (64.48, 271.4, 256.13)
WHEEL_ORIGINS = {
    "a": (74.479, 234.777, 215.630),
    "b": (74.479, 234.777, 296.630),
}
# The four drive units are one part placed four times, so only this one is exported.
MODULE_LABEL = 4
WHEEL_SOLIDS = {"a": slice(5, 45), "b": slice(45, 85)}
# Base components visible from outside; the casing hides the rest.
BASE_PARTS = {
    "casing": (8, 28, 29),
    "cover": (34,),
    "lidar": (9, 10, 13, 14, 15),
    # fills the cutout in the casing, which is open without it
    "panel": (23, 24, 25, 26, 27, 30, 31, 32),
}
# The CAD's outer skin and drive colours are placeholders; these are the built robot's.
PAINT = {"casing": (0.25, 0.25, 0.26), "panel": (0.25, 0.25, 0.26), "drive": (0.35, 0.35, 0.35)}
# Component 29's first solid duplicates the casing's outer surface and z-fights with
# it; its other solids are the service-bay door, which the casing needs.
SKIP_SOLIDS = {29: (0,)}
# The tread ring is the only collision geometry, so it stays its own mesh.
TYRE_HEX = "404040"
DEFAULT_RGB = (0.6, 0.6, 0.6)
# MuJoCo's STL decoder rejects a mesh with more than 200000 triangles.
MAX_TRIANGLES = 180_000
# Chord tolerance per part; the drive unit is mostly hidden.  The tyres are the
# contact geometry: at the default tolerance they come out as 27-gons whose
# 0.35 mm facets exceed the contact penetration, so a unit's two wheels unload
# in turn and slip, which drifts the platform and the odometry with it.
DEFLECTION = {"drive": 8.0, "tyre_a": 0.05, "tyre_b": 0.05}
DEFAULT_DEFLECTION = 5.0
ANGULAR_DEFLECTION = {"tyre_a": 0.05, "tyre_b": 0.05}  # [rad]
DEFAULT_ANGULAR_DEFLECTION = 0.5


def compound(shapes):
    builder = BRep_Builder()
    result = TopoDS_Compound()
    builder.MakeCompound(result)
    for shape in shapes:
        builder.Add(result, shape)
    return result


def sub_shapes(shape, kind):
    result = []
    explorer = TopExp_Explorer(shape, kind)
    while explorer.More():
        result.append(explorer.Current())
        explorer.Next()
    return result


def area(shape):
    props = GProp_GProps()
    BRepGProp.SurfaceProperties_s(shape, props)
    return props.Mass()


def hex_rgb(rgb):
    return "".join(f"{round(255 * channel):02x}" for channel in rgb)


def style_rgb(styles, shape):
    """The sRGB surface colour XCAF resolved for this exact shape, if any."""
    if not styles.Contains(shape):
        return None
    style = styles.FindFromKey(shape)
    if not style.IsSetColorSurf():
        return None
    return style.GetColorSurf().Values(Quantity_TOC_sRGB)


def color_groups(styles, shape, inherited=None):
    """Map sRGB colour -> faces of `shape`, resolving face over solid over part."""
    groups = defaultdict(list)
    outer = style_rgb(styles, shape) or inherited
    for solid in sub_shapes(shape, TopAbs_SOLID):
        solid_rgb = style_rgb(styles, solid) or outer
        for face in sub_shapes(solid, TopAbs_FACE):
            groups[style_rgb(styles, face) or solid_rgb or DEFAULT_RGB].append(face)
    return groups


def dominant_rgb(groups):
    return max(groups, key=lambda rgb: sum(area(face) for face in groups[rgb]))


def write_stl(faces, origin, path, deflection, angular):
    transform = gp_Trsf()
    transform.SetTranslation(gp_Vec(*(-value for value in origin)))
    shape = BRepBuilderAPI_Transform(compound(faces), transform, True).Shape()
    BRepMesh_IncrementalMesh(shape, deflection, False, angular, True)
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    if not writer.Write(shape, str(path)):
        raise RuntimeError(f"Could not write {path}")
    return sum(triangles(face) for face in sub_shapes(shape, TopAbs_FACE))


def triangles(face):
    location = TopLoc_Location()
    triangulation = BRep_Tool.Triangulation_s(TopoDS.Face_s(face), location)
    return triangulation.NbTriangles() if triangulation is not None else 0


def write_part(name, groups, origin, output_dir, manifest, report):
    rgb = PAINT.get(name) or dominant_rgb(groups)
    faces = [face for group in groups.values() for face in group]
    deflection = DEFLECTION.get(name, DEFAULT_DEFLECTION)
    angular = ANGULAR_DEFLECTION.get(name, DEFAULT_ANGULAR_DEFLECTION)
    count = write_stl(faces, origin, output_dir / f"{name}.stl", deflection, angular)
    if count > MAX_TRIANGLES:
        raise RuntimeError(f"{name}.stl has {count} triangles, over MuJoCo's mesh limit")
    manifest.append({"part": name, "file": f"{name}.stl", "rgb": [round(c, 4) for c in rgb],
                     "hex": hex_rgb(rgb), "triangles": count})
    report.append(f"{name:8s} {hex_rgb(rgb)}  faces={len(faces):5d}  triangles={count:6d}"
                  f"  deflection={deflection} mm")


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
    component_shapes = [shape_tool.GetShape_s(components.Value(i)) for i in range(1, components.Length() + 1)]

    output_dir.mkdir(parents=True, exist_ok=True)
    manifest, report = [], []

    for name, labels in BASE_PARTS.items():
        groups = defaultdict(list)
        for label in labels:
            shape = component_shapes[label - 1]
            if label in SKIP_SOLIDS:
                solids = sub_shapes(shape, TopAbs_SOLID)
                shape = compound([s for i, s in enumerate(solids) if i not in SKIP_SOLIDS[label]])
            for rgb, faces in color_groups(styles, shape).items():
                groups[rgb].extend(faces)
        write_part(name, groups, BASE_ORIGIN, output_dir, manifest, report)

    module = component_shapes[MODULE_LABEL - 1]
    module_solids = sub_shapes(module, TopAbs_SOLID)
    inherited = style_rgb(styles, module)
    body_solids = module_solids[:5] + module_solids[85:]
    write_part("drive", color_groups(styles, compound(body_solids), inherited),
               PIVOT_ORIGIN, output_dir, manifest, report)
    for side, solids in WHEEL_SOLIDS.items():
        groups = color_groups(styles, compound(module_solids[solids]), inherited)
        tyre = {rgb: faces for rgb, faces in groups.items() if hex_rgb(rgb) == TYRE_HEX}
        rim = {rgb: faces for rgb, faces in groups.items() if hex_rgb(rgb) != TYRE_HEX}
        write_part(f"tyre_{side}", tyre, WHEEL_ORIGINS[side], output_dir, manifest, report)
        write_part(f"wheel_{side}", rim, WHEEL_ORIGINS[side], output_dir, manifest, report)

    (output_dir / "colors.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print("\n".join(report))


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit(f"usage: {Path(sys.argv[0]).name} STEP_FILE OUTPUT_DIR")
    main(Path(sys.argv[1]), Path(sys.argv[2]))
