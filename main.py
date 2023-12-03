# %%
from contextlib import suppress
from math import *
from os import getenv
from build123d import *
from tqdm import tqdm
with suppress(ImportError):  # Optional
    import ocp_vscode
from OCP.gp import (gp_Quaternion, gp_Extrinsic_XYZ, gp_Vec)
import random
random.seed(0)

# %%
# ================== PARAMETERS ==================
# 3D printing basics
tol = 0.1 * MM  # Tolerance (tighter than usual)
wall_min = 0.4 * MM  # Minimum wall width
wall = 3 * wall_min  # Recommended width for most walls of this print
eps = 1e-5 * MM  # A small number
# Whether to generate the final model or a simplified one
final_model = getenv("final") is not None

# Sampling surface parameters
samples = 15, 10

# Protection parameters
prot_offset = 2 * MM
prot_thickness = 2 * wall
prot_clip_x = -2 * CM
prot_clip_z = 1.75 * CM

# Sewing parameters
sew_hole_radius = (1.5 * MM) if final_model else (5 * MM)
sew_hole_sep = 2 * 2 * sew_hole_radius

# ================== MODELLING ==================

# Load the boot model, which must be aligned with the base at the origin, centered in X and Y and with forward direction in X+
# Slow import...
boot = Mesher().read(
    "boot.stl" if final_model else "boot-simplified.stl")[0]
boot_bb = boot.bounding_box()

boot_rel = boot & Box(boot_bb.max.X + 2 - prot_clip_x, boot_bb.size.Y, boot_bb.max.Z, align=(
    Align.MIN, Align.CENTER, Align.MIN)).translate((prot_clip_x, boot_bb.center().Y, 0))
boot_rel_bb = boot_rel.bounding_box()


# %%

# TODO: New sampling plan: faces().normal.Z > 0, take vertices(), smooth them and add a special bottom layer
# Create a surface of all relevant up-facing faces
surf = boot_rel.faces().group_by(
    lambda f: f.normal_at(f.center()).Z > eps)[-1]
shell = Shell.make_shell(surf)

# Take 2D point samples from the surface
surf_array = []
for x_rel_int in range(samples[0]):
    x_rel = x_rel_int / (samples[0] - 1)
    x_abs = (x_rel) * (boot_rel_bb.size.X) + boot_rel_bb.min.X - eps
    # print("Sampling at X=%f" % x_abs)

    plane = Face.make_rect(1 * M, 1 * M, Plane.YZ).translate((x_abs, 0, 0))
    tmp = shell.intersect(extrude(plane, amount=1)).edges().group_by(Axis.X)[0]
    with BuildLine() as curve:
        full_sample = tmp.vertices().sort_by(Axis.Y)
        for a, b in zip(full_sample, full_sample[1:]):
            print(a.vertex().center(), b.vertex().center())
            if a != b:
                Line(a.vertex().center(), b.vertex().center())

    samples_dat = [
        curve.line@t for t in map(lambda t: t / (samples[1]-1), range(samples[1]))]

    # TODO: Add a bottom layer of points at a fixed Z

    surf_array.append(samples_dat)
    # del tmp

test = [p for l in surf_array for p in l]
test2 = Face.make_surface_from_array_of_points(surf_array, max_deg=1)
# swp = sweep(surf_sweep_sections, Line([(0,0,0), (1,0,0)]))
# test = sweep(surf_sweep_sections, tmp, multisection=True)
# print("  %d points" % len(test.vertices()))
# del test

# with BuildPart() as obj:
#     pass
# surf = Face.make_surface_from_array_of_points(surf_points)
# add(surf.thicken(-prot_thickness))

# # Clean cut for post-processing!
# add(boot_prot_relevant_filter, mode=Mode.INTERSECT)

# # # Extras: holes along edges
# obj_corner_edges = obj.edges().group_by(
#     Axis.Z)[0].group_by(SortBy.LENGTH)[-1]  # Not outer, but works anyway
# obj_corner_edges += obj.edges().group_by(
#     Axis.X)[0].group_by(SortBy.LENGTH)[-1]
# top_face = obj.faces().group_by(Axis.Z)[-2][0]
# for edge, inside_dir in [(obj_corner_edges[0], Vector(0, 0, 1)), (obj_corner_edges[-1], Vector(1, 0, 0))]:
#     # For each edge, compute and add holes
#     steps = edge.length / sew_hole_sep
#     step_relative_size = steps / int(steps)
#     for step in tqdm(range(1, int(steps))):
#         progress = step / steps * step_relative_size

#         # Extract tangent from edge
#         tangent = edge % progress

#         # Extract basic data from edge
#         pos = edge@progress + inside_dir *\
#             (sew_hole_radius + prot_thickness)

#         # Maybe: project the position on the surface

#         # Compute the orientation of the hole
#         wanted_up = tangent.cross(inside_dir)
#         quat = gp_Quaternion(gp_Vec(0, 0, 1), gp_Vec(
#             wanted_up.X, wanted_up.Y, wanted_up.Z))
#         angles = quat.GetEulerAngles(gp_Extrinsic_XYZ)
#         loc = Location(pos, (degrees(angles[0]), degrees(
#             angles[1]), degrees(angles[2])))

#         # Build the hole
#         with BuildPart(loc, mode=Mode.SUBTRACT):
#             Cylinder(sew_hole_radius, boot_bb.size.Y)

#         # Clean up the hole with a fillet
#         fillet(obj.edges(Select.LAST), prot_thickness/2.1)

# # Extras: extrude bottom and fillet all remaining sharp edges
# extrude(obj.faces().group_by(Axis.Z)[
#         0], prot_clip_z * 0.75, dir=(0, 0, -1))
# fillet_edges = obj.edges().group_by(Axis.X)[0]
# fillet_edges -= fillet_edges.group_by(SortBy.LENGTH)[0]
# fillet_edges -= fillet_edges.group_by(SortBy.LENGTH)[0]
# fillet(fillet_edges, prot_thickness/2.1)
# fillet_edges = obj.edges().group_by(Axis.Z)[0]
# fillet(fillet_edges, prot_thickness/2.1)

if "ocp_vscode" in locals():
    ocp_vscode.reset_show()
    ocp_vscode.show_all()

# %%
# ================== SHOWING/EXPORTING ==================

export = True
try:
    if "show_object" in locals():
        show_object(obj, "boot-protector")  # type: ignore
        export = False
    elif "ocp_vscode" in locals():
        ocp_vscode.reset_show()
        ocp_vscode.show_all()
        export = False
except Exception as ex:
    print("Cannot show model, exporting to STL instead (%s)" % ex)

if export:
    obj.part.export_stl("boot-protector.stl")
