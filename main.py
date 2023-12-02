# %%
from contextlib import suppress
from math import *
from os import getenv
from typing import Optional
from build123d import *
from tqdm import tqdm
with suppress(ImportError):  # Optional
    import ocp_vscode

# %%
# ================== PARAMETERS ==================
# 3D printing basics
tol = 0.1 * MM  # Tolerance (tighter than usual)
wall_min = 0.4 * MM  # Minimum wall width
wall = 3 * wall_min  # Recommended width for most walls of this print
eps = 1e-5 * MM  # A small number

# Sampling surface parameters
samples_yaw = 8
samples_pitch = 14

# Protection parameters
prot_offset = 2 * MM
prot_thickness = 2 * wall
prot_clip_x = -2 * CM
prot_clip_z = 1.75 * CM


# ================== MODELLING ==================

# Load the boot model, which must be aligned with the base at the origin, centered in X and Y and with forward direction in X+
# Slow import...
boot = Mesher().read("boot.stl" if getenv(
    "final_boot") else "boot-simplified.stl")[0]
boot_bb = boot.bounding_box()

boot_relevant = boot & Box(boot_bb.max.X + 2 - prot_clip_x, boot_bb.size.Y, boot_bb.max.Z, align=(
    Align.MIN, Align.CENTER, Align.MIN)).translate((prot_clip_x, boot_bb.center().Y, 0))

boot_prot_relevant_filter = Box(boot_bb.max.X + 2 - prot_clip_x + prot_thickness, boot_bb.size.Y + prot_thickness*4, boot_bb.max.Z, align=(
    Align.MIN, Align.CENTER, Align.MIN)).translate((prot_clip_x, boot_bb.center().Y, prot_clip_z))
boot_prot_relevant = boot & boot_prot_relevant_filter


# %%

def take_subsample(yaw_rel: float, pitch_rel: float) -> tuple[Vector, Vector]:
    """Returns the result of sampling the surface of the boot, given "yaw" and "pitch" angles"""
    def mix(v1, v2, by):
        return v1 * (1 - by) + v2 * by

    # HACK: Focus yaw samples towards the bottom of the boot
    yaw_rel = mix(yaw_rel ** 3, yaw_rel, abs(pitch_rel-0.5) * 2)
    # HACK: Focus pitch samples towards the center of the boot
    pitch_rel = tan(1.9 * (pitch_rel - 0.5)) / 3 + 0.5

    # HACK: Extend the surface samples a little bit to cut them cleanly later
    extend_surf = prot_thickness * 3

    offset = Vector(prot_clip_x * yaw_rel - extend_surf,
                    0, prot_clip_z - extend_surf)
    with BuildLine(Plane.XY.move(Location(offset, (0, yaw_rel * -90, 0)))) as support_line:
        CenterArc((0, 0), boot_bb.size.X, -89, 178)

    ray_from = support_line.line@pitch_rel
    ray_to = offset

    # Grab the position of the first intersection of the ray with the boot
    pos, normal = boot.find_intersection(
        Axis(ray_from, ray_to - ray_from))[0]
    return pos + normal * prot_offset, normal


all_collisions = []
for yaw_sample in tqdm(map(lambda x: x / (samples_yaw-1), range(samples_yaw))):
    all_collisions_line = []
    for pitch_sample in tqdm(map(lambda x: x / (samples_pitch - 1), range(samples_pitch))):
        if abs(pitch_sample - 0.5) > 0.5:
            continue  # Ignore extreme pitch samples
        loc, _ = take_subsample(yaw_sample, pitch_sample)
        all_collisions_line.append(loc)
    all_collisions.append(all_collisions_line)
all_collisions_debug = [item for row in all_collisions for item in row]

with BuildPart() as obj:
    surf = Face.make_surface_from_array_of_points(all_collisions)
    add(surf.thicken(-prot_thickness))

    # Clean cut for post-processing!
    add(boot_prot_relevant_filter, mode=Mode.INTERSECT)

    # # Extras: holes
    obj_corner_edges = obj.edges().group_by(Axis.Z)[0]
    obj_corner_edges += obj.edges().group_by(Axis.X)[0]

    # # Extras: extrude bottom and fillet
    # extrude(obj.faces().group_by(Axis.Z, tol_digits=1)
    #         [0], prot_clip_z * 0.75, dir=(0, 0, -1))
    # fillet_bottom = obj.edges().group_by(Axis.Z)[0].filter_by(
    #     lambda x: x.length > prot_thickness * 2)
    # fillet(fillet_bottom[0], prot_thickness/2)
    pass

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
