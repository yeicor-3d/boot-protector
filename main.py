# %%
from contextlib import suppress
from math import *
from os import getenv
from build123d import *
from tqdm import tqdm
with suppress(ImportError):  # Optional
    import ocp_vscode
from OCP.gp import (gp_Quaternion, gp_Extrinsic_XYZ, gp_Vec)

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
samples_yaw = 8
samples_pitch = 14

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


# TODO: New sampling plan: faces().normal.Z > 0, take vertices(), smooth them and add a special bottom layer
all_collisions = []
for yaw_sample in tqdm(map(lambda x: x / (samples_yaw-1), range(samples_yaw))):
    all_collisions_line = []
    for pitch_sample in tqdm(map(lambda x: x / (samples_pitch - 1), range(samples_pitch))):
        if abs(pitch_sample - 0.5) > 0.5:
            continue  # Ignore extreme pitch samples
        pos, _ = take_subsample(yaw_sample, pitch_sample)
        all_collisions_line.append(pos)
    all_collisions.append(all_collisions_line)
all_collisions_debug = [item for row in all_collisions for item in row]

with BuildPart() as obj:
    surf = Face.make_surface_from_array_of_points(all_collisions)
    add(surf.thicken(-prot_thickness))

    # Clean cut for post-processing!
    add(boot_prot_relevant_filter, mode=Mode.INTERSECT)

    # # Extras: holes along edges
    obj_corner_edges = obj.edges().group_by(
        Axis.Z)[0].group_by(SortBy.LENGTH)[-1]  # Not outer, but works anyway
    obj_corner_edges += obj.edges().group_by(
        Axis.X)[0].group_by(SortBy.LENGTH)[-1]
    top_face = obj.faces().group_by(Axis.Z)[-2][0]
    for edge, inside_dir in [(obj_corner_edges[0], Vector(0, 0, 1)), (obj_corner_edges[-1], Vector(1, 0, 0))]:
        # For each edge, compute and add holes
        steps = edge.length / sew_hole_sep
        step_relative_size = steps / int(steps)
        for step in tqdm(range(1, int(steps))):
            progress = step / steps * step_relative_size

            # Extract tangent from edge
            tangent = edge % progress

            # Extract basic data from edge
            pos = edge@progress + inside_dir *\
                (sew_hole_radius + prot_thickness)

            # Maybe: project the position on the surface

            # Compute the orientation of the hole
            wanted_up = tangent.cross(inside_dir)
            quat = gp_Quaternion(gp_Vec(0, 0, 1), gp_Vec(
                wanted_up.X, wanted_up.Y, wanted_up.Z))
            angles = quat.GetEulerAngles(gp_Extrinsic_XYZ)
            loc = Location(pos, (degrees(angles[0]), degrees(
                angles[1]), degrees(angles[2])))

            # Build the hole
            with BuildPart(loc, mode=Mode.SUBTRACT):
                Cylinder(sew_hole_radius, boot_bb.size.Y)

            # Clean up the hole with a fillet
            fillet(obj.edges(Select.LAST), prot_thickness/2.1)

    # Extras: extrude bottom and fillet all remaining sharp edges
    extrude(obj.faces().group_by(Axis.Z)[
            0], prot_clip_z * 0.75, dir=(0, 0, -1))
    fillet_edges = obj.edges().group_by(Axis.X)[0]
    fillet_edges -= fillet_edges.group_by(SortBy.LENGTH)[0]
    fillet_edges -= fillet_edges.group_by(SortBy.LENGTH)[0]
    fillet(fillet_edges, prot_thickness/2.1)
    fillet_edges = obj.edges().group_by(Axis.Z)[0]
    fillet(fillet_edges, prot_thickness/2.1)

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
