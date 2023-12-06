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
final_model = True

# Sampling surface parameters
samples = 50, 50  # Number of samples along X and Y
spline_vertices = 3  # Mitigates instability in the spline
min_samples_dist = 2 * MM  # Mitigates instability in the spline
boot_front_dist = tol  # Mitigates instability in the surface

# Protection parameters
prot_offset = 2 * MM
prot_thickness = 2 * wall
prot_clip_x = -2 * CM  # From the center of the boot

# Sewing parameters
sew_hole_radius = (1.5 * MM) if final_model else (5 * MM)
sew_hole_sep = 2 * 2 * sew_hole_radius

# ================== MODELLING ==================

# Load the boot model, which must be aligned with the base at the origin, centered in X and Y and with forward direction in X+
# Slow import...
boot = Mesher().read(
    "boot.stl" if final_model else "boot-simplified.stl")[0]
boot_bb = boot.bounding_box()

boot_rel_clip = Box(
    boot_bb.max.X + 2 - prot_clip_x + prot_offset + prot_thickness,
    boot_bb.size.Y + 4 * (prot_offset + prot_thickness),
    boot_bb.max.Z + prot_offset + prot_thickness,
    align=(Align.MIN, Align.CENTER, Align.MIN))\
    .translate((boot_bb.center().X + prot_clip_x, boot_bb.center().Y, boot_bb.min.Z))
boot_rel = boot & boot_rel_clip
boot_rel_bb = boot_rel.bounding_box()

# %%

boot_rel_off = boot_rel.offset_3d([], 3)

# %%

if "ocp_vscode" in locals():
    ocp_vscode.reset_show()
    ocp_vscode.show_all()

# %%


def remove_duplicates_keep_order(seq):
    seen = set()
    seen_add = seen.add
    return [x for x in seq if not (x in seen or seen_add(x))]


# Create a surface of all relevant up-facing faces
surf_raw = boot_rel.faces().group_by(
    lambda f: f.normal_at(f.center()).Z > eps)[-1]

# Remove false-positives by taking only the largest connected component
surf_raw_filtered = sorted(Face.sew_faces(surf_raw), key=lambda f: len(f))[-1]

# Build a shell to intersect later
shell_raw = Shell.make_shell(surf_raw_filtered)

# Take 2D point samples from the surface
surf_lines = []
for x_rel_int in tqdm(reversed(range(samples[0]))):
    x_rel = x_rel_int / (samples[0] - 1)
    x_abs = (x_rel) * (boot_rel_bb.size.X - boot_front_dist) + \
        boot_rel_bb.min.X

    face_sz = boot_rel_bb.size.Y
    plane = Face.make_rect(face_sz, face_sz, Plane.YZ).translate(
        (x_abs, boot_rel_bb.center().Y, boot_rel_bb.center().Z))
    tmp = shell_raw.intersect(extrude(plane, amount=1)
                              ).edges().group_by(Axis.X)[0]
    with BuildLine() as curve:
        full_line_points = set(map(lambda v: v.center(), tmp.vertices()))
        # Sort them by angle from the center
        full_line_points = sorted(
            full_line_points, key=lambda p: atan2(p.Y, p.Z))

        # # HACK: Ignore sequences of points that are too close, which cause instability in the spline
        full_line_points_new = [p for i, p in enumerate(full_line_points) if i == 0 or i == len(full_line_points)-1 or ((
            p - full_line_points[i-1]).length > min_samples_dist/2 and (
            p - full_line_points[i+1]).length > min_samples_dist/2)]
        if len(full_line_points_new) == 1:  # Special degenerate case of a single point
            full_line_points_new.append(full_line_points[-1])
        print("Reduced %d points to %d" %
              (len(full_line_points), len(full_line_points_new)))
        full_line_points = full_line_points_new

        # Draw the spline, splitting input data in smoothing_neighbors chunks to avoid explosions
        # spline = Spline(full_line_points)
        chunks = [full_line_points[i:i+spline_vertices]
                  for i in range(0, len(full_line_points), spline_vertices-2)]
        for chunk in chunks:
            if len(chunk) < 2:
                continue  # Ignore single-point chunks
            spl = Spline(chunk)

    curve_samples = [curve.line@(i/(samples[1] - 1))
                     for i in range(samples[1])]

    surf_lines.append(curve_samples)
    # del tmp

# HACK: Extend the surface to close it and for a clean cut later
extend_by = boot_bb.size.Z/6
minz = min([p.Z for l in surf_lines for p in l])
print("Min Z: %f" % minz)
for line in surf_lines:
    left_bottom = line[0]
    right_bottom = line[-1]
    line.append(Vector(right_bottom.X, right_bottom.Y, minz - extend_by))
    line.append(Vector(left_bottom.X, left_bottom.Y, minz - extend_by))
    line.append(left_bottom)

# # Also on the X axis
surf_lines.append([Vector(p.X - extend_by, p.Y, p.Z) for p in surf_lines[-1]])

# surf_array_debug = [p for l in surf_lines for p in l]

if "ocp_vscode" in locals():
    ocp_vscode.reset_show()
    ocp_vscode.show_all()

# %%


line0 = surf_lines[0]
surf_lines_poly = [Polyline(pts, close=True) for pts in surf_lines]

surf = Solid.make_loft(surf_lines_poly, ruled=True)

if "ocp_vscode" in locals():
    ocp_vscode.reset_show()
    ocp_vscode.show_all()

# %%

# HACK: Force triangulation to avoid extrusion issues

surf.tessellate(0.1 * MM)
surf.export_step("/tmp/surf.step")

# %%

surf_imported = import_step("/tmp/surf.step")
# surf_imported = Mesher().read("/tmp/surf.3mf")
# surf_imported = Mesher().read("/tmp/surf.stl")
# surf_imported = import_stl("/tmp/surf.stl")

if "ocp_vscode" in locals():
    ocp_vscode.reset_show()
    ocp_vscode.show_all()

# %%

surf_top = surf_imported.faces()
surf_top -= surf_top.group_by(Axis.X)[0]
surf_top -= surf_top.group_by(Axis.Z)[0]

# # Thicken the surface (slow operation)
# with BuildPart() as surf_thick:
#     cut = boot_rel_clip.translate(
#         (0, 0, minz - boot_rel_clip.bounding_box().min.Z))
#     # # surf_top_edges = surf_top.edges()
#     # add(extrude(surf_top, 10, (0, 0, 1)), mode=Mode.ADD)
#     # # add(surf.thicken(-prot_offset), mode=Mode.PRIVATE)
#     # add(cut, mode=Mode.INTERSECT)  # Clean cut for extensibility

if "ocp_vscode" in locals():
    ocp_vscode.reset_show()
    ocp_vscode.show_all()

# %%

surf_top_thick = extrude(surf_top, 10, (1, 0, 1))

# %%

# if 'front_hack' in locals():
#     raise Exception(
#         "Front hack already exists, restart and re-run all cells above :(")

# HACK: Need to add back the front face, which is removed by the make_surface hack
# Face.make_plane(Plane.YZ)
front_hack = surf_thick.edges().group_by(Axis.X, tol_digits=1)[-1][0]
front_hack_top_pos = front_hack.vertices().group_by(Axis.Z)[-1][0].center()
front_hack_end = surf_thick.edges().group_by(Axis.X)[-4][0]
front_hack_end_pos = front_hack_end.vertices().group_by(
    Axis.Z)[-1][0].center()
front_hack_proj, _ = Wire.make_wire(
    [front_hack]).close().project_to_viewport((0, 0, 0))
front_hack_proj_wire = Wire.make_wire(front_hack_proj)
front_hack_face = Face.make_from_wires(front_hack_proj_wire)
front_hack_face = front_hack_face.rotate(
    Axis.Y, 90).rotate(Axis.X, 90).rotate(Axis.Z, 180)
front_hack_face_top_pos = front_hack_face.vertices().group_by(
    Axis.Z)[-1][0].center()
front_hack_face = front_hack_face.translate(
    front_hack_top_pos - front_hack_face_top_pos +
    Vector(0, front_hack_face.bounding_box().size.Y, 0))
extrude_by = front_hack_top_pos.X - front_hack_end_pos.X
front_hack_solid = extrude(front_hack_face, amount=extrude_by)
assert front_hack_solid.is_valid(), "Invalid front hack solid"

if "ocp_vscode" in locals():
    ocp_vscode.reset_show()
    ocp_vscode.show_all()

# %%

obj = surf_thick.part + front_hack_solid

if "ocp_vscode" in locals():
    ocp_vscode.reset_show()
    ocp_vscode.show_all()

# %%

obj = surf_thick.part

# # Extras: holes along edges
obj_corner_edges_z = obj.edges().group_by(
    Axis.Z)[0].filter_by(lambda e: e.length > 2*sew_hole_radius).sort_by(SortBy.LENGTH)[-3:-1]
print(obj_corner_edges_z)
obj_corner_edge_x = obj.edges().group_by(
    Axis.X)[0].filter_by(lambda e: e.length > 2*sew_hole_radius).sort_by(SortBy.LENGTH)[-1]
top_face = obj.faces().group_by(Axis.Z)[-2][0]
for edge, inside_dir in [(obj_corner_edges_z[0], Vector(0, 0, 1)), (obj_corner_edges_z[1], Vector(0, 0, 1)), (obj_corner_edge_x, Vector(1, 0, 0))]:
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
        with BuildPart(loc, mode=Mode.ADD) as tetet:
            Cylinder(sew_hole_radius, boot_bb.size.Y)

        # Clean up the hole with a fillet
        # fillet(obj.edges(Select.LAST), prot_thickness/2.1)

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
