# %%
from contextlib import suppress
from math import *
from os import getenv
from build123d import *
from tqdm import tqdm
with suppress(ImportError):  # Optional
    import ocp_vscode
from OCP.gp import (gp_Quaternion, gp_Extrinsic_XYZ, gp_Vec)
import numpy as np

# %%
# ================== PARAMETERS ==================
# 3D printing basics
tol = 0.1 * MM  # Tolerance (tighter than usual)
wall_min = 0.4 * MM  # Minimum wall width
wall = 3 * wall_min  # Recommended width for most walls of this print
eps = 1e-5 * MM  # A small number
# Whether to generate the final model or a simplified one
final_model = getenv("final") is not None

# Protection parameters
prot_clip_x = -6 * MM
prot_clip_x_radius = 6 * CM
prot_z_extra = 5 * MM  # Distance from floor (which is at origin)
prot_thickness = 2 * wall  # Thickness of the protection (external)

# Sewing parameters
sew_hole_radius = (1.5 * MM) if final_model else (5 * MM)
sew_hole_sep = 2 * 2 * sew_hole_radius

# ================== MODELLING ==================

# Load the boot model, which must be aligned with the base at the origin, centered in X and Y and with forward direction in X+
# Slow import...
boot = Mesher().read(
    "boot-protector-1-core.stl" if final_model else "boot-protector-1-core-simpl.stl")[0]
boot_bb = boot.bounding_box()

# %%

# Make proper clean edge cuts for further modification
# X cut (manual cylinder)

boot_rel = boot
boot_rel -= Cylinder(prot_clip_x_radius, boot_bb.size.Z).translate(
    (-prot_clip_x_radius + prot_clip_x, 0, boot_bb.center().Z))
print(boot_rel)
boot_rel = boot_rel.solids()[0]
print(boot_rel)

# %%

# Z cut (automatic based on normals)

up_faces = boot_rel.faces().filter_by(lambda f: f.normal_at(None).Z > eps)
largest_component = sorted(Face.sew_faces(up_faces), key=lambda f: -len(f))[0]
shell = Shell.make_shell(largest_component)

faces = shell.faces()
edge_to_faces = {}
for face in faces:
    for edge in face.edges():
        edge_to_faces.setdefault(edge, []).append(face)
corner_edges = [edge for edge, faces in edge_to_faces.items()
                if len(faces) < 2]
rel_corner_edges = [edge for edge in corner_edges if abs(
    (edge % 0.5).Z) < 0.5 and (edge @ 0.5).Z < shell.center().Z]

# - Remove fully disconnected edges
vert_to_edges = {}
for edge in rel_corner_edges:
    for vert in edge.vertices():
        vert_to_edges.setdefault(vert, []).append(edge)
rel_corner_edges = list(
    edges for _, edges in vert_to_edges.items() if len(edges) > 1)
rel_corner_edges = list(
    set(edge for edges in rel_corner_edges for edge in edges))

# - Given all relevant corner edges, create a single plane for a clean Z cut
rel_positions = [edge @ 0.5 for edge in rel_corner_edges]
coords = np.array([pos.to_tuple() for pos in rel_positions])
G = coords.sum(axis=0) / len(coords)
u, s, vh = np.linalg.svd(coords - G)
u_norm = vh[2, :]
u_forw = vh[0, :]
print("Plane normal: %s" % u_norm)
zcut_face = Face.make_rect(boot_bb.size.Y * 3, boot_bb.size.Y * 3,
                           Plane(G, Vector(*u_forw), Vector(*u_norm)))
zcut_solid = extrude(zcut_face, amount=-boot_bb.size.Z * 2, dir=(0, 0, 1))
# rel_plane = Plane(sum(rel_positions) / len(rel_positions), rel_normal)

# - Actual cut
boot_rel -= zcut_solid

# TODO: Make hull of all the faces to intersect with the uneven cut

# %%

# # Extras: holes along edges
with BuildPart() as obj:
    add(boot_rel.solid())

    # .filter_by(lambda e: e.length > 2*sew_hole_radius).sort_by(SortBy.LENGTH)[-3:-1]
    obj_bottom_face = obj.faces().sort_by(SortBy.AREA)[-1]
    obj_bottom_wire = obj_bottom_face.wires().sort_by(SortBy.LENGTH)[-1]
    # The wire follows the exterior and interior edges of the bottom face, but we only want the exterior ones
    # - We extract it by querying points of the wire to find the most X- of both side of Y
    # This is very hacky and weak, but it works for now
    minxSample1 = min([sample_at/1000 * 0.5 for sample_at in range(1000)],
                      key=lambda k: (obj_bottom_wire@k).X)
    minx1 = obj_bottom_wire@minxSample1
    sample2From = minxSample1 + 0.1
    minXSample2 = min([sample2From + (sample_at/1000 * (1.0 - sample2From))
                       for sample_at in range(1000)], key=lambda k: (obj_bottom_wire@k).X)
    minx2 = obj_bottom_wire@minXSample2
    print("Min X1: %f -> %s, Min X2: %f -> %s" %
          (minxSample1, minx1, minXSample2, minx2))
    # print("Done trimming")

    # HACK: The following timeouts, so we modify samples manually resulting in ugly code
    # obj_bottom_wire_outer = obj_bottom_wire.trim(minxSample1, minXSample2)

    def wire_sample_interp(k): return (
        k * (minXSample2 - minxSample1) + minxSample1)

    # For each wire, compute and add holes
    wire = obj_bottom_wire
    steps = wire.length * (minXSample2 - minxSample1) / sew_hole_sep
    step_relative_size = steps / int(steps)
    inside_dir = Vector(0, 0, 1)
    for step in tqdm(range(int(steps))):
        progress = wire_sample_interp(
            (step + 0.5) / steps * step_relative_size)

        # Extract tangent from wire
        tangent = wire % progress

        # Extract basic data from wire
        pos = wire@progress + inside_dir * (sew_hole_radius + prot_thickness)

        # Maybe: project the position on the surface

        # Compute the orientation of the hole
        wanted_up = tangent.cross(inside_dir)
        quat = gp_Quaternion(gp_Vec(0, 0, 1), gp_Vec(
            wanted_up.X, wanted_up.Y, wanted_up.Z))
        angles = quat.GetEulerAngles(gp_Extrinsic_XYZ)
        loc = Location(pos, (degrees(angles[0]), degrees(
            angles[1]), degrees(angles[2])))

        # Build the hole
        with Locations(loc):
            Cylinder(sew_hole_radius, boot_bb.size.Y, mode=Mode.SUBTRACT)

        # Clean up the hole with a fillet
        try:
            to_fillet = obj.edges(Select.NEW).filter_by(
                lambda e: e % 0.4 != e % 0.6)
            # FIXME: Not filleting due to lots of sub-ellipses...
            # print("Fillet %s" % [(e.geom_type(), e.length) for e in to_fillet])
            # fillet(to_fillet, prot_thickness/2.1)
        except Exception as ex:
            print("Cannot fillet: %s" % ex)

# %%
# ================== SHOWING/EXPORTING ==================

export = True
try:
    if "show_object" in locals():
        show_object(obj, "boot-protector-2-with-holes")  # type: ignore
        export = False
    elif "ocp_vscode" in locals():
        ocp_vscode.reset_show()
        ocp_vscode.show_all()
        export = False
except Exception as ex:
    print("Cannot show model, exporting to STL instead (%s)" % ex)

if export:
    obj.part.export_stl("boot-protector-2-with-holes.stl")
