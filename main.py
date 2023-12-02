# %%
from contextlib import suppress
from os import getenv
from build123d import *

with suppress(ImportError):
    # Optional, for visualizing the model in VSCode instead of CQ-editor or exporting to STL
    import ocp_vscode

# %%
# ================== PARAMETERS ==================
# 3D printing basics
tol = 0.1 * MM  # Tolerance (tighter than usual)
wall_min = 0.4 * MM  # Minimum wall width
wall = 3 * wall_min  # Recommended width for most walls of this print
eps = 1e-5 * MM  # A small number

# ...
protection_thickness = 2 * wall
samples_pitch = 10  # Number of pitch angles (up-down front) to sample
samples_yaw = 10  # Number of yaw angles (left-right front) to sample


# ================== MODELLING ==================

# Load the boot model, which must be aligned with the base at the origin, centered in X and Y and with forward direction in X+
# Slow import...
boot = Mesher().read("boot.stl" if getenv(
    "final_boot") else "boot-simplified.stl")[0]

boot_half_x = boot.bounding_box().size.X / 2
boot_half_y = boot.bounding_box().size.Y / 2

# %%

all_lines = []
for yaw_sample in map(lambda x: x / (samples_yaw-1), range(samples_yaw)):
    with BuildLine(Plane.XY.rotated((0, -90 * yaw_sample, 0))) as line_tmp:
        CenterArc((0, 0, 0), boot_half_x + 1, -70, 140)
    all_lines.append(line_tmp)

all_collisions = []
for pitch_sample in map(lambda x: x / (samples_pitch-1), range(samples_pitch)):
    all_collisions_line = []
    for line in all_lines:
        # Throw a ray to the center of the boot
        ray_from = line._obj@pitch_sample
        ray_to = Vector(0, 0, 0)
        loc = boot.find_intersection(
            Axis(ray_from, ray_to - ray_from))[0][0]  # Position of first
        # print("Ray from %s to %s: %s" % (ray_from, ray_to, res))
        all_collisions_line.append(loc)
    all_collisions.append(all_collisions_line)

with BuildPart() as obj:
    add(Face.make_surface_from_array_of_points(
        all_collisions).thicken(protection_thickness))

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
