# %%
from contextlib import suppress
from os import getenv
from build123d import *
with suppress(ImportError):  # Optional
    import ocp_vscode

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
prot_clip_z_offset = 25 * MM
prot_clip_z_rotation = (0, 0, 0)
prot_z_extra = 8 * MM  # Vertical wall to add below cut
prot_offset = 1 * MM  # Offset of the protection from the boot
prot_thickness = 2 * wall  # Thickness of the protection (external)

# Sewing parameters
sew_hole_radius = (1.5 * MM) if final_model else (5 * MM)
sew_hole_sep = 2 * 2 * sew_hole_radius

# ================== MODELLING ==================

# Load the boot model, which must be aligned with the base at the origin, centered in X and Y and with forward direction in X+
# Slow import...
# boot = Mesher().read("boot-0.stl" if final_model else "boot-0-simpl.stl")
boot = import_step("boot-0-simpl.step")
# assert len(boot) == 1
# boot = boot[0]
boot.fix()

if 'ocp_vscode' in locals():
    ocp_vscode.show_all()
assert boot.is_valid()
print(boot)
boot = boot.solid()
boot_bb = boot.bounding_box()

# %%

with BuildPart() as obj:
    add(boot)

    with Locations(Location((prot_clip_x-prot_clip_x_radius, boot_bb.center().Y, boot_bb.min.Z))):
        Box(boot_bb.size.X/2, boot_bb.size.Y, boot_bb.size.Z, align=(
            Align.MAX, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
        Cylinder(prot_clip_x_radius, boot_bb.size.Z, align=(
            Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)

    with Locations(Location((boot_bb.min.X, boot_bb.min.Y, 0))):
        Box(boot_bb.size.X, boot_bb.size.Y, prot_clip_z_offset,
            align=Align.MIN, mode=Mode.SUBTRACT)

    assert obj.part.is_valid()
    back_faces = faces().filter_by(lambda f: f.normal_at(
        None).Z < eps and f.normal_at(None).X < eps and abs(f.normal_at(None).Y) < 0.4)
    op = back_faces + [faces().sort_by(Axis.Z)[-1]]
    # BROKEN!!! offset(openings=op, kind=Kind.INTERSECTION)

if 'ocp_vscode' in locals():
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
        # export = False
except Exception as ex:
    print("Cannot show model, exporting to STL instead (%s)" % ex)

if export:
    print("Exporting to STL")
    obj.part.export_stl("boot-protector.stl")

# %%
