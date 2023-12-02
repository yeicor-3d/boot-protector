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


# ================== MODELLING ==================

# Slow import...
boot = Mesher().read("boot.stl" if getenv(
    "final_boot") else "boot-simplified.stl")[0]

# %%

with BuildPart() as obj:
    Box(20*CM, 20*CM, 20*CM)
    add(boot, mode=Mode.SUBTRACT)

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
