from build123d import *
from contextlib import suppress
with suppress(ImportError): import ocp_vscode # Optional, for visualizing the model in VSCode instead of CQ-editor or exporting to STL


# ================== PARAMETERS ==================
# 3D printing basics
tol = 0.1 * MM  # Tolerance (tighter than usual)
wall_min = 0.4 * MM  # Minimum wall width
wall = 3 * wall_min  # Recommended width for most walls of this print
eps = 1e-5 * MM  # A small number

# ...


# ================== MODELLING ==================

boot = Compound.make_compound([import_stl('boot.stl')])
obj = boot

# ================== SHOWING/EXPORTING ==================

export = True
try:
    if 'ocp_vscode' in locals():
        import socket
        tmp_socket = socket.socket()
        tmp_socket.connect(('localhost', ocp_vscode.get_port()))
        tmp_socket.close()
        ocp_vscode.reset_show()
        ocp_vscode.show_all()
        export = False  # If the connection fails, export to STL instead
    elif 'show_object' in locals():
        show_object(obj, 'boot-protector')  # type: ignore
except Exception as ex:
    print("Cannot show model, exporting to STL instead (%s)" % ex)
    
if export:
    if hasattr(obj, 'part'):
        obj.part.export_stl('boot-protector.stl')
    else:
        export_stl(obj, 'boot-protector.stl')
