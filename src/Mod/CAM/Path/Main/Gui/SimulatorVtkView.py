# SPDX-License-Identifier: LGPL-2.1-or-later
"""VTK viewport for the CAM legacy simulator, driven directly from
cStock's height grid.

Replaces the per-tick path

    (self.cutMaterial.Mesh, self.cutMaterialIn.Mesh) = self.voxSim.GetResultMesh()

which rebuilds two Mesh::MeshObjects and makes FreeCAD reconstruct the
whole Coin3D scene graph every tick. Measured on a 2.53M-point stock that
cost ~96 ms per update. This path costs ~0.6 ms, because the geometry is
built once and only a rectangle of texture is written per tick.

Requires the VolSim/PathSim additions in volsim_viewport.diff:
GetHeightBuffer() and ConsumeViewDirty().

SIX CONSTRAINTS fail silently if broken -- no exception, no GLSL error,
just a wrong picture:

  1. VBO shift-scale left on, so vertexMC arrives normalised while the
     height is in world units, inflating Z by a large factor
  2. UV taken from vertexMC.xy instead of the tcoord attribute, sampling
     zero everywhere
  3. a float32 texture with linear filtering returning black
  4. a supplied normals array, blackening the walls
  5. a single headlight, leaving vertical walls unlit
  6. a collapse test that is not quad-invariant, so half a quad is pushed
     outside the clip volume and the rest stretches to meet it

"Constraint N" below refers to this list.
"""

import ctypes
import os
import sys
import time
import types

import numpy as np

import vtkmodules.vtkInteractionStyle  # noqa: F401  (registers the styles)
import vtkmodules.vtkRenderingOpenGL2  # noqa: F401  (registers the backend)

# Base the interactor widget on QOpenGLWidget rather than QVTKRenderWindow-
# Interactor's default plain QWidget.
#
# The default sets WA_PaintOnScreen, making the widget a NATIVE window.
# Native windows bypass Qt's compositing: they do not clip against
# translucent siblings and they ignore normal stacking, so under a theme
# that makes the tree dock translucent the GL surface escapes its parent
# and paints over unrelated widgets -- under themed styles only, which is
# what makes it easy to miss.
#
# QOpenGLWidget renders into an FBO that Qt composites into the backing
# store, so stylesheets and translucency cannot displace it. FreeCAD's own
# OpenGL simulator (DlgCAMSimulator) is a QOpenGLWidget for the same
# reason.
QVTK_BASE_ERRORS = []


def _ensure_qopenglwidget():
    """Make PySide6.QtOpenGLWidgets importable if the class exists anywhere.

    VTK's wrapper imports QOpenGLWidget from one specific module per Qt
    binding. Qt6 moved the class into QtOpenGLWidgets, and some PySide6
    packagings omit that module while still shipping the class elsewhere.
    If it can be found, a synthetic module is registered under the name
    VTK expects, so the QOpenGLWidget base becomes available and the
    widget is composited instead of native.

    Returns a note describing what was found, for the report.
    """
    import importlib

    try:
        importlib.import_module("PySide6.QtOpenGLWidgets")
        return "PySide6.QtOpenGLWidgets already present"
    except Exception:
        pass

    candidates = (
        "PySide6.QtWidgets",
        "PySide6.QtOpenGL",
        "PySide6.QtGui",
        "PySide.QtWidgets",
        "PySide.QtGui",
    )
    for name in candidates:
        try:
            mod = importlib.import_module(name)
        except Exception:
            continue
        cls = getattr(mod, "QOpenGLWidget", None)
        if cls is None:
            continue
        shim = types.ModuleType("PySide6.QtOpenGLWidgets")
        shim.QOpenGLWidget = cls
        sys.modules["PySide6.QtOpenGLWidgets"] = shim
        try:
            # Attach to the package too, or `from PySide6 import
            # QtOpenGLWidgets` still fails.
            pyside6 = importlib.import_module("PySide6")
            setattr(pyside6, "QtOpenGLWidgets", shim)
        except Exception:
            pass
        return "shimmed PySide6.QtOpenGLWidgets from {}".format(name)

    return "QOpenGLWidget not found in " + ", ".join(candidates)


def plan_display_grid(nx, ny, max_cells=None, viewport=None, bytes_per_cell=None, max_bytes=None):
    """Work out the display grid for a simulation grid.

    Two different resolutions live here and confusing them is easy: the
    SIMULATION grid is cut fidelity, set by the Accuracy slider, and the
    DISPLAY grid is the staircase geometry drawn for it, at about 816
    bytes and 6 triangles per cell. The simulation can run far finer than
    the viewport draws, since both sample the same height texture.

    The display grid is capped by the VIEWPORT PIXELS. More display cells
    than pixels cannot be seen -- they are subpixel geometry, paid for in
    memory and triangles and thrown away by the rasteriser. The cap is
    per-axis, because with a single uniform step a wide stock would
    otherwise still oversample across its long side.

    `max_cells` is an optional extra ceiling, applied on top.
    `bytes_per_cell` is the geometry cost of one display cell, which
    doubles when the underside is drawn, and `max_bytes` an optional
    ceiling on the total. The byte ceiling is the one to pass when the
    cost per cell varies: scaling `max_cells` instead cannot express it,
    because `step` is a whole number and the grid lands wherever the
    rounding puts it -- 12% over the budget, in the case that prompted
    this.

    Returns (dx, dy, step, bytes).
    """
    import math as _math

    step = 1
    if viewport:
        vw, vh = (max(1, int(v)) for v in viewport)
        step = max(
            1,
            int(_math.ceil(nx / float(vw))),
            int(_math.ceil(ny / float(vh))),
        )
    if max_cells:
        step = max(step, int(_math.ceil(max(nx, ny) / float(max(1, max_cells)))))
    per_cell = bytes_per_cell or BYTES_PER_DISPLAY_CELL
    dx = int(_math.ceil(nx / step))
    dy = int(_math.ceil(ny / step))
    # One display cell per simulation cell is the finest there is, so a
    # budget too small for even that cannot be met: stop rather than loop.
    while max_bytes and dx * dy * per_cell > max_bytes and step < max(nx, ny):
        step += 1
        dx = int(_math.ceil(nx / step))
        dy = int(_math.ceil(ny / step))
    return dx, dy, step, dx * dy * per_cell


def _import_interactor():
    """Import QVTKRenderWindowInteractor on the least problematic base.

    The default plain QWidget sets WA_PaintOnScreen, making a NATIVE
    window: not clipped to its parent and outside Qt's compositing, so
    under some themes the GL surface spills over neighbouring docks.
    QOpenGLWidget renders into an FBO that Qt composites normally, which
    is what FreeCAD's own DlgCAMSimulator uses.

    NEVER RAISES. Returning (None, note) instead means an unusable VTK Qt
    layer leaves the simulator command registered and the rest of CAM
    working; the failure surfaces when the viewport is actually created.
    Raising here removed CAM_Simulator from the menu entirely.
    """
    import importlib

    name = "vtkmodules.qt.QVTKRenderWindowInteractor"

    def plain_import():
        try:
            module = importlib.import_module(name)
            return getattr(module, "QVTKRenderWindowInteractor", None)
        except Exception as exc:
            QVTK_BASE_ERRORS.append("plain import: {}".format(exc))
            return None

    try:
        import vtkmodules.qt
    except Exception as exc:
        QVTK_BASE_ERRORS.append("vtkmodules.qt: {}".format(exc))
        return None, "unavailable"

    QVTK_BASE_ERRORS.append(_ensure_qopenglwidget())

    # Already imported elsewhere -- FEM also uses VTK. Reloading to force a
    # different base is not worth it: a failed reload leaves a broken
    # module in sys.modules and every later attempt fails against the
    # wreckage, which is exactly how this module stopped importing.
    if name in sys.modules:
        cls = plain_import()
        if cls is not None:
            return cls, "{} (already imported elsewhere)".format(
                getattr(vtkmodules.qt, "QVTKRWIBase", "QWidget")
            )

    for base in ("QOpenGLWidget", "QGLWidget", "QWidget"):
        try:
            vtkmodules.qt.QVTKRWIBase = base
            module = importlib.import_module(name)
            return module.QVTKRenderWindowInteractor, base
        except Exception as exc:
            QVTK_BASE_ERRORS.append("{}: {}".format(base, exc))
            # A failed import can leave a partial module behind; drop it so
            # the next base starts clean.
            sys.modules.pop(name, None)

    cls = plain_import()
    if cls is not None:
        return cls, "QWidget (last resort)"
    return None, "unavailable"


QVTKRenderWindowInteractor, _ = _import_interactor()
from vtkmodules.util.numpy_support import numpy_to_vtk, vtk_to_numpy
from vtkmodules.vtkCommonCore import (
    VTK_FLOAT,
    VTK_UNSIGNED_CHAR,
    vtkIdTypeArray,
    vtkPoints,
)
from vtkmodules.vtkCommonDataModel import (
    vtkCellArray,
    vtkDataObject,
    vtkImageData,
    vtkPolyData,
)
from vtkmodules.vtkCommonMath import vtkMatrix4x4
from vtkmodules.vtkInteractionStyle import vtkInteractorStyleTrackballCamera
from vtkmodules.vtkRenderingCore import (
    vtkTextActor,
    vtkActor,
    vtkLightKit,
    vtkPolyDataMapper,
    vtkRenderer,
    vtkTexture,
)

try:
    from vtkmodules.vtkRenderingOpenGL2 import vtkOpenGLVertexBufferObject

    _SHIFT_SCALE_OFF = vtkOpenGLVertexBufferObject.DISABLE_SHIFT_SCALE
except Exception:  # pragma: no cover - older VTK
    _SHIFT_SCALE_OFF = 0

# Measured footprint of the static geometry, CPU arrays plus VTK's GPU
# copies: 4 top verts + ~8 wall verts per cell, each carrying position,
# tcoord and colourUV, plus a 32-bit index buffer.
BYTES_PER_DISPLAY_CELL = 816

# How many spans below each column's primary the viewport draws. One
# covers a single cavity -- a pocket cut in one orientation and seen from
# another. Two covers a cavity behind a cavity, which is what a SECOND
# workplane change produces: a column can then run through a pocket made
# in the first orientation and another made in the second, and at one
# layer the deeper one is capped from above and left open below.
#
# Layers cost geometry only where a column holds the span, so a part
# with one cavity pays for one whatever this says.
#
# Only the columns that actually hold such a span get geometry, so a
# stock with none costs nothing, and a part with a pocket pays for the
# pocket rather than for the grid.
#
# It was drawn and turned off once: the seams where the layer's surfaces
# met the primary's along the cavity's edge looked worse than the arch
# left without them. That verdict was reached on a build with two faults
# since fixed, and both of them put stray surfaces exactly there --
# downward faces taking the column's cut flag, so every layer's
# underside was orange and survived Uncut being turned off, and empty
# cells at a silhouette reading as machined rather than as outside. So
# it is on again, to be judged on a build where the edge of a cavity is
# drawn with the rules the rest of the part is.
#
# What neither fix touches is the geometry: one height per cell is still
# one height per cell, and the primary's own bottom wall still descends
# to the neighbour's underside through material the layer now fills. If
# a seam remains, that double face is the first place to look.
SPAN_LAYERS = 2

# The same with the underside drawn: a second cap and a second wall per
# cell, so twice the vertices. Paid only where the stock has an underside
# above the floor -- a box sitting in the document frame has none, and
# costs exactly what it always did.
UNDERSIDE_BYTES_PER_DISPLAY_CELL = 2 * BYTES_PER_DISPLAY_CELL

# Geometry is duplicated on the GPU, and there is no portable way to ask
# how much VRAM exists -- the extensions are vendor specific
# (GL_NVX_gpu_memory_info, GL_ATI_meminfo) and VTK does not surface them.
# So size from system RAM and stay well clear of it.
MEMORY_BUDGET_FRACTION = 0.15


def available_memory_bytes():
    """Usable system RAM in bytes, or None if it cannot be determined."""
    try:
        # Linux: what the kernel thinks is usable without swapping, which
        # is a better basis than MemTotal.
        with open("/proc/meminfo") as fh:
            for line in fh:
                if line.startswith("MemAvailable:"):
                    return int(line.split()[1]) * 1024
    except OSError:
        pass
    if sys.platform.startswith("win"):
        try:

            class _MemStatus(ctypes.Structure):
                _fields_ = [
                    ("dwLength", ctypes.c_ulong),
                    ("dwMemoryLoad", ctypes.c_ulong),
                    ("ullTotalPhys", ctypes.c_ulonglong),
                    ("ullAvailPhys", ctypes.c_ulonglong),
                    ("ullTotalPageFile", ctypes.c_ulonglong),
                    ("ullAvailPageFile", ctypes.c_ulonglong),
                    ("ullTotalVirtual", ctypes.c_ulonglong),
                    ("ullAvailVirtual", ctypes.c_ulonglong),
                    ("ullAvailExtendedVirtual", ctypes.c_ulonglong),
                ]

            st = _MemStatus()
            st.dwLength = ctypes.sizeof(_MemStatus)
            if ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(st)):
                return int(st.ullAvailPhys)
        except Exception:
            pass
    try:
        # POSIX fallback: total rather than available.
        return os.sysconf("SC_PAGE_SIZE") * os.sysconf("SC_PHYS_PAGES")
    except (ValueError, OSError, AttributeError):
        pass
    return None


_ID_DTYPE = np.int64 if vtkIdTypeArray().GetDataTypeSize() == 8 else np.int32
_VTK_ID_TYPE = vtkIdTypeArray().GetDataType()

UNCUT_COLOUR = np.array([176, 179, 189], dtype=np.uint8)
CUT_COLOUR = np.array([214, 118, 46], dtype=np.uint8)

_VS_DEC = (
    "//VTK::PositionVC::Dec\n"
    "uniform sampler2D heightTex;\n"
    "uniform sampler2D bottomTex;\n"
    "uniform sampler2D cutTex;\n"
    "in vec2 colourUV;\n"
)

# Height from `tcoord`; colour from whichever of the two neighbours is
# lower, written into tcoordVCVSOutput so VTK's own fragment shader picks
# it up with no fragment replacement. Both corner pairs of a wall reach
# the same answer, so a wall is coloured uniformly instead of splitting
# half cut, half uncut -- which is the CutMaterial / CutMaterialIn
# distinction, carried by one texture instead of two meshes.
# Cells with no material left are collapsed rather than drawn.
#
# The grid covers the stock's BOUNDING BOX, so a round or irregular
# stock leaves corner cells at the floor; drawn, they form a flat plate
# the shape of the bounding box. The geometry is static and cannot drop
# them, so the shader pushes them outside the clip volume per frame --
# which also stops drawing a pocket cut clean through.
#
# All four corners of a top quad sample the same texel, and both corner
# pairs of a wall sample the same two, so a collapsed quad collapses
# whole. A wall between an empty cell and a full one keeps its material
# side and is still drawn: that is the outer wall of a round stock.
_EMPTY_EPSILON = "0.0005"

# Opacity of the original-stock shell. Low enough to read the cut surface
# through it, high enough to show where the stock used to be. Two layers
# are crossed when looking through the solid, so it reads darker than the
# figure suggests.
SHELL_OPACITY = 0.10

# quadKind says which of the four quad sets a vertex belongs to, and so
# which surface it follows: 0 cell top, 1 wall between two tops, 2 cell
# underside, 3 wall between two undersides. One shader, four uniforms,
# rather than four shaders.
KIND_TOP = 0
KIND_WALL = 1
KIND_BOTTOM = 2
KIND_BOTTOM_WALL = 3

# The cut mask carries THREE states, because the shader asks it two
# different questions: has this cell been machined, and was it ever stock
# at all.
#
# "Never cut" used to stand in for "never was stock". The proxy fails at
# the silhouette of a tilted stock, where a column legitimately holds
# less than one cell of material: nick it and the cell is both empty and
# cut, which is what a pocket taken clean through looks like. The skirt
# test then stopped recognising the outer wall as outward, so Uncut no
# longer hid it and it was coloured as machined -- over the full
# thickness of the part, since a wall beside an empty cell drops to its
# neighbour's underside. At step > 1 the logical_or over a block that
# straddles the edge spread it a whole display cell wide.
#
# Presence is now asked of the SEEDED geometry instead, which is what the
# question was always about: original - bottom >= res, the same one-cell
# rule as cStock::ColumnSpan. It travels in the mask rather than being
# inferred from the cut flag.
MASK_OUTSIDE = 0.0  # never held material: outside the stock's solid
MASK_STOCK = 0.5  # stock, untouched
MASK_CUT = 1.0  # stock, machined

# The thresholds, as GLSL literals. InterpolateOff means the shader reads
# the three values exactly, so any cut between them would do; the
# midpoints are the ones that stay right if the values are ever nudged.
# The mask is written in refresh() in this same file, so the encoding
# cannot fall out of step with the shader that reads it.
_MASK_STOCK_MIN = "0.25"
_MASK_CUT_MIN = "0.75"

_VS_IMPL = (
    "float hSelf  = texture(heightTex, tcoord).r;\n"
    "float hOther = texture(heightTex, colourUV).r;\n"
    "float bSelf  = texture(bottomTex, tcoord).r;\n"
    "float bOther = texture(bottomTex, colourUV).r;\n"
    # The cut flag is per COLUMN, and it says the column's TOP has been
    # machined. The underside quads read the same texel, so every cell
    # the tool touched drew the pocket's own outline on the bottom of the
    # part -- orange, and left standing with Uncut off, while the
    # untouched undersides around it were hidden.
    #
    # A downward face is stock whatever happened above it. Its colour is
    # taken from the texture's border ring instead, which refresh never
    # writes and which therefore holds UNCUT_COLOUR for the life of the
    # scene: with nearest sampling and EdgeClampOn, (0,0) lands on that
    # first texel exactly, whatever the grid size.
    #
    # quadKind is a uniform, so all four vertices of a quad reach the
    # same answer here and constraint 6 is untouched.
    #
    # isLayer covers a span BELOW the primary for the same reason one
    # step further out: the mask is the primary column's, so a layer's
    # faces would take the state of material two spans above them, and a
    # cavity floor would turn orange because something cut the top of the
    # same column. Nothing can say whether a carried-over cavity was
    # machined -- the transfer re-seeds and the engine keeps no machined
    # flag -- so it is drawn as stock, which is what the export does too.
    "bool stockFace = quadKind > 1.5 || isLayer > 0.5;\n"
    "tcoordVCVSOutput = stockFace ? vec2(0.0) : ((hSelf <= hOther) ? tcoord : colourUV);\n"
    # A column holds material between its underside and its top. Below a
    # tilted or round stock the underside is well above the floor, and
    # before it was read here every wall at the silhouette ran to the
    # floor: the skirt the Uncut button exists to hide.
    "bool matSelf  = (hSelf  - bSelf ) > " + _EMPTY_EPSILON + ";\n"
    "bool matOther = (hOther - bOther) > " + _EMPTY_EPSILON + ";\n"
    # Where this vertex sits, in normalised height.
    #
    # The walls are the interesting pair. Between two columns that both
    # hold material, a top wall is the step from one top to the other and
    # a bottom wall the step between the two undersides -- each collapses
    # to nothing where the two agree. Where the NEIGHBOUR holds nothing,
    # the top wall takes that corner down to this column's own underside
    # instead, so it spans the full thickness of the material and closes
    # the stock along its silhouette; the bottom wall then collapses,
    # having nothing left to cover.
    #
    # Every vertex of a quad reads both texels and applies the same rule,
    # so all four reach the same answer -- the symmetry the skirt test
    # below also depends on.
    #
    # Two spans that do not overlap at all -- this cell's span and the
    # neighbour's at the same rank lie wholly one above the other -- are
    # different pieces of material, and the rules above would join them
    # across the air between. That happens wherever a turn or a cut shifts
    # which span is which rank: a thin roof left over a cavity is the
    # primary in one column while the bulk below is the primary in the
    # next. The top wall ran from the lower span's top to the upper span's
    # top and the bottom wall from the lower span's underside to the upper
    # span's; both crossed the gap, drawing fins and railings along the rims
    # of carried-over cavities.
    #
    # Where the spans are apart, the top wall becomes the UPPER span's own
    # face (the lower cell's corners drop to the upper span's underside,
    # as the silhouette branch already does) and the bottom wall the LOWER
    # span's face (the upper cell's corners rise to the lower span's top).
    # Nothing is drawn across the gap. Collapsing the walls instead removes
    # the same artefact but deletes the real faces with it.
    #
    # The test must switch off only the lower cell for the top wall. Turned
    # off for both, both corners fall through to hSelf and the quad is
    # drawn exactly as before -- the first wording of this fix did that.
    #
    # `apart` is symmetric in the two texels, so every vertex of a quad
    # reaches the same verdict. It changes nothing where either cell is
    # empty or the spans overlap, so every other case is untouched.
    "bool apart = matSelf && matOther && (bSelf >= hOther || bOther >= hSelf);\n"
    "bool wallSelf  = matSelf  && !(apart && hSelf  < hOther);\n"
    "bool wallOther = matOther && !(apart && hOther < hSelf);\n"
    # A column the tool cut clean through, beside one that holds material.
    # refresh() leaves such a texel empty -- both surfaces equal -- but at
    # its ORIGINAL underside rather than the floor. Below that height the
    # column never held material, so the neighbour's face there is the
    # original stock face; above it the tool opened the face. The two
    # walls split it there: the top wall keeps the part above, a machined
    # face, and the bottom wall takes the part below, a stock face, which
    # Uncut hides and which is coloured as stock. Before, the top wall took
    # all of it, so a thin skin surfaced off beside a tall face turned the
    # whole face orange and left it standing with Uncut off -- in stripes,
    # wherever a sliver of skin survived between passes and the face there
    # became a bottom wall instead.
    #
    # Only the carved cell's corners move, and each reads both texels, so a
    # quad stays whole. A column cut through its full depth has its
    # original underside at the floor; the split then sits at the
    # neighbour's own underside and both walls are drawn exactly as before.
    "bool carvedSelf = !matSelf && matOther\n"
    "                  && texture(cutTex, tcoord).r >= " + _MASK_CUT_MIN + ";\n"
    "float zSplit = min(max(bOther, hSelf), hOther);\n"
    "float zUnit;\n"
    "if (quadKind < 0.5) { zUnit = hSelf; }\n"
    "else if (carvedSelf && quadKind > 0.5 && quadKind < 1.5) { zUnit = zSplit; }\n"
    "else if (quadKind < 1.5) { zUnit = wallSelf ? hSelf : (wallOther ? bOther : hSelf); }\n"
    "else if (quadKind < 2.5) { zUnit = bSelf; }\n"
    "else if (carvedSelf) { zUnit = zSplit; }\n"
    "else if (apart && hSelf > hOther) { zUnit = hOther; }\n"
    "else { zUnit = matSelf ? bSelf : (matOther ? bOther : bSelf); }\n"
    "float zW = heightOffset + zUnit * heightScale;\n"
    "vec4 dispMC = vec4(vertexMC.x, vertexMC.y, zW, 1.0);\n"
    # showUncut is a uniform rather than a rebuild: hiding untouched
    # material is a view option, and regenerating the geometry to change
    # it would cost a full rebuild per click.
    #
    # A wall is shown when its LOWER cell is cut -- the wall between cut
    # and uncut material is the side of the cut and stays; see `untouched`.
    "float cutSelf  = texture(cutTex, tcoord).r;\n"
    "float cutOther = texture(cutTex, colourUV).r;\n"
    "bool empty = !matSelf && !matOther;\n"
    # Untouched is still "neither cell has been machined", so the test is
    # against the cut state alone: an outside cell reads below this too,
    # exactly as it did when the mask was a 0/1 cut flag.
    # A downward face has not been machined, so Uncut governs it like any
    # other untouched material and hides it. What that leaves is a part
    # open from below -- which is what a 3-axis job has always looked
    # like with Uncut off, having no underside geometry at all.
    #
    # A wall is judged by its LOWER cell, not by either. The face between
    # two columns was made by the tool only if material was taken from
    # the side that is now lower: that is where the air is. Where a cut
    # cell stands ABOVE an untouched one, the wall between them is the
    # original surface's own riser, and with the old any-cell test it
    # stayed drawn -- a grey strip along the edge of every cut that runs
    # out onto a sloping original surface, floating once Uncut hid the
    # tops beside it. Flat 3-axis stock never shows it, since there a cut
    # cell is always below its untouched neighbours; every 3+2 re-seed
    # does, because m_original is then the transferred staircase.
    #
    # Symmetric, like `empty` and `outward`: both vertices of a wall read
    # the same two texels and pick the same lower one, and a tie falls
    # back to max() of both, which is also what a top quad reaches --
    # its colourUV is its own texel -- so tops are unchanged.
    "float cutLower = hSelf < hOther ? cutSelf"
    " : (hOther < hSelf ? cutOther : max(cutSelf, cutOther));\n"
    "bool untouched = stockFace || cutLower < " + _MASK_CUT_MIN + ";\n"
    # The stock's outer wall at the silhouette. One of its cells is cut,
    # so `untouched` lets it through, and with Uncut off it would be left
    # standing around an otherwise hidden solid.
    #
    # It used to be a false extrusion to the floor and is now the true
    # thickness of the material, the underside being read above -- but it
    # is still untouched stock, so Uncut still governs it.
    #
    # "Outside" is a cell holding no material that never held any -- the
    # mask is part of the test because a pocket taken clean through holds
    # no material either, and must keep its walls.
    #
    # It asks the mask for that directly. Asking whether the cell had
    # been cut was the same question only while every empty cell was
    # either outside the stock or cut clean through; a sub-cell column at
    # a tilted silhouette is neither, and a nick in one used to hand its
    # wall to the cut side (see MASK_OUTSIDE).
    #
    # MUST stay symmetric, like `empty` and `untouched`, so every vertex
    # of a quad reaches the same answer. Test one texel instead of both
    # and a skirt wall collapses two corners and keeps two; the survivors
    # stretch to a fixed point in clip space and draw as a striped sheet
    # anchored to the screen, sweeping as the camera moves.
    "bool selfOutside  = !matSelf  && cutSelf  < " + _MASK_STOCK_MIN + ";\n"
    "bool otherOutside = !matOther && cutOther < " + _MASK_STOCK_MIN + ";\n"
    "bool outward = selfOutside || otherOutside;\n"
    "bool hidden = showUncut < 0.5 && (untouched || outward);\n"
    "if (empty || hidden) {\n"
    "  gl_Position = vec4(2.0, 2.0, 2.0, 1.0);\n"
    "} else {\n"
    "  gl_Position = MCDCMatrix * dispMC;\n"
    "}\n"
    "vertexVCVSOutput = MCVCMatrix * dispMC;\n"
)


def reduce_layer_blocks(top, bottom, present, step, floor_z):
    """One display texel per step x step block of a span layer, as ONE REAL
    span: the span of the cell with the block's highest top at this rank,
    and that cell's own underside.

    It used to be the HULL of the rank's spans across the block, lowest
    underside to highest top. Wherever a rank shift falls inside a block --
    rank k is a roof remnant in one cell and the bulk in the next, the rims
    of every carried-over cavity -- the hull runs from the bulk's underside
    to the roof's top and claims the air between them as solid. No wall
    rule can take that back, since the shader is told there is material
    there; it is why the fix in _VS_IMPL cleared the railings at step 1 and
    left them at step 2. Measured on the harness grid at step 2: hull
    texels claimed 52.7 mm of air and their walls stood 90.0 mm in it; one
    real span claims none. The cost is extent: 97.9% of the layer's
    material is shown at its own rank against 100% for the hull, the
    difference being material the hull showed only by also painting the
    air around it.

    Cells that do not hold the span (`present` false) take no part, as
    before; a block with none reads as the floor.
    """
    s0 = np.arange(0, top.shape[0], step)
    s1 = np.arange(0, top.shape[1], step)
    hi = np.where(present, top, -np.inf)
    block_t = np.maximum.reduceat(np.maximum.reduceat(hi, s0, axis=0), s1, axis=1)
    # Spread each block's top back over its cells to find the one that has
    # it. reduceat starts a block at every multiple of `step`, so repeating
    # each value `step` times and trimming lines up, ragged edge included.
    grown = np.repeat(np.repeat(block_t, step, axis=0), step, axis=1)[: top.shape[0], : top.shape[1]]
    lo = np.where(present & (hi == grown), bottom, np.inf)
    block_b = np.minimum.reduceat(np.minimum.reduceat(lo, s0, axis=0), s1, axis=1)
    gone = ~np.isfinite(block_t)
    return np.where(gone, floor_z, block_t), np.where(gone, floor_z, block_b)


# FreeCAD's navigation styles differ mainly in which button rotates and
# which pans. VTK's trackball rotates on the LEFT button, which is the one
# FreeCAD reserves for selection, so left-dragging in the viewport rotates
# unexpectedly. These map the buttons the way the corresponding FreeCAD
# style does.
NAV_CAD = "cad"
NAV_BLENDER = "blender"
NAV_TRACKBALL = "trackball"


def navigation_style_from_preferences():
    """Read the user's configured FreeCAD navigation style, mapped onto
    the subset implemented here. Unknown styles fall back to trackball
    rather than guessing at bindings we have not reproduced."""
    try:
        import FreeCAD

        name = FreeCAD.ParamGet("User parameter:BaseApp/Preferences/View").GetString(
            "NavigationStyle", ""
        )
    except Exception:
        return NAV_CAD
    name = (name or "").lower()
    if "blender" in name:
        return NAV_BLENDER
    if "cad" in name or "gesture" in name or not name:
        return NAV_CAD
    return NAV_TRACKBALL


class CamNavigationStyle(vtkInteractorStyleTrackballCamera):
    """Mouse bindings closer to FreeCAD's, plus its view-direction keys.

    CAD style     middle drag pans, middle+right drags rotates, wheel
                  zooms, and left alone does nothing -- FreeCAD uses the
                  left button for selection.
    Blender style middle drags rotates, shift+middle pans, wheel zooms.
    Trackball     VTK's own behaviour, unchanged.
    """

    def __init__(self, view, style=NAV_CAD):
        self._view = view
        self._style = style
        self._middle = False
        self._right = False
        self.AddObserver("LeftButtonPressEvent", self._on_left_down)
        self.AddObserver("LeftButtonReleaseEvent", self._on_left_up)
        self.AddObserver("MiddleButtonPressEvent", self._on_middle_down)
        self.AddObserver("MiddleButtonReleaseEvent", self._on_middle_up)
        self.AddObserver("RightButtonPressEvent", self._on_right_down)
        self.AddObserver("RightButtonReleaseEvent", self._on_right_up)
        # Both: which of the two carries a usable KeySym varies by
        # platform and by how Qt forwards the event.
        self.AddObserver("CharEvent", self._on_char)
        self.AddObserver("KeyPressEvent", self._on_char)

    # -- helpers -------------------------------------------------------
    def _shift(self):
        iren = self.GetInteractor()
        return bool(iren and iren.GetShiftKey())

    def _stop(self):
        self.EndRotate()
        self.EndPan()
        self.EndDolly()

    # -- buttons -------------------------------------------------------
    def _focus_widget(self):
        try:
            self._view.widget.setFocus()
        except Exception:
            pass

    def _on_left_down(self, obj, event):
        # Clicking the viewport should make it the keyboard target, the
        # same as any other view.
        self._focus_widget()
        if self._style == NAV_TRACKBALL:
            self.OnLeftButtonDown()
        # CAD and Blender: the left button selects, so it must not move the
        # camera. Deliberately swallowed rather than passed to the base
        # class, whose trackball rotates on left-drag.

    def _on_left_up(self, obj, event):
        if self._style == NAV_TRACKBALL:
            self.OnLeftButtonUp()

    def _on_right_down(self, obj, event):
        self._right = True
        if self._style == NAV_TRACKBALL:
            self.OnRightButtonDown()
            return
        if self._style == NAV_CAD and self._middle:
            # Middle already held: the pair means rotate.
            self.EndPan()
            self.StartRotate()

    def _on_right_up(self, obj, event):
        self._right = False
        if self._style == NAV_TRACKBALL:
            self.OnRightButtonUp()
            return
        self._stop()
        if self._style == NAV_CAD and self._middle:
            # Back to panning: the middle button is still down.
            self.StartPan()

    def _on_middle_down(self, obj, event):
        self._middle = True
        self._focus_widget()
        if self._style == NAV_TRACKBALL:
            self.OnMiddleButtonDown()
        elif self._style == NAV_BLENDER:
            self.StartPan() if self._shift() else self.StartRotate()
        else:  # CAD
            self.StartRotate() if self._right else self.StartPan()

    def _on_middle_up(self, obj, event):
        self._middle = False
        if self._style == NAV_TRACKBALL:
            self.OnMiddleButtonUp()
            return
        self._stop()

    # -- keys ----------------------------------------------------------
    def _on_char(self, obj, event):
        iren = self.GetInteractor()
        if iren is None:
            return
        key = iren.GetKeySym()
        if not key:
            # Some platforms leave KeySym empty and only fill KeyCode.
            code = iren.GetKeyCode()
            key = code if code else None
        # FreeCAD's standard view keys, so muscle memory carries over.
        views = {
            "1": "front",
            "2": "top",
            "3": "right",
            "4": "rear",
            "5": "bottom",
            "6": "left",
            "0": "iso",
        }
        if key in views:
            self._view.set_view(views[key])
        elif key in ("f", "F"):
            self._view.set_view("fit")
        # Swallow the event: VTK's default CharEvent bindings include
        # things like 'w' for wireframe and 'e' for exit, neither of which
        # should be reachable from a simulation viewport.


class _ResizeBridge(object):
    """Forces the VTK render window to follow the Qt widget's size.

    QVTKRenderWindowInteractor is supposed to do this in its own
    resizeEvent, but when it does not the render window keeps whatever
    size it had at Initialize() -- the viewport draws into a fixed square
    in the corner and never fills its container. Driving it from an event
    filter does not depend on that path working.

    Implemented as a plain object with a QObject built at runtime, so the
    module still imports where PySide is unavailable (headless use).
    """

    def __init__(self, view):
        from PySide import QtCore

        bridge = self

        class _Filter(QtCore.QObject):
            def eventFilter(self, obj, event):
                if event.type() in (
                    QtCore.QEvent.Resize,
                    QtCore.QEvent.Show,
                    QtCore.QEvent.WindowStateChange,
                ):
                    bridge._view.sync_render_window_size()
                return False

        self._view = view
        self._filter = _Filter(view.widget)
        view.widget.installEventFilter(self._filter)


def _axis_angle(r):
    """Axis and angle (radians, 0 to pi) of a 3x3 rotation matrix.

    Through the quaternion (Shepperd's method), not acos of the trace: the
    trace loses the axis near 180 degrees, and a half turn is an ordinary
    3+2 workplane change.
    """
    r = np.asarray(r, dtype=np.float64)
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        q = (0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s)
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        q = ((r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s)
    elif r[1, 1] > r[2, 2]:
        s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        q = ((r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s)
    else:
        s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        q = ((r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s)
    w, v = q[0], np.array(q[1:], dtype=np.float64)
    if w < 0.0:  # the same rotation; this sign is the short way round
        w, v = -w, -v
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return np.array([0.0, 0.0, 1.0]), 0.0
    return v / n, 2.0 * float(np.arctan2(n, w))


def _rotation(axis, angle):
    """3x3 rotation by `angle` radians about unit `axis` (Rodrigues)."""
    k = np.asarray(axis, dtype=np.float64)
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + np.sin(angle) * kx + (1.0 - np.cos(angle)) * (kx @ kx)


class VtkStockView:
    """Renders a cStock's height grid with static geometry.

    Geometry is a function of grid dimensions only, so it is rebuilt when
    the stock is reseeded or the resolution changes -- never per tick.

    DISPLAY RESOLUTION IS CAPPED, AND MUST BE.

    Static geometry costs roughly 800 B per displayed cell across the CPU
    arrays and VTK's GPU copies (4 top verts + ~8 wall verts, each with
    position, tcoord and colourUV, plus a 32-bit index buffer). The
    simulation grid is 0.01 * accuracy * maxlen, so a 100 mm stock at
    accuracy 0.025 is 4000x4000 -- 16M cells, around 13 GB. Building
    geometry per simulation cell is not viable at real settings.

    So the display grid is capped independently: the simulation runs at
    full resolution while the geometry is at most max_cells on a side,
    with the height texture downsampled to match. Detail finer than one
    display cell is lost, but the block reduction takes the MINIMUM
    height, so a cut is never under-represented -- a pocket shows at its
    true depth rather than being averaged away.

    Note that capping can only ever shrink the display grid relative to
    the simulation grid, so it cannot make startup slower. attach() is
    instrumented so the actual distribution of setup time is visible
    rather than guessed at.
    """

    def __init__(self, parent=None, max_cells=None, nav_style=None):
        if QVTKRenderWindowInteractor is None:
            raise RuntimeError(
                "VTK's Qt interactor is unavailable in this build: " + "; ".join(QVTK_BASE_ERRORS)
            )
        self.widget = QVTKRenderWindowInteractor(parent)
        # Expand to whatever the layout gives it, and allow shrinking:
        # without a low minimum the widget can pin its container's size.
        #
        # QSizePolicy lives in QtGui in FreeCAD's PySide shim and in
        # QtWidgets elsewhere. Looking only in QtWidgets raises ImportError
        # here, which a bare except then hides -- leaving the policy unset
        # and the widget at QVTKRenderWindowInteractor's default 400x300.
        policy_cls = None
        for module_name in ("QtGui", "QtWidgets"):
            try:
                mod = __import__("PySide." + module_name, fromlist=[module_name])
                policy_cls = getattr(mod, "QSizePolicy", None)
                if policy_cls is not None:
                    break
            except ImportError:
                continue
        if policy_cls is not None:
            self.widget.setSizePolicy(policy_cls.Expanding, policy_cls.Expanding)
        else:
            print(
                "SimulatorVtkView: QSizePolicy not found; the viewport "
                "may not expand to fill its container"
            )
        self.widget.setMinimumSize(1, 1)
        self._relax_native_attributes()
        # Keyboard events reach VTK only through the widget, and only if it
        # can take focus. Without this the view keys do nothing at all --
        # no error, the keystrokes simply go somewhere else.
        try:
            from PySide import QtCore as _QtC

            self.widget.setFocusPolicy(_QtC.Qt.StrongFocus)
        except Exception:
            pass

        self.renderer = vtkRenderer()
        self.renderer.SetBackground(0.16, 0.17, 0.21)
        self.render_window = self.widget.GetRenderWindow()
        self.render_window.AddRenderer(self.renderer)
        self._started = False
        self._last_size = (0, 0)
        self._resize_bridge = None
        self._nav_name = nav_style or navigation_style_from_preferences()
        self._nav_style = None

        # Constraint 5.
        self._light_kit = vtkLightKit()
        self._light_kit.SetKeyLightIntensity(0.85)
        self._light_kit.SetKeyToFillRatio(2.2)
        self._light_kit.AddLightsToRenderer(self.renderer)

        self._max_cells = self._choose_max_cells(max_cells)
        self._sim = None
        self._actors = []
        # While the camera is being dragged, the simulation tick is still
        # firing every 8 ms -- each one a cut, a full texture re-upload and
        # a render, all competing with the frame the user is trying to
        # move. Updates are deferred for the duration instead of coarsening
        # the geometry: the cut keeps advancing in C++, nothing is dropped,
        # and the view stays at full resolution throughout.
        self._heights = None
        self._originals = None
        # Per-cell underside, or None when there is none to honour: the
        # self-test's synthetic scenes, or a module predating
        # GetBottomBuffer. See refresh() for how it is used.
        self._bottoms = None
        # False only when a real stock is attached and its module could
        # not supply the underside -- the one case where the picture
        # silently disagrees with the engine. The controller reports it.
        self.has_bottoms = True
        # Whether the underside is anywhere off the floor, and so worth
        # drawing. Decided per attach, in attach(), before the display
        # grid is planned: the geometry doubles when it is.
        self._has_underside = False
        # One boolean display-grid mask per drawn span below the primary:
        # which cells hold that span, and so get geometry. Empty when the
        # stock has no such spans, or the module cannot report them.
        self._layer_masks = []
        # Scratch for one GetSpanLayers call over the dirty rectangle,
        # allocated once at the full grid's size.
        self._layer_tops = None
        self._layer_bottoms = None
        self._layer_tex = []
        self._tool_actor = None
        # The tool's placement in the working frame, kept so it can be
        # recomposed when the working frame itself changes.
        self._tool_rows = None
        self._base_actor = None
        self._shell_actor = None
        # Grid-to-world transform for the heightmap actors. Identity for
        # plain 3-axis work; under 3+2 the grid is rebuilt in the new
        # working frame after each workplane change, and this carries it
        # back to document coordinates so the part stays put on screen
        # and the shell and model -- which are never transformed -- keep
        # lining up with it. Also composed into the tool placement, which
        # arrives in the working frame.
        self._stock_matrix = None  # vtkMatrix4x4, or None for identity
        self._stock_matrix_np = None  # the same, as a 4x4 float array
        # A camera turn in progress (see turn_camera), and its Qt timer --
        # made on first use, so a run with no workplane change never has
        # one.
        self._turn = None
        self._turn_timer = None
        # The operation banner (see show_banner) and its hide timer.
        self._banner = None
        self._banner_timer = None
        self._shell_visible = True
        self._show_uncut = True
        self._overlay = None
        # Stock geometry the camera was last framed for. The view is only
        # re-framed when this changes, so a reseed at the same size leaves
        # the user's viewpoint alone.
        self._framed_signature = None

    def _relax_native_attributes(self):
        """Try to stop the widget being a native, uncomposited window.

        QVTKRenderWindowInteractor sets WA_PaintOnScreen, which forces a
        native window: Qt then neither composites nor z-orders it, so under
        a theme where the docks overlay the MDI area the GL surface is
        drawn on top of them. The proper fix is a QOpenGLWidget base, but
        Qt6 moved that class into QtOpenGLWidgets and not every PySide6
        build ships it.

        This is a probe, not a guaranteed fix -- VTK may need the native
        surface to render at all. If the viewport comes up blank this is
        the first thing to put back.
        """
        try:
            from PySide import QtCore
        except Exception:
            return
        try:
            attr = getattr(QtCore.Qt, "WA_PaintOnScreen", None)
            if attr is not None and self.widget.testAttribute(attr):
                self.widget.setAttribute(attr, False)
        except Exception:
            pass

    def start(self):
        """Initialise the interactor. Must be called AFTER the widget has
        been placed in its final parent and shown.

        Initialising in __init__ creates the OpenGL context and native
        window while the widget is still parentless -- a top-level window
        of its own -- so the render window keeps that size and stops
        tracking the layout. Reparenting afterwards moves the widget but
        not the surface it draws into, which looks like a viewport stuck
        at a fixed resolution.
        """
        if self._started:
            return
        self.widget.Initialize()
        # Held on self: VTK keeps only a weak reference to the style, so a
        # locally-scoped one is collected and the viewport silently loses
        # all mouse handling.
        self._nav_style = CamNavigationStyle(self, self._nav_name)
        self.render_window.GetInteractor().SetInteractorStyle(self._nav_style)
        self.widget.Start()
        self.widget.setFocus()
        try:
            self._resize_bridge = _ResizeBridge(self)
        except Exception:
            self._resize_bridge = None
        # Re-check the size at the start of every frame. render=False:
        # calling Render() from inside a render would recurse.
        self.render_window.AddObserver(
            "StartEvent", lambda obj, evt: self.sync_render_window_size(False)
        )
        self.sync_render_window_size()
        self._started = True

    # ------------------------------------------------------------------
    # Attach / rebuild
    # ------------------------------------------------------------------
    def attach(self, sim, keep_camera=False):
        """Wrap a PathSim's stock. Call once after BeginSimulation(), and
        again after anything that recreates the stock -- the memoryviews
        do not survive that.

        `keep_camera` suppresses re-framing outright. The guard below
        compares grid extents, which is the right test when the grid is
        always in document coordinates; after a 3+2 reseed it is not --
        the extents are of a box in the new working frame, and they
        change on every rotation even though the part has not moved on
        screen at all, because set_stock_matrix puts it back where it
        was. Left to the guard, every workplane change would throw away
        the user's viewpoint.
        """
        self._sim = sim
        (
            h_mv,
            o_mv,
            nx,
            ny,
            res,
            origin_x,
            origin_y,
            floor_z,
            plane_z,
        ) = sim.GetHeightBuffer()

        # index = x * ny + y, matching Array2D::operator[]. Reshaping
        # rather than transposing keeps the upload a straight copy: the
        # texture's row axis is X and its column axis is Y, and the mesh's
        # texture coordinates are built to match.
        self._heights = np.frombuffer(h_mv, dtype=np.float32).reshape(nx, ny)
        self._originals = np.frombuffer(o_mv, dtype=np.float32).reshape(nx, ny)

        # The underside. Without it the view can only say "material from
        # the floor up to the top", which is false in a tilted frame: a
        # cut through the part leaves a top below its bottom but above
        # the floor, the engine counts the cell empty, and the view drew
        # a solid column there. Wrapped once, since the buffer lives as long
        # as the stock; its values change only when a cut promotes a lower
        # span, and refresh reads it live over the rectangle the cut marked.
        if hasattr(sim, "GetBottomBuffer"):
            self._bottoms = np.frombuffer(sim.GetBottomBuffer(), dtype=np.float32).reshape(nx, ny)
            self.has_bottoms = True
        else:
            self._bottoms = None
            self.has_bottoms = False

        # Is there an underside to draw? A stock sitting flat in its own
        # frame has none: every cell's underside is the floor, where
        # nothing can see it. A tilted or round one does, and so does any
        # 3+2 re-seed, and drawing it doubles the geometry per cell.
        #
        # Read once here rather than per frame. Cutting lowers tops and
        # can promote a lower span, which moves an underside DOWN, never
        # up from the floor -- so a stock with no underside at seed time
        # still has none later, and the decision holds until the next
        # attach, which every re-seed already forces.
        self._has_underside = bool(
            self._bottoms is not None and np.any(self._bottoms > floor_z + 1e-6)
        )

        # Spans below the primary, if the module can report them. Read in
        # full once here: the masks decide what geometry exists, and a
        # span cannot appear later -- cutting promotes lower spans into
        # the primary, which only ever removes them.
        self._layer_masks = []
        self._layer_tops = self._layer_bottoms = None
        if SPAN_LAYERS > 0 and hasattr(sim, "GetSpanLayers"):
            count = SPAN_LAYERS * nx * ny
            self._layer_tops = np.zeros(count, dtype=np.float32)
            self._layer_bottoms = np.zeros(count, dtype=np.float32)
            try:
                sim.GetSpanLayers(
                    0, 0, nx - 1, ny - 1, SPAN_LAYERS, self._layer_tops, self._layer_bottoms
                )
            except Exception:
                self._layer_tops = self._layer_bottoms = None

        # One display cell per `step` simulation cells on each axis.
        # Drawing the underside doubles the cost of a cell, so the grid
        # is held to what the flat one would have taken rather than
        # quietly costing twice the memory the cap was chosen for.
        per_cell = (
            UNDERSIDE_BYTES_PER_DISPLAY_CELL if self._has_underside else BYTES_PER_DISPLAY_CELL
        )
        # The ceiling is what this same stock would have taken without an
        # underside -- the figure the cap was actually chosen against, not
        # the cap itself, which the step's rounding usually leaves well
        # under.
        budget = None
        if self._has_underside:
            budget = plan_display_grid(nx, ny, self._max_cells, self.viewport_pixels())[3]
        self._dx, self._dy, self._step, _bytes = plan_display_grid(
            nx, ny, self._max_cells, self.viewport_pixels(), per_cell, budget
        )

        self._nx, self._ny = nx, ny
        self._res = res
        self._origin = (origin_x, origin_y)
        # The stock's true size. The last column and row of cells stop at
        # it; a module predating GetGridLength had whole cells throughout.
        if hasattr(sim, "GetGridLength"):
            self._length = tuple(sim.GetGridLength())
        else:
            self._length = (nx * res, ny * res)
        self._floor_z = floor_z
        # Guard against a degenerate stock height; the shader divides by
        # nothing but a zero span would make every cell land on the floor.
        self._span = max(plane_z - floor_z, 1e-6)

        # Which display cells hold each span below the primary. Needed
        # before the textures, which are allocated one pair per layer, and
        # before the geometry, which is built only where these are set.
        self._layer_masks = self._span_layer_masks()

        self._build_textures()
        self._build_geometry()
        self.refresh(full=True)
        # _build_geometry makes fresh actors on every attach, so the
        # setting would revert after each reseed.
        self.show_uncut_material(self._show_uncut)

        # Only frame the stock when there is something new to frame.
        # attach() runs again on every reseed -- which Play triggers -- and
        # resetting there threw away whatever view the user had set up.
        # The camera follows the stock's WORLD EXTENT, not its grid.
        #
        # nx, ny and res all change when the Accuracy slider moves, but
        # the stock is the same physical object at the same place -- so
        # including them here re-framed the view on every accuracy change
        # and threw away the user's viewpoint for no reason. What should
        # re-frame it is the stock actually being different: a new job, a
        # moved or resized stock.
        stock_signature = (origin_x, origin_y, floor_z, plane_z) + tuple(self._length)
        reframe = not keep_camera and not self._same_stock(
            stock_signature, self._framed_signature
        )
        if reframe:
            self.reset_camera()
            self._framed_signature = stock_signature

    # Millimetres. Below anything a user would call a different stock, and
    # above float noise in the extents. (The grid once overhung the stock
    # by up to a cell, which made this tolerance essential; it now covers
    # the stock exactly, and a module predating that still overhangs.)
    STOCK_TOLERANCE = 2.0

    @classmethod
    def _same_stock(cls, a, b):
        """Whether two stock extents are the same physical stock."""
        if a is None or b is None:
            return False
        return all(abs(x - y) <= cls.STOCK_TOLERANCE for x, y in zip(a, b))

    def viewport_pixels(self):
        """Render window size in device pixels, or None before the widget
        has been realised. Used to cap the display grid: more cells than
        pixels is geometry nobody can see."""
        try:
            w, h = self.render_window.GetSize()
        except Exception:
            return None
        if w <= 1 or h <= 1:
            return None
        return int(w), int(h)

    @property
    def max_cells(self):
        """The display-grid ceiling in use."""
        return self._max_cells

    @property
    def draws_underside(self):
        """Whether the geometry includes undersides, which doubles the
        cost of a display cell and so coarsens the display grid. False
        until the first attach."""
        return self._has_underside

    def _choose_max_cells(self, requested):
        """Pick a display-grid ceiling that fits in memory.

        An explicit value is honoured but still clamped to the GL texture
        limit, since the height texture is (max_cells + 2) square and
        would fail to allocate above it.
        """
        gl_max = self._gl_max_texture_size()
        if requested is not None:
            n = int(requested)
        else:
            avail = available_memory_bytes()
            budget = 512 * 1024 * 1024 if avail is None else avail * MEMORY_BUDGET_FRACTION
            # A safety net only: the viewport-pixel cap in
            # plan_display_grid is what normally decides the display grid,
            # since cells finer than pixels cannot be seen. This just
            # stops a very large window asking for more than the machine
            # can hold.
            n = int((budget / BYTES_PER_DISPLAY_CELL) ** 0.5)
            n = max(128, min(4096, n))
        if gl_max:
            n = min(n, gl_max - 2)
        # Floor last, after every clamp: a display grid below this is
        # useless, and zero is a division by zero in attach().
        return max(128, n)

    # Any GL implementation that can run this has at least 2048; OpenGL
    # itself guarantees 1024. A smaller answer means the question was asked
    # before a context existed, not that the limit is small.
    MIN_PLAUSIBLE_TEXTURE_SIZE = 1024

    def _gl_max_texture_size(self):
        """Maximum texture edge, or None if it cannot be trusted yet.

        Queried during __init__, before start() creates the GL context,
        this returns a meaningless small number. Accepting it made
        max_cells collapse to 0 and attach() divide by zero. Values below
        the guaranteed minimum are treated as "unknown" rather than as a
        real limit.
        """
        try:
            from vtkmodules.vtkRenderingOpenGL2 import vtkTextureObject

            size = int(vtkTextureObject.GetMaximumTextureSize(self.render_window))
        except Exception:
            return None
        if size < self.MIN_PLAUSIBLE_TEXTURE_SIZE:
            return None
        return size

    def _layer_present(self, top, bottom):
        """Which cells hold a span worth drawing.

        The SAME rule the primary is held to in refresh(): thinner than
        one grid cell and it goes. Dropping the rule here and keeping only
        a zero test drew sub-cell slivers of buried span where the primary
        in the same column had correctly gone -- thin sheets lying over
        the features the layer exists to show, and more of them after
        every transfer, since each one leaves slivers along the edges of a
        cavity.
        """
        return (top - bottom) >= self._res

    def _span_layer_masks(self):
        """A display-grid boolean per layer: where that span has material.

        Empty when there are no such spans to draw -- the ordinary case,
        and the one that must cost nothing.
        """
        if self._layer_tops is None:
            return []
        masks = []
        for k in range(SPAN_LAYERS):
            top, bottom = self._layer_view(k)
            present = self._layer_present(top, bottom)
            if not present.any():
                # Layers are filled from the primary down, so once one is
                # empty every deeper one is too.
                break
            masks.append(self._to_display_grid(present))
        return masks

    def _layer_view(self, k, width=None, height=None):
        """Layer `k` of the scratch buffers as two (width, height) arrays."""
        w = self._nx if width is None else width
        h = self._ny if height is None else height
        size = w * h
        start = k * size
        return (
            self._layer_tops[start : start + size].reshape(w, h),
            self._layer_bottoms[start : start + size].reshape(w, h),
        )

    def _to_display_grid(self, present):
        """Reduce a simulation-grid boolean to the display grid: a display
        cell is set when any simulation cell in it is."""
        step = self._step
        if step == 1:
            return present[: self._dx, : self._dy]
        s0 = np.arange(0, present.shape[0], step)
        s1 = np.arange(0, present.shape[1], step)
        return np.logical_or.reduceat(np.logical_or.reduceat(present, s0, axis=0), s1, axis=1)

    def _float_texture(self, tx, ty):
        """A single-component float texture and the array behind it.

        Returns (array, image, numpy view, texture). Single-component
        uint8 comes back as zero everywhere through
        SetColorModeToDirectScalars on this build, hence float32.
        """
        zeros = np.zeros((tx, ty), dtype=np.float32)
        arr = numpy_to_vtk(zeros.ravel(), deep=1, array_type=VTK_FLOAT)
        img = vtkImageData()
        img.SetDimensions(ty, tx, 1)
        img.GetPointData().SetScalars(arr)
        view = vtk_to_numpy(img.GetPointData().GetScalars()).reshape(tx, ty)
        tex = vtkTexture()
        tex.SetInputData(img)
        tex.SetColorModeToDirectScalars()
        tex.InterpolateOff()
        tex.RepeatOff()
        tex.EdgeClampOn()
        return arr, img, view, tex

    def _build_textures(self):
        # One texel border, pinned to the floor, so the stock's outer
        # sides are ordinary walls to a zero-height neighbour rather than
        # a special case.
        tx, ty = self._dx + 2, self._dy + 2
        self._tx, self._ty = tx, ty

        hn = np.zeros((tx, ty), dtype=np.float32)
        self._h_arr = numpy_to_vtk(hn.ravel(), deep=1, array_type=VTK_FLOAT)
        self._h_arr.SetName("height")
        self._h_img = vtkImageData()
        self._h_img.SetDimensions(ty, tx, 1)
        self._h_img.GetPointData().SetScalars(self._h_arr)
        self._h_view = vtk_to_numpy(self._h_img.GetPointData().GetScalars()).reshape(tx, ty)

        self._h_tex = vtkTexture()
        self._h_tex.SetInputData(self._h_img)
        self._h_tex.SetColorModeToDirectScalars()
        self._h_tex.InterpolateOff()  # Constraint 3.
        self._h_tex.RepeatOff()
        self._h_tex.EdgeClampOn()

        cn = np.empty((tx, ty, 3), dtype=np.uint8)
        cn[:, :] = UNCUT_COLOUR
        self._c_arr = numpy_to_vtk(cn.reshape(-1, 3), deep=1, array_type=VTK_UNSIGNED_CHAR)
        self._c_arr.SetName("colour")
        self._c_img = vtkImageData()
        self._c_img.SetDimensions(ty, tx, 1)
        self._c_img.GetPointData().SetScalars(self._c_arr)
        self._c_view = vtk_to_numpy(self._c_img.GetPointData().GetScalars()).reshape(tx, ty, 3)

        self._c_tex = vtkTexture()
        self._c_tex.SetInputData(self._c_img)
        self._c_tex.SetColorModeToDirectScalars()
        self._c_tex.InterpolateOff()
        self._c_tex.RepeatOff()
        self._c_tex.EdgeClampOn()

        # 1 where material has been removed, 0 where the cell is
        # untouched. float32 and single-component, exactly like the height
        # texture -- a single-component uint8 comes back as zero
        # everywhere through SetColorModeToDirectScalars on this build.
        mn = np.zeros((tx, ty), dtype=np.float32)
        self._m_arr = numpy_to_vtk(mn.ravel(), deep=1, array_type=VTK_FLOAT)
        self._m_arr.SetName("cutmask")
        self._m_img = vtkImageData()
        self._m_img.SetDimensions(ty, tx, 1)
        self._m_img.GetPointData().SetScalars(self._m_arr)
        self._m_view = vtk_to_numpy(self._m_img.GetPointData().GetScalars()).reshape(tx, ty)

        self._m_tex = vtkTexture()
        self._m_tex.SetInputData(self._m_img)
        self._m_tex.SetColorModeToDirectScalars()
        self._m_tex.InterpolateOff()
        self._m_tex.RepeatOff()
        self._m_tex.EdgeClampOn()

        # The underside, normalised exactly like the height texture. Zero
        # everywhere for a stock sitting flat in its own frame, and for a
        # module with no GetBottomBuffer -- which is the old behaviour,
        # material from the floor up, reached without a special case.
        bn = np.zeros((tx, ty), dtype=np.float32)
        self._b_arr = numpy_to_vtk(bn.ravel(), deep=1, array_type=VTK_FLOAT)
        self._b_arr.SetName("bottom")
        self._b_img = vtkImageData()
        self._b_img.SetDimensions(ty, tx, 1)
        self._b_img.GetPointData().SetScalars(self._b_arr)
        self._b_view = vtk_to_numpy(self._b_img.GetPointData().GetScalars()).reshape(tx, ty)

        self._b_tex = vtkTexture()
        self._b_tex.SetInputData(self._b_img)
        self._b_tex.SetColorModeToDirectScalars()
        self._b_tex.InterpolateOff()
        self._b_tex.RepeatOff()
        self._b_tex.EdgeClampOn()

        # One pair per drawn span below the primary. The shader reads
        # `heightTex` and `bottomTex`; binding a layer's pair to those
        # same names on its own actors draws that layer with no shader
        # change at all.
        self._layer_tex = []
        for _ in range(len(self._layer_masks)):
            top = self._float_texture(tx, ty)
            bottom = self._float_texture(tx, ty)
            self._layer_tex.append((top, bottom))

    # ------------------------------------------------------------------
    # Static geometry: cell tops plus inter-cell walls
    # ------------------------------------------------------------------
    def _quad_polydata(self, pos, tc, cuv):
        pos = np.ascontiguousarray(np.vstack(pos), dtype=np.float32)
        tc = np.ascontiguousarray(np.vstack(tc), dtype=np.float32)
        cuv = np.ascontiguousarray(np.vstack(cuv), dtype=np.float32)
        base = np.arange(0, len(pos), 4, dtype=_ID_DTYPE)
        faces = np.empty((len(base) * 2, 3), dtype=_ID_DTYPE)
        faces[0::2, 0], faces[0::2, 1], faces[0::2, 2] = base, base + 1, base + 2
        faces[1::2, 0], faces[1::2, 1], faces[1::2, 2] = base, base + 2, base + 3

        points = vtkPoints()
        points.SetData(numpy_to_vtk(pos, deep=1))
        pd = vtkPolyData()
        pd.SetPoints(points)
        cells = vtkCellArray()
        cells.SetData(
            numpy_to_vtk(
                np.arange(0, 3 * (len(faces) + 1), 3, dtype=_ID_DTYPE),
                deep=1,
                array_type=_VTK_ID_TYPE,
            ),
            numpy_to_vtk(np.ascontiguousarray(faces.ravel()), deep=1, array_type=_VTK_ID_TYPE),
        )
        try:
            cells.ConvertTo32BitStorage()
        except AttributeError:
            pass
        pd.SetPolys(cells)
        # Constraint 4: no normals array.
        a = numpy_to_vtk(tc, deep=1, array_type=VTK_FLOAT)
        a.SetName("TCoords")
        pd.GetPointData().SetTCoords(a)
        b = numpy_to_vtk(cuv, deep=1, array_type=VTK_FLOAT)
        b.SetName("ColourUV")
        pd.GetPointData().AddArray(b)
        return pd

    def _build_geometry(self):
        """Build the drawn geometry: the primary span, and one set more
        for each span layer below it that any column holds."""
        for actor in self._actors:
            self.renderer.RemoveActor(actor)
        self._actors = []
        self._add_layer_geometry(None, self._h_tex, self._b_tex, self._has_underside)
        for mask, ((_, _, _, top_tex), (_, _, _, bottom_tex)) in zip(
            self._layer_masks, self._layer_tex
        ):
            # Always with an underside: a span below a primary is material
            # with air under it, which is exactly the case that needs one.
            self._add_layer_geometry(mask, top_tex, bottom_tex, True)

    def _add_layer_geometry(self, mask, height_tex, bottom_tex, underside):
        # Display grid, not simulation grid. One display cell spans
        # `step` simulation cells, so its world size is step * res.
        nx, ny = self._dx, self._dy
        res = self._res * self._step
        tx, ty = self._tx, self._ty
        ox, oy = self._origin
        lx, ly = self._length
        top_z = self._floor_z + self._span  # bounds only; shader sets Z

        # Edge k of the display grid, in world units. The far edge is the
        # stock's own: the last display cell is narrower wherever the
        # stock is not a whole number of them.
        def ex(k):
            return ox + np.minimum(k * res, lx)

        def ey(k):
            return oy + np.minimum(k * res, ly)

        # `mask` is None for the primary span, which every cell has, and
        # a display-grid boolean for a layer below it, which few do. Cells
        # outside it get no geometry at all rather than collapsed
        # geometry: a pocket seen from another orientation touches a
        # narrow band of columns, and building the whole grid for it would
        # cost as much again as the primary.
        cells = np.ones((nx, ny), dtype=bool) if mask is None else np.asarray(mask, dtype=bool)
        if not cells.any():
            return

        # --- cell tops: four private corners each, so tops stay flat ---
        gx, gy = np.meshgrid(np.arange(1, nx + 1), np.arange(1, ny + 1), indexing="ij")
        keep = cells.ravel()
        gx = gx.ravel()[keep].astype(np.float32)
        gy = gy.ravel()[keep].astype(np.float32)
        n = gx.size
        wx0 = ex(gx - 1.0)
        wx1 = ex(gx)
        wy0 = ey(gy - 1.0)
        wy1 = ey(gy)

        tp = np.empty((n, 4, 3), dtype=np.float32)
        tp[:, 0, 0], tp[:, 0, 1] = wx0, wy0
        tp[:, 1, 0], tp[:, 1, 1] = wx1, wy0
        tp[:, 2, 0], tp[:, 2, 1] = wx1, wy1
        tp[:, 3, 0], tp[:, 3, 1] = wx0, wy1
        # Z here is never drawn -- the shader builds its position from
        # vertexMC.xy and its own zUnit -- but VTK reads it for the
        # actor's BOUNDS, and the bounds are what ResetCameraClippingRange
        # sizes the near and far planes from. All four corners at top_z
        # made a cap claim to be a flat plane at the top of the stock
        # while drawing near the floor, so with only that actor visible
        # the clipping planes closed around a surface that was not there
        # and cut most of the real one away, leaving a band that moved
        # with the camera. Spanning the same range the walls do keeps the
        # claim honest whatever the shader does with the height.
        tp[:, 0:2, 2] = self._floor_z
        tp[:, 2:4, 2] = top_z

        # Texture row axis is X, column axis is Y, matching the buffer.
        tt = np.empty((n, 4, 2), dtype=np.float32)
        tt[:, :, 0] = ((gy + 0.5) / ty)[:, None]
        tt[:, :, 1] = ((gx + 0.5) / tx)[:, None]
        tops = self._quad_polydata([tp.reshape(-1, 3)], [tt.reshape(-1, 2)], [tt.reshape(-1, 2)])

        # The same quads again, wound the other way, for the underside.
        # Only where there is one: a stock sitting flat has its underside
        # on the floor, where nothing can see it, and paying twice the
        # geometry to draw it there would halve the display grid for
        # nothing. The shader puts these at the bottom texture's height.
        bottoms = None
        if underside:
            bp = tp[:, ::-1, :].copy()
            bottoms = self._quad_polydata(
                [bp.reshape(-1, 3)], [tt.reshape(-1, 2)], [tt.reshape(-1, 2)]
            )

        # --- walls between every adjacent texel pair -------------------
        wp, wt, wc = [], [], []
        for axis in (0, 1):
            if axis == 0:
                a, b = np.meshgrid(np.arange(0, nx + 1), np.arange(1, ny + 1), indexing="ij")
            else:
                a, b = np.meshgrid(np.arange(1, nx + 1), np.arange(0, ny + 1), indexing="ij")
            a = a.ravel()
            b = b.ravel()
            # An edge is drawn when EITHER side holds the span: that is
            # the wall closing it off. Padded so an edge at the grid's rim
            # sees a neighbour that holds nothing.
            padded = np.zeros((nx + 2, ny + 2), dtype=bool)
            padded[1:-1, 1:-1] = cells
            if axis == 0:
                keep = padded[a, b] | padded[a + 1, b]
            else:
                keep = padded[a, b] | padded[a, b + 1]
            a = a[keep].astype(np.float32)
            b = b[keep].astype(np.float32)
            nq = a.size
            if nq == 0:
                continue
            p = np.empty((nq, 4, 3), dtype=np.float32)
            t = np.empty((nq, 4, 2), dtype=np.float32)
            c = np.empty((nq, 4, 2), dtype=np.float32)

            if axis == 0:
                xw = ex(a)
                p[:, 0, 0], p[:, 0, 1] = xw, ey(b - 1.0)
                p[:, 1, 0], p[:, 1, 1] = xw, ey(b)
                p[:, 2, 0], p[:, 2, 1] = xw, ey(b)
                p[:, 3, 0], p[:, 3, 1] = xw, ey(b - 1.0)
                lu, lv = (b + 0.5) / ty, (a + 0.5) / tx
                ru, rv = (b + 0.5) / ty, (a + 1.5) / tx
            else:
                yw = ey(b)
                p[:, 0, 0], p[:, 0, 1] = ex(a - 1.0), yw
                p[:, 1, 0], p[:, 1, 1] = ex(a), yw
                p[:, 2, 0], p[:, 2, 1] = ex(a), yw
                p[:, 3, 0], p[:, 3, 1] = ex(a - 1.0), yw
                lu, lv = (b + 0.5) / ty, (a + 0.5) / tx
                ru, rv = (b + 1.5) / ty, (a + 0.5) / tx

            p[:, 0, 2] = p[:, 1, 2] = self._floor_z
            p[:, 2, 2] = p[:, 3, 2] = top_z

            # Two corners follow one neighbour's height, two the other's,
            # so the quad is exactly the vertical step between them and
            # collapses to zero area when the two heights agree. No
            # geometry is added or removed as the surface changes.
            t[:, 0, 0], t[:, 0, 1] = lu, lv
            t[:, 1, 0], t[:, 1, 1] = lu, lv
            t[:, 2, 0], t[:, 2, 1] = ru, rv
            t[:, 3, 0], t[:, 3, 1] = ru, rv
            c[:, 0, 0], c[:, 0, 1] = ru, rv
            c[:, 1, 0], c[:, 1, 1] = ru, rv
            c[:, 2, 0], c[:, 2, 1] = lu, lv
            c[:, 3, 0], c[:, 3, 1] = lu, lv

            wp.append(p.reshape(-1, 3))
            wt.append(t.reshape(-1, 2))
            wc.append(c.reshape(-1, 2))

        if not wp:
            return
        walls = self._quad_polydata(wp, wt, wc)

        # `mask` is None only for the primary span.
        layer = mask is not None

        def add(pd, kind):
            actor = self._make_actor(pd, kind, height_tex, bottom_tex, layer)
            self._actors.append(actor)

        add(tops, KIND_TOP)
        add(walls, KIND_WALL)
        if underside:
            # The wall quads a second time: the same positions and the
            # same texel pairs, following the undersides instead of the
            # tops. Between two columns that both hold material this is
            # the step between their undersides; elsewhere it collapses.
            add(bottoms, KIND_BOTTOM)
            add(self._quad_polydata(wp, wt, wc), KIND_BOTTOM_WALL)

    def _make_actor(self, pd, kind=KIND_TOP, height_tex=None, bottom_tex=None, layer=False):
        mapper = vtkPolyDataMapper()
        mapper.SetInputData(pd)
        mapper.ScalarVisibilityOff()
        mapper.StaticOn()
        try:
            mapper.SetVBOShiftScaleMethod(_SHIFT_SCALE_OFF)  # Constraint 1.
        except AttributeError:
            pass
        mapper.MapDataArrayToVertexAttribute(
            "colourUV", "ColourUV", vtkDataObject.FIELD_ASSOCIATION_POINTS, -1
        )

        actor = vtkActor()
        actor.SetMapper(mapper)
        # Identity in 3-axis work; under 3+2 this is what carries the grid
        # out of the working frame it was re-seeded in and back into
        # document coordinates. Set here rather than only in
        # set_stock_matrix because these actors are new on every attach.
        actor.SetUserMatrix(self._stock_matrix)
        prop = actor.GetProperty()
        prop.SetColor(1.0, 1.0, 1.0)
        prop.SetAmbient(0.34)
        prop.SetDiffuse(0.72)
        prop.SetSpecular(0.05)
        prop.BackfaceCullingOff()
        prop.SetTexture("heightTex", height_tex if height_tex is not None else self._h_tex)
        prop.SetTexture("bottomTex", bottom_tex if bottom_tex is not None else self._b_tex)
        prop.SetTexture("cutTex", self._m_tex)
        actor.SetTexture(self._c_tex)

        sp = actor.GetShaderProperty()
        sp.GetVertexCustomUniforms().SetUniformf("heightScale", self._span)
        sp.GetVertexCustomUniforms().SetUniformf("heightOffset", self._floor_z)
        sp.GetVertexCustomUniforms().SetUniformf("showUncut", 1.0 if self._show_uncut else 0.0)
        sp.GetVertexCustomUniforms().SetUniformf("quadKind", float(kind))
        # Set on every actor, not only the layers: a uniform the
        # shader reads and nothing writes is undefined, and this one
        # decides whether a face can be coloured as machined at all.
        sp.GetVertexCustomUniforms().SetUniformf("isLayer", 1.0 if layer else 0.0)
        sp.AddVertexShaderReplacement("//VTK::PositionVC::Dec", True, _VS_DEC, False)
        sp.AddVertexShaderReplacement(
            "//VTK::PositionVC::Impl", True, _VS_IMPL, False  # Constraint 2.
        )
        self.renderer.AddActor(actor)
        return actor

    # ------------------------------------------------------------------
    # Cutting tool
    # ------------------------------------------------------------------
    @staticmethod
    def _shape_to_polydata(shape, tolerance):
        """Tessellate a Part shape into a vtkPolyData, or None.

        Shared by the tool and the base shape. Both are plain geometry:
        no height shader, no textures, and no shift-scale concern, since
        neither writes a position component by hand.
        """
        if shape is None:
            return None
        pts, facets = shape.tessellate(tolerance)
        if not pts or not facets:
            return None

        pos = np.empty((len(pts), 3), dtype=np.float32)
        for i, v in enumerate(pts):
            pos[i, 0], pos[i, 1], pos[i, 2] = v.x, v.y, v.z
        tris = np.asarray(facets, dtype=_ID_DTYPE)

        points = vtkPoints()
        points.SetData(numpy_to_vtk(pos, deep=1))
        pd = vtkPolyData()
        pd.SetPoints(points)
        cells = vtkCellArray()
        cells.SetData(
            numpy_to_vtk(
                np.arange(0, 3 * (len(tris) + 1), 3, dtype=_ID_DTYPE),
                deep=1,
                array_type=_VTK_ID_TYPE,
            ),
            numpy_to_vtk(np.ascontiguousarray(tris.ravel()), deep=1, array_type=_VTK_ID_TYPE),
        )
        pd.SetPolys(cells)
        return pd

    def set_tool(self, shape, tolerance=0.1):
        """Tessellate the tool once and keep it. The shape only changes
        when the operation changes, so per-tick cost is a 4x4 matrix and
        nothing else -- the same principle as the stock geometry."""
        self.clear_tool()
        pd = self._shape_to_polydata(shape, tolerance)
        if pd is None:
            return

        mapper = vtkPolyDataMapper()
        mapper.SetInputData(pd)
        mapper.ScalarVisibilityOff()
        mapper.StaticOn()

        actor = vtkActor()
        actor.SetMapper(mapper)
        prop = actor.GetProperty()
        prop.SetColor(0.35, 0.72, 0.95)
        prop.SetOpacity(0.55)
        prop.SetAmbient(0.25)
        prop.SetSpecular(0.25)
        prop.SetSpecularPower(35)
        # Plain geometry: no height shader, no textures, and no
        # shift-scale concern, since nothing here writes a position
        # component by hand.
        self.renderer.AddActor(actor)
        self._tool_actor = actor

    # ------------------------------------------------------------------
    # Base shape (the target model)
    # ------------------------------------------------------------------
    def show_uncut_material(self, visible):
        """Draw, or not, the cells nothing has touched yet.

        The heightmap holds one height per cell, so below a curved or
        tilted silhouette the untouched material is drawn as a straight
        extrusion to the floor -- a shape the stock never had. Hiding it
        leaves only the cut surface, with the translucent shell showing
        the true original extent.

        A uniform, not a rebuild: this is a view option, and regenerating
        the geometry per click would cost a full rebuild.
        """
        self._show_uncut = bool(visible)
        value = 1.0 if self._show_uncut else 0.0
        for actor in self._actors:
            try:
                actor.GetShaderProperty().GetVertexCustomUniforms().SetUniformf("showUncut", value)
            except Exception:
                pass

    def set_stock_shell(self, shape, tolerance=0.2):
        """A translucent shell of the original stock, never modified.

        The heightmap can only hold the topmost surface per cell, so a
        sphere or a tilted block renders as a silhouette extruded to the
        floor. Drawing the real solid alongside it and cutting one against
        the other was tried and abandoned: the grid's silhouette is
        stair-stepped and the solid's is smooth, so the two disagree by a
        fraction of a cell along every boundary, and no per-fragment test
        reconciles them.

        A shell that is never modified has no boundary to reconcile. It
        also shows the ORIGINAL extent for the whole run, so how much has
        come off is visible rather than only what is left.

        Depth writing is off. With it on, the near face of the shell hides
        the cut surface behind it -- the shell encloses the entire scene,
        so everything is behind it.
        """
        self.clear_stock_shell()
        pd = self._shape_to_polydata(shape, tolerance)
        if pd is None:
            return False

        mapper = vtkPolyDataMapper()
        mapper.SetInputData(pd)
        mapper.ScalarVisibilityOff()
        mapper.StaticOn()

        actor = vtkActor()
        actor.SetMapper(mapper)
        prop = actor.GetProperty()
        prop.SetColor(0.62, 0.66, 0.72)
        prop.SetOpacity(SHELL_OPACITY)
        prop.SetAmbient(0.45)
        prop.SetDiffuse(0.55)
        prop.SetSpecular(0.0)
        # Both faces: looking through the shell means seeing its far side
        # as well, and culling one of them makes the far side vanish where
        # it should still tint what is behind it.
        prop.BackfaceCullingOff()
        prop.FrontfaceCullingOff()

        actor.ForceTranslucentOn()
        mapper.SetResolveCoincidentTopologyToPolygonOffset()
        self.renderer.AddActor(actor)
        self._shell_actor = actor
        self.show_stock_shell(self._shell_visible)
        return True

    def show_stock_shell(self, visible):
        self._shell_visible = bool(visible)
        if self._shell_actor is not None:
            self._shell_actor.SetVisibility(self._shell_visible)

    def clear_stock_shell(self):
        if self._shell_actor is not None:
            self.renderer.RemoveActor(self._shell_actor)
            self._shell_actor = None

    def set_base_shape(self, shape, tolerance=0.1):
        """Draw the target model inside the stock.

        Opaque and depth-tested, deliberately. While material still covers
        it the model is hidden; where a cut has gone past the model, its
        surface shows through. So it reads directly as a gouge indicator
        rather than needing the two surfaces compared by eye.
        """
        self.clear_base_shape()
        pd = self._shape_to_polydata(shape, tolerance)
        if pd is None:
            return

        mapper = vtkPolyDataMapper()
        mapper.SetInputData(pd)
        mapper.ScalarVisibilityOff()
        mapper.StaticOn()

        actor = vtkActor()
        actor.SetMapper(mapper)
        prop = actor.GetProperty()
        prop.SetColor(0.30, 0.78, 0.42)
        prop.SetAmbient(0.22)
        prop.SetDiffuse(0.78)
        prop.SetSpecular(0.15)
        self.renderer.AddActor(actor)
        self._base_actor = actor

    def show_base_shape(self, visible):
        if self._base_actor is not None:
            self._base_actor.SetVisibility(bool(visible))

    def has_base_shape(self):
        return self._base_actor is not None

    def clear_base_shape(self):
        if self._base_actor is not None:
            self.renderer.RemoveActor(self._base_actor)
            self._base_actor = None

    # ------------------------------------------------------------------
    # Working-frame transform (3+2)
    # ------------------------------------------------------------------
    @staticmethod
    def _matrix_rows(m):
        """A FreeCAD Matrix as a flat row-major tuple of 16 floats.

        Read through the A11..A44 attributes rather than the .A tuple:
        both exist, but only the attributes are documented row-major, and
        this module deliberately does not import FreeCAD, so there is
        nothing here to check the convention against at runtime.
        """
        return (
            m.A11,
            m.A12,
            m.A13,
            m.A14,
            m.A21,
            m.A22,
            m.A23,
            m.A24,
            m.A31,
            m.A32,
            m.A33,
            m.A34,
            m.A41,
            m.A42,
            m.A43,
            m.A44,
        )

    @classmethod
    def _to_vtk_matrix(cls, rows):
        vm = vtkMatrix4x4()
        vm.DeepCopy(tuple(rows))
        return vm

    def set_stock_matrix(self, matrix=None):
        """Place the heightmap in the world.

        `matrix` maps grid coordinates to document coordinates -- under
        3+2 that is the inverse of the workplane rotation the grid was
        re-seeded in. None means identity, which is the whole of 3-axis
        work.

        Applied as an actor UserMatrix, not baked into the geometry: the
        staircase is built once per attach and a reseed rebuilds it
        anyway, but the tool needs the same transform every tick and a
        4x4 is the only thing that should change at that rate.

        Stored, and re-applied by _make_actor, because _build_geometry
        makes fresh actors on every attach -- the same reason
        show_uncut_material is re-applied there.
        """
        if matrix is None:
            self._stock_matrix = None
            self._stock_matrix_np = None
        else:
            rows = self._matrix_rows(matrix)
            self._stock_matrix = self._to_vtk_matrix(rows)
            self._stock_matrix_np = np.asarray(rows, dtype=np.float64).reshape(4, 4)
        for actor in self._actors:
            actor.SetUserMatrix(self._stock_matrix)
        # The tool's matrix was composed against the previous stock
        # matrix and is now stale by exactly this change.
        self._reapply_tool_matrix()

    # -- turning the camera with the working frame --------------------
    # How long a workplane change takes to turn the camera, for a quarter
    # turn. Scaled with the angle, so a small tilt does not drag and a half
    # turn gets more time to follow, within the limits below. The
    # simulator pauses cutting while it plays. 0 turns it at once.
    CAMERA_TURN_MS = 1500
    CAMERA_TURN_MIN_MS = 800
    CAMERA_TURN_MAX_MS = 2200
    CAMERA_TURN_TICK_MS = 16

    def turn_camera(self, matrix, animate=True):
        """Orbit the camera by the rotation part of `matrix`.

        For a 3+2 workplane change the caller passes new^-1 * old, the
        frames being document -> working. The heightmap stays in document
        coordinates, so without this the part holds still on screen and
        the tool tilts; with it, the camera keeps its pose relative to the
        WORKING frame, so the tool comes down the screen as it did before
        and the part turns, as it does when it is re-clamped. Only the
        view moves: nothing here touches the simulation.

        The pivot is the focal point, what the user was looking at, so it
        stays where it is on screen; distance, zoom and projection are
        kept.

        A turn requested while one is running is folded into it: the
        camera carries on from where it is to where both turns together
        end, in one movement. So turns compose exactly however quickly
        workplanes change, and a pair that cancels -- Stop and replay
        turning back to the document frame, then straight into the first
        operation's -- barely moves the camera at all.
        """
        rows = np.asarray(self._matrix_rows(matrix), dtype=np.float64).reshape(4, 4)
        q = rows[:3, :3]
        turn = self._turn
        if turn is not None:
            # From the turn's start: its own full rotation, then this one.
            # From the camera as it is now: that, less what has played.
            played = _rotation(turn["axis"], turn["angle"] * turn["eased"])
            q = q @ _rotation(turn["axis"], turn["angle"]) @ played.T
            self._turn = None
        axis, angle = _axis_angle(q)
        if angle < 1e-9:
            self._stop_camera_turn()
            return
        cam = self.renderer.GetActiveCamera()
        self._turn = {
            "axis": axis,
            "angle": angle,
            "position": np.array(cam.GetPosition(), dtype=np.float64),
            "focal": np.array(cam.GetFocalPoint(), dtype=np.float64),
            "up": np.array(cam.GetViewUp(), dtype=np.float64),
            "t0": time.perf_counter(),
            "eased": 0.0,  # how much of the turn is on screen
            "ms": self._camera_turn_ms(angle),
        }
        timer = self._camera_turn_timer() if animate and self.CAMERA_TURN_MS > 0 else None
        if timer is None:
            self._finish_camera_turn()
            return
        timer.start(self.CAMERA_TURN_TICK_MS)

    def _camera_turn_ms(self, angle):
        """Duration of a turn through `angle` radians."""
        ms = self.CAMERA_TURN_MS * angle / (0.5 * np.pi)
        return min(max(ms, self.CAMERA_TURN_MIN_MS), self.CAMERA_TURN_MAX_MS)

    @property
    def camera_turning(self):
        """Whether a turn is still playing. The simulator holds its cuts
        until it is not."""
        return self._turn is not None

    def finish_camera_turn(self):
        """Land a turn in progress at its end, now."""
        self._finish_camera_turn()

    def _camera_turn_timer(self):
        """The turn's Qt timer, or None if Qt cannot be reached, in which
        case the camera turns at once. Imported here, not at the top: this
        module otherwise needs Qt only through VTK's widget."""
        if self._turn_timer is None:
            try:
                from PySide import QtCore
            except ImportError:
                try:
                    from PySide6 import QtCore
                except ImportError:
                    return None
            self._turn_timer = QtCore.QTimer()
            self._turn_timer.timeout.connect(self._camera_turn_tick)
        return self._turn_timer

    def _camera_turn_tick(self):
        if self._turn is None:
            self._stop_camera_turn()
            return
        fraction = (time.perf_counter() - self._turn["t0"]) * 1000.0 / self._turn["ms"]
        if fraction >= 1.0:
            self._finish_camera_turn()
        else:
            self._apply_camera_turn(fraction)

    def _apply_camera_turn(self, fraction):
        """Put the camera `fraction` of the way through the turn, eased
        in and out. Each step is computed from the pose the turn started
        at, never from the last step, so nothing accumulates."""
        turn = self._turn
        eased = 0.5 - 0.5 * np.cos(np.pi * min(max(fraction, 0.0), 1.0))
        turn["eased"] = eased
        r = _rotation(turn["axis"], turn["angle"] * eased)
        focal = turn["focal"]
        position = focal + r @ (turn["position"] - focal)
        up = r @ turn["up"]
        cam = self.renderer.GetActiveCamera()
        cam.SetFocalPoint(*focal)
        cam.SetPosition(*position)
        cam.SetViewUp(*up)
        cam.OrthogonalizeViewUp()
        self.renderer.ResetCameraClippingRange()
        if self.render_window is not None:
            self.render_window.Render()

    def _finish_camera_turn(self):
        """Land a turn in progress at its end."""
        if self._turn is not None:
            self._apply_camera_turn(1.0)
        self._stop_camera_turn()

    def _stop_camera_turn(self):
        """Abandon a turn where it is: for anything that sets the camera
        outright, which a later tick would otherwise undo."""
        self._turn = None
        if self._turn_timer is not None:
            self._turn_timer.stop()
        # However the turn ended, a banner waiting for it can now go.
        self._arm_banner_hide()

    # -- the operation banner ------------------------------------------
    # How long the banner stays up once the camera has stopped turning.
    BANNER_HOLD_MS = 1500

    def show_banner(self, text):
        """Name what is coming, top centre, while the camera turns.

        Stays up for the whole turn, however long it takes, and for
        BANNER_HOLD_MS after it; with no turn running, for BANNER_HOLD_MS.
        A later banner replaces an earlier one and restarts the count.

        Needs Qt for the timer that takes it down again. Without one it
        is not shown at all, rather than left up for good.

        """
        timer = self._banner_hide_timer()
        if timer is None:
            return
        if self._banner is None:
            actor = vtkTextActor()
            prop = actor.GetTextProperty()
            prop.SetFontSize(15)
            prop.BoldOn()
            # Right-justified and pinned to the top right, so the text
            # grows leftwards from a fixed corner instead of shifting
            # under itself as the operation name changes length.
            prop.SetJustificationToRight()
            prop.SetVerticalJustificationToTop()
            prop.SetLineSpacing(1.2)
            prop.SetColor(0.96, 0.97, 0.99)
            try:
                # A dark band behind it, so it reads against the stock.
                prop.SetBackgroundColor(0.08, 0.09, 0.11)
                prop.SetBackgroundOpacity(0.6)
            except AttributeError:
                prop.ShadowOn()
            actor.GetPositionCoordinate().SetCoordinateSystemToNormalizedViewport()
            actor.SetPosition(0.985, 0.97)
            # AddViewProp, not AddActor2D: the same thing, and present in
            # every VTK, where AddActor2D is not.
            self.renderer.AddViewProp(actor)
            self._banner = actor
        self._banner.SetInput(text)
        self._banner.SetVisibility(True)
        timer.stop()
        self._arm_banner_hide()
        if self.render_window is not None:
            self.render_window.Render()

    def _banner_hide_timer(self):
        """The banner's single-shot hide timer, or None without Qt."""
        if self._banner_timer is None:
            try:
                from PySide import QtCore
            except ImportError:
                try:
                    from PySide6 import QtCore
                except ImportError:
                    return None
            self._banner_timer = QtCore.QTimer()
            self._banner_timer.setSingleShot(True)
            self._banner_timer.timeout.connect(self._hide_banner)
        return self._banner_timer

    def _arm_banner_hide(self):
        """Start the banner's countdown, unless a turn is still playing:
        the turn's end calls this again."""
        if self._banner is None or not self._banner.GetVisibility() or self._turn is not None:
            return
        if self._banner_timer is not None:
            self._banner_timer.start(self.BANNER_HOLD_MS)

    def _hide_banner(self):
        if self._banner is not None and self._banner.GetVisibility():
            self._banner.SetVisibility(False)
            if self.render_window is not None:
                self.render_window.Render()

    def _reapply_tool_matrix(self):
        if self._tool_actor is None or self._tool_rows is None:
            return
        rows = self._tool_rows
        if self._stock_matrix_np is not None:
            rows = tuple(
                (self._stock_matrix_np @ np.asarray(rows, dtype=np.float64).reshape(4, 4)).ravel()
            )
        self._tool_actor.SetUserMatrix(self._to_vtk_matrix(rows))

    def set_tool_placement(self, placement):
        """Position the tool from a FreeCAD Placement.

        curpos is in the simulator's working frame, the same frame the
        heightmap is built in. In plain 3-axis work that is also the
        document frame and the placement goes straight on the actor; with
        a workplane rotation the stock actors carry a grid-to-world
        matrix, and the tool has to be put through the same one or it
        would mill the air beside the part.
        """
        if self._tool_actor is None:
            return
        self._tool_rows = self._matrix_rows(placement.toMatrix())
        self._reapply_tool_matrix()

    def show_tool(self, visible):
        if self._tool_actor is not None:
            self._tool_actor.SetVisibility(bool(visible))

    def shutdown(self):
        """Release everything while the GL context is still valid.

        Left to Python's garbage collector, the widget is destroyed at an
        arbitrary later moment -- typically when the next simulator run
        allocates enough to trigger a collection. By then the X window is
        gone, so VTK's DestroyWindow calls ReleaseGraphicsResources, which
        calls MakeCurrent on a dead context and dereferences null. That is
        a segfault inside the GC, with a backtrace pointing at whatever
        happened to be running.

        Order matters: drop the props, then finalise the render window,
        then release the references. Reversing any pair puts a VTK object
        back in the collector's hands.
        """
        self._stop_camera_turn()
        if self._banner_timer is not None:
            self._banner_timer.stop()
        self._banner = None  # RemoveAllViewProps below takes the actor
        try:
            self.clear_tool()
            self.clear_base_shape()
            self.clear_stock_shell()
        except Exception:
            pass
        try:
            self.renderer.RemoveAllViewProps()
        except Exception:
            pass
        self._actors = []
        try:
            # Releases the GL resources now, on a live context.
            if hasattr(self.widget, "Finalize"):
                self.widget.Finalize()
            elif self.render_window is not None:
                self.render_window.Finalize()
        except Exception:
            pass

        # Break every reference to VTK objects and to the C++ buffers, so
        # nothing is left for the collector to touch after the widget goes.
        self._sim = None
        self._heights = None
        self._originals = None
        self._bottoms = None
        self._h_view = None
        self._c_view = None
        self._h_arr = None
        self._h_img = None
        self._h_tex = None
        self._c_arr = None
        self._c_img = None
        self._c_tex = None
        self._b_view = None
        self._b_arr = None
        self._b_img = None
        self._b_tex = None
        self._layer_tex = []
        self._layer_masks = []
        self._layer_tops = None
        self._layer_bottoms = None
        # The cut mask was left out of this list when it was added, so its
        # image and texture outlived the widget along with it.
        self._m_view = None
        self._m_arr = None
        self._m_img = None
        self._m_tex = None
        self._nav_style = None
        self._resize_bridge = None
        self.renderer = None
        self.render_window = None

    def clear_tool(self):
        if self._tool_actor is not None:
            self.renderer.RemoveActor(self._tool_actor)
            self._tool_actor = None
        # Belongs to the actor that has just gone; keeping it would put
        # the previous operation's last position on the next tool before
        # its first move.
        self._tool_rows = None

    # ------------------------------------------------------------------
    # Per tick
    # ------------------------------------------------------------------
    def refresh(self, full=False):
        """Upload whatever changed and redraw. Replaces the GetResultMesh
        assignment; cost is proportional to the dirty rectangle, not to
        stock size."""
        # A full refresh needs no simulation: the dirty rectangle is the
        # whole grid and the data comes from self._heights. Only the
        # incremental path asks the simulator what changed.
        if self._sim is None and not full:
            return
        if full:
            rect = (0, 0, self._nx - 1, self._ny - 1)
        else:
            rect = self._sim.ConsumeViewDirty()
            if rect is None:
                return

        x0, y0, x1, y1 = rect
        step = self._step

        # Expand the dirty rectangle out to display-cell boundaries: a
        # block's value depends on every simulation cell inside it, so
        # reducing a partial block would use incomplete data and show up
        # as flicker along the tool path.
        dx0, dy0 = x0 // step, y0 // step
        dx1 = min(x1 // step, self._dx - 1)
        dy1 = min(y1 // step, self._dy - 1)
        sx0, sy0 = dx0 * step, dy0 * step
        sx1 = min((dx1 + 1) * step, self._nx)
        sy1 = min((dy1 + 1) * step, self._ny)
        if sx1 <= sx0 or sy1 <= sy0:
            return

        sub_h = self._heights[sx0:sx1, sy0:sy1]
        sub_o = self._originals[sx0:sx1, sy0:sy1]

        # The cut mask comes from the TRUE heights, before the underside
        # is folded in below. Folded first, a cell seeded thinner than one
        # cell -- the edge of a tilted or round stock -- would read as
        # "below its original", be flagged cut, and colour its
        # neighbours' walls orange and let them through with Uncut off.
        flag = sub_h < (sub_o - 1e-6)

        # Was this column ever stock? Asked of the SEEDED pair -- the
        # original top against the underside, which cutting never moves --
        # and held to the same one-cell rule as the material test below,
        # so a column too thin to draw was never drawable rather than
        # becoming outside the moment it is touched. This is what the
        # shader's skirt test reads; inferring it from the cut flag is
        # what put a machined face on the outer wall (see MASK_OUTSIDE).
        sub_b = None
        if self._bottoms is not None:
            sub_b = self._bottoms[sx0:sx1, sy0:sy1]
            solid = (sub_o - sub_b) >= self._res
        else:
            # No GetBottomBuffer: material runs from the floor up, so the
            # seeded top alone says whether there was any. The old model,
            # reached without a special case.
            solid = sub_o > (self._floor_z + self._res)

        # A cell holds material only while its top is at least one
        # simulation cell above its bottom -- the same test, at the same
        # threshold, as cStock::ColumnSpan and the tessellation sweeps,
        # so the picture and the exported mesh agree about which cells
        # exist. Where it fails the height goes to the floor, and the
        # shader's existing `empty` test collapses the cell. No shader
        # change, so constraint 6 is untouched: `empty` is already
        # written over max() of both texels.
        #
        # Walls beside such a cell still run down to the floor, not to
        # the neighbour's bottom -- the geometry has no underside. This
        # stops the view drawing material the engine says is gone; it
        # does not draw the air under a tilted part.
        #
        # Guarded on sub_b, not on self._bottoms: without the underside
        # there is nothing to fold and nothing to pin, and reaching this
        # with sub_b still None is a subtraction against None on every
        # tick. It got here once, nested one level too deep.
        if sub_b is not None:
            # Both surfaces are pinned to the floor together, so a cell
            # the engine counts empty reads as empty on the GPU too --
            # there the test is top minus underside, and an underside
            # left where it was would leave a sliver of material behind.
            thin = (sub_h - sub_b) < self._res
            # A column the tool has cut clean through keeps its ORIGINAL
            # underside in the engine (PromoteBelow leaves m_bottom alone
            # when nothing lies below). That is the lowest the tool can
            # have exposed a neighbour's face beside it: below it the
            # column never held material, so the neighbour's face there
            # is original. Kept here, before the fold loses it; see
            # `carved` in _VS_IMPL for what the shader does with it.
            carved = np.where(thin & flag & solid, sub_b, np.float32(np.inf))
            sub_h = np.where(thin, np.float32(self._floor_z), sub_h)
            sub_b = np.where(thin, np.float32(self._floor_z), sub_b)
        else:
            carved = None

        if step == 1:
            block = sub_h
            cut = flag
            ever = solid
        else:
            # reduceat rather than reshape, so a ragged final block at the
            # grid edge needs no padding. Minimum, not mean: a pocket then
            # shows at its true depth instead of being averaged back
            # towards the surface.
            #
            # The same minimum means one emptied simulation cell empties
            # its whole display block, so a through-cut reads up to one
            # display block wider than it is. That is the existing rule
            # -- a block straddling the edge of a round stock already
            # drops to the floor the same way -- not a new one.
            s0 = np.arange(0, sx1 - sx0, step)
            s1 = np.arange(0, sy1 - sy0, step)
            block = np.minimum.reduceat(np.minimum.reduceat(sub_h, s0, axis=0), s1, axis=1)
            cut = np.logical_or.reduceat(np.logical_or.reduceat(flag, s0, axis=0), s1, axis=1)
            # AND, to match the MINIMUM above: one cell that was never
            # stock takes the block's surfaces to the floor, so the same
            # cell must be what accounts for them. Reduced the other way
            # the two disagree, and a block straddling the stock's edge
            # reads as a hole through it -- which is the bug this is.
            #
            # The cost is the mirror error, and the quieter one: a cut
            # that breaks out THROUGH the outer wall reads as the wall
            # for up to one display cell either side of where it does.
            ever = np.logical_and.reduceat(np.logical_and.reduceat(solid, s0, axis=0), s1, axis=1)
            if sub_b is not None:
                # Minimum for the underside as well, not maximum. Taking
                # the highest underside against the lowest top would thin
                # every block of a tilted stock from both ends at once,
                # and a stock thinner than a display block would vanish
                # rather than merely read a little thick. Cuts are
                # governed by the tops, which stay conservative.
                sub_b = np.minimum.reduceat(np.minimum.reduceat(sub_b, s0, axis=0), s1, axis=1)
                # The lowest original underside among the block's carved
                # cells: the widest band the tool can have opened.
                carved = np.minimum.reduceat(np.minimum.reduceat(carved, s0, axis=0), s1, axis=1)

        # +1 on both axes: the texture carries a one-texel border.
        tsx = slice(dx0 + 1, dx1 + 2)
        tsy = slice(dy0 + 1, dy1 + 2)

        np.clip(
            (block - self._floor_z) / self._span,
            0.0,
            1.0,
            out=self._h_view[tsx, tsy],
        )
        machined = cut & ever
        target = self._c_view[tsx, tsy]
        target[...] = UNCUT_COLOUR
        # `ever` as well as `cut`: a wall takes its colour from the LOWER
        # of the two texels, which at the silhouette is the empty cell
        # beyond the stock. Coloured from the cut flag alone, a block
        # there straddling the edge painted the stock's own outer wall as
        # a machined face.
        target[machined] = CUT_COLOUR
        self._m_view[tsx, tsy] = np.where(
            ever,
            np.where(cut, np.float32(MASK_CUT), np.float32(MASK_STOCK)),
            np.float32(MASK_OUTSIDE),
        )
        if sub_b is not None:
            np.clip(
                (sub_b - self._floor_z) / self._span,
                0.0,
                1.0,
                out=self._b_view[tsx, tsy],
            )
        if carved is not None and self._has_underside:
            # An empty texel's two surfaces only have to be EQUAL for the
            # shader to read it as empty; the floor was never required.
            # So a machined empty texel carries its original underside in
            # both, and every test that asks whether it holds material
            # still says no. The block is empty whenever any cell in it is
            # thin (the minimum above), so a finite value never lands on a
            # texel that holds material. Only with underside geometry:
            # the split it drives hands the lower part of a face to the
            # bottom walls, which are not built without one.
            keep = np.isfinite(carved) & machined
            if keep.any():
                level = np.clip((carved - self._floor_z) / self._span, 0.0, 1.0)
                h_t = self._h_view[tsx, tsy]
                b_t = self._b_view[tsx, tsy]
                h_t[keep] = level[keep]
                b_t[keep] = level[keep]

        self._h_arr.Modified()
        self._h_img.Modified()
        self._c_arr.Modified()
        self._c_img.Modified()
        self._m_arr.Modified()
        self._m_img.Modified()
        self._b_arr.Modified()
        self._b_img.Modified()
        # The EXPANDED rectangle, the same one the primary reduced over:
        # the layer's blocks have to start on display-cell boundaries or
        # its reduction would straddle them.
        #
        # The primary's underside was passed down here for a while, to
        # clamp each layer under it: per CELL a lower span is under the
        # primary's bottom by construction, and the opposite reductions
        # break that per BLOCK. The clamp was correct about the ordering
        # and wrong to use THAT array, which has been through the fold --
        # so wherever the primary holds less than a cell of material, as
        # it does directly above a side-facing cavity, the ceiling read
        # as the floor and took the whole layer with it. The cavity then
        # kept only a rim of cells, each hanging a full-height wall: the
        # railings, made worse by the thing meant to cure them.
        #
        # Whatever replaces it has to take the underside from BEFORE the
        # fold, and leave a layer unclamped where the primary holds
        # nothing at all, since then there is nothing above it.
        self._refresh_span_layers(sx0, sy0, sx1 - 1, sy1 - 1, dx0, dy0, dx1, dy1)
        self.render_window.Render()

    def _refresh_span_layers(self, x0, y0, x1, y1, dx0, dy0, dx1, dy1):
        """Re-read the spans below the primary over the dirty rectangle
        and upload them.

        Per tick, because cutting through a primary promotes the span
        below it: the layers change exactly where the tool has been, which
        is the rectangle already in hand. It arrives expanded to
        display-cell boundaries, so the reduction below lines up with the
        texels it writes.
        """
        if not self._layer_masks or self._layer_tops is None:
            return
        w = x1 - x0 + 1
        h = y1 - y0 + 1
        if w <= 0 or h <= 0:
            return
        try:
            self._sim.GetSpanLayers(
                x0, y0, x1, y1, len(self._layer_masks), self._layer_tops, self._layer_bottoms
            )
        except Exception:
            return

        tsx = slice(dx0 + 1, dx1 + 2)
        tsy = slice(dy0 + 1, dy1 + 2)
        step = self._step
        for k, (
            (top_arr, top_img, top_view, _),
            (bottom_arr, bottom_img, bottom_view, _),
        ) in enumerate(self._layer_tex):
            top, bottom = self._layer_view(k, w, h)
            present = self._layer_present(top, bottom)
            if step == 1:
                # Pinned to the floor together, exactly as the primary's
                # two surfaces are: a span too thin to draw must read as
                # nothing at all, not as a sliver.
                floor = np.float32(self._floor_z)
                block_t = np.where(present, top, floor)
                block_b = np.where(present, bottom, floor)
            else:
                block_t, block_b = reduce_layer_blocks(top, bottom, present, step, self._floor_z)
            np.clip((block_t - self._floor_z) / self._span, 0.0, 1.0, out=top_view[tsx, tsy])
            np.clip((block_b - self._floor_z) / self._span, 0.0, 1.0, out=bottom_view[tsx, tsy])
            top_arr.Modified()
            top_img.Modified()
            bottom_arr.Modified()
            bottom_img.Modified()

    def _device_pixel_ratio(self):
        for name in ("devicePixelRatioF", "devicePixelRatio"):
            fn = getattr(self.widget, name, None)
            if fn is not None:
                try:
                    ratio = float(fn())
                    if ratio > 0.0:
                        return ratio
                except Exception:
                    continue
        return 1.0

    def sync_render_window_size(self, render=True):
        """Push the widget's current size into the render window.

        Called before every frame, not from a resize event. FreeCAD's own
        OpenGL simulator does exactly this -- its resizeGL is empty and
        paintGL calls updateWindowScale() each frame -- with the comment
        that the devicePixelRatio seen during the first resize can be
        wrong. Relying on resize events left this viewport at
        QVTKRenderWindowInteractor's default 400x300, which is the 4:3
        image that would not go away.
        """
        try:
            ratio = self._device_pixel_ratio()
            w = max(1, int(round(self.widget.width() * ratio)))
            h = max(1, int(round(self.widget.height() * ratio)))
        except Exception:
            return
        if (w, h) == self._last_size:
            return
        self._last_size = (w, h)
        self.render_window.SetSize(w, h)
        iren = self.render_window.GetInteractor()
        if iren is not None:
            iren.SetSize(w, h)
            iren.ConfigureEvent()
        if render:
            self.render_window.Render()

    def set_view(self, name):
        """Standard view directions, matching FreeCAD's numbering."""
        directions = {
            "front": ((0.0, -1.0, 0.0), (0.0, 0.0, 1.0)),
            "rear": ((0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            "top": ((0.0, 0.0, 1.0), (0.0, 1.0, 0.0)),
            "bottom": ((0.0, 0.0, -1.0), (0.0, -1.0, 0.0)),
            "right": ((1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
            "left": ((-1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
            "iso": ((-1.0, -1.0, 1.0), (0.0, 0.0, 1.0)),
        }
        self._stop_camera_turn()
        cam = self.renderer.GetActiveCamera()
        if name != "fit":
            if name not in directions:
                return
            offset, up = directions[name]
            focal = cam.GetFocalPoint()
            dist = cam.GetDistance() or 1.0
            norm = float(np.linalg.norm(offset)) or 1.0
            cam.SetPosition(
                focal[0] + offset[0] / norm * dist,
                focal[1] + offset[1] / norm * dist,
                focal[2] + offset[2] / norm * dist,
            )
            cam.SetViewUp(*up)
        # ResetCamera keeps the direction and re-frames, so this doubles as
        # "fit" and stops a view change leaving the stock off-screen.
        self.renderer.ResetCamera()
        self.renderer.ResetCameraClippingRange()
        self.render_window.Render()

    def set_overlay_text(self, text):
        """Status text drawn in the viewport rather than the control bar.

        The bar had run out of room, and these readouts belong next to
        what they describe anyway. A vtkTextActor rather than a Qt label:
        it scales and moves with the view, and costs nothing per frame.
        """
        if self._overlay is None:
            actor = vtkTextActor()
            actor.GetTextProperty().SetFontSize(15)
            actor.GetTextProperty().SetColor(0.72, 0.75, 0.80)
            actor.GetTextProperty().SetLineSpacing(1.25)
            actor.SetDisplayPosition(12, 10)
            actor.GetPositionCoordinate().SetCoordinateSystemToDisplay()
            self.renderer.AddViewProp(actor)  # AddActor2D is gone in VTK 9.7
            self._overlay = actor
        self._overlay.SetInput(text)

    # The one control over how close the startup view sits. ResetCamera
    # fits every actor, then this scales that: below 1 pulls back, above
    # 1 pushes in.
    #
    # It was 0.92, pulling back to keep the tool in shot -- a tool parked
    # 170 mm above a 27 mm stock is well outside the fit. Above 1 crops
    # that headroom first, so if the tool disappears at the start of a
    # run this is why.
    START_ZOOM = 1.15

    def _to_world(self, point):
        """A point in grid coordinates, in document coordinates."""
        if self._stock_matrix_np is None:
            return point
        p = self._stock_matrix_np @ np.array([point[0], point[1], point[2], 1.0], dtype=np.float64)
        return (float(p[0]), float(p[1]), float(p[2]))

    def reset_camera(self):
        # A turn still running would overwrite this on its next tick.
        self._stop_camera_turn()
        lx, ly = self._length
        ox, oy = self._origin
        cx = ox + lx * 0.5
        cy = oy + ly * 0.5
        top = self._floor_z + self._span
        cam = self.renderer.GetActiveCamera()
        # Computed in grid coordinates, because that is the frame the
        # extents are in, then mapped out through the same matrix the
        # actors carry. Under 3+2 the two frames differ and placing the
        # camera at raw grid coordinates would point it away from the
        # part -- which is only reachable through the view keys, since a
        # reseed keeps the camera rather than re-framing.
        #
        # These offsets set the view DIRECTION and nothing more. The
        # ResetCamera below keeps the direction and recomputes the
        # distance to fit the actors, so scaling them changes the angle
        # and not the framing. START_ZOOM is the only thing that moves
        # the camera in or out.
        cam.SetFocalPoint(*self._to_world((cx, cy, self._floor_z + self._span * 0.35)))
        cam.SetPosition(
            *self._to_world(
                (
                    cx - lx * 0.8,
                    cy - ly * 1.0,
                    top + self._span * 2.4,
                )
            )
        )
        cam.SetViewUp(0, 0, 1)
        self.renderer.ResetCamera()
        cam.Zoom(self.START_ZOOM)
        self.renderer.ResetCameraClippingRange()
        self.render_window.Render()
