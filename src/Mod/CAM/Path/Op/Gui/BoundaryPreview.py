# SPDX-License-Identifier: LGPL-2.1-or-later
# SPDX-FileCopyrightText: 2026 Dimitrios Pana <dimitriospana75@gmail.com>
# SPDX-FileNotice: Part of the FreeCAD project.

################################################################################
#                                                                              #
#   FreeCAD is free software: you can redistribute it and/or modify            #
#   it under the terms of the GNU Lesser General Public License as             #
#   published by the Free Software Foundation, either version 2.1              #
#   of the License, or (at your option) any later version.                     #
#                                                                              #
#   FreeCAD is distributed in the hope that it will be useful,                 #
#   but WITHOUT ANY WARRANTY; without even the implied warranty                #
#   of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                    #
#   See the GNU Lesser General Public License for more details.                #
#                                                                              #
#   You should have received a copy of the GNU Lesser General Public           #
#   License along with FreeCAD. If not, see https://www.gnu.org/licenses       #
#                                                                              #
################################################################################

"""
Boundary preview for operation task panels.

Draws what an operation will work within (its machining regions, keep-out
zones, filled holes) in the 3D view while its task panel is open, switched on
and off by a check box on the operation's page. It starts off each time the
panel opens: building the regions can be heavy on large models. Any operation
can use it.

Operation side
==============

The operation's proxy provides one method:

    previewRegions(obj) -> [(shape, role), ...]

Shapes are in the operation's work plane frame (the frame its toolpath is
computed in, see previewFramePlacement below). role is one of:

    OUTLINE   "outline"   the area the boundary starts from    flat, dashed
    REGION    "region"    where the tool centre may go         flat, solid
    KEEPOUT   "keepout"   zones the tool must stay out of      flat, solid magenta
    FILL      "fill"      faces filled in (capped holes)       in place, transparent

Flat roles take flat regions (any Z) and are drawn as outlines at one height
above the work, so they never hide in the model. FILL shapes are drawn where
they are, shaded and transparent with their outline.

and may provide:

    previewMessages(obj) -> [str, ...]
        Notes on the regions just built (ignored selections, failures),
        shown under the check box. Called right after previewRegions().
    previewDrawHeight(obj) -> float
        Frame Z for the flat roles. Default: 1 mm above Safe Height.
    previewFramePlacement(obj) -> FreeCAD.Placement
        Frame -> world. Default: the operation's work plane
        (PathUtil.workplaneForOp), so the drawing sits on a 3+2 work plane the
        way obj.Placement places the toolpath.

Page side
=========

The page has a QCheckBox (and optionally a QLabel for the notes), creates
a BoundaryPreviewController in initPage() and forwards updateData() and
cleanupPage() to it:

    self.boundaryPreview = BoundaryPreview.BoundaryPreviewController(
        obj,
        self.form.showBoundaryPreview,
        properties=("Base", "BoundaryAdjustment", ...),  # changes that redraw
        status=self.form.boundaryPreviewStatus,          # optional
    )

    def updateData(self, obj, prop):
        self.boundaryPreview.updateData(obj, prop)

    def cleanupPage(self, obj):
        self.boundaryPreview.cleanup()

BoundaryOverlay can also be used on its own, to draw (shape, role) pairs
without a check box or a timer.
"""

from typing import NamedTuple

import FreeCAD
import FreeCADGui
from PySide import QtCore, QtGui

import Path
import Path.Base.Util as PathUtil

translate = FreeCAD.Qt.translate

Path.Log.setLevel(Path.Log.Level.INFO, Path.Log.thisModule())


# ---------------------------------------------------------------------------
# Roles and styles
# ---------------------------------------------------------------------------

OUTLINE = "outline"
REGION = "region"
KEEPOUT = "keepout"
FILL = "fill"

# Defaults avoid the colours CAM and FreeCAD already draw with (feed green,
# rapid red, highlight orange lines, selection green / preselection yellow). Packed
# 0xRRGGBBAA like CAM's other colours, and overridable in
# User parameter:BaseApp/Preferences/Mod/CAM.
BOUNDARY_COLOR = 0x0099FFFF  # sky blue: outline and region
BOUNDARY_KEEPOUT_COLOR = 0xE61AE6FF  # magenta: keep-out zones
BOUNDARY_FILL_COLOR = 0xFFA500FF  # orange, shaded: filled holes

CAM_PARAMS = "User parameter:BaseApp/Preferences/Mod/CAM"

# Discretization for drawing (mm)
DRAW_DEFLECTION = 0.05


class Style(NamedTuple):
    """How one role is drawn."""

    flat: bool  # drawn as outlines at the draw height (else in place, shaded)
    color_preference: str
    color: int  # default, 0xRRGGBBAA
    line_width: float
    dashed: bool
    transparency: float  # shaded faces only (0 opaque .. 1 invisible)


STYLES = {
    OUTLINE: Style(True, "DefaultBoundaryColor", BOUNDARY_COLOR, 1.5, True, 0.0),
    REGION: Style(True, "DefaultBoundaryColor", BOUNDARY_COLOR, 3.0, False, 0.0),
    KEEPOUT: Style(True, "DefaultBoundaryKeepoutColor", BOUNDARY_KEEPOUT_COLOR, 2.0, False, 0.0),
    FILL: Style(False, "DefaultBoundaryFillColor", BOUNDARY_FILL_COLOR, 1.5, False, 0.6),
}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def prefColor(name, default):
    """(r, g, b) floats from the CAM preference *name*, packed 0xRRGGBBAA."""
    try:
        value = FreeCAD.ParamGet(CAM_PARAMS).GetUnsigned(name, default)
    except Exception:
        value = default
    return tuple(((value >> shift) & 0xFF) / 255.0 for shift in (24, 16, 8))


def regionPolylines(shape, z=None, deflection=DRAW_DEFLECTION):
    """
    Polylines along every wire of *shape* (outer wires and holes).

    Args:
        shape (Part.Shape | None): Faces or wires.
        z (float, optional): Draw every point at this height; None keeps
            each point's own Z.
        deflection (float): Discretization of curved edges (mm).

    Returns:
        list: [(x, y, z), ...] polylines; empty for None or a null shape.
    """
    lines = []
    if shape is None or shape.isNull():
        return lines
    for wire in shape.Wires:
        try:
            pts = wire.discretize(Deflection=deflection)
        except Exception:
            continue
        if len(pts) >= 2:
            lines.append([(p.x, p.y, p.z if z is None else z) for p in pts])
    return lines


def regionTriangles(shape, deflection=DRAW_DEFLECTION):
    """
    Triangles covering the faces of *shape*, for shaded drawing.

    Returns:
        tuple: ([(x, y, z), ...], [(i, j, k), ...]); empty lists for None or
            a null shape.
    """
    if shape is None or shape.isNull() or not shape.Faces:
        return [], []
    try:
        points, triangles = shape.tessellate(deflection)
    except Exception as e:
        Path.Log.debug(f"Boundary preview: tessellation failed: {e}")
        return [], []
    return [(p.x, p.y, p.z) for p in points], [tuple(t) for t in triangles]


def defaultDrawHeight(obj):
    """Frame Z for the flat roles: just above Safe Height, clear of the model
    and of the toolpath, rapids included."""
    for prop in ("SafeHeight", "ClearanceHeight", "StartDepth"):
        quantity = getattr(obj, prop, None)
        if quantity is not None:
            return quantity.Value + 1.0
    return 0.0


def defaultFramePlacement(obj):
    """Frame -> world: the operation's work plane (identity without one)."""
    return PathUtil.workplaneForOp(obj)


# ---------------------------------------------------------------------------
# Drawing
# ---------------------------------------------------------------------------


class BoundaryOverlay:
    """
    Draws (shape, role) pairs in the 3D view (see the module docstring).

    Drawn on top of the model (SoAnnotation) and unpickable, so it never gets
    in the way of selecting geometry.
    """

    def __init__(self):
        self.root = None
        self.sceneGraph = None

    @property
    def isShown(self):
        """True while something is drawn."""
        return self.root is not None

    def show(self, regions, height=0.0, placement=None):
        """
        Replaces the drawing.

        Args:
            regions (list): (shape, role) pairs, in the operation's frame. An
                unknown role is skipped (logged at debug level).
            height (float): Frame Z for the flat roles.
            placement (FreeCAD.Placement, optional): Frame -> world, applied
                to everything drawn.
        """
        self.clear()
        if FreeCADGui.ActiveDocument is None:
            return
        nodes = []
        for shape, role in regions:
            style = STYLES.get(role)
            if style is None:
                Path.Log.debug(f"Boundary preview: unknown role {role!r}")
                continue
            nodes.extend(self._nodes(shape, style, height))
        if not nodes:
            return
        try:
            from pivy import coin

            self.sceneGraph = FreeCADGui.ActiveDocument.ActiveView.getSceneGraph()
        except Exception as e:
            Path.Log.debug(f"Boundary preview unavailable: {e}")
            return

        root = coin.SoAnnotation()
        pick = coin.SoPickStyle()
        pick.style = coin.SoPickStyle.UNPICKABLE
        root.addChild(pick)
        if placement is not None and not placement.isIdentity():
            xform = coin.SoTransform()
            base = placement.Base
            xform.translation.setValue(base.x, base.y, base.z)
            xform.rotation.setValue(*placement.Rotation.Q)
            root.addChild(xform)
        for node in nodes:
            root.addChild(node)
        self.sceneGraph.addChild(root)
        self.root = root

    def clear(self):
        """Removes the drawing, if any."""
        if self.root is not None and self.sceneGraph is not None:
            try:
                self.sceneGraph.removeChild(self.root)
            except Exception as e:
                Path.Log.debug(f"Boundary preview removal failed: {e}")
        self.root = None

    # -- coin nodes --

    @staticmethod
    def _lines(lines, color, width, dashed):
        from pivy import coin

        sep = coin.SoSeparator()
        col = coin.SoBaseColor()
        col.rgb = color
        style = coin.SoDrawStyle()
        style.lineWidth = width
        if dashed:
            style.linePattern = 0x0F0F
        coords = coin.SoCoordinate3()
        pts = [p for line in lines for p in line]
        coords.point.setValues(0, len(pts), pts)
        lset = coin.SoLineSet()
        lset.numVertices.setValues(0, len(lines), [len(line) for line in lines])
        for node in (col, style, coords, lset):
            sep.addChild(node)
        return sep

    @staticmethod
    def _faces(points, triangles, color, transparency):
        from pivy import coin

        sep = coin.SoSeparator()
        mat = coin.SoMaterial()
        mat.diffuseColor = color
        mat.transparency = transparency
        hints = coin.SoShapeHints()
        hints.vertexOrdering = coin.SoShapeHints.UNKNOWN_ORDERING  # light both sides
        coords = coin.SoCoordinate3()
        coords.point.setValues(0, len(points), points)
        index = [i for tri in triangles for i in (*tri, -1)]
        faceset = coin.SoIndexedFaceSet()
        faceset.coordIndex.setValues(0, len(index), index)
        for node in (mat, hints, coords, faceset):
            sep.addChild(node)
        return sep

    def _nodes(self, shape, style, height):
        """Coin nodes drawing one shape in one style (may be empty)."""
        color = prefColor(style.color_preference, style.color)
        nodes = []
        if not style.flat:
            points, triangles = regionTriangles(shape)
            if triangles:
                nodes.append(self._faces(points, triangles, color, style.transparency))
        lines = regionPolylines(shape, height if style.flat else None)
        if lines:
            nodes.append(self._lines(lines, color, style.line_width, style.dashed))
        return nodes


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------


class BoundaryPreviewController:
    """
    Runs one operation page's boundary preview.

    Ties a check box to a BoundaryOverlay, unchecked at first: rebuilds the
    drawing shortly after the last change to any of the given properties
    (debounced, as building regions can take a moment on large models), shows
    the operation's notes, and removes the drawing when the page closes.
    """

    def __init__(self, obj, checkbox, properties, *, status=None, delay=300):
        """
        Args:
            obj: The operation object. Its proxy provides previewRegions()
                (see the module docstring).
            checkbox (QCheckBox): The page's on / off switch.
            properties (iterable): Property names whose change redraws.
            status (QLabel, optional): Shows the operation's notes while the
                preview is on; hidden when there are none.
            delay (int): Milliseconds of quiet before a redraw.
        """
        self.obj = obj
        self.checkbox = checkbox
        self.properties = frozenset(properties)
        self.status = status
        self.overlay = BoundaryOverlay()

        self.timer = QtCore.QTimer()
        self.timer.setSingleShot(True)
        self.timer.setInterval(delay)
        self.timer.timeout.connect(self.refresh)

        self.checkbox.blockSignals(True)
        self.checkbox.setChecked(False)
        self.checkbox.blockSignals(False)
        self.checkbox.toggled.connect(self.schedule)

    # -- page forwards --

    def updateData(self, obj, prop):
        """Forward of the page's updateData(): redraws on a relevant change."""
        if prop in self.properties:
            self.schedule()

    def cleanup(self):
        """Forward of the page's cleanupPage(): the drawing goes with the page."""
        self.timer.stop()
        self.overlay.clear()

    # -- drawing --

    def schedule(self, *args):
        """Redraws after the delay; a further call restarts it."""
        self.timer.start()

    def refresh(self):
        """Rebuilds the drawing from the operation's previewRegions()."""
        if not self.checkbox.isChecked():
            self.overlay.clear()
            self._setStatus([])
            return

        obj = self.obj
        proxy = obj.Proxy
        notes = []
        QtGui.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try:
            regions = list(proxy.previewRegions(obj))
            if hasattr(proxy, "previewMessages"):
                notes = list(proxy.previewMessages(obj))
            if hasattr(proxy, "previewDrawHeight"):
                height = proxy.previewDrawHeight(obj)
            else:
                height = defaultDrawHeight(obj)
            if hasattr(proxy, "previewFramePlacement"):
                placement = proxy.previewFramePlacement(obj)
            else:
                placement = defaultFramePlacement(obj)
        except Exception as e:
            message = translate("CAM_BoundaryPreview", "Boundary preview failed: {}").format(e)
            Path.Log.warning(message)
            regions, notes, height, placement = [], [message], 0.0, None
        finally:
            QtGui.QApplication.restoreOverrideCursor()

        self.overlay.show(regions, height, placement)
        self._setStatus(notes)

    def _setStatus(self, lines):
        if self.status is None:
            return
        self.status.setText("\n".join(lines))
        self.status.setVisible(bool(lines))
