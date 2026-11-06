# SPDX-License-Identifier: LGPL-2.1-or-later
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

Draws an operation's flat machining regions in the 3D view while its task
panel is open, switched on and off by a check box on the operation's page.
Any operation can use it.

The operation's proxy provides:

    previewRegions(obj) -> [(shape, role), ...]
        Flat regions in the operation's work plane frame. role is one of:
          "outline"  the area the boundary starts from    (dashed)
          "region"   where the tool centre may go          (solid)
          "keepout"  zones the tool must stay out of       (solid, magenta)

and may provide:

    previewMessages(obj) -> [str, ...]
        Notes on the regions just built (ignored selections, failures),
        shown under the summary. Called right after previewRegions().

and may override, when the defaults do not suit:

    previewDrawHeight(obj) -> float
        Frame Z to draw at. Default: 1 mm above Safe Height.
    previewFramePlacement(obj) -> FreeCAD.Placement
        Frame -> world. Default: the operation's work plane
        (PathUtil.workplaneForOp), so the drawing sits on a 3+2 work plane the
        way obj.Placement places the toolpath.

The page creates a BoundaryPreviewController in initPage() and forwards
updateData() and cleanupPage() to it:

    self.boundaryPreview = BoundaryPreview.BoundaryPreviewController(
        obj, self.form.showBoundaryPreview, PREVIEW_PROPERTIES, "MyOpBoundaryPreview",
        status=self.form.boundaryPreviewStatus,  # optional QLabel
    )
    ...
    def updateData(self, obj, prop):
        self.boundaryPreview.updateData(obj, prop)

    def cleanupPage(self, obj):
        self.boundaryPreview.cleanup()
"""

import FreeCAD
import FreeCADGui
from PySide import QtCore, QtGui

import Path
import Path.Base.Util as PathUtil

translate = FreeCAD.Qt.translate

Path.Log.setLevel(Path.Log.Level.INFO, Path.Log.thisModule())


# ---------------------------------------------------------------------------
# Styles and preferences
# ---------------------------------------------------------------------------

# Defaults avoid the colours CAM and FreeCAD already draw with (feed green,
# rapid red, highlight orange, selection green / preselection yellow). Packed
# 0xRRGGBBAA like CAM's other colours, and overridable in
# User parameter:BaseApp/Preferences/Mod/CAM.
BOUNDARY_COLOR = 0x0099FFFF  # sky blue: outline and region
BOUNDARY_KEEPOUT_COLOR = 0xE61AE6FF  # magenta: keep-out zones

CAM_PARAMS = "User parameter:BaseApp/Preferences/Mod/CAM"

# role -> (colour preference, default colour, line width, dashed)
ROLE_STYLES = {
    "outline": ("DefaultBoundaryColor", BOUNDARY_COLOR, 1.5, True),
    "region": ("DefaultBoundaryColor", BOUNDARY_COLOR, 3.0, False),
    "keepout": ("DefaultBoundaryKeepoutColor", BOUNDARY_KEEPOUT_COLOR, 2.0, False),
}

# Discretization of curved edges for drawing (mm)
DRAW_DEFLECTION = 0.05


def previewPreference(name, value=None):
    """
    Get (or set, when value is given) a preview's on / off choice.

    Kept in the CAM preferences under *name*, so each operation remembers its
    own. Default: off.
    """
    try:
        params = FreeCAD.ParamGet(CAM_PARAMS)
        if value is not None:
            params.SetBool(name, bool(value))
            return bool(value)
        return params.GetBool(name, False)
    except Exception:
        return False if value is None else bool(value)


def prefColor(name, default):
    """(r, g, b) floats from the CAM preference *name*, packed 0xRRGGBBAA."""
    try:
        value = FreeCAD.ParamGet(CAM_PARAMS).GetUnsigned(name, default)
    except Exception:
        value = default
    return tuple(((value >> shift) & 0xFF) / 255.0 for shift in (24, 16, 8))


def regionPolylines(shape, z=0.0, deflection=DRAW_DEFLECTION):
    """
    Polylines for drawing a flat region.

    Args:
        shape (Part.Shape | None): A flat region (faces or a compound).
        z (float): The height to draw at, in the region's frame.
        deflection (float): Discretization of curved edges (mm).

    Returns:
        list: One [(x, y, z), ...] polyline per wire (outer wires and holes).
            Empty for None or a null shape.
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
            lines.append([(p.x, p.y, z) for p in pts])
    return lines


def describeRegion(shape):
    """(areas, holes, area in mm²) of a flat region; zeros for None."""
    if shape is None or shape.isNull():
        return 0, 0, 0.0
    faces = shape.Faces
    holes = sum(len(f.Wires) - 1 for f in faces)
    return len(faces), holes, shape.Area


def summaryLines(regions):
    """
    Short, translated lines describing the drawn regions, by role.

    Args:
        regions (list): (shape, role) tuples, as previewRegions() returns.

    Returns:
        list: Lines of text (empty when there is nothing).
    """
    lines = []
    for shape, role in regions:
        areas, holes, area = describeRegion(shape)
        if role == "region":
            lines.append(
                translate(
                    "CAM_BoundaryPreview", "Tool area: {} region(s), {} hole(s), {:.1f} mm²"
                ).format(areas, holes, area)
            )
        elif role == "keepout":
            lines.append(translate("CAM_BoundaryPreview", "Keep-out: {} zone(s)").format(areas))
    return lines


def defaultDrawHeight(obj):
    """Frame Z for drawing: just above Safe Height, clear of the model and of
    the toolpath, rapids included."""
    for prop in ("SafeHeight", "ClearanceHeight", "StartDepth"):
        quantity = getattr(obj, prop, None)
        if quantity is not None:
            return quantity.Value + 1.0
    return 0.0


# ---------------------------------------------------------------------------
# Drawing
# ---------------------------------------------------------------------------


class BoundaryOverlay:
    """
    Coin overlay drawing flat polylines in the 3D view.

    Drawn on top of the model (SoAnnotation) and unpickable, so it never gets
    in the way of selecting geometry.
    """

    def __init__(self):
        self.root = None
        self.sceneGraph = None

    def show(self, groups, placement=None):
        """
        Replaces the drawing.

        Args:
            groups (list): (polylines, (r, g, b), line_width, dashed) tuples.
                Groups without polylines are skipped.
            placement (FreeCAD.Placement, optional): Frame -> world, applied
                to every polyline.
        """
        self.clear()
        groups = [g for g in groups if g[0]]
        if not groups or FreeCADGui.ActiveDocument is None:
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
        for lines, color, width, dashed in groups:
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
            root.addChild(sep)
        self.sceneGraph.addChild(root)
        self.root = root

    @property
    def isShown(self):
        """True while something is drawn."""
        return self.root is not None

    def clear(self):
        """Removes the drawing, if any."""
        if self.root is not None and self.sceneGraph is not None:
            try:
                self.sceneGraph.removeChild(self.root)
            except Exception as e:
                Path.Log.debug(f"Boundary preview removal failed: {e}")
        self.root = None


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------


class BoundaryPreviewController:
    """
    Runs one operation page's boundary preview.

    Ties a check box to the overlay: remembers the choice, rebuilds the
    drawing shortly after the last change to any of the given properties
    (debounced, as building regions can take a moment on large models), and
    removes it when the page closes.
    """

    def __init__(self, obj, checkbox, properties, preference, delay=300, status=None):
        """
        Args:
            obj: The operation object. Its proxy provides previewRegions()
                (see the module docstring).
            checkbox (QCheckBox): The page's on / off switch.
            properties (iterable): Property names whose change redraws.
            preference (str): CAM preference name remembering the choice.
            delay (int): Milliseconds of quiet before a redraw.
            status (QLabel, optional): Shows a summary of the regions and the
                operation's notes while the preview is on; hidden otherwise.
        """
        self.obj = obj
        self.status = status
        self.checkbox = checkbox
        self.properties = frozenset(properties)
        self.preference = preference
        self.overlay = BoundaryOverlay()

        self.timer = QtCore.QTimer()
        self.timer.setSingleShot(True)
        self.timer.setInterval(delay)
        self.timer.timeout.connect(self.refresh)

        self.checkbox.blockSignals(True)
        self.checkbox.setChecked(previewPreference(preference))
        self.checkbox.blockSignals(False)
        self.checkbox.toggled.connect(self._toggled)
        self.schedule()

    def _toggled(self, checked):
        previewPreference(self.preference, checked)
        self.schedule()

    def schedule(self, *args):
        """Redraws after the delay; a further call restarts it."""
        self.timer.start()

    def updateData(self, obj, prop):
        """Forward of the page's updateData(): redraws on a relevant change."""
        if prop in self.properties:
            self.schedule()

    def cleanup(self):
        """Forward of the page's cleanupPage(): the drawing goes with the page."""
        self.timer.stop()
        self.overlay.clear()

    def _setStatus(self, lines):
        if self.status is None:
            return
        self.status.setText("\n".join(lines))
        self.status.setVisible(bool(lines))

    def refresh(self):
        """Rebuilds the drawing from the operation's previewRegions()."""
        obj = self.obj
        if not self.checkbox.isChecked():
            self.overlay.clear()
            self._setStatus([])
            return

        proxy = obj.Proxy
        notes = []
        QtGui.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try:
            regions = proxy.previewRegions(obj)
            if hasattr(proxy, "previewMessages"):
                notes = list(proxy.previewMessages(obj))
            if hasattr(proxy, "previewDrawHeight"):
                z = proxy.previewDrawHeight(obj)
            else:
                z = defaultDrawHeight(obj)
            if hasattr(proxy, "previewFramePlacement"):
                placement = proxy.previewFramePlacement(obj)
            else:
                placement = PathUtil.workplaneForOp(obj)
        except Exception as e:
            message = translate("CAM_BoundaryPreview", "Boundary preview failed: {}").format(e)
            Path.Log.warning(message)
            regions, notes = [], [message]
            placement = None
        finally:
            QtGui.QApplication.restoreOverrideCursor()

        groups = []
        for shape, role in regions:
            style = ROLE_STYLES.get(role)
            if style is None:
                Path.Log.debug(f"Boundary preview: unknown role {role!r}")
                continue
            pref, default, width, dashed = style
            groups.append((regionPolylines(shape, z), prefColor(pref, default), width, dashed))
        self.overlay.show(groups, placement)
        self._setStatus(summaryLines(regions) + notes)
