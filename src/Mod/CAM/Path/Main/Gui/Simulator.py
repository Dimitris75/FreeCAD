# SPDX-License-Identifier: LGPL-2.1-or-later

# ***************************************************************************
# *   Copyright (c) 2017 Shai Seger <shaise at gmail>                       *
# *                                                                         *
# *   This program is free software; you can redistribute it and/or modify  *
# *   it under the terms of the GNU Lesser General Public License (LGPL)    *
# *   as published by the Free Software Foundation; either version 2 of     *
# *   the License, or (at your option) any later version.                   *
# *   for detail see the LICENCE text file.                                 *
# *                                                                         *
# *   This program is distributed in the hope that it will be useful,       *
# *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
# *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
# *   GNU Library General Public License for more details.                  *
# *                                                                         *
# *   You should have received a copy of the GNU Library General Public     *
# *   License along with this program; if not, write to the Free Software   *
# *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  *
# *   USA                                                                   *
# *                                                                         *
# ***************************************************************************

import FreeCAD
import Path
import Path.Base.Util as PathUtil
import Path.Dressup.Utils as PathDressup
import PathScripts.PathUtils as PathUtils
import PathSimulator
import math
import weakref

from FreeCAD import Vector

# lazily loaded modules
from lazy_loader.lazy_loader import LazyLoader

Mesh = LazyLoader("Mesh", globals(), "Mesh")
Part = LazyLoader("Part", globals(), "Part")

from . import SimulatorVtkView
from . import SimulatorWindow

if FreeCAD.GuiUp:
    import FreeCADGui
    from PySide import QtGui, QtCore


class CAMSimTaskUi:
    def __init__(self, parent):
        # this will create a Qt widget from our ui file
        self.form = FreeCADGui.PySideUic.loadUi(":/panels/TaskPathSimulator.ui")
        self.parent = parent

    def accept(self):
        self.parent.accept()
        FreeCADGui.Control.closeDialog()

    def reject(self):
        self.parent.cancel()
        FreeCADGui.Control.closeDialog()


# Last job simulated, remembered per document across restarts.
#
# A fresh PathSimulation is built for every run, so this cannot live on
# the instance; and a module-level dict would only last the session, which
# is the case that actually matters -- opening FreeCAD and running the
# simulator on a multi-job document lands on the first job every time.
_LAST_JOB_GROUP = "User parameter:BaseApp/Preferences/Mod/CAM/SimulatorLastJob"


def _rememberedJob(docName):
    try:
        return FreeCAD.ParamGet(_LAST_JOB_GROUP).GetString(docName, "")
    except Exception:
        return ""


def _rememberJob(docName, jobName):
    try:
        FreeCAD.ParamGet(_LAST_JOB_GROUP).SetString(docName, jobName)
    except Exception:
        pass


# Colour of the mesh put into the document by "To document" and OK:
# the viewport's cut colour, so the exported result reads as the thing
# that was simulated, and stands apart from the model and FreeCAD's
# default grey. RGB, 0 to 1.
#
# One colour for the whole mesh. Painting machined faces apart from
# untouched stock works -- a Mesh::Feature takes a FaceColors list, the
# way FreeCAD's own importer colours an imported OBJ -- but the engine
# cannot supply the split: a workplane change re-seeds the grid, and
# every face already cut then reads as the stock's own surface. It would
# have been two honest colours in 3-axis work and one everywhere else,
# which is worse than one colour everywhere. Fixing it properly means
# carrying a machined flag through the transfer, in C++.
RESULT_COLOUR = (214 / 255.0, 118 / 255.0, 46 / 255.0)

# The finest the Accuracy slider goes, as a percentage of the stock's
# larger side, while any ticked operation uses a rotated workplane. A
# rotated operation re-seeds a grid sized by the rotated extent, up to the
# stock's diagonal squared, and a workplane change holds two grids at
# once. 3-axis jobs keep the slider's full range (0.025%).
FINEST_ACCURACY_3PLUS2 = 0.05


_AXIS_NAMES = (
    ((1, 0, 0), "+X"),
    ((-1, 0, 0), "-X"),
    ((0, 1, 0), "+Y"),
    ((0, -1, 0), "-Y"),
    ((0, 0, 1), "+Z"),
    ((0, 0, -1), "-Z"),
)


def _describeWorkplane(wp):
    """A workplane for people: "Workplane +X, tilted 90 deg from +Z".

    Named when it is an axis, else its direction to two places. The tilt
    is the angle a 3+2 machine actually turns through, whatever the
    direction around the vertical.
    """
    length = math.sqrt(wp.x * wp.x + wp.y * wp.y + wp.z * wp.z)
    if length < 1e-12:
        return "Workplane undefined"
    v = (wp.x / length, wp.y / length, wp.z / length)
    name = None
    for axis, label in _AXIS_NAMES:
        if all(abs(a - b) < 1e-6 for a, b in zip(v, axis)):
            name = label
            break
    if name is None:
        # + 0.0 turns -0.0 into 0.0, which would otherwise print as "-0.00".
        name = "({:.2f}, {:.2f}, {:.2f})".format(*(round(c, 2) + 0.0 for c in v))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, v[2]))))
    if tilt < 0.05:
        return "Workplane {}".format(name)
    return "Workplane {}, tilted {:.0f}\u00b0 from +Z".format(name, tilt)


# Arc endpoints closer than this are the same point. In G-code that makes
# the arc a full circle, not an empty one.
_ARC_CLOSED_TOL = 1e-6


def _arcPoints(sx, sy, sz, ex, ey, ez, i, j, ccw, resolution):
    """Points along an XY-plane arc, as straight segments, ending exactly
    at (ex, ey, ez). Z moves linearly with the angle, which is a helix.

    Segment count is ceil(|sweep| * sqrt(radius / resolution)), unchanged
    from the inline version this replaces, with two cases it crashed on:

    - Coincident endpoints are a full circle. The angle difference is
      zero, so the count was zero and the next line divided by it.
    - A zero radius gave a zero count the same way. Now one segment.

    The last point is the programmed endpoint rather than the last point
    computed on the circle: the machine finishes where the program says,
    even when I and J are a little off.
    """
    cx, cy = sx + i, sy + j
    r = math.hypot(i, j)
    a0 = math.atan2(sy - cy, sx - cx)
    if math.hypot(ex - sx, ey - sy) < _ARC_CLOSED_TOL:
        da = 2.0 * math.pi if ccw else -2.0 * math.pi
    else:
        da = math.atan2(ey - cy, ex - cx) - a0
        if ccw:
            da = da % (2.0 * math.pi)
        else:
            da = -((-da) % (2.0 * math.pi))
    n = max(1, int(Path.Geom.ceil(abs(da) * math.sqrt(r / resolution))))
    points = []
    for k in range(1, n):
        a = a0 + da * k / n
        points.append((cx + r * math.cos(a), cy + r * math.sin(a), sz + (ez - sz) * k / n))
    points.append((ex, ey, ez))
    return points


# The running simulation, for the Python console:
#
#     from Path.Main.Gui import Simulator
#     sim = Simulator.current()
#
# A weak reference, so a closed simulation's grid -- hundreds of MB at a
# fine accuracy -- is not kept alive until the next one opens. It used to
# be a local in CommandPathSimulate.Activated, so the console handle the
# comments pointed at did not exist.
_current = None


def current():
    """The live PathSimulation, or None when none is open."""
    return _current() if _current is not None else None


def _cellsFor(length, resolution):
    """Grid cells along an axis, as cStock sizes it: exactly enough to
    cover the length, the last one clipped to the edge."""
    return max(1, int(math.ceil(length / resolution - 1e-4)))


def TSError(msg):
    QtGui.QMessageBox.information(None, "Path Simulation", msg)


class PathSimulation:
    def __init__(self):
        self.timer = QtCore.QTimer()
        QtCore.QObject.connect(self.timer, QtCore.SIGNAL("timeout()"), self.PerformCut)
        self.stdrot = FreeCAD.Rotation(Vector(0, 0, 1), 0)
        self.iprogress = 0
        self.numCommands = 0
        self.accuracy = 0.1
        self.resetSimulation = False
        self.jobs = []
        # Commands are batched on a fixed tick. Not for redraw cost --
        # that is ~0.6 ms -- but to keep the tick rate predictable. The
        # interval is the ceiling on how finely the animation can be
        # paced.
        self.tickInterval = 8  # ms, the fastest the timer will fire
        self.cmdsPerTick = 1
        # Speed is G-code MOVES PER SECOND: independent of how long the
        # operation is, and the thing the user is actually choosing. A
        # rate per operation would run a 200-move facing pass and a
        # 60,000-move adaptive path at wildly different speeds from the
        # same slider position.
        #
        # Geometric across the slider: each step is a constant ratio,
        # which is how speed reads to the eye.
        self.minMoveRate = 5.0
        self.maxMoveRate = 200000.0
        # Fast-forward ignores the speed slider: it exists to get to the
        # end, not to be watched. Commands are still processed in batches
        # rather than one long loop, so the window stays responsive and
        # Stop still works.
        self.fastForward = False
        self.ffBatch = 5000
        # accuracy = (1.1 - 0.1 * slider) / accuracyResolutionFactor, so
        # this sets the finest accuracy the slider can reach: 0.1 / 4.0 =
        # 0.025. Grid cells go as the square of this, and the display
        # geometry is currently built per simulation cell, so a 100 mm
        # stock at 0.025 is a 4000x4000 grid -- around 13 GB of geometry.
        self.accuracyResolutionFactor = 4.0
        # VTK viewport. When active, the per-tick path uploads only the
        # cells this tick touched rather than rebuilding two Mesh
        # objects and the Coin3D scene graph: ~0.6 ms/tick against ~96 ms
        # on a 2.5M point stock, and it does not scale with stock size.
        self.vtkView = None
        # The buffers VtkStockView wraps live inside cStock and do not
        # survive anything that recreates it, so every reseed must
        # re-attach. Guarded rather than left to discipline.
        self._vtkAttachedGeneration = -1
        self._stockGeneration = 0
        # SetupSimulation runs once when the dialog opens and again on the
        # first tick after Play, because nothing watches the operation
        # checkboxes and that second run is how a change to them is picked
        # up. Re-seeding the grid and rebuilding the viewport geometry a
        # second time is the pause between pressing Play and the first
        # cut, so the heavy part is skipped when nothing relevant changed
        # and the stock is still untouched.
        self._lastSetupSignature = None
        self._stockDirty = False
        # Set properly in SetupSimulation / SimulateMill. Initialised here
        # so that a failure before those run surfaces as the actual error,
        # rather than a cascade of AttributeErrors from the toolbar
        # buttons -- which stay clickable after a failed Activate.
        self.activeOps = []
        self.operations = []
        self.jobs = []
        self.ioperation = 0
        self.tool = None
        # The current operation's tool shape, as handed to SetToolShape and
        # drawn in the viewport. A plain shape, not a document object: the
        # old hidden "CutTool" Part feature was only ever a carrier for it,
        # and it sat in the document for the whole run -- in the tree, in
        # File > Export, and left behind for good whenever the run did not
        # end through RemoveTool.
        self.toolShape = None
        # The Accuracy slider's own maximum, before a 3+2 job trims it.
        # Read from the slider on first use: the window does not exist yet.
        self._accuracySliderTop = None
        self.taskForm = None
        # The simulation window owns every widget; these are reached
        # through properties below so the logic reads the same as when
        # they were attributes here.
        self.simWindow = None
        # Compound of the job's model shapes, drawn inside the stock so
        # gouges show through. None when the job has no model.
        self.baseShape = None
        # Whether the user asked for the model. Kept here because a reseed
        # discards the tessellated copy and it has to come back.
        self.showBaseShape = False
        # Off by default: a reference overlay, useful when the
        # heightmap's silhouette is misleading and clutter the rest of
        # the time.
        self.showStockShell = False
        self.showUncut = True
        # Guards re-entry: CloseVtkView closes the window, which fires
        # closeEvent, which would call back into CloseVtkView.
        self._vtkClosing = False
        # --- 3+2 ------------------------------------------------------
        # The working frame the grid is currently built in. A job starts
        # unrotated, and identity here makes the first workplane change a
        # straight rotation rather than a rotate-then-un-rotate pair.
        # Set properly in SetupSimulation; initialised here so a failure
        # before that surfaces as itself rather than an AttributeError.
        self.currentWorkplane = Vector(0, 0, 1)
        self.currentTransformMatrix = FreeCAD.Matrix()
        # Top of the stock in the CURRENT working frame, which is where
        # the tool starts each operation. Not the Job stock's ZMax once a
        # rotation has been applied.
        self.frameTopZ = None
        # Cell size, and the tessellation tolerance derived from it.
        # SetupSimulation computes the real value. The default matters
        # because onJobChange can draw the model and the stock shell
        # before any simulation is set up.
        self.resolution = 0.0
        # One warning per run when the module is too old to reseed,
        # rather than one per operation.
        self._reseedUnavailableReported = False
        # One warning per run when the module can re-seed only through
        # the mesh, which fills every cavity the new frame cannot see.
        self._transferUnavailableReported = False
        # One warning per run when the module predates MaterialVolume().
        self._volumeFallbackReported = False
        # Engine bytes per grid cell, for the Accuracy label. See
        # _bytesPerCell.
        self._cellBytes = None
        # One warning per run when the module predates GetBottomBuffer(),
        # and the viewport therefore cannot show a through-cut in a
        # tilted frame as empty.
        self._bottomsUnavailableReported = False

    # Widgets live in the simulation window. Forwarding rather than
    # reaching through self.simWindow at every use keeps the simulation
    # logic readable and leaves one place to change if the window is
    # restructured again.
    @property
    def comboJobs(self):
        return self.simWindow.comboJobs

    @property
    def listOperations(self):
        return self.simWindow.listOperations

    @property
    def sliderAccuracy(self):
        return self.simWindow.sliderAccuracy

    @property
    def sliderSpeed(self):
        return self.simWindow.sliderSpeed

    def UpdateProgress(self):
        if self.numCommands <= 0:
            return
        percent = self.iprogress * 100 / self.numCommands
        bar = getattr(self.taskForm.form, "progressBar", None)
        if bar is not None:
            bar.setValue(int(percent))
        if self.simWindow is not None:
            self.simWindow.setProgress(percent)

    def Activate(self):
        self.initdone = False
        self.taskForm = CAMSimTaskUi(self)

        # The window has to exist first: every control now lives in it, so
        # populating the job list or reading the accuracy slider before it
        # is built would touch widgets that do not yet exist.
        self._setupVtkView()

        self.sliderSpeed.valueChanged.connect(self.onSpeedBarChange)
        self.onSpeedBarChange()
        self.sliderAccuracy.valueChanged.connect(self.onAccuracyBarChange)
        midAccuracy = (self.sliderAccuracy.minimum() + self.sliderAccuracy.maximum()) // 2
        self.sliderAccuracy.setValue(midAccuracy)
        self.onAccuracyBarChange()
        self.simWindow.followsVisibility.setChecked(self.followsVisibilityPref())
        self._populateJobSelection()
        self.comboJobs.currentIndexChanged.connect(self.onJobChange)
        self.onJobChange()

        FreeCADGui.Control.showDialog(self.taskForm)
        self.disableAnim = False
        self.firstDrill = True
        self.voxSim = PathSimulator.PathSim()
        self.SimulateMill()
        self.initdone = True

    def _setPanelStatus(self, text):
        """The task panel's status line.

        The panel keeps only a progress bar and this label: every control
        lives in the simulation window. It is not empty on purpose --
        anyone embedding the viewport in FreeCAD's own window will want
        somewhere to report, and the dialog would otherwise be a bare OK
        button.
        """
        label = getattr(self.taskForm.form, "labelStatus", None)
        if label is not None:
            label.setText(text)

    def _preferredJobName(self, jobList):
        """Which job to open on.

        In order: whatever the user has selected in the tree, then the job
        last simulated in this document, then the only job if there is
        one, then the first.

        The selection test uses findParentJob, so an operation, a tool
        controller or the stock all count. Matching the Job object alone
        lands on the wrong job in a document with several.
        """
        names = [j.Name for j in jobList]

        selection = FreeCADGui.Selection.getSelectionEx()
        selInfo = "nothing selected"
        if selection:
            selObj = selection[0].Object
            try:
                job = PathUtils.findParentJob(selObj)
            except Exception as e:
                job = None
                selInfo = "findParentJob({}) raised {}".format(getattr(selObj, "Name", "?"), e)
            else:
                selInfo = "selected {} -> job {}".format(
                    getattr(selObj, "Name", "?"),
                    getattr(job, "Name", None),
                )
            # Compared by Name, not identity: findObjects and
            # findParentJob need not hand back the same Python wrapper for
            # the same document object, so `job in jobList` can be False
            # for a job that is plainly in the list.
            if job is not None and getattr(job, "Name", None) in names:
                FreeCADGui.Selection.clearSelection()
                Path.Log.debug("CAM simulation job choice: {} (from selection)\n".format(job.Name))
                return job.Name

        remembered = _rememberedJob(FreeCAD.ActiveDocument.Name)
        if remembered and remembered in names:
            Path.Log.debug("CAM simulation job choice: {} (last used)\n".format(remembered))
            return remembered

        chosen = names[0] if names else ""
        Path.Log.debug(
            "CAM simulation job choice: {} (first of {}); {}; "
            "remembered={}\n".format(chosen, names, selInfo, remembered)
        )
        return chosen

    def _populateJobSelection(self):
        jobList = FreeCAD.ActiveDocument.findObjects("Path::FeaturePython", "Job.*")
        if not jobList:
            return
        wanted = self._preferredJobName(jobList)

        # self.jobs must be cleared with the combo. The combo is rebuilt
        # from scratch while the list was only appended to, so any second
        # call left the two out of step and comboJobs.currentIndex() then
        # selected the wrong entry from self.jobs -- seeding the simulator
        # from a different job than the one shown.
        self.jobs = []
        self.comboJobs.blockSignals(True)
        self.comboJobs.clear()
        setJobIdx = 0
        for i, j in enumerate(jobList):
            self.comboJobs.addItem(j.ViewObject.Icon, j.Label)
            self.jobs.append(j)
            if j.Name == wanted:
                setJobIdx = i
        self.comboJobs.setCurrentIndex(setJobIdx)
        self.comboJobs.blockSignals(False)

    def _setupVtkView(self):
        """Create the simulation window. It owns the viewport and every
        control; this class owns the simulation and reads them through
        it."""
        self.simWindow = SimulatorWindow.SimulationWindow(self)
        self.simWindow.start()
        self.vtkView = self.simWindow.view

    def _updateSpeedLabel(self):
        if self.simWindow is None:
            return
        rate = self.TargetMoveRate()
        if rate >= 10000:
            text = "{:,.0f} moves/s".format(round(rate, -3))
        elif rate >= 100:
            text = "{:,.0f} moves/s".format(rate)
        else:
            text = "{:.0f} moves/s".format(rate)
        self.simWindow.setSpeedLabel(text)

    def _updateWindowStatus(self):
        if self.vtkView is None or self.simWindow is None:
            return
        # No percentage here: the progress bar beside it already shows it,
        # and the two were reading a point apart because they update at
        # different moments.
        #
        # ioperation increments as each operation finishes, so after the
        # last one it points one past the list -- "op 2 of 1". Clamped:
        # the run ends showing the operation it ended on.
        total = max(1, len(self.activeOps))
        current = min(self.ioperation + 1, total)
        status = "op {} of {}".format(current, total)
        self.simWindow.setStatus(status)
        name = self.operation.Label if self.operation is not None else ""
        self._setPanelStatus("{}  {}".format(status, name).strip())

    def onSimulationWindowClosed(self):
        """The window's own close button was used.

        Everything must stop here. The task dialog remains open otherwise,
        with a timer still ticking against a widget Qt has destroyed --
        which is why pressing anything afterwards crashed FreeCAD rather
        than raising. Closing from the task panel worked only because that
        path tears down in the right order.
        """
        if self._vtkClosing:
            return
        self._vtkClosing = True
        try:
            self.timer.stop()
        except Exception:
            pass
        try:
            self.EndSimulation()
        except Exception:
            pass
        # Drop the view first: nothing may render after this point. The
        # GL teardown has to happen here as well -- closing by the window
        # button reaches this and not CloseVtkView, and leaving VTK to the
        # collector is what segfaults.
        self.vtkView = None
        self._vtkAttachedGeneration = -1
        if self.simWindow is not None:
            try:
                self.simWindow.releaseView()
            except Exception:
                pass
        try:
            FreeCADGui.Control.closeDialog()
        except Exception:
            pass

    def _operationBanner(self, wp):
        """Two lines for the viewport: the operation about to run, and
        the workplane the part is turning to."""
        total = max(1, len(self.activeOps))
        return "Op {} of {}: {}\n{}".format(
            min(self.ioperation + 1, total), total, self.operation.Label, _describeWorkplane(wp)
        )

    def _vtkCameraTurning(self):
        """Whether the viewport is still playing a camera turn."""
        return bool(getattr(self.vtkView, "camera_turning", False))

    def _vtkTurnCamera(self, newMatrix, oldMatrix):
        """Turn the viewport's camera from one working frame to another.

        Both matrices map document -> working frame. The camera keeps its
        pose relative to the working frame, which in document coordinates
        (where the viewport draws) is a turn by new^-1 * old about the
        focal point. Animated, except under fast-forward, which draws
        nothing until the end anyway.

        A viewport predating turn_camera simply keeps the camera still,
        as before; nothing is lost but the turn.
        """
        turn = getattr(self.vtkView, "turn_camera", None)
        if turn is None:
            return
        try:
            turn(
                newMatrix.inverse().multiply(oldMatrix),
                animate=not getattr(self, "disableAnim", False),
            )
        except Exception as e:
            # A camera is not worth stopping a run for.
            Path.Log.debug("CAM simulation: could not turn the camera: {}\n".format(e))

    def _vtkAttach(self, keep_camera=False):
        """Bind the viewport to the current stock buffers. Must be called
        after BeginSimulation and after every reseed -- the memoryviews
        point into cStock and are invalidated when it is recreated.

        `keep_camera` is for a 3+2 reseed: the grid is rebuilt in a new
        working frame, so its extents change, but the viewport puts it
        back in document coordinates and the part does not move on
        screen. Letting the view's own guard decide would re-frame on
        every workplane change.
        """
        if self.vtkView is None:
            return
        self.vtkView.attach(self.voxSim, keep_camera=keep_camera)
        self._vtkAttachedGeneration = self._stockGeneration

        if not getattr(self.vtkView, "has_bottoms", True):
            # A warning, not debug: the picture is then wrong in exactly
            # the case 3+2 exists for, and wrong without any error.
            if not self._bottomsUnavailableReported:
                self._bottomsUnavailableReported = True
                Path.Log.warning(
                    "CAM simulation: this build of the simulator module has "
                    "no GetBottomBuffer(), so the viewport cannot see the "
                    "stock's underside. A cut through the part in a rotated "
                    "workplane will still be drawn as material; the "
                    "exported mesh and volumes are unaffected.\n"
                )

        # A translucent shell of the original stock. Never modified, so
        # there is no boundary to reconcile with the heightmap -- and it
        # shows where material used to be, not only what is left.
        try:
            self.vtkView.set_stock_shell(self.stock, max(self.resolution, 0.01))
        except Exception as e:
            Path.Log.warning(f"CAM simulation: could not draw the stock shell ({e})\n")
        self.vtkView.show_uncut_material(self.showUncut)
        self.vtkView.show_stock_shell(self.showStockShell)

    def _vtkRefresh(self):
        """One tick's redraw. Silently does nothing once the view is gone
        -- a tick can still be in flight when the window closes."""
        if self.vtkView is None:
            return
        if self._vtkAttachedGeneration != self._stockGeneration:
            # Stale buffers: reading them would touch freed memory.
            self._vtkAttach()
        self.vtkView.refresh()

    def onShowModelToggled(self, checked):
        """Model button."""
        self.showBaseShape = bool(checked)
        self._applyBaseShape()

    def onShowUncutToggled(self, checked):
        """Uncut button: material the simulation has not touched."""
        self.showUncut = bool(checked)
        if self.vtkView is not None:
            self.vtkView.show_uncut_material(self.showUncut)
            self.vtkView.render_window.Render()

    def onShowStockToggled(self, checked):
        """Stock button: the translucent original-stock shell."""
        self.showStockShell = bool(checked)
        if self.vtkView is not None:
            self.vtkView.show_stock_shell(self.showStockShell)
            self.vtkView.render_window.Render()

    def _applyBaseShape(self):
        """Make the view match showBaseShape.

        Called again after every reseed: BeginSimulation discards the
        tessellated copy because the resolution may have changed, and
        without this the model silently vanished the moment Play was
        pressed while the button still looked switched on.

        Tessellated on first use rather than at job selection -- most runs
        never turn it on, and tessellating a complex model costs real time.
        """
        if self.vtkView is None:
            return
        if self.showBaseShape and not self.vtkView.has_base_shape():
            if self.baseShape is None:
                return
            # Grid resolution is the meaningful limit: detail finer than a
            # cell cannot be compared against the simulated surface anyway.
            self.vtkView.set_base_shape(self.baseShape, max(self.resolution, 0.01))
        self.vtkView.show_base_shape(self.showBaseShape)
        self.vtkView.render_window.Render()

    def _vtkSetTool(self, shape):
        """Hand the tool geometry to the VTK viewport.

        The shape is tessellated in its OWN local frame. obj.Shape on a
        Part::FeaturePython comes back with the object's Placement already
        applied, so tessellating it directly bakes in whatever position the
        tool happened to be at, and set_tool_placement() then applies
        curpos on top of that -- the tool ends up offset by its previous
        position. Coin3D never hit this because it applies Placement to the
        raw shape once.

        The tolerance is for DISPLAY only and must not follow the
        simulation accuracy. Passing 0.01 * self.accuracy meant 0.00025 mm
        deflection on a tool a few millimetres across -- millions of
        triangles, measured at 117 seconds. Scale it to the tool instead,
        with a floor so a tiny tool cannot drive it back down.
        """
        if self.vtkView is None:
            return
        localShape = shape.copy()
        localShape.Placement = FreeCAD.Placement()
        try:
            diag = localShape.BoundBox.DiagonalLength
        except Exception:
            diag = 10.0
        tolerance = max(0.02, diag / 300.0)
        self.vtkView.set_tool(localShape, tolerance)
        self.vtkView.show_tool(True)

    def _setupSignature(self):
        """Everything that would change the seeded stock. If this matches
        the last seed and nothing has been cut since, the existing grid is
        still exactly what a fresh seed would produce."""
        ops = tuple(
            self.listOperations.item(i).text()
            for i in range(self.listOperations.count())
            if self.listOperations.item(i).checkState() == QtCore.Qt.CheckState.Checked
        )
        bb = self.job.Stock.Shape.BoundBox
        return (
            self.job.Name,
            ops,
            round(self.accuracy, 9),
            round(bb.XMin, 6),
            round(bb.YMin, 6),
            round(bb.ZMin, 6),
            round(bb.XLength, 6),
            round(bb.YLength, 6),
            round(bb.ZLength, 6),
        )

    def SetupSimulation(self):
        self.activeOps = []
        self.numCommands = 0
        self.ioperation = 0
        for i in range(self.listOperations.count()):
            if self.listOperations.item(i).checkState() == QtCore.Qt.CheckState.Checked:
                self.firstDrill = True
                self.activeOps.append(self.operations[i])
                self.numCommands += len(self.operations[i].Path.Commands)

        self.stock = self.job.Stock.Shape
        # Cell size is fixed for the whole job, from the Job stock in
        # DOCUMENT coordinates, and every re-seed reuses it.
        #
        # This is a decision, not an oversight. Accuracy is a physical
        # tolerance: the size of the smallest feature the grid can hold.
        # Re-deriving it from each rotated bounding box would keep the
        # cell COUNT roughly constant and let the cell SIZE drift between
        # orientations, so the same Accuracy setting would mean a
        # different tolerance for each operation, and every re-seed would
        # resample onto a different pitch and compound the staircase
        # error it already carries. Fixing the size and letting the count
        # follow the rotated extent is the other way round, and it is the
        # one that keeps Accuracy meaning something.
        #
        # The cost is that a rotated grid can hold more cells than the
        # unrotated one -- bounded by the stock's diagonal, so at worst
        # about (diagonal / max(X, Y)) squared. _updateGridEstimate shows
        # that bound when the job has a rotated operation, and each
        # re-seed logs the grid it actually built.
        maxlen = self.stock.BoundBox.XLength
        if maxlen < self.stock.BoundBox.YLength:
            maxlen = self.stock.BoundBox.YLength
        self.resolution = 0.01 * self.accuracy * maxlen

        signature = self._setupSignature()
        reseed = self._stockDirty or signature != self._lastSetupSignature

        if reseed:
            # The grid about to be seeded is the Job's stock in document
            # coordinates, before any operation's workplane. The frame
            # state is reset with the seed, not before it: when the seed
            # is skipped the existing grid may already be in the first
            # operation's working frame, and saying otherwise would make
            # the next workplane comparison re-rotate a grid that is
            # already rotated -- twice, since the first rotation would
            # then be composed on top of itself.
            previousMatrix = self.currentTransformMatrix
            self.currentWorkplane = Vector(0, 0, 1)
            self.currentTransformMatrix = FreeCAD.Matrix()
            self.frameTopZ = self.stock.BoundBox.ZMax
            self.initialPos = Vector(0, 0, self.frameTopZ)
            if self.vtkView is not None:
                self.vtkView.set_stock_matrix(None)
                # The camera was turned with the frame on the way, so it
                # turns back with it -- otherwise a replay after Stop
                # starts with the view tilted against a part that is back
                # in its own frame. Before the attach: if the attach
                # re-frames (a different stock), its reset cancels this.
                self._vtkTurnCamera(self.currentTransformMatrix, previousMatrix)

            self.voxSim.BeginSimulation(self.stock, self.resolution)
            self._stockGeneration += 1
            self._vtkAttach()
            self._lastSetupSignature = signature
            self._stockDirty = False
            # Resolution may have changed, so the tessellated model is
            # stale. Dropped and then restored, rather than left dangling.
            if self.vtkView is not None:
                self.vtkView.clear_base_shape()
                self._applyBaseShape()
        self.busy = False
        self.tool = None
        # Start at the first operation that has a tool. This called
        # SetupOperation(0) on every pass, so a toolless first operation
        # was retried N times and never skipped. ioperation has to follow,
        # since PerformCut resumes from it; and the skipped operations'
        # commands are counted as done, because numCommands includes them
        # and the bar would otherwise stop short of the end.
        self.iprogress = 0
        for i in range(len(self.activeOps)):
            self.ioperation = i
            self.SetupOperation(i)
            if self.tool is not None:
                break
            self.iprogress += len(self.activeOps[i].Path.Commands)
        self.UpdateProgress()

    def _operationWorkplane(self):
        """The operation's workplane normal, or +Z when it has none.

        Operations predating 3+2, and every operation in a 3-axis job,
        have no Workplane property at all.
        """
        wp = getattr(self.operation, "Workplane", None)
        if wp is None:
            return Vector(0, 0, 1)
        return wp

    def _anyRotatedOperation(self):
        """Whether any checked operation asks for a workplane off +Z.

        Read straight off the property, with no rotation solve: this runs
        on every move of the Accuracy slider, and solving the kinematic
        chain there would be absurd for a label.
        """
        up = Vector(0, 0, 1)
        try:
            for i in range(self.listOperations.count()):
                item = self.listOperations.item(i)
                if item.checkState() != QtCore.Qt.CheckState.Checked:
                    continue
                wp = getattr(self.operations[i], "Workplane", None)
                if wp is not None and not wp.isEqual(up, 1e-6):
                    return True
        except Exception:
            pass
        return False

    def _solveWorkplaneMatrix(self, wp):
        """The geometric rotation that takes document coordinates into
        the working frame for workplane `wp`, or None.

        Uses the same solver Path/Op/Base.py uses to generate the real
        toolpath. A workplane's full orientation depends on the machine's
        kinematic chain, not just the target up-vector, so there is no
        way to derive this independently of that solver and still agree
        with the G-code being simulated.
        """
        machine = self.job.Proxy.getMachine() if self.job else None
        if machine is None or not machine.has_rotary_axes:
            Path.Log.warning(
                "CAM simulation: operation {} asks for a workplane "
                "rotation but the job's machine has no rotary axes -- the "
                "stock will not be re-oriented, so this operation is "
                "simulated in the wrong frame.\n".format(self.operation.Label)
            )
            return None

        try:
            import Path.Base.Generator.rotation as rotation

            result = rotation.solve_orientation(machine, wp)
            if not result.success:
                Path.Log.error(
                    "CAM simulation: cannot solve the workplane "
                    "orientation for operation {}: {}\n".format(self.operation.Label, result.reason)
                )
                return None
            chain = rotation.build_kinematic_chain(machine)
            return rotation.compute_rotation_matrix(chain, result.angles).toMatrix()
        except Exception as e:
            Path.Log.error(
                "CAM simulation: error solving the workplane rotation for "
                "operation {}: {}\n".format(self.operation.Label, e)
            )
            return None

    def _gridVolume(self):
        """Volume of material the grid currently holds, in mm3.

        Prefers the C++ MaterialVolume(), measured between the top and
        bottom surfaces. The fallback sums from the top down to the
        floor, so on a module without it a tilted re-seed still reports
        its invented material rather than hiding it. The two figures are
        not comparable and nothing in the number says which it is, so the
        fallback warns once.
        """
        if hasattr(self.voxSim, "MaterialVolume"):
            return self.voxSim.MaterialVolume()

        if not self._volumeFallbackReported:
            self._volumeFallbackReported = True
            Path.Log.warning(
                "CAM simulation: this build has no MaterialVolume(), so "
                "re-seed volumes are measured from the top surface down "
                "to the floor and will overstate a tilted stock.\n"
            )
        heights, _originals, _nx, _ny, res, _ox, _oy, floor_z, _pz = self.voxSim.GetHeightBuffer()
        import numpy as np

        h = np.frombuffer(heights, dtype=np.float32)
        return float(np.clip(h - floor_z, 0.0, None).sum()) * res * res

    def _reportReseedVolume(self, before, after, surfaceArea, lossless):
        """Say what the re-seed did to the amount of material held.

        `surfaceArea` bounds what resampling onto a new grid can change:
        at most one cell layer over the surface, area * cell size.

        LOSSLESS (the crossing transfer, ReseedFromStock): every run of
        material reaches the new grid as its own span, so the volume
        should change by resampling alone, either way. Measured in the
        engine harness at +/-0.07% at most on realistic sources; -2.2% on
        random stacks of sub-cell spans that no grid can hold. A change
        beyond the allowance IN EITHER DIRECTION is a fault and a
        warning. A gain in particular is what the min/max fill does.

        MIN/MAX (ReseedFromMesh, a module one build behind): each column
        gets the interval from the lowest triangle over it to the
        highest, which contains all its material and possibly more. So a
        GAIN is expected -- cavities the new frame cannot see into,
        filled solid: +6% at 30 degrees on a pocketed block, +19% at 90
        -- and reported at debug, since a warning that always fires
        teaches the reader to skip warnings. A LOSS beyond the allowance
        means the mesh read back was wrong -- a missing face, an unclosed
        wall -- and SeedFromMesh never inspects topology, so nothing else
        would say so. That is a warning.
        """
        if before <= 0.0:
            # Not silent: a check that cannot run must say so.
            Path.Log.warning(
                "CAM simulation: re-seeding into the workplane of {} could not "
                "be checked -- the grid read {:.1f} mm3 before it.\n".format(
                    self.operation.Label, before
                )
            )
            return
        change = after - before
        percent = 100.0 * change / before
        allowance = surfaceArea * self.resolution
        method = "transferred" if lossless else "re-seeded through the mesh"
        if lossless and abs(change) > allowance:
            Path.Log.warning(
                "CAM simulation: the stock {} into the workplane of {} changed "
                "from {:.1f} to {:.1f} mm3 ({:+.2f}%). The transfer should "
                "change it by resampling alone, at most about {:.1f} mm3 here "
                "(one cell over the surface).\n".format(
                    method, self.operation.Label, before, after, percent, allowance
                )
            )
        elif not lossless and change < -allowance:
            Path.Log.warning(
                "CAM simulation: re-seeding into the workplane of {} lost "
                "{:.1f} of {:.1f} mm3 ({:+.1f}%). Resampling can lose at "
                "most about {:.1f} mm3 here (one cell over the surface), "
                "so the mesh the re-seed read back was probably wrong -- a "
                "missing face or an unclosed wall -- and material has gone "
                "that was not cut.\n".format(
                    self.operation.Label, -change, before, percent, allowance
                )
            )
        else:
            Path.Log.debug(
                "CAM simulation: stock {} into the workplane of {}: "
                "{:.1f} -> {:.1f} mm3 ({:+.2f}%), allowance {:.1f} mm3.{}\n".format(
                    method,
                    self.operation.Label,
                    before,
                    after,
                    percent,
                    allowance,
                    (
                        ""
                        if lossless
                        else " A gain is cavities the new frame cannot see into, filled solid."
                    ),
                )
            )

    def _reportTransfer(self, report):
        """Surface what SeedFromStock found.

        `imbalanced` counts columns whose crossings did not balance: the
        source surface had a gap over them, and they were filled from
        their lowest crossing to their highest -- the old behaviour,
        silently, if nothing said so. It has never been non-zero in the
        engine harness, so any value here is a bug, and a warning.

        A report without the key is a module this file does not know. It
        is reported too: a check that cannot run must say so.
        """
        imbalanced = None if report is None else report.get("imbalanced")
        if imbalanced is None:
            Path.Log.warning(
                "CAM simulation: the stock transfer for {} returned no "
                "'imbalanced' count, so whether it was exact is unknown.\n".format(
                    self.operation.Label
                )
            )
        elif imbalanced > 0:
            Path.Log.warning(
                "CAM simulation: carrying the stock into the workplane of {} "
                "met {:,} column(s) with a gap in the source surface; they "
                "were filled from their lowest crossing to their highest, "
                "which can put back material that was cut. This is a bug in "
                "the transfer.\n".format(self.operation.Label, imbalanced)
            )
        if report is not None:
            Path.Log.debug(
                "CAM simulation: transfer for {}: {:,} source faces, {:,} "
                "crossings, {:,} columns, {:,} spans below a top span, "
                "{} imbalanced; {:.1f} ms surface, {:.1f} ms spans.\n".format(
                    self.operation.Label,
                    report.get("faces", 0),
                    report.get("crossings", 0),
                    report.get("columns", 0),
                    report.get("extraSpans", 0),
                    imbalanced,
                    report.get("surfaceMs", 0.0),
                    report.get("assignMs", 0.0),
                )
            )

    def _reseedByTransfer(self, step, bbox):
        """Carry the grid itself into the new frame: ReseedFromStock.

        Returns the source surface area for the volume check. Raises
        whatever the binding raises; the stock is then untouched.
        """
        report = self.voxSim.ReseedFromStock(
            step,
            bbox.XMin,
            bbox.YMin,
            bbox.ZMin,
            bbox.XLength,
            bbox.YLength,
            bbox.ZLength,
            self.resolution,
        )
        self._reportTransfer(report)
        return report.get("area") or self.stock.Area

    def _reseedByMesh(self, step, bbox):
        """The min/max re-seed, for a module without ReseedFromStock:
        tessellate, transform, rasterise. Fills every cavity the new frame
        cannot see into. Returns the mesh area for the volume check.

        InvalidateTessellation is a no-op on a current module, where every
        GetResultMesh is a full pass. It stays for an older one, whose
        incremental tessellator returned a mesh with holes without it.
        """
        self.voxSim.InvalidateTessellation()
        meshOuter, meshInner = self.voxSim.GetResultMesh()
        combined = Mesh.Mesh()
        combined.addMesh(meshOuter)
        combined.addMesh(meshInner)
        # The stock's own area is the fallback: smaller than a cut
        # mesh's, so the allowance errs tight rather than loose.
        meshArea = getattr(combined, "Area", None) or self.stock.Area
        combined.transform(step)
        self.voxSim.ReseedFromMesh(
            combined,
            bbox.XMin,
            bbox.YMin,
            bbox.ZMin,
            bbox.XLength,
            bbox.YLength,
            bbox.ZLength,
            self.resolution,
        )
        Path.Log.debug(
            "CAM simulation: re-seeded {} through a mesh of {:,} facets.\n".format(
                self.operation.Label, combined.CountFacets
            )
        )
        return meshArea

    def _handleWorkplaneChange(self):
        """Carry the cut stock into this operation's working frame.

        Between operations a 3+2 job re-orients the part. The grid is
        3-axis in whatever frame it was built in, so continuing to cut
        into it would simulate the new operation against the old
        orientation -- wrong, and wrong quietly. Instead the grid's
        material is carried through the rotation onto a fresh grid in the
        new frame (ReseedFromStock, the crossing transfer), span by span,
        so a pocket that faces sideways afterwards stays a pocket.

        A module without ReseedFromStock falls back to the mesh round
        trip, which fills such cavities solid, and says so once.

        Nothing here happens in a 3-axis job: every operation reports the
        same workplane and the first comparison returns immediately.
        """
        wp = self._operationWorkplane()
        if wp.isEqual(self.currentWorkplane, 1e-6):
            return

        transfer = hasattr(self.voxSim, "ReseedFromStock")
        if not transfer and not hasattr(self.voxSim, "ReseedFromMesh"):
            if not self._reseedUnavailableReported:
                self._reseedUnavailableReported = True
                TSError(
                    "This build of the CAM simulator module is out of "
                    "date: it cannot re-seed the stock, so operations on a "
                    "rotated workplane cannot be simulated in their own "
                    "orientation. The run will continue in the current "
                    "one, and the result will not be correct."
                )
            return
        if not transfer and not self._transferUnavailableReported:
            # A warning, not debug: the result is then wrong in exactly the
            # case the transfer exists for, and wrong without any error.
            self._transferUnavailableReported = True
            Path.Log.warning(
                "CAM simulation: this build of the simulator module has no "
                "ReseedFromStock(), so the stock is carried between "
                "workplanes through a mesh, and any cavity the new "
                "orientation cannot see into is filled solid (+20% on a "
                "pocketed block turned 90 degrees). Rebuild the module.\n"
            )

        new_matrix = self._solveWorkplaneMatrix(wp)
        if new_matrix is None:
            return  # already reported; leave the frame alone

        # Read before anything is destroyed: this is the old grid's
        # volume, and there is no getting it back afterwards.
        try:
            volumeBefore = self._gridVolume()
        except Exception:
            volumeBefore = 0.0

        # The grid is in the OLD working frame. Un-rotate to document
        # coordinates, then into the new frame -- new_matrix alone is
        # correct only for the first rotation of a job, where the old
        # matrix happens to be identity.
        old_matrix = self.currentTransformMatrix
        step = new_matrix.multiply(old_matrix.inverse())

        # Size the grid from the Job's stock rotated into the new frame,
        # not from what is left of it: the grid has to cover the full
        # extent the part could occupy.
        rotated_stock = self.stock.copy().transformShape(new_matrix, False, False)
        bbox = rotated_stock.BoundBox

        try:
            if transfer:
                surfaceArea = self._reseedByTransfer(step, bbox)
            else:
                surfaceArea = self._reseedByMesh(step, bbox)
        except Exception as e:
            # ReseedFromStock builds the new grid beside the old and swaps
            # only when done, so the stock, its buffers and the frame are
            # all still the old ones: leave them be, as when the solve
            # fails. (ReseedFromMesh gives no such promise, which is one
            # more reason it is only the fallback.)
            Path.Log.error(
                "CAM simulation: could not carry the stock into the "
                "workplane of {}: {}. It stays in the previous orientation, "
                "so this operation is simulated in the wrong frame.\n".format(
                    self.operation.Label, e
                )
            )
            if not transfer:
                self._stockGeneration += 1  # the buffers may be gone
            return

        # The cell size is the job's, so the count is whatever the
        # rotated extent needs. Logged because it is the one number that
        # changes silently with orientation, and a grid that has grown by
        # a factor of two should be visible somewhere.
        Path.Log.debug(
            "CAM simulation: re-seeded for {} at {:.4f} mm/cell, grid "
            "{}x{} over X [{:.3f}, {:.3f}] Y [{:.3f}, {:.3f}].\n".format(
                self.operation.Label,
                self.resolution,
                _cellsFor(bbox.XLength, self.resolution),
                _cellsFor(bbox.YLength, self.resolution),
                bbox.XMin,
                bbox.XMax,
                bbox.YMin,
                bbox.YMax,
            )
        )

        # cStock is gone and with it the buffers the viewport wrapped.
        self._stockGeneration += 1
        # _stockDirty is deliberately NOT set. It means "material has been
        # removed", and nothing has been cut here. A rotated but uncut
        # grid is exactly what a fresh setup would produce for the same
        # first operation, so leaving it alone lets SetupSimulation's
        # guard skip the re-seed Play would otherwise force.

        self.currentWorkplane = wp
        self.currentTransformMatrix = new_matrix
        self.frameTopZ = bbox.ZMax
        self.initialPos = Vector(0, 0, self.frameTopZ)

        if self.vtkView is not None:
            # The grid's coordinates are now local to the new frame. The
            # viewport draws it back in document coordinates, so the part
            # stays where it is on screen and the stock shell and model,
            # which are never transformed, still line up with it. What
            # visibly rotates is the tool axis, which is the truth of the
            # thing.
            self.vtkView.set_stock_matrix(new_matrix.inverse())
            self._vtkAttach(keep_camera=True)
            # And turn the camera with the frame, so the tool still comes
            # down the screen and the part is seen to turn, as on the
            # machine. The view only: see _vtkTurnCamera.
            self._vtkTurnCamera(new_matrix, old_matrix)
            # What is coming, while the part turns into place for it.
            banner = getattr(self.vtkView, "show_banner", None)
            if banner is not None:
                try:
                    banner(self._operationBanner(wp))
                except Exception as e:
                    Path.Log.debug("CAM simulation: could not show the banner: {}\n".format(e))

        try:
            self._reportReseedVolume(volumeBefore, self._gridVolume(), surfaceArea, transfer)
        except Exception as e:
            Path.Log.warning(
                "CAM simulation: could not measure the re-seeded stock ({}).\n".format(e)
            )

    def SetupOperation(self, itool):
        self.operation = self.activeOps[itool]
        self._handleWorkplaneChange()
        try:
            self.tool = PathDressup.toolController(self.operation).Tool
        except Exception:
            self.tool = None

        if self.tool is not None:
            # The same shape the CutTool feature used to hand on: assigning
            # a Shape to a Part::Feature outside a recompute moves the
            # feature's Placement to the shape's, so its Shape came back as
            # the tool's own, whatever the feature's placement had been.
            self.toolShape = self.tool.Shape

            if not self.toolShape.isValid() or self.toolShape.isNull():
                self.EndSimulation()
                raise RuntimeError(
                    "Path Simulation: Error in tool geometry - {}".format(self.tool.Name)
                )

            # Was 0.01 * self.accuracy, missing the maxlen factor that
            # self.resolution carries -- so the tool was profiled ~maxlen
            # times finer than a grid cell can represent (0.00025 mm
            # against a 0.025 mm cell on a 100 mm stock). Measured at 245
            # seconds. The grid resolution is the meaningful limit here,
            # the same argument BeginSimulation already makes for the
            # stock tessellation.
            self.voxSim.SetToolShape(self.toolShape, self.resolution)
        self.icmd = 0
        self.curpos = FreeCAD.Placement(self.initialPos, self.stdrot)
        if self.tool is not None:
            # Tessellated once per operation, not per tick: the shape only
            # changes when the operation does. Position comes from the
            # matrix set below, never from the geometry.
            self._vtkSetTool(self.toolShape)
        if self.vtkView is not None:
            self.vtkView.set_tool_placement(self.curpos)
        self.opCommands = PathUtils.getPathWithPlacement(self.operation).Commands
        self.UpdateCmdsPerTick()
        self._updateSpeedLabel()

    def SimulateMill(self):
        index = self.comboJobs.currentIndex()
        if 0 <= index < len(self.jobs):
            self.job = self.jobs[index]
        self.busy = False
        # No CutTool document object: see self.toolShape.

        # No CutMaterial / CutMaterialIn objects: the cut stock is drawn in
        # the VTK window, and nothing here needs a document Mesh. If the
        # result is wanted in the 3D view later, it comes from
        # GetResultMesh() on demand rather than being maintained
        # throughout the run.

        # Recompute BEFORE reading the stock, not after.
        #
        # The recompute can update the Job's Stock. (It was first needed
        # because adding the old CutTool object touched the document; kept
        # because a stale stock is still possible.) Seeding first meant the grid was built from a
        # pre-recompute stock while the reseed at Play used the
        # post-recompute one -- a different bounding box, so the viewport
        # legitimately re-framed and the camera appeared to reset. Only on
        # the first open, since afterwards the document is already
        # computed and nothing changes.
        FreeCAD.ActiveDocument.recompute()

        self.initialPos = Vector(0, 0, self.job.Stock.Shape.BoundBox.ZMax)
        self.SetupSimulation()
        self.resetSimulation = True

    def ApplyVoxelCommand(self, cmd):
        """Stamp one G-code command into the voxel stock. No mesh rebuild here —
        that's the expensive part and is done once per batch by the caller."""
        if cmd.Name in ("G0", "G00", "G1", "G01", "G2", "G02", "G3", "G03"):
            self.firstDrill = True
            if cmd.Name in ("G2", "G02", "G3", "G03") and (cmd.k or 0) == 0:
                # Omitted words are modal: an arc without X, Y or Z keeps
                # the current one. The Z used to be found by walking back
                # through earlier commands, which fell back to 0 -- a plunge
                # to the bottom of the stock -- when the operation had set
                # none yet. curpos already holds the modal value.
                here = self.curpos.Base
                ex = here.x if cmd.x is None else cmd.x
                ey = here.y if cmd.y is None else cmd.y
                ez = here.z if cmd.z is None else cmd.z
                points = _arcPoints(
                    here.x,
                    here.y,
                    here.z,
                    ex,
                    ey,
                    ez,
                    cmd.i or 0.0,
                    cmd.j or 0.0,
                    cmd.Name in ("G3", "G03"),
                    self.resolution,
                )
                cmd.Name = "G1"
                for x, y, z in points:
                    cmd.x, cmd.y, cmd.z = x, y, z
                    self.curpos = self.voxSim.ApplyCommand(self.curpos, cmd)
            else:
                self.curpos = self.voxSim.ApplyCommand(self.curpos, cmd)
        if cmd.Name == "G80":
            self.firstDrill = True
        if cmd.Name in ("G73", "G81", "G82", "G83"):
            extendcommands = []
            if self.firstDrill:
                extendcommands.append(Path.Command("G0", {"Z": cmd.r}))
                self.firstDrill = False
            extendcommands.append(Path.Command("G0", {"X": cmd.x, "Y": cmd.y, "Z": cmd.r}))
            extendcommands.append(Path.Command("G1", {"X": cmd.x, "Y": cmd.y, "Z": cmd.z}))
            extendcommands.append(Path.Command("G1", {"X": cmd.x, "Y": cmd.y, "Z": cmd.r}))
            for ecmd in extendcommands:
                self.curpos = self.voxSim.ApplyCommand(self.curpos, ecmd)

    def PerformCut(self, singleStep=False):
        if self.resetSimulation:
            self.resetSimulation = False
            self.SetupSimulation()

        if self.busy:
            return
        if singleStep:
            # An explicit step does not wait: land the turn and step.
            finish = getattr(self.vtkView, "finish_camera_turn", None)
            if finish is not None:
                finish()
        elif self._vtkCameraTurning():
            # The part is still turning on screen, so cutting waits. Were
            # it to carry on, these ticks would starve the turn's own timer
            # -- a half-second animation drawn in two frames reads as a
            # jump -- and the new operation's first cuts would happen
            # while the part was still moving.
            return
        self.busy = True

        # Batching is for a predictable tick rate, not for redraw cost.
        # SimStep always processes exactly one command.
        batchSize = 1 if singleStep else self.cmdsPerTick
        ended = False
        for _ in range(batchSize):
            if self.icmd >= len(self.opCommands):
                self.ioperation += 1
                if self.ioperation >= len(self.activeOps):
                    ended = True
                    break
                self.SetupOperation(self.ioperation)
                if not singleStep and self._vtkCameraTurning():
                    break  # a workplane change started a turn: let it play
                continue

            cmd = self.opCommands[self.icmd]
            self.ApplyVoxelCommand(cmd)
            self._stockDirty = True
            self.icmd += 1
            self.iprogress += 1

        self.UpdateProgress()
        self._updateWindowStatus()
        if not self.disableAnim:
            if self.vtkView is not None:
                self.vtkView.set_tool_placement(self.curpos)
            self._vtkRefresh()

        self.busy = False
        if ended:
            self.EndSimulation()

    def onJobChange(self):
        index = self.comboJobs.currentIndex()
        if not (0 <= index < len(self.jobs)):
            # The combo and the job list disagree. Silently indexing here
            # is how the simulator ended up seeded from the wrong job.
            Path.Log.error(
                "CAM simulation: job list out of step with the selector "
                "({} entries, index {})\n".format(len(self.jobs), index)
            )
            return
        j = self.jobs[index]
        self.job = j
        # So the next run in this document opens on the same job, in this
        # session and after a restart.
        _rememberJob(FreeCAD.ActiveDocument.Name, j.Name)
        if self.taskForm is not None:
            self._setPanelStatus(j.Label)
        self.operations = []

        follows = self.followsVisibilityPref()
        active = [op for op in j.Operations.OutList if PathUtil.opProperty(op, "Active")]
        # If nothing is visible, following visibility would tick nothing
        # and leave no operations to simulate. Fall back to all of them.
        allHidden = bool(active) and all(not op.Visibility for op in active)

        # Repopulating fires itemChanged for every row; that would run the
        # selection handler mid-rebuild, against a half-filled list.
        self.listOperations.blockSignals(True)
        self.listOperations.clear()
        for op in active:
            listItem = QtGui.QListWidgetItem(op.ViewObject.Icon, op.Label)
            listItem.setFlags(listItem.flags() | QtCore.Qt.ItemIsUserCheckable)
            hide = follows and not op.Visibility and not allHidden
            listItem.setCheckState(
                QtCore.Qt.CheckState.Unchecked if hide else QtCore.Qt.CheckState.Checked
            )
            self.operations.append(op)
            self.listOperations.addItem(listItem)
        self.listOperations.blockSignals(False)
        self.onOperationItemChange()

        # The target model, compounded once per job. Drawn inside the
        # stock so a cut past it shows through.
        try:
            models = [o.Shape for o in j.Model.OutList]
            self.baseShape = Part.makeCompound(models) if models else None
        except Exception:
            self.baseShape = None
        if self.simWindow is not None:
            self.simWindow.setModelAvailable(self.baseShape is not None)

        # Stock and Model default OFF, for every job. They used to default
        # ON for a 3+2 job, where the shell shows what a re-seeded grid has
        # lost or gained and the model shows through where a cut went past
        # it; both are a click away on the control bar for that check, and
        # left on by default they are clutter.
        #
        # Reset on every job change, so a new job never inherits a toggle
        # from the last one.
        # SetupSimulation owns this, but it has not run yet and the two
        # toggles below both tessellate at max(resolution, 0.01). Left at
        # the default that floor would tessellate the model to 0.01 mm on
        # a job change, which is far finer than anything here needs.
        try:
            bb = j.Stock.Shape.BoundBox
            self.resolution = 0.01 * self.accuracy * max(bb.XLength, bb.YLength)
        except Exception:
            pass

        self.showStockShell = False
        self.showBaseShape = False
        if self.simWindow is not None:
            self.simWindow.setViewToggles(self.showStockShell, self.showBaseShape)

        if self.vtkView is not None:
            # Belongs to the previous job.
            self.vtkView.clear_base_shape()
            self._applyBaseShape()
            self.vtkView.show_stock_shell(self.showStockShell)

        self._updateGridEstimate()
        if self.initdone:
            self.SetupSimulation()

    @staticmethod
    def followsVisibilityPref():
        # Defaults on: simulating what you can see is the expected
        # behaviour, and the all-hidden guard means it cannot leave the
        # selection empty.
        return FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/CAM").GetBool(
            "SimulatorFollowsVisibility", True
        )

    def onFollowsVisibilityChanged(self, checked):
        FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/CAM").SetBool(
            "SimulatorFollowsVisibility", bool(checked)
        )
        self.onJobChange()

    def onOperationItemChange(self, _item=None):
        """Play is only meaningful with at least one operation ticked.

        Previously nothing watched the checkboxes, so an empty selection
        reached InvalidOperation and returned silently -- a Play button
        that did nothing and said nothing.
        """
        anyChecked = any(
            self.listOperations.item(i).checkState() == QtCore.Qt.CheckState.Checked
            for i in range(self.listOperations.count())
        )
        if self.simWindow is not None:
            self.simWindow.setPlayEnabled(anyChecked)
        self._applyAccuracyLimit()

    def _applyAccuracyLimit(self):
        """Stop the Accuracy slider at FINEST_ACCURACY_3PLUS2 in a 3+2 job.

        Done by trimming the slider's range, not by clamping the value, so
        every position still means what it does in a 3-axis job and the
        label never shows a figure the simulation is not using. Qt pulls a
        value past the new maximum back onto it and emits valueChanged, so
        onAccuracyBarChange runs and the label follows.

        Depends on the ticked operations, so it runs from here: this
        handler already follows the checkboxes, and onJobChange calls it.
        """
        if self.simWindow is None:
            return
        slider = self.sliderAccuracy
        if self._accuracySliderTop is None:
            self._accuracySliderTop = slider.maximum()
        top = self._accuracySliderTop
        limited = False
        if self._anyRotatedOperation():
            # accuracy = (1.1 - 0.1 * v) / factor, solved for v. The epsilon
            # keeps the limit itself reachable against float rounding.
            limit = int(
                math.floor(
                    11.0 - 10.0 * FINEST_ACCURACY_3PLUS2 * self.accuracyResolutionFactor + 1e-6
                )
            )
            limit = max(slider.minimum(), min(top, limit))
            limited = limit < top
        else:
            limit = top
        if slider.maximum() != limit:
            slider.setMaximum(limit)
        slider.setToolTip(
            "Finest accuracy is {}% while an operation uses a rotated workplane.".format(
                FINEST_ACCURACY_3PLUS2
            )
            if limited
            else ""
        )

    def TargetMoveRate(self):
        """Moves per second the slider is asking for."""
        lo, hi = self.sliderSpeed.minimum(), self.sliderSpeed.maximum()
        span = max(1, hi - lo)
        t = (self.sliderSpeed.value() - lo) / float(span)
        ratio = self.maxMoveRate / self.minMoveRate
        return self.minMoveRate * (ratio**t)

    def UpdateCmdsPerTick(self):
        """Turn the requested move rate into a batch size and a tick rate.

        Above one move per tick the batch grows and the timer stays at its
        fastest. Below that the batch cannot go under one move, so the
        timer interval stretches instead -- which is the only way to run
        slower than tickInterval allows.
        """
        if self.fastForward:
            # Not "very fast": as fast as the event loop allows. Deriving
            # this from the slider meant fast-forward crawled whenever the
            # slider was set slow, since UpdateCmdsPerTick runs again at
            # every operation and reset the interval.
            self.cmdsPerTick = self.ffBatch
            self.timer.setInterval(0)
            return

        rate = self.TargetMoveRate()
        perTick = rate * self.tickInterval / 1000.0
        if perTick >= 1.0:
            self.cmdsPerTick = int(round(perTick))
            interval = self.tickInterval
        else:
            self.cmdsPerTick = 1
            interval = int(round(1000.0 / max(rate, 0.1)))
        self.timer.setInterval(interval)

    def onSpeedBarChange(self):
        # The speed control is the simulation window's own slider now.
        # TaskPathSimulator.ui still defines sliderSpeed/labelGPerSec, but
        # they are hidden and unread; they can go when the .ui is next
        # touched.
        self.UpdateCmdsPerTick()
        self._updateSpeedLabel()

    def onAccuracyBarChange(self):
        self.accuracy = (1.1 - 0.1 * self.sliderAccuracy.value()) / self.accuracyResolutionFactor
        self.simWindow.setAccuracyLabel(str(round(self.accuracy, 3)) + "%")
        self._updateGridEstimate()

    def _updateGridEstimate(self):
        """Show the simulation grid and the display grid side by side."""
        if self.simWindow is None:
            return
        try:
            bb = self.job.Stock.Shape.BoundBox
        except Exception:
            self.simWindow.setGridLabel("")
            return
        maxlen = max(bb.XLength, bb.YLength)
        resolution = 0.01 * self.accuracy * maxlen
        if resolution <= 0.0:
            self.simWindow.setGridLabel("")
            return
        nx = _cellsFor(bb.XLength, resolution)
        ny = _cellsFor(bb.YLength, resolution)
        cap = self.vtkView.max_cells if self.vtkView is not None else None
        viewport = self.vtkView.viewport_pixels() if self.vtkView is not None else None
        # Plan it the way attach() does, or the label describes a grid the
        # viewport will not build. Drawing undersides doubles the cost of a
        # display cell, and attach() holds the result to what the flat grid
        # would have taken, which costs a step. Quoting the flat plan here
        # named a finer grid and less memory than a 3+2 job actually uses.
        #
        # Before the first attach the view cannot say, so a rotated
        # operation stands in for it: every re-seed into a tilted frame
        # has an underside.
        underside = self._anyRotatedOperation()
        if self.vtkView is not None and self.vtkView.draws_underside:
            underside = True
        budget = None
        per_cell = None
        if underside:
            budget = SimulatorVtkView.plan_display_grid(nx, ny, cap, viewport)[3]
            per_cell = SimulatorVtkView.UNDERSIDE_BYTES_PER_DISPLAY_CELL
        dx, dy, step, nbytes = SimulatorVtkView.plan_display_grid(
            nx, ny, cap, viewport, per_cell, budget
        )
        # nbytes is the DISPLAY geometry only. The simulation grid's own
        # state is the larger of the two by some way on a fine grid, and
        # quoting only the viewport's figure understated what the slider
        # actually costs by more than double.
        simbytes = float(nx) * ny * self._bytesPerCell()
        if step == 1:
            label = "sim {}x{} ({:.0f} MB), view 1:1 ({:.0f} MB)".format(
                nx, ny, simbytes / 1e6, nbytes / 1e6
            )
        else:
            label = "sim {}x{} ({:.0f} MB), view {}x{} (1:{}, {:.0f} MB)".format(
                nx, ny, simbytes / 1e6, dx, dy, step, nbytes / 1e6
            )

        # The figures above are for the stock as it sits in the document.
        # Cell size is fixed for the job, so a rotated operation re-seeds
        # a grid sized by the ROTATED extent, which is larger. Any
        # rotation of the stock fits inside a sphere of its diagonal, so
        # that squared is an upper bound -- loose, since both axes cannot
        # reach the diagonal at once, but cheap and honest, and it stops
        # the label quietly understating the job by a factor of two.
        if self._anyRotatedOperation():
            diag = math.sqrt(bb.XLength**2 + bb.YLength**2 + bb.ZLength**2)
            nd = _cellsFor(diag, resolution)
            label += "; rotated ops at most {}x{}".format(nd, nd)

        self.simWindow.setGridLabel(label)

    def _bytesPerCell(self):
        """Simulation bytes per grid cell, as the engine reports it.

        Asked of a fresh PathSim rather than self.voxSim, which does not
        exist yet the first time the Accuracy label is drawn; the figure
        needs no stock. Cached, since it cannot change within a session.

        21 is the fallback for a module predating BytesPerCell(), and is
        right for the one that added the underside: five float arrays and
        a char. It is only a label, so the fallback is silent.
        """
        if self._cellBytes is None:
            try:
                self._cellBytes = int(PathSimulator.PathSim().BytesPerCell())
            except Exception:
                self._cellBytes = 21
        return self._cellBytes

    def GuiBusy(self, isBusy):
        if self.simWindow is not None:
            self.simWindow.setBusy(isBusy)

    def EndSimulation(self):
        self.UpdateProgress()
        self.timer.stop()
        self.GuiBusy(False)
        self.ViewShape()
        self.resetSimulation = True

    def SimStop(self):
        if self.vtkView is not None:
            self.vtkView.show_tool(False)
        self.iprogress = 0
        self.EndSimulation()

    def InvalidOperation(self):
        if len(self.activeOps) == 0:
            return True
        if self.tool is None:
            TSError("No tool assigned for the operation")
            return True
        return False

    def SimFF(self):
        if self.InvalidOperation():
            return
        self.fastForward = True
        self.disableAnim = True
        self.UpdateCmdsPerTick()
        self.GuiBusy(True)
        self.timer.start(0)

    def SimStep(self):
        if self.InvalidOperation():
            return
        self.fastForward = False
        self.disableAnim = False
        self.UpdateCmdsPerTick()
        self.PerformCut(singleStep=True)

    def SimPlay(self):
        if self.InvalidOperation():
            return
        self.fastForward = False
        self.disableAnim = False
        self.UpdateCmdsPerTick()
        self.GuiBusy(True)
        self.timer.start(self.timer.interval() or self.tickInterval)

    def ViewShape(self):
        """Bring the viewport fully up to date. Called when the run pauses
        or ends."""
        self._vtkRefresh()

    def SimPause(self):
        self.fastForward = False
        self.ViewShape()
        self.GuiBusy(False)
        self.timer.stop()

    def CloseVtkView(self):
        self._vtkClosing = True
        if self.vtkView is not None:
            try:
                self.simWindow.releaseView()
                self.simWindow.close()
            except Exception:
                pass
            self.vtkView = None
            self.simWindow = None
            self._vtkAttachedGeneration = -1

    def RemoveTool(self):
        # Nothing in the document to remove any more; the name stays for
        # accept() and cancel().
        self.toolShape = None

    def CreateResultMesh(self):
        """Put the simulated stock into the document as one Mesh.

        The cut and uncut surfaces come back as two meshes and are
        combined here: they are two halves of one solid, and nobody wants
        them as separate objects. This is the only remaining caller of
        GetResultMesh, and it runs on demand -- doing it every tick was
        the ~96 ms cost the viewport exists to avoid.
        """
        if self.voxSim is None:
            return None
        if not hasattr(self.voxSim, "InvalidateTessellation"):
            TSError(
                "This build of the CAM simulator module is out of date: "
                "InvalidateTessellation is missing, and the exported mesh "
                "would be incomplete."
            )
            return None

        if self.simWindow is not None:
            self.simWindow.setExportBusy(True, "Building mesh...")
        try:
            # A no-op on a current module, whose GetResultMesh is always a
            # full pass; kept for an older one, whose incremental
            # tessellator returned a mesh with holes without it. The
            # button works before anything has been cut, giving the uncut
            # stock.
            self.voxSim.InvalidateTessellation()
            meshOuter, meshInner = self.voxSim.GetResultMesh()
            combined = Mesh.Mesh()
            combined.addMesh(meshOuter)
            combined.addMesh(meshInner)
            # The grid is in the CURRENT working frame, not the document's.
            # After a 3+2 re-seed that frame is rotated, and the mesh has
            # to be carried back or the result lands in the document
            # rotated against the model it is meant to be compared with.
            # currentTransformMatrix maps document -> working frame and
            # always describes the grid as it actually is (identity in
            # 3-axis work, unchanged when a re-seed was refused), so its
            # inverse is the whole correction.
            #
            # Invisible on the first test job, which went one rotation and
            # back and so ended at identity.
            combined.transform(self.currentTransformMatrix.inverse())
            # No explicit Label: it then follows the object Name, giving
            # SimulationResult, SimulationResult001 and so on. Setting a
            # Label with a space in it broke that numbering and read
            # inconsistently against every other CAM object.
            obj = FreeCAD.ActiveDocument.addObject("Mesh::Feature", "SimulationResult")
            obj.Mesh = combined
            self._colourResult(obj)
            FreeCAD.ActiveDocument.recompute()
            if self.simWindow is not None:
                # Short: the control bar has no room, and the full
                # detail is in the tooltip.
                self.simWindow.setExportBusy(
                    False, "Mesh added: {:,} facets".format(combined.CountFacets)
                )
            return obj
        except Exception as e:
            if self.simWindow is not None:
                self.simWindow.setExportBusy(False, "")
            TSError("Could not create the result mesh: {}".format(e))
            return None

    def _colourResult(self, obj):
        """Give the exported mesh RESULT_COLOUR instead of the default grey.

        ShapeAppearance on current FreeCAD, ShapeColor on releases that
        predate it. The existing material is edited rather than replaced,
        so the user's default shininess and transparency survive.

        Cosmetic, so a failure is logged at debug and the export stands.
        """
        vobj = getattr(obj, "ViewObject", None)
        if vobj is None:
            return
        try:
            properties = vobj.PropertiesList
            if "ShapeAppearance" in properties:
                materials = vobj.ShapeAppearance
                material = materials[0] if materials else FreeCAD.Material()
                material.DiffuseColor = RESULT_COLOUR
                vobj.ShapeAppearance = (material,)
            elif "ShapeColor" in properties:
                vobj.ShapeColor = RESULT_COLOUR
        except Exception as e:
            Path.Log.debug("CAM simulation: could not colour the result mesh: {}\n".format(e))

    def onExportResult(self):
        """Export button."""
        self.CreateResultMesh()

    def accept(self):
        # Restores what the old OK button did: leave the simulated stock
        # in the document as a mesh.
        self.CreateResultMesh()
        self.EndSimulation()
        self.CloseVtkView()
        self.RemoveTool()

    def cancel(self):
        self.EndSimulation()
        self.CloseVtkView()
        self.RemoveTool()


class CommandPathSimulate:
    def GetResources(self):
        return {
            "Pixmap": "CAM_Simulator",
            "MenuText": QtCore.QT_TRANSLATE_NOOP("CAM_Simulator", "Legacy CAM Simulator"),
            "Accel": "P, M",
            "ToolTip": QtCore.QT_TRANSLATE_NOOP("CAM_Simulator", "Simulates G-code on stock"),
        }

    def IsActive(self):
        if FreeCAD.ActiveDocument is not None:
            for o in FreeCAD.ActiveDocument.Objects:
                if o.Name[:3] == "Job":
                    return True
        return False

    def Activated(self):
        global _current
        pathSimulation = PathSimulation()
        _current = weakref.ref(pathSimulation)
        pathSimulation.Activate()


if FreeCAD.GuiUp:
    # register the FreeCAD command
    FreeCADGui.addCommand("CAM_Simulator", CommandPathSimulate())
    FreeCAD.Console.PrintLog("Loading PathSimulator Gui… done\n")
