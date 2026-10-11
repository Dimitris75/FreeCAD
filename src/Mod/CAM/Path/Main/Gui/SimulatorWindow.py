# SPDX-License-Identifier: LGPL-2.1-or-later
"""The CAM simulation window: the VTK viewport and every control.

Split out of Simulator.py, which had grown to a thousand lines and was
both the simulation controller and the whole user interface.

WHY A SEPARATE TOP-LEVEL WINDOW

Hosting the viewport inside FreeCAD was tried three ways -- an MDI
sub-window, a dock, and stacked over the 3D view -- and all three failed
the same way. QVTKRenderWindowInteractor can only be built on a plain
QWidget in this build, because FreeCAD's PySide6 ships no QtOpenGLWidgets
module, and that makes it a NATIVE window. Native windows take no part in
Qt's compositing or z-ordering, so under themes where the docks overlay
the document area the GL surface painted over the tree view and the Tasks
panel regardless of the geometry it reported.

A top-level window owns its whole surface, so none of that arises. The
cost is that FreeCAD's View menu does not act on it, which is why the
transport controls live here rather than in the task panel.

The window owns the widgets; Simulator.py owns the simulation and reads
them through this object.
"""

from PySide import QtCore, QtGui

from . import SimulatorVtkView


class SimulationWindow(QtGui.QWidget):
    """Viewport plus controls.

    Subclassing QWidget is not decoration: closeEvent has to be
    overridden, and assigning a closeEvent attribute to an instance does
    nothing in PySide -- the virtual is dispatched on the C++ side and the
    Python attribute is never consulted. Without a real override the
    window is destroyed with the simulation still running, the timer still
    ticking, and the next tick rendering into a dead widget. That is a
    hard crash, not an exception.
    """

    def __init__(self, controller):
        super().__init__()
        self._controller = controller
        self.view = None
        self._readouts = {}

        self.setWindowTitle("CAM Simulation")
        self.resize(1600, 900)

        outer = QtGui.QHBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        right = QtGui.QVBoxLayout()
        right.setContentsMargins(0, 0, 0, 0)
        right.setSpacing(0)

        # max_cells=None: display grid capped by viewport pixels, with a
        # memory-derived ceiling as a safety net.
        # nav_style=None: taken from the user's FreeCAD navigation setting.
        self.view = SimulatorVtkView.VtkStockView(parent=self)
        right.addWidget(self.view.widget, 1)
        right.addWidget(self._buildControlBar())

        outer.addWidget(self._buildSidePanel())
        outer.addLayout(right, 1)

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------
    def _buildSidePanel(self):
        """Job and operation selection: what to simulate, chosen once."""
        panel = QtGui.QWidget()
        panel.setFixedWidth(220)
        col = QtGui.QVBoxLayout(panel)
        col.setContentsMargins(6, 6, 6, 6)

        col.addWidget(QtGui.QLabel("Job"))
        self.comboJobs = QtGui.QComboBox()
        col.addWidget(self.comboJobs)

        col.addWidget(QtGui.QLabel("Operations"))
        self.listOperations = QtGui.QListWidget()
        self.listOperations.itemChanged.connect(self._controller.onOperationItemChange)
        col.addWidget(self.listOperations, 1)

        self.followsVisibility = QtGui.QCheckBox("Follow operation visibility")
        self.followsVisibility.setToolTip(
            "Tick only the operations that are visible in the 3D view.\n"
            "Ignored when every operation is hidden, since that would\n"
            "leave nothing to simulate."
        )
        self.followsVisibility.clicked.connect(self._controller.onFollowsVisibilityChanged)
        col.addWidget(self.followsVisibility)

        note = QtGui.QLabel("Volumetric simulation, inaccuracies are inherent")
        note.setWordWrap(True)
        col.addWidget(note)
        return panel

    def _buildControlBar(self):
        """Playback: the things adjusted while watching the result.

        Accuracy sits beside speed rather than in the side panel because
        both are dials you turn while looking at the viewport, so they
        belong where your eye already is.
        """
        bar = QtGui.QWidget()
        row = QtGui.QHBoxLayout(bar)
        row.setContentsMargins(6, 4, 6, 4)

        def button(label, slot, tip):
            b = QtGui.QToolButton()
            b.setText(label)
            b.setToolTip(tip)
            b.clicked.connect(slot)
            row.addWidget(b)
            return b

        c = self._controller
        button("\u25a0", c.SimStop, "Stop and reset")
        self.btnPlay = button("\u25b6", c.SimPlay, "Play")
        self.btnPause = button("\u2016", c.SimPause, "Pause")
        self.btnStep = button("\u25b6|", c.SimStep, "Single step")
        self.btnFF = button("\u25b6\u25b6", c.SimFF, "Run to end without drawing")

        self.btnExport = QtGui.QToolButton()
        self.btnExport.setText("To document")
        self.btnExport.setToolTip(
            "Put the simulated stock into the document as a mesh.\n"
            "Also done automatically when the task panel is closed with OK."
        )
        self.btnExport.clicked.connect(c.onExportResult)
        row.addWidget(self.btnExport)

        self.btnUncut = QtGui.QToolButton()
        self.btnUncut.setText("Uncut")
        self.btnUncut.setToolTip(
            "Show material the simulation has not touched.\n"
            "It is drawn from one height per cell, so below a curved or\n"
            "tilted stock it extrudes straight down to the floor -- a\n"
            "shape the stock never had. Turn it off and the translucent\n"
            "shell shows the true extent instead."
        )
        self.btnUncut.setCheckable(True)
        self.btnUncut.setChecked(True)
        self.btnUncut.clicked.connect(c.onShowUncutToggled)
        row.addWidget(self.btnUncut)

        self.btnStock = QtGui.QToolButton()
        self.btnStock.setText("Stock")
        self.btnStock.setToolTip(
            "Show the original stock as a translucent shell.\n"
            "It is never cut -- it marks where material used to be."
        )
        self.btnStock.setCheckable(True)
        self.btnStock.clicked.connect(c.onShowStockToggled)
        row.addWidget(self.btnStock)

        self.btnModel = QtGui.QToolButton()
        self.btnModel.setText("Model")
        self.btnModel.setToolTip(
            "Show the target model inside the stock.\n"
            "It stays hidden while material covers it, and shows through\n"
            "wherever a cut has gone past the model."
        )
        self.btnModel.setCheckable(True)
        self.btnModel.clicked.connect(c.onShowModelToggled)
        row.addWidget(self.btnModel)

        row.addSpacing(12)
        row.addWidget(QtGui.QLabel("Speed"))
        self.sliderSpeed = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.sliderSpeed.setMinimum(1)
        self.sliderSpeed.setMaximum(10)
        self.sliderSpeed.setValue(5)
        self.sliderSpeed.setFixedWidth(140)
        row.addWidget(self.sliderSpeed)

        row.addSpacing(12)
        row.addWidget(QtGui.QLabel("Accuracy"))
        self.sliderAccuracy = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.sliderAccuracy.setMinimum(1)
        self.sliderAccuracy.setMaximum(10)
        self.sliderAccuracy.setFixedWidth(140)
        row.addWidget(self.sliderAccuracy)

        # Stretch first, so progress and status sit hard against the
        # right edge instead of drifting with the width of whatever is to
        # their left.
        row.addStretch(1)

        self.progressBar = QtGui.QProgressBar()
        self.progressBar.setRange(0, 100)
        self.progressBar.setFixedWidth(180)
        row.addWidget(self.progressBar)

        row.addSpacing(8)
        self.labelStatus = QtGui.QLabel("")
        # Fixed width, elided text: a status message must never be able to
        # widen the window. "Simulation Result added, 33,652 facets" did
        # exactly that.
        self.labelStatus.setFixedWidth(150)
        self.labelStatus.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.labelStatus.setTextInteractionFlags(QtCore.Qt.NoTextInteraction)
        row.addWidget(self.labelStatus)
        return bar

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def start(self):
        """Show the window and bring the viewport up.

        The widget must be parented and on screen before VtkStockView.start
        initialises the interactor: initialising it while parentless binds
        the GL surface to a window of its own, and the viewport then stops
        tracking the layout.
        """
        self.show()
        QtGui.QApplication.processEvents()
        self.view.start()
        self.view.widget.setFocus()
        self.fitViewportAspect()

    def fitViewportAspect(self, ratio=16.0 / 9.0):
        """Size the window so the VIEWPORT is 16:9, not the window.

        The side panel and control bar are not part of the render area, so
        a 16:9 window gives a viewport that is not. Measuring the actual
        chrome and adding it back is exact whatever the theme does to
        widget sizes; fixed offsets are not.
        """
        QtGui.QApplication.processEvents()
        chromeW = self.width() - self.view.widget.width()
        chromeH = self.height() - self.view.widget.height()
        if chromeW < 0 or chromeH < 0:
            return
        viewH = max(360, self.view.widget.height())
        viewW = int(round(viewH * ratio))
        self.resize(viewW + chromeW, viewH + chromeH)
        QtGui.QApplication.processEvents()
        self.view.sync_render_window_size()

    def closeEvent(self, event):
        self._controller.onSimulationWindowClosed()
        super().closeEvent(event)

    def releaseView(self):
        """Tear the viewport down deterministically.

        Two separate failures come from letting Python's collector do this
        instead. The native surface can outlive the window and stay on
        screen as a frozen rectangle; and VTK, destroyed after its X
        window has gone, calls MakeCurrent on a dead GL context and
        segfaults inside the collector.

        shutdown() releases the GL resources while the context is still
        live, and only then is the widget destroyed.
        """
        if self.view is None:
            return
        view, self.view = self.view, None
        widget = getattr(view, "widget", None)
        try:
            view.shutdown()
        except Exception:
            pass
        if widget is not None:
            try:
                widget.setParent(None)
                # deleteLater, not immediate: this may run from inside the
                # widget's own close event.
                widget.deleteLater()
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Display updates, called by the controller
    # ------------------------------------------------------------------
    def setModelAvailable(self, available):
        """Grey the Model button when the job has no model to show."""
        self.btnModel.setEnabled(bool(available))
        if not available:
            self.btnModel.setChecked(False)

    def setViewToggles(self, stock, model):
        """Put the Stock and Model buttons in a given state.

        setChecked does not emit clicked, so this changes the buttons
        without running the controller's handlers -- the caller has
        already set the state those handlers would set.
        """
        self.btnStock.setChecked(bool(stock))
        if self.btnModel.isEnabled():
            self.btnModel.setChecked(bool(model))

    def setPlayEnabled(self, enabled):
        """Play, Step and Fast-forward are only meaningful with something
        selected to run."""
        self._playAllowed = bool(enabled)
        self.btnPlay.setEnabled(bool(enabled))
        self.btnStep.setEnabled(bool(enabled))
        self.btnFF.setEnabled(bool(enabled))

    def setExportBusy(self, busy, message=""):
        """Show that the export is running, and what it produced.

        Building the mesh is one blocking call with no progress to report,
        so the bar goes indeterminate rather than pretending to measure
        something. Without any feedback the button looks like it did
        nothing -- the work happens, the document gains an object, and the
        window is unchanged.
        """
        self.btnExport.setEnabled(not busy)
        if busy:
            self.progressBar.setRange(0, 0)  # indeterminate
        else:
            self.progressBar.setRange(0, 100)
        if message:
            self.setStatus(message)
        QtGui.QApplication.processEvents()

    def setBusy(self, isBusy):
        if not getattr(self, "_playAllowed", True):
            # An empty selection outranks the busy state: nothing to run.
            isBusy = True
        self.btnPlay.setEnabled(not isBusy)
        self.btnPause.setEnabled(isBusy)
        self.btnStep.setEnabled(not isBusy)
        self.btnFF.setEnabled(not isBusy)

    def setProgress(self, percent):
        self.progressBar.setValue(int(percent))

    def setStatus(self, text):
        """Set the status text, elided to the label's width so a long
        message cannot resize the window."""
        metrics = self.labelStatus.fontMetrics()
        self.labelStatus.setText(
            metrics.elidedText(text, QtCore.Qt.ElideRight, self.labelStatus.width() - 4)
        )
        self.labelStatus.setToolTip(text)

    # The slider readouts are drawn in the viewport now: the control bar
    # had run out of room, and they belong next to what they describe.
    def setSpeedLabel(self, text):
        self._readouts["speed"] = text
        self._updateOverlay()

    def setAccuracyLabel(self, text):
        self._readouts["accuracy"] = text
        self._updateOverlay()

    def setGridLabel(self, text):
        self._readouts["grid"] = text
        self._updateOverlay()

    def _updateOverlay(self):
        if self.view is None:
            return
        lines = [
            self._readouts.get("speed", ""),
            "  ".join(
                part
                for part in (
                    self._readouts.get("accuracy", ""),
                    self._readouts.get("grid", ""),
                )
                if part
            ),
        ]
        self.view.set_overlay_text("\n".join(l for l in lines if l))
