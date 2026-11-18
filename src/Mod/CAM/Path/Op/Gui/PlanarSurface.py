# SPDX-License-Identifier: LGPL-2.1-or-later
# SPDX-FileCopyrightText: 2025 sliptonic <shopinthewoods@gmail.com>
# SPDX-FileCopyrightText: 2026 Dimitris75 <dimitriospana75@gmail.com>
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

import FreeCAD
import FreeCADGui
from PySide import QtCore
from PySide.QtCore import QT_TRANSLATE_NOOP

import Path
import Path.Base.Gui.Util as PathGuiUtil
import Path.Op.Gui.Base as PathOpGui
import Path.Op.Gui.BoundaryPreview as BoundaryPreview
import Path.Op.Gui.FeatureListPage as FeatureListPage
import Path.Op.PlanarSurface as PathPlanarSurface

__title__ = "CAM Planar Surface Operation UI"
__author__ = "sliptonic (Brad Collette)"
__url__ = "https://www.freecad.org"
__doc__ = "Surface 3D operation page controller and command implementation."

translate = FreeCAD.Qt.translate

Path.Log.setLevel(Path.Log.Level.INFO, Path.Log.thisModule())


# Properties that change the boundary preview: a change redraws it
PREVIEW_PROPERTIES = (
    "Base",
    "AvoidGeometry",
    "FillHoleFaces",
    "Strategy",
    "BoundBox",
    "BoundaryAdjustment",
    "AvoidGeometryOverlap",
    "HandleMultipleFeatures",
    "BoundaryOutline",
    "LinearDeflection",
    "OptimizeMeshConversion",
    "ToolController",
    "Workplane",
    "SafeHeight",
    "SampleInterval",
)


# Widgets handled generically (see _readWidget / _writeWidget): (widget, property)
EXTRA_FIELDS = (
    ("cutMode", "CutMode"),
    ("waterlineOrder", "WaterlineOrder"),
    # -- Advanced --
    ("leadInOut", "LeadInOut"),
    ("leadFeed", "LeadFeed"),
    ("leadLiftDistance", "LeadLiftDistance"),
    ("finishSteepWalls", "FinishSteepWalls"),
    ("steepMaxDraftAngle", "SteepMaxDraftAngle"),
    ("steepMinDraftAngle", "SteepMinDraftAngle"),
    ("steepStepDown", "SteepStepDown"),
    ("steepOrder", "SteepOrder"),
    ("finishFillets", "FinishFillets"),
    ("filletMaxRadius", "FilletMaxRadius"),
    ("filletIncludeConcave", "FilletIncludeConcave"),
    ("filletStepOver", "FilletStepOver"),
    ("filletEdgeTrim", "FilletEdgeTrim"),
    ("volumetricFeedPercent", "VolumetricFeedPercent"),
)

# Remembers whether the Advanced section was left open
ADVANCED_PARAM = "PlanarSurfaceAdvancedOpen"


def _camParams():
    return FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/CAM")


class TaskPanelBaseGeometryPage(FeatureListPage.FeatureListBaseGeometryPage):
    """Base Geometry page with the Avoid Geometry (every strategy) and Fill
    Holes (Z-Level Hybrid) lists beside Base (see Path.Op.Gui.FeatureListPage)."""

    FEATURE_LISTS = (
        FeatureListPage.FeatureList(
            prop="AvoidGeometry",
            title=QT_TRANSLATE_NOOP("CAM_PlanarSurface", "Avoid Geometry"),
            hint=QT_TRANSLATE_NOOP(
                "CAM_PlanarSurface",
                "Faces, or closed loops of edges, the tool must not cut. "
                "The area to machine is set on the Base Geometry sub-tab.",
            ),
            accepted=("Face", "Edge"),
            context="CAM_PlanarSurface",
        ),
        FeatureListPage.FeatureList(
            prop="FillHoleFaces",
            title=QT_TRANSLATE_NOOP("CAM_PlanarSurface", "Fill Holes"),
            hint=QT_TRANSLATE_NOOP(
                "CAM_PlanarSurface",
                "Wall faces of holes to fill. Filled holes are capped so the tool does not enter them.",
            ),
            accepted=("Face",),
            visible=lambda obj: getattr(obj, "Strategy", "") in ("ZLevelHybrid", "SurfaceScan"),
            context="CAM_PlanarSurface",
        ),
    )
    VISIBILITY_PROPERTIES = ("Strategy",)


class TaskPanelOpPage(PathOpGui.TaskPanelPage):
    """Page controller class for the Surface operation."""

    def taskPanelBaseGeometryPage(self, obj, features):
        """taskPanelBaseGeometryPage(obj, features) ... Base Geometry page with the
        per-strategy face lists (Avoid / Fill Holes) as extra sub-tabs."""
        return TaskPanelBaseGeometryPage(obj, features)

    def initPage(self, obj):
        """initPage(obj) ... initialize the task panel page"""
        self.setTitle("Planar Surface - " + obj.Label)
        self.form.advancedToggle.setChecked(_camParams().GetBool(ADVANCED_PARAM, False))
        self.updateVisibility()
        self.form.accuracySlider.setPageStep(1)

        # Boundary preview (see Path.Op.Gui.BoundaryPreview)
        self.boundaryPreview = BoundaryPreview.BoundaryPreviewController(
            obj,
            self.form.showBoundaryPreview,
            properties=PREVIEW_PROPERTIES,
            status=self.form.boundaryPreviewStatus,
        )

    def getToolTipList(self):
        """getToolTipList() ... Collect list of tuples (widget_name: str, property_name: str)"""
        tuples = []
        tuples.append(("strategySelect", "Strategy"))
        tuples.append(("cutPattern", "CutPattern"))
        tuples.append(("cutPatternZLevel", "CutPatternZLevel"))
        tuples.append(("layerMode", "LayerMode"))
        tuples.append(("sampleInterval", "SampleInterval"))
        tuples.append(("adaptiveSampling", "AdaptiveSampling"))
        tuples.append(("minSampleInterval", "MinSampleInterval"))
        tuples.append(("boundBoxSelect", "BoundBox"))
        tuples.append(("boundaryAdjustment", "BoundaryAdjustment"))
        tuples.append(("boundaryOutline", "BoundaryOutline"))
        tuples.append(("stockToLeave", "StockToLeave"))
        tuples.append(("depthOffset", "DepthOffset"))
        tuples.append(("avoidGeometryOverlap", "AvoidGeometryOverlap"))
        tuples.append(("stepOver", "StepOver"))
        tuples.append(("profileEdges", "ProfileEdges"))
        tuples.append(("cutPatternAngle", "CutPatternAngle"))
        tuples.append(("cutPatternReversed", "CutPatternReversed"))
        tuples.append(("clearPlanarOnly", "ClearPlanarOnly"))
        tuples.append(("ignoreOuter", "IgnoreOuter"))
        tuples.append(("useStartPoint", "UseStartPoint"))
        tuples.append(("keepToolDown", "KeepToolDown"))
        tuples.append(("optimizeEnabled", "OptimizeLinearPaths"))
        tuples.append(("adaptivePatternAccuracy", "AdaptiveAccuracy"))
        tuples.append(("liftDistance", "LiftDistance"))
        tuples.append(("keepToolDownThreshold", "KeepToolDownThreshold"))
        tuples.append(("helixMaxRampAngle", "HelixMaxRampAngle"))
        tuples.append(("helixMaxDiameter", "HelixMaxDiameterPercent"))
        tuples.append(("forceInsideOut", "ForceInsideOut"))
        tuples.append(("finishingProfile", "FinishingProfile"))
        tuples.extend(EXTRA_FIELDS)
        return tuples

    def getForm(self):
        """getForm() ... returns UI"""
        form = FreeCADGui.PySideUic.loadUi(":/panels/PageOpPlanarSurfaceEdit.ui")
        comboToPropertyMap = [
            ("strategySelect", "Strategy"),
            ("boundBoxSelect", "BoundBox"),
            ("layerMode", "LayerMode"),
            ("cutPattern", "CutPattern"),
            ("cutPatternZLevel", "CutPatternZLevel"),
            ("profileEdges", "ProfileEdges"),
            ("adaptivePatternAccuracy", "AdaptiveAccuracy"),
            ("cutMode", "CutMode"),
            ("waterlineOrder", "WaterlineOrder"),
            ("steepOrder", "SteepOrder"),
        ]
        enumTups = PathPlanarSurface.ObjectSurface.propertyEnumerations(dataType="raw")
        PathGuiUtil.populateCombobox(form, enumTups, comboToPropertyMap)
        return form

    def getFields(self, obj):
        """getFields(obj) ... transfers values from UI to obj's properties"""
        # -- Strategy --
        if obj.Strategy != str(self.form.strategySelect.currentData()):
            obj.Strategy = str(self.form.strategySelect.currentData())

        obj.CutPattern = self.form.cutPattern.currentData()

        obj.CutPatternZLevel = self.form.cutPatternZLevel.currentData()

        if obj.LayerMode != str(self.form.layerMode.currentData()):
            obj.LayerMode = str(self.form.layerMode.currentData())

        # -- Performance and Accuracy --
        PathGuiUtil.updateInputField(obj, "SampleInterval", self.form.sampleInterval)
        PathGuiUtil.updateInputField(obj, "MinSampleInterval", self.form.minSampleInterval)

        if obj.AdaptiveSampling != self.form.adaptiveSampling.isChecked():
            obj.AdaptiveSampling = self.form.adaptiveSampling.isChecked()

        # -- Boundary Control --
        if obj.BoundBox != str(self.form.boundBoxSelect.currentData()):
            obj.BoundBox = str(self.form.boundBoxSelect.currentData())

        PathGuiUtil.updateInputField(obj, "BoundaryAdjustment", self.form.boundaryAdjustment)
        PathGuiUtil.updateInputField(obj, "StockToLeave", self.form.stockToLeave)
        PathGuiUtil.updateInputField(obj, "DepthOffset", self.form.depthOffset)
        PathGuiUtil.updateInputField(obj, "AvoidGeometryOverlap", self.form.avoidGeometryOverlap)

        # -- Clearing Options --
        if obj.StepOver != self.form.stepOver.value():
            obj.StepOver = self.form.stepOver.value()

        obj.ProfileEdges = self.form.profileEdges.currentData()

        PathGuiUtil.updateInputField(obj, "CutPatternAngle", self.form.cutPatternAngle)

        if obj.CutPatternReversed != self.form.cutPatternReversed.isChecked():
            obj.CutPatternReversed = self.form.cutPatternReversed.isChecked()

        if obj.ClearPlanarOnly != self.form.clearPlanarOnly.isChecked():
            obj.ClearPlanarOnly = self.form.clearPlanarOnly.isChecked()

        if obj.IgnoreOuter != self.form.ignoreOuter.isChecked():
            obj.IgnoreOuter = self.form.ignoreOuter.isChecked()

        if obj.BoundaryOutline != self.form.boundaryOutline.isChecked():
            obj.BoundaryOutline = self.form.boundaryOutline.isChecked()

        if obj.UseStartPoint != self.form.useStartPoint.isChecked():
            obj.UseStartPoint = self.form.useStartPoint.isChecked()

        # -- Optimization --
        if obj.OptimizeLinearPaths != self.form.optimizeEnabled.isChecked():
            obj.OptimizeLinearPaths = self.form.optimizeEnabled.isChecked()

        if obj.KeepToolDown != self.form.keepToolDown.isChecked():
            obj.KeepToolDown = self.form.keepToolDown.isChecked()

        # -- Adaptive Pattern Settings --
        if obj.AdaptiveAccuracy != str(self.form.adaptivePatternAccuracy.currentData()):
            obj.AdaptiveAccuracy = str(self.form.adaptivePatternAccuracy.currentData())

        PathGuiUtil.updateInputField(obj, "LiftDistance", self.form.liftDistance)
        PathGuiUtil.updateInputField(obj, "KeepToolDownThreshold", self.form.keepToolDownThreshold)
        PathGuiUtil.updateInputField(obj, "HelixMaxRampAngle", self.form.helixMaxRampAngle)

        if obj.HelixMaxDiameterPercent != self.form.helixMaxDiameter.value():
            obj.HelixMaxDiameterPercent = self.form.helixMaxDiameter.value()

        if obj.ForceInsideOut != self.form.forceInsideOut.isChecked():
            obj.ForceInsideOut = self.form.forceInsideOut.isChecked()

        if obj.FinishingProfile != self.form.finishingProfile.isChecked():
            obj.FinishingProfile = self.form.finishingProfile.isChecked()

        # -- Cut mode, Waterline order and Advanced --
        for name, prop in EXTRA_FIELDS:
            self._readWidget(obj, getattr(self.form, name), prop)

    def _readWidget(self, obj, widget, prop):
        """Transfers one widget's value to obj.prop (combo, check box, spin box or input field)."""
        if widget.inherits("QComboBox"):
            value = str(widget.currentData())
        elif widget.inherits("QCheckBox"):
            value = widget.isChecked()
        elif widget.inherits("QSpinBox"):
            value = widget.value()
        else:
            PathGuiUtil.updateInputField(obj, prop, widget)
            return
        if getattr(obj, prop) != value:
            setattr(obj, prop, value)

    def _writeWidget(self, obj, widget, prop):
        """Transfers obj.prop to one widget (see _readWidget)."""
        value = getattr(obj, prop)
        if widget.inherits("QComboBox"):
            self.selectInComboBox(value, widget)
        elif widget.inherits("QCheckBox"):
            widget.setChecked(bool(value))
        elif widget.inherits("QSpinBox"):
            widget.setValue(int(value))
        else:
            widget.setText(value.UserString)

    def setFields(self, obj):
        """setFields(obj) ... transfers obj's property values to UI"""

        # -- Strategy --
        self.selectInComboBox(obj.Strategy, self.form.strategySelect)
        self.selectInComboBox(obj.CutPattern, self.form.cutPattern)
        self.selectInComboBox(obj.CutPatternZLevel, self.form.cutPatternZLevel)
        self.selectInComboBox(obj.LayerMode, self.form.layerMode)

        # -- Performance and Accuracy --
        self.form.sampleInterval.setText(
            FreeCAD.Units.Quantity(obj.SampleInterval.Value, FreeCAD.Units.Length).UserString
        )
        self.form.minSampleInterval.setText(
            FreeCAD.Units.Quantity(obj.MinSampleInterval.Value, FreeCAD.Units.Length).UserString
        )

        if obj.AdaptiveSampling:
            self.form.adaptiveSampling.setCheckState(QtCore.Qt.Checked)
        else:
            self.form.adaptiveSampling.setCheckState(QtCore.Qt.Unchecked)

        self._syncAccuracyLabel()

        # -- Boundary Control --
        self.selectInComboBox(obj.BoundBox, self.form.boundBoxSelect)

        self.form.boundaryAdjustment.setText(
            FreeCAD.Units.Quantity(obj.BoundaryAdjustment.Value, FreeCAD.Units.Length).UserString
        )
        self.form.stockToLeave.setText(
            FreeCAD.Units.Quantity(obj.StockToLeave.Value, FreeCAD.Units.Length).UserString
        )
        self.form.depthOffset.setText(
            FreeCAD.Units.Quantity(obj.DepthOffset.Value, FreeCAD.Units.Length).UserString
        )
        self.form.avoidGeometryOverlap.setText(
            FreeCAD.Units.Quantity(obj.AvoidGeometryOverlap.Value, FreeCAD.Units.Length).UserString
        )

        # -- Clearing Options --
        self.form.stepOver.setValue(obj.StepOver)
        self.selectInComboBox(obj.ProfileEdges, self.form.profileEdges)

        self.form.cutPatternAngle.setText(
            FreeCAD.Units.Quantity(obj.CutPatternAngle, FreeCAD.Units.Angle).UserString
        )

        if obj.CutPatternReversed:
            self.form.cutPatternReversed.setCheckState(QtCore.Qt.Checked)
        else:
            self.form.cutPatternReversed.setCheckState(QtCore.Qt.Unchecked)

        if obj.ClearPlanarOnly:
            self.form.clearPlanarOnly.setCheckState(QtCore.Qt.Checked)
        else:
            self.form.clearPlanarOnly.setCheckState(QtCore.Qt.Unchecked)

        if obj.IgnoreOuter:
            self.form.ignoreOuter.setCheckState(QtCore.Qt.Checked)
        else:
            self.form.ignoreOuter.setCheckState(QtCore.Qt.Unchecked)

        if obj.BoundaryOutline:
            self.form.boundaryOutline.setCheckState(QtCore.Qt.Checked)
        else:
            self.form.boundaryOutline.setCheckState(QtCore.Qt.Unchecked)

        if obj.UseStartPoint:
            self.form.useStartPoint.setCheckState(QtCore.Qt.Checked)
        else:
            self.form.useStartPoint.setCheckState(QtCore.Qt.Unchecked)

        # -- Optimization --
        if obj.OptimizeLinearPaths:
            self.form.optimizeEnabled.setCheckState(QtCore.Qt.Checked)
        else:
            self.form.optimizeEnabled.setCheckState(QtCore.Qt.Unchecked)

        if obj.KeepToolDown:
            self.form.keepToolDown.setCheckState(QtCore.Qt.Checked)
        else:
            self.form.keepToolDown.setCheckState(QtCore.Qt.Unchecked)

        # -- Adaptive Pattern Settings --
        self.selectInComboBox(obj.AdaptiveAccuracy, self.form.adaptivePatternAccuracy)

        self.form.liftDistance.setText(
            FreeCAD.Units.Quantity(obj.LiftDistance.Value, FreeCAD.Units.Length).UserString
        )
        self.form.keepToolDownThreshold.setText(f"{obj.KeepToolDownThreshold.Value:.2f}")

        self.form.helixMaxRampAngle.setText(
            FreeCAD.Units.Quantity(obj.HelixMaxRampAngle, FreeCAD.Units.Angle).UserString
        )
        self.form.helixMaxDiameter.setValue(obj.HelixMaxDiameterPercent)

        if obj.ForceInsideOut:
            self.form.forceInsideOut.setCheckState(QtCore.Qt.Checked)
        else:
            self.form.forceInsideOut.setCheckState(QtCore.Qt.Unchecked)

        if obj.FinishingProfile:
            self.form.finishingProfile.setCheckState(QtCore.Qt.Checked)
        else:
            self.form.finishingProfile.setCheckState(QtCore.Qt.Unchecked)

        # -- Cut mode, Waterline order and Advanced --
        for name, prop in EXTRA_FIELDS:
            self._writeWidget(obj, getattr(self.form, name), prop)

        # -- Update Visibility --
        self.updateVisibility()

    def getSignalsForUpdate(self, obj):
        """getSignalsForUpdate(obj) ... return list of signals for updating obj"""
        signals = []
        # -- Strategy --
        signals.append(self.form.strategySelect.currentIndexChanged)
        signals.append(self.form.cutPattern.currentIndexChanged)
        signals.append(self.form.cutPatternZLevel.currentIndexChanged)
        signals.append(self.form.layerMode.currentIndexChanged)
        # -- Performance and Accuracy --
        signals.append(self.form.accuracySlider.valueChanged)
        signals.append(self.form.sampleInterval.editingFinished)
        signals.append(self.form.minSampleInterval.editingFinished)
        # -- Boundary Control --
        signals.append(self.form.boundBoxSelect.currentIndexChanged)
        signals.append(self.form.boundaryAdjustment.editingFinished)
        signals.append(self.form.stockToLeave.editingFinished)
        signals.append(self.form.depthOffset.editingFinished)
        signals.append(self.form.avoidGeometryOverlap.editingFinished)
        # -- Clearing Options --
        signals.append(self.form.stepOver.editingFinished)
        signals.append(self.form.profileEdges.currentIndexChanged)
        signals.append(self.form.cutPatternAngle.editingFinished)
        # -- Adaptive Pattern Settings
        signals.append(self.form.adaptivePatternAccuracy.currentIndexChanged)
        signals.append(self.form.liftDistance.editingFinished)
        signals.append(self.form.keepToolDownThreshold.editingFinished)
        signals.append(self.form.helixMaxRampAngle.editingFinished)
        signals.append(self.form.helixMaxDiameter.editingFinished)
        # -- Cut mode, Waterline order and Advanced --
        for name, _prop in EXTRA_FIELDS:
            widget = getattr(self.form, name)
            if widget.inherits("QComboBox"):
                signals.append(widget.currentIndexChanged)
            elif widget.inherits("QCheckBox"):
                signals.append(widget.toggled)
            else:
                signals.append(widget.editingFinished)

        if hasattr(self.form.useStartPoint, "checkStateChanged"):  # Qt version >= 6.7.0
            # -- Performance and Accuracy
            signals.append(self.form.adaptiveSampling.checkStateChanged)
            # -- Boundary Control --
            signals.append(self.form.boundaryOutline.checkStateChanged)
            # -- Clearing Options --
            signals.append(self.form.cutPatternReversed.checkStateChanged)
            signals.append(self.form.clearPlanarOnly.checkStateChanged)
            signals.append(self.form.ignoreOuter.checkStateChanged)
            signals.append(self.form.useStartPoint.checkStateChanged)
            # -- Optimization --
            signals.append(self.form.keepToolDown.checkStateChanged)
            signals.append(self.form.optimizeEnabled.checkStateChanged)
            # -- Adaptive Pattern Settings --
            signals.append(self.form.forceInsideOut.checkStateChanged)
            signals.append(self.form.finishingProfile.checkStateChanged)

        else:  # Qt version < 6.7.0
            # -- Performance and Accuracy
            signals.append(self.form.adaptiveSampling.stateChanged)
            # -- Boundary Control --
            signals.append(self.form.boundaryOutline.stateChanged)
            # -- Clearing Options --
            signals.append(self.form.cutPatternReversed.stateChanged)
            signals.append(self.form.clearPlanarOnly.stateChanged)
            signals.append(self.form.ignoreOuter.stateChanged)
            signals.append(self.form.useStartPoint.stateChanged)
            # -- Optimization --
            signals.append(self.form.keepToolDown.stateChanged)
            signals.append(self.form.optimizeEnabled.stateChanged)
            # -- Adaptive Pattern Settings --
            signals.append(self.form.forceInsideOut.stateChanged)
            signals.append(self.form.finishingProfile.stateChanged)

        return signals

    def _onAccuracySliderChanged(self, level):
        """Populate UI fields and non-UI properties from the selected accuracy preset."""
        presets = PathPlanarSurface.ObjectSurface.ACCURACY_PRESETS
        preset = presets.get(level, presets[4])
        self.form.sampleInterval.setText(
            FreeCAD.Units.Quantity(preset["sample_interval"], FreeCAD.Units.Length).UserString
        )
        self.form.minSampleInterval.setText(
            FreeCAD.Units.Quantity(preset["min_sample_interval"], FreeCAD.Units.Length).UserString
        )
        self.form.accuracyDescription.setText(
            "{} - {}".format(preset["name"], preset["description"])
        )
        obj = self.obj
        if hasattr(obj, "AngularDeflection"):
            obj.AngularDeflection = preset["angular_deflection"]
        if hasattr(obj, "LinearDeflection"):
            obj.LinearDeflection = preset["linear_deflection"]
        if hasattr(obj, "MeshSimplification"):
            obj.MeshSimplification = preset["mesh_simplification"]

        self.updateVisibility()

    def _syncAccuracyLabel(self):
        """Check if current property values match a preset; update slider and label."""
        obj = self.obj
        presets = PathPlanarSurface.ObjectSurface.ACCURACY_PRESETS

        try:
            angular = obj.AngularDeflection.Value
            linear = obj.LinearDeflection.Value
            simplification = obj.MeshSimplification
            sample = obj.SampleInterval.Value
            min_sample = obj.MinSampleInterval.Value
        except AttributeError as e:
            # Accuracy properties are not created yet (e.g. mid-restore); nothing to match.
            Path.Log.debug(f"Accuracy properties unavailable, skipping preset sync: {e}")
            self.form.accuracyDescription.setText("Custom accuracy settings**")
            return

        for lvl, preset in presets.items():
            if (
                abs(angular - preset["angular_deflection"]) < 1e-6
                and abs(linear - preset["linear_deflection"]) < 1e-6
                and simplification == preset["mesh_simplification"]
                and abs(sample - preset["sample_interval"]) < 0.001
                and abs(min_sample - preset["min_sample_interval"]) < 0.001
            ):
                self.form.accuracySlider.blockSignals(True)
                self.form.accuracySlider.setValue(lvl)
                self.form.accuracySlider.blockSignals(False)
                self.form.accuracyDescription.setText(f"{preset['name']} - {preset['description']}")
                return

        self.form.accuracyDescription.setText("Custom accuracy settings**")

    def updateVisibility(self, sentObj=None):
        """Main visibility controller. Acts as a conductor, gathering the current state
        and dispatching control to specialized helper functions for each UI group.
        """
        # 1. Gather state
        strategy = self.form.strategySelect.currentData()
        cut_pattern = self.form.cutPattern.currentData()
        cut_pattern_zlevel = self.form.cutPatternZLevel.currentData()

        try:
            sample_interval = FreeCAD.Units.Quantity(self.form.sampleInterval.text()).Value
        except ValueError:
            # Field holds an unparseable quantity while the user is still typing.
            sample_interval = 1.0  # Default to a safe value

        # Dispatch to helpers
        self._updateStrategyWidgets(strategy, cut_pattern, cut_pattern_zlevel)
        self._updatePerformanceWidgets(strategy, sample_interval)
        self._updateBoundaryWidgets(strategy)
        self._updateClearingWidgets(strategy, cut_pattern, cut_pattern_zlevel)
        self._updateOptimizationWidgets(strategy)
        self._updateAdaptivePatternWidgets(strategy, cut_pattern_zlevel)
        self._updateAdvancedWidgets(strategy)

    def _updateStrategyWidgets(self, strategy, cut_pattern, cut_pattern_zlevel):
        """Manages widgets in the 'Strategy' group."""
        is_surface_scan = strategy == "SurfaceScan"
        is_zlevel = strategy == "ZLevelHybrid"

        self.form.cutPattern.setVisible(is_surface_scan)
        self.form.cutPattern_label.setVisible(is_surface_scan)

        self.form.cutPatternZLevel.setVisible(is_zlevel)
        self.form.cutPatternZLevel_label.setVisible(is_zlevel)

        self.form.layerMode.setVisible(is_surface_scan)
        self.form.layerMode_label.setVisible(is_surface_scan)

        is_waterline = strategy == "Waterline"
        self.form.waterlineOrder.setVisible(is_waterline)
        self.form.waterlineOrder_label.setVisible(is_waterline)

    def _updatePerformanceWidgets(self, strategy, sample_interval):
        """Manages widgets in the 'Performance and Accuracy' group."""
        is_zlevel = strategy == "ZLevelHybrid"
        is_waterline = strategy == "Waterline"

        self.form.performanceAccuracyGroup.setVisible(not is_zlevel)

        adaptive_threshold = 0.30  # adaptive_threshold also in /Op/Surface.py
        is_adaptive_useful = sample_interval >= adaptive_threshold

        # The checkbox itself is only visible for strategies that can use it
        can_show_adaptive = strategy == "SurfaceScan" or is_waterline
        self.form.adaptiveSampling.setVisible(can_show_adaptive)

        # It's only ENABLED if it's both visible and useful
        self.form.adaptiveSampling.setEnabled(can_show_adaptive and is_adaptive_useful)

        # The Min Sample Interval field is only visible and enabled if adaptive is checked and active
        is_min_sample_visible = (
            self.form.adaptiveSampling.isVisible() and self.form.adaptiveSampling.isChecked()
        )
        self.form.minSampleInterval.setVisible(is_min_sample_visible)
        self.form.minSampleInterval_label.setVisible(is_min_sample_visible)

        is_min_sample_enabled = (
            self.form.adaptiveSampling.isEnabled() and self.form.adaptiveSampling.isChecked()
        )
        self.form.minSampleInterval.setEnabled(is_min_sample_enabled)
        self.form.minSampleInterval_label.setEnabled(is_min_sample_enabled)

    def _updateBoundaryWidgets(self, strategy):
        """Manages widgets in the 'Boundary Control' group."""
        is_waterline = strategy == "Waterline"
        is_zlevel = strategy == "ZLevelHybrid"

        is_surface_scan = strategy == "SurfaceScan"

        # Waterline: Boundary adjustment, Depth offset and Avoid geometry overlap
        # apply to its Base / Avoid boundary; Boundary box has no meaning there.
        self.form.boundaryGroup.setVisible(True)
        self.form.boundBoxSelect.setVisible(not is_waterline)
        self.form.boundBoxSelect_label.setVisible(not is_waterline)
        self.form.stockToLeave.setVisible(is_zlevel)
        self.form.stockToLeave_label.setVisible(is_zlevel)
        self.form.profileEdges.setVisible(is_surface_scan)
        self.form.profileEdges_label.setVisible(is_surface_scan)
        self.form.boundaryOutline.setVisible(is_surface_scan)
        self._updateSelectionWidgets()

    def _updateSelectionWidgets(self):
        """
        Greys out settings that need a selection: Avoid geometry overlap without
        Avoid Geometry, Outline only without Base Geometry.
        """
        has_avoid = bool(getattr(self.obj, "AvoidGeometry", None))
        self.form.avoidGeometryOverlap.setEnabled(has_avoid)
        self.form.avoidGeometryOverlap_label.setEnabled(has_avoid)
        self.form.boundaryOutline.setEnabled(bool(getattr(self.obj, "Base", None)))

    def updateData(self, obj, prop):
        """updateData(obj, prop) ... the Base / Avoid lists are edited on the Base Geometry page."""
        if prop in ("Base", "AvoidGeometry"):
            self._updateSelectionWidgets()
        if hasattr(self, "boundaryPreview"):
            self.boundaryPreview.updateData(obj, prop)

    def cleanupPage(self, obj):
        """cleanupPage(obj) ... the boundary preview goes with the panel."""
        if hasattr(self, "boundaryPreview"):
            self.boundaryPreview.cleanup()

    def _updateClearingWidgets(self, strategy, cut_pattern, cut_pattern_zlevel):
        """Manages widgets in the 'Clearing Options' group."""
        is_surface_scan = strategy == "SurfaceScan"
        is_zlevel = strategy == "ZLevelHybrid"

        self.form.clearingOptionsGroup.setVisible(is_surface_scan or is_zlevel)

        # Pattern Angle is enabled for linear patterns in either strategy
        is_linear_surface = is_surface_scan and cut_pattern in ["Line", "ZigZag"]
        is_linear_zlevel = is_zlevel and cut_pattern_zlevel in ["Line", "ZigZag", "Grid"]
        self.form.cutPatternAngle.setEnabled(is_linear_surface or is_linear_zlevel)
        self.form.cutPatternAngle_label.setEnabled(is_linear_surface or is_linear_zlevel)

        # Step Over and Reverse Cut Pattern are enabled if any pattern is chosen
        has_surface_scan = is_surface_scan and cut_pattern is not None
        has_zlevel_pattern = is_zlevel and cut_pattern_zlevel != "None"
        self.form.stepOver.setEnabled(has_surface_scan or has_zlevel_pattern)
        self.form.stepOver_label.setEnabled(has_surface_scan or has_zlevel_pattern)

        # Z-Level specific checkboxes
        self.form.clearPlanarOnly.setVisible(is_zlevel)
        self.form.ignoreOuter.setVisible(is_zlevel)
        self.form.useStartPoint.setVisible(is_zlevel)

    def _updateOptimizationWidgets(self, strategy):
        """Manages widgets in the 'Optimization' group."""
        is_surface_scan = strategy == "SurfaceScan"
        is_zlevel = strategy == "ZLevelHybrid"
        self.form.optimizationGroup.setVisible(not is_zlevel)
        self.form.keepToolDown.setVisible(is_surface_scan)

    def _updateAdaptivePatternWidgets(self, strategy, cut_pattern_zlevel):
        """Manages widgets in the 'Adaptive Pattern Settings' group."""
        is_adaptive_pattern = strategy == "ZLevelHybrid" and cut_pattern_zlevel == "Adaptive"
        self.form.adaptivePatternGroup.setVisible(is_adaptive_pattern)
        self.form.clearPlanarOnly.setEnabled(
            not is_adaptive_pattern and cut_pattern_zlevel != "None"
        )
        self.form.ignoreOuter.setEnabled(not is_adaptive_pattern)
        self.form.cutPatternReversed.setEnabled(not is_adaptive_pattern)
        self.form.useStartPoint.setEnabled(not is_adaptive_pattern)

    def _updateAdvancedWidgets(self, strategy):
        """Manages the 'Advanced' section, with the same rules as the property editor."""
        is_surface_scan = strategy == "SurfaceScan"
        has_leads = is_surface_scan or strategy == "Waterline"
        finishing = is_surface_scan and self.form.layerMode.currentData() == "Single-pass"

        self.form.leadsGroup.setVisible(has_leads)
        self.form.finishingGroup.setVisible(finishing)
        self.form.feedGroup.setVisible(is_surface_scan)

        leads_on = self.form.leadInOut.isChecked()
        for name in ("leadFeed", "leadLiftDistance"):
            getattr(self.form, name).setEnabled(leads_on)
            getattr(self.form, name + "_label").setEnabled(leads_on)
        # Finishing Passes: each switch's settings only while it is on
        steep_on = self.form.finishSteepWalls.isChecked()
        for name in ("steepMaxDraftAngle", "steepMinDraftAngle", "steepStepDown", "steepOrder"):
            getattr(self.form, name).setVisible(steep_on)
            getattr(self.form, name + "_label").setVisible(steep_on)
        fillets_on = self.form.finishFillets.isChecked()
        for name in ("filletMaxRadius", "filletStepOver", "filletEdgeTrim"):
            getattr(self.form, name).setVisible(fillets_on)
            getattr(self.form, name + "_label").setVisible(fillets_on)
        self.form.filletIncludeConcave.setVisible(fillets_on)

        # Nothing to show (Z-Level): no Advanced section at all
        has_advanced = has_leads or finishing
        is_open = self.form.advancedToggle.isChecked()
        self.form.advancedToggle.setVisible(has_advanced)
        self.form.advancedContent.setVisible(has_advanced and is_open)
        self.form.advancedToggle.setArrowType(
            QtCore.Qt.DownArrow if is_open else QtCore.Qt.RightArrow
        )

    def _onAdvancedToggled(self, is_open):
        _camParams().SetBool(ADVANCED_PARAM, is_open)
        self.updateVisibility()

    def registerSignalHandlers(self, obj):
        self.form.strategySelect.currentIndexChanged.connect(self.updateVisibility)
        self.form.layerMode.currentIndexChanged.connect(self.updateVisibility)
        self.form.advancedToggle.toggled.connect(self._onAdvancedToggled)
        for name in ("leadInOut", "finishSteepWalls", "finishFillets"):
            getattr(self.form, name).toggled.connect(self.updateVisibility)
        self.form.cutPattern.currentIndexChanged.connect(self.updateVisibility)
        self.form.cutPatternZLevel.currentIndexChanged.connect(self.updateVisibility)
        self.form.adaptivePatternAccuracy.currentIndexChanged.connect(self.updateVisibility)

        if hasattr(self.form.adaptiveSampling, "checkStateChanged"):
            self.form.adaptiveSampling.checkStateChanged.connect(self.updateVisibility)
        else:
            self.form.adaptiveSampling.stateChanged.connect(self.updateVisibility)

        self.form.accuracySlider.valueChanged.connect(self._onAccuracySliderChanged)
        self.form.sampleInterval.editingFinished.connect(self.updateVisibility)
        self.form.performanceAccuracyGroup.toggled.connect(self.updateVisibility)
        self.form.sampleInterval.editingFinished.connect(self._syncAccuracyLabel)
        self.form.minSampleInterval.editingFinished.connect(self._syncAccuracyLabel)


Command = PathOpGui.SetupOperation(
    "PlanarSurface",
    PathPlanarSurface.Create,
    TaskPanelOpPage,
    "CAM_PlanarSurface",
    QT_TRANSLATE_NOOP("CAM_PlanarSurface", "Planar Surface"),
    QT_TRANSLATE_NOOP("CAM_PlanarSurface", "Creates a Planar Surface operation from a model"),
    PathPlanarSurface.SetupProperties,
)

FreeCAD.Console.PrintLog("Loading PathPlanarSurfaceGui... done\n")
