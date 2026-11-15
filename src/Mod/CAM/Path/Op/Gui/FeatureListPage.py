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
#                                                                              #
#   You should have received a copy of the GNU Lesser General Public           #
#   License along with FreeCAD. If not, see https://www.gnu.org/licenses       #
#                                                                              #
################################################################################

"""
Base Geometry page with extra feature lists.

Some operations need more selections than Base: geometry to avoid, holes to
fill, and so on. This page keeps the stock Base Geometry form unchanged on a
first sub-tab and adds one sub-tab per extra list, each with its own Add /
Remove / Clear and a short hint.

An operation's GUI module subclasses FeatureListBaseGeometryPage and lists
its feature lists:

    class TaskPanelBaseGeometryPage(FeatureListPage.FeatureListBaseGeometryPage):
        FEATURE_LISTS = (
            FeatureListPage.FeatureList(
                prop="AvoidGeometry",
                title=QT_TRANSLATE_NOOP("CAM_MyOp", "Avoid Geometry"),
                hint=QT_TRANSLATE_NOOP("CAM_MyOp", "Faces the tool must not cut."),
                accepted=("Face", "Edge"),
                context="CAM_MyOp",
            ),
        )

and returns it from its page's taskPanelBaseGeometryPage(obj, features).

The operation's proxy provides, for every list property:

    addFeature(obj, prop, base, sub) -> bool
        Adds one sub-element (mapping the selected object to the Job's model
        clone as for Base); returns whether it was added.

A list can be shown for some operation states only (a strategy, say): give
it visible=callable(obj) -> bool, and name the properties that change it in
the subclass's VISIBILITY_PROPERTIES.
"""

from typing import Callable, NamedTuple, Optional

import FreeCAD
import FreeCADGui
from PySide import QtGui

import Path
import Path.Op.Gui.Base as PathOpGui

translate = FreeCAD.Qt.translate

Path.Log.setLevel(Path.Log.Level.INFO, Path.Log.thisModule())


class FeatureList(NamedTuple):
    """One extra feature list on the Base Geometry page."""

    prop: str  # the operation's App::PropertyLinkSubListGlobal
    title: str  # sub-tab title (QT_TRANSLATE_NOOP'd in *context*)
    hint: str  # text above the list (QT_TRANSLATE_NOOP'd in *context*)
    accepted: tuple = ("Face",)  # sub-element prefixes the list takes
    visible: Optional[Callable] = None  # visible(obj) -> bool; None: always
    context: str = "PathOp"  # translation context of title and hint


class FeatureListBaseGeometryPage(PathOpGui.TaskPanelBaseGeometryPage):
    """
    Base Geometry page with one extra sub-tab per FEATURE_LISTS entry.

    The stock Base Geometry form is loaded unchanged and moved onto the first
    sub-tab: ``self.form`` stays the original panel, so every inherited method
    that reaches ``self.form.baseList`` etc. keeps working.
    """

    # FeatureList entries, in sub-tab order
    FEATURE_LISTS = ()
    # Properties whose change can show or hide a list (see FeatureList.visible)
    VISIBILITY_PROPERTIES = ()

    def getForm(self):
        panel = super().getForm()

        # Move the stock content, item by item with its grid position, onto
        # the first sub-tab. Moving items (rather than the QLayout itself)
        # keeps the widgets inside ``panel``'s tree, so the attribute names
        # set by loadUi remain valid.
        stock_layout = panel.layout()
        base_tab = QtGui.QWidget()
        base_layout = QtGui.QGridLayout(base_tab)
        items = []
        while stock_layout.count():
            row, col, rspan, cspan = stock_layout.getItemPosition(0)
            items.append((stock_layout.takeAt(0), row, col, rspan, cspan))
        for item, row, col, rspan, cspan in items:
            if item.widget():
                base_layout.addWidget(item.widget(), row, col, rspan, cspan)
            else:
                base_layout.addItem(item, row, col, rspan, cspan)

        self.subTabs = QtGui.QTabWidget()
        self.subTabs.addTab(base_tab, translate("PathOp", "Base Geometry"))

        # One form per list, each with a fixed title. Tabs are shown or hidden,
        # never renamed: QTabWidget does not re-measure a renamed tab that is
        # off screen, which clips the new title.
        self.featureLists = {spec.prop: spec for spec in self.FEATURE_LISTS}
        self.featureForms = {}
        for spec in self.FEATURE_LISTS:
            form = FreeCADGui.PySideUic.loadUi(":/panels/PageFeatureList.ui")
            form.featureHint.setText(translate(spec.context, spec.hint))
            self.subTabs.addTab(form, translate(spec.context, spec.title))
            self.featureForms[spec.prop] = form

        stock_layout.addWidget(self.subTabs, 0, 0)
        return panel

    # ---- Helpers ----

    def _setTabVisible(self, form, visible):
        index = self.subTabs.indexOf(form)
        if hasattr(self.subTabs, "setTabVisible"):  # Qt >= 5.15
            self.subTabs.setTabVisible(index, visible)
        else:
            self.subTabs.setTabEnabled(index, visible)
        if not visible and self.subTabs.currentIndex() == index:
            self.subTabs.setCurrentIndex(0)

    def _updateFeatureTabs(self, obj):
        for prop, spec in self.featureLists.items():
            visible = spec.visible(obj) if spec.visible else True
            self._setTabVisible(self.featureForms[prop], visible)

    def _fillFeatureList(self, obj, prop):
        feature_list = self.featureForms[prop].featureList
        feature_list.blockSignals(True)
        feature_list.clear()
        for base, subs in getattr(obj, prop, None) or []:
            for sub in subs:
                item = QtGui.QListWidgetItem(f"{base.Label}.{sub}")
                item.setData(self.DataObject, base)
                item.setData(self.DataObjectSub, sub)
                feature_list.addItem(item)
        feature_list.blockSignals(False)

    # ---- Page interface ----

    def setFields(self, obj):
        super().setFields(obj)
        for prop in self.featureForms:
            self._fillFeatureList(obj, prop)
        self._updateFeatureTabs(obj)

    def registerSignalHandlers(self, obj):
        super().registerSignalHandlers(obj)
        for prop, form in self.featureForms.items():
            form.featureList.itemSelectionChanged.connect(
                lambda p=prop: self.featureItemActivated(p)
            )
            form.addFeature.clicked.connect(lambda _=False, p=prop: self.addFeature(p))
            form.deleteFeature.clicked.connect(lambda _=False, p=prop: self.deleteFeature(p))
            form.clearFeature.clicked.connect(lambda _=False, p=prop: self.clearFeature(p))

    def pageUpdateData(self, obj, prop):
        super().pageUpdateData(obj, prop)
        if prop in self.VISIBILITY_PROPERTIES:
            self._updateFeatureTabs(obj)
        elif prop in self.featureForms:
            self._fillFeatureList(obj, prop)

    def updateSelection(self, obj, selection):
        super().updateSelection(obj, selection)
        subs = [sub for sel in selection for sub in (sel.SubElementNames or [])]
        for prop, form in self.featureForms.items():
            accepted = self.featureLists[prop].accepted
            form.addFeature.setEnabled(any(sub.startswith(accepted) for sub in subs))

    # ---- List actions ----

    def featureItemActivated(self, prop):
        FreeCADGui.Selection.clearSelection()
        for item in self.featureForms[prop].featureList.selectedItems():
            FreeCADGui.Selection.addSelection(
                item.data(self.DataObject), item.data(self.DataObjectSub)
            )

    def addFeature(self, prop):
        accepted = self.featureLists[prop].accepted
        added = False
        for sel in FreeCADGui.Selection.getSelectionEx():
            if not hasattr(sel.Object, "Shape"):
                continue
            for sub in sel.SubElementNames:
                if sub.startswith(accepted):
                    added |= bool(self.obj.Proxy.addFeature(self.obj, prop, sel.Object, sub))
        if added:
            self._fillFeatureList(self.obj, prop)
            self.setDirty()

    def deleteFeature(self, prop):
        feature_list = self.featureForms[prop].featureList
        for item in feature_list.selectedItems():
            feature_list.takeItem(feature_list.row(item))
        links = []
        for i in range(feature_list.count()):
            item = feature_list.item(i)
            links.append((item.data(self.DataObject), str(item.data(self.DataObjectSub))))
        setattr(self.obj, prop, links)
        self._fillFeatureList(self.obj, prop)
        self.setDirty()

    def clearFeature(self, prop):
        setattr(self.obj, prop, [])
        self._fillFeatureList(self.obj, prop)
        self.setDirty()
