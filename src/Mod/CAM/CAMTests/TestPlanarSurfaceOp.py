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
import Part
import Path
import math
import os
import tempfile
import unittest
import Path.Main.Workplane as PathWorkplane
import Path.Main.Job as PathJob
from CAMTests.PathTestUtils import PathTestWithAssets

Path.Log.setLevel(Path.Log.Level.INFO, Path.Log.thisModule())
Path.Log.trackModule(Path.Log.thisModule())

# Check if OCL is available
try:
    import ocl
except ImportError:
    try:
        import opencamlib as ocl
    except ImportError:
        ocl = None

_ocl_available = ocl is not None

if _ocl_available:
    import Path.Op.PlanarSurface as PathPlanarSurface


@unittest.skipUnless(_ocl_available, "OpenCamLib not available")
class TestPlanarSurfaceOp(PathTestWithAssets):
    """Integration tests for the unified Surface operation.

    These tests create a FreeCAD document with a Job, model geometry,
    and tool controller, then execute the Surface operation to verify
    end-to-end functionality.
    """

    def setUp(self):
        super().setUp()
        self.doc = FreeCAD.newDocument("TestPlanarSurface")

    def tearDown(self):
        super().tearDown()
        FreeCAD.closeDocument(self.doc.Name)

    def _createJobWithBox(self, length=100, width=100, height=10):
        """Helper: create a Job with a simple box model and endmill tool."""
        box = self.doc.addObject("Part::Feature", "TestBox")
        box.Shape = Part.makeBox(length, width, height)

        job = PathJob.Create("Job", [box])

        # Load a 5mm endmill from assets and assign to tool controller
        toolbit = self.assets.get("toolbit://5mm_Endmill")
        loaded_tool = toolbit.attach_to_doc(doc=self.doc)
        job.Tools.Group[0].Tool = loaded_tool

        self.doc.recompute()
        return job

    def _createJobWithSphere(self, radius=50):
        """Helper: create a Job with a simple sphere model and endmill tool."""
        sphere = self.doc.addObject("Part::Feature", "TestSphere")
        sphere.Shape = Part.makeSphere(radius)

        job = PathJob.Create("Job", [sphere])

        # Load a 5mm endmill from assets and assign to tool controller
        toolbit = self.assets.get("toolbit://5mm_Endmill")
        loaded_tool = toolbit.attach_to_doc(doc=self.doc)
        job.Tools.Group[0].Tool = loaded_tool

        self.doc.recompute()
        return job

    # -- Property definition tests --

    def test00(self):
        """
        Verifies that the Surface operation can be created and has the Strategy property.

        INPUT:
        - Function: PathPlanarSurface.ObjectSurface()
        - Parameters: None (just instantiation check)
        - Input data: A new Surface operation object

        EXPECTED OUTPUT:
        - Operation object should be created successfully
        - Should have a Strategy property with default value "SurfaceScan"
        - Confirms the operation is properly registered and initialized
        """
        self._createJobWithBox()

        op = FreeCAD.ActiveDocument.addObject("Path::FeaturePython", "PlanarSurface")
        proxy = PathPlanarSurface.ObjectSurface(op, "PlanarSurface")
        proxy.initOperation(op)

        self.assertTrue(hasattr(op, "Strategy"))
        self.doc.recompute()

    def test01(self):
        """
        Verifies that the Strategy property has the correct enumeration values.

        INPUT:
        - Function: ObjectPlanarSurface.propertyEnumerations()
        - Parameters: dataType="data"
        - Input data: Class method call for enumeration data

        EXPECTED OUTPUT:
        - Should contain "Strategy" enumeration
        - Strategy should include: SurfaceScan, Waterline, ZLevelHybrid
        - These are the four supported 3D surfacing strategies
        """
        enums = PathPlanarSurface.ObjectSurface.propertyEnumerations()
        enum_dict = {name: values for name, values in enums}

        self.assertIn("Strategy", enum_dict)
        strategies = enum_dict["Strategy"]
        self.assertIn("SurfaceScan", strategies)
        self.assertIn("Waterline", strategies)
        self.assertIn("ZLevelHybrid", strategies)

    def test02(self):
        """
        Verifies that the CutPattern property has the correct enumeration values.

        INPUT:
        - Function: ObjectSurface.propertyEnumerations()
        - Parameters: dataType="data"
        - Input data: Class method call for enumeration data

        EXPECTED OUTPUT:
        - Should contain "CutPattern" enumeration
        - CutPattern should include: Line, ZigZag, Circular, CircularZigZag, Spiral, Offset
        - These are the scan patterns available for SurfaceScan strategy
        """
        enums = PathPlanarSurface.ObjectSurface.propertyEnumerations()
        enum_dict = {name: values for name, values in enums}

        self.assertIn("CutPattern", enum_dict)
        patterns = enum_dict["CutPattern"]
        self.assertIn("Line", patterns)
        self.assertIn("ZigZag", patterns)
        self.assertIn("Circular", patterns)
        self.assertIn("Spiral", patterns)

    def test03(self):
        """
        Verifies that the CutPatternZLevel property has the correct enumeration values.

        INPUT:
        - Function: ObjectSurface.propertyEnumerations()
        - Parameters: dataType="data"
        - Input data: Class method call for enumeration data

        EXPECTED OUTPUT:
        - Should contain "CutPatternZLevel" enumeration
        - CutPatternZLevel should include: None, Line, ZigZag, Offset, Grid
        - These are the scan patterns available for Z-Level Hybrid strategy
        """
        enums = PathPlanarSurface.ObjectSurface.propertyEnumerations()
        enum_dict = {name: values for name, values in enums}

        self.assertIn("CutPatternZLevel", enum_dict)
        patterns = enum_dict["CutPatternZLevel"]
        self.assertIn("None", patterns)
        self.assertIn("Line", patterns)
        self.assertIn("ZigZag", patterns)
        self.assertIn("Offset", patterns)
        self.assertIn("Grid", patterns)

    def test04(self):
        """
        Verifies that opFeatures returns the expected feature flags.

        INPUT:
        - Function: ObjectSurface.opFeatures()
        - Parameters: obj (FreeCAD operation object)
        - Input data: A new Surface operation

        EXPECTED OUTPUT:
        - Should include FeatureTool, FeatureDepths, FeatureHeights
        - Should include FeatureStepDown, FeatureCoolant, FeatureBaseFaces
        - These flags control which base class behaviors are enabled
        """
        import Path.Op.Base as PathOp

        proxy = PathPlanarSurface.ObjectSurface.__new__(PathPlanarSurface.ObjectSurface)
        features = proxy.opFeatures(None)

        self.assertTrue(features & PathOp.FeatureTool)
        self.assertTrue(features & PathOp.FeatureDepths)
        self.assertTrue(features & PathOp.FeatureHeights)
        self.assertTrue(features & PathOp.FeatureStepDown)
        self.assertTrue(features & PathOp.FeatureCoolant)
        self.assertTrue(features & PathOp.FeatureBaseFaces)

    # -- SurfaceScan execution tests --

    @unittest.skipUnless(_ocl_available, "OpenCamLib not available")
    def test10(self):
        """
        Executes the SurfaceScan strategy on a simple box and verifies G-code output.

        INPUT:
        - Function: ObjectSurface.opExecute()
        - Parameters: Strategy=SurfaceScan, CutPattern=Line on a 100x100x10mm box
        - Input data: Simple rectangular solid with 5mm endmill

        EXPECTED OUTPUT:
        - Operation should execute without errors
        - Should produce G-code commands (non-empty path)
        - Path should contain both G0 (rapid) and G1 (cut) moves
        - Verifies the full SurfaceScan pipeline works end-to-end
        """
        job = self._createJobWithBox()

        op = FreeCAD.ActiveDocument.addObject("Path::FeaturePython", "PlanarSurface")
        proxy = PathPlanarSurface.ObjectSurface(op, "PlanarSurface")
        proxy.initOperation(op)

        op.Strategy = "SurfaceScan"
        op.CutPattern = "Line"
        op.StepOver = 50.0
        op.SampleInterval = 5.0

        # Set the Base geometry to the job's model
        op.Base = job.Model.Group

        job.Operations.addObject(op)
        self.doc.recompute()

        # Execute the operation
        op.Proxy.execute(op)

        # Verify output
        self.assertTrue(
            len(op.Path.Commands) > 0,
            "SurfaceScan should produce G-code commands",
        )

        # Check for both rapid and cut moves
        cmd_names = [c.Name for c in op.Path.Commands]
        self.assertIn("G0", cmd_names, "Should contain rapid moves")
        self.assertIn("G1", cmd_names, "Should contain cutting moves")

    @unittest.skipUnless(_ocl_available, "OpenCamLib not available")
    def test11(self):
        """
        Executes the SurfaceScan (Adaptive) strategy on a simple box and verifies G-code output.

        INPUT:
        - Function: ObjectSurface.opExecute()
        - Parameters: Strategy=SurfaceScan, CutPattern=Line on a 100x100x10mm box
        - Input data: Simple rectangular solid with 5mm endmill

        EXPECTED OUTPUT:
        - Operation should execute without errors
        - Should produce G-code commands (non-empty path)
        - Path should contain both G0 (rapid) and G1 (cut) moves
        - Verifies the full SurfaceScan pipeline works end-to-end
        """
        job = self._createJobWithBox()

        op = FreeCAD.ActiveDocument.addObject("Path::FeaturePython", "PlanarSurface")
        proxy = PathPlanarSurface.ObjectSurface(op, "PlanarSurface")
        proxy.initOperation(op)

        op.Strategy = "SurfaceScan"
        op.CutPattern = "Line"
        op.StepOver = 50.0
        op.AdaptiveSampling = True
        op.SampleInterval = 5.0
        op.MinSampleInterval = 3.0

        # Set the Base geometry to the job's model
        op.Base = job.Model.Group

        job.Operations.addObject(op)
        self.doc.recompute()

        # Execute the operation
        op.Proxy.execute(op)

        # Verify output
        self.assertTrue(
            len(op.Path.Commands) > 0,
            "SurfaceScan should produce G-code commands",
        )

        # Check for both rapid and cut moves
        cmd_names = [c.Name for c in op.Path.Commands]
        self.assertIn("G0", cmd_names, "Should contain rapid moves")
        self.assertIn("G1", cmd_names, "Should contain cutting moves")

    @unittest.skipUnless(_ocl_available, "OpenCamLib not available")
    def test12(self):
        """
        Executes the SurfaceScan strategy with ZigZag pattern on a box.

        INPUT:
        - Function: ObjectSurface.opExecute()
        - Parameters: Strategy=SurfaceScan, CutPattern=ZigZag on a 100x100x10mm box
        - Input data: Simple rectangular solid with 5mm endmill

        EXPECTED OUTPUT:
        - Operation should execute without errors
        - Should produce G-code commands (non-empty path)
        - ZigZag pattern alternates line direction for efficient machining
        """
        job = self._createJobWithBox()

        op = FreeCAD.ActiveDocument.addObject("Path::FeaturePython", "PlanarSurface")
        proxy = PathPlanarSurface.ObjectSurface(op, "PlanarSurface")
        proxy.initOperation(op)

        op.Strategy = "SurfaceScan"
        op.CutPattern = "ZigZag"
        op.StepOver = 50.0
        op.SampleInterval = 5.0

        # Set the Base geometry to the job's model
        op.Base = job.Model.Group

        job.Operations.addObject(op)
        self.doc.recompute()

        op.Proxy.execute(op)

        self.assertTrue(
            len(op.Path.Commands) > 0,
            "SurfaceScan ZigZag should produce G-code commands",
        )

    # -- Waterline execution tests --

    @unittest.skipUnless(_ocl_available, "OpenCamLib not available")
    def test20(self):
        """
        Executes the Waterline strategy on a box and verifies G-code output.

        INPUT:
        - Function: ObjectSurface.opExecute()
        - Parameters: Strategy=Waterline on a 100x100x10mm box
        - Input data: Simple rectangular solid with 5mm endmill

        EXPECTED OUTPUT:
        - Operation should execute without errors
        - Should produce G-code commands (non-empty path)
        - Waterline creates constant-Z contour paths around the model
        """
        job = self._createJobWithBox()

        op = FreeCAD.ActiveDocument.addObject("Path::FeaturePython", "PlanarSurface")
        proxy = PathPlanarSurface.ObjectSurface(op, "PlanarSurface")
        proxy.initOperation(op)

        op.Strategy = "Waterline"
        op.SampleInterval = 2.0

        # Set the Base geometry to the job's model
        op.Base = job.Model.Group

        job.Operations.addObject(op)
        self.doc.recompute()

        op.Proxy.execute(op)

        self.assertTrue(
            len(op.Path.Commands) > 0,
            "Waterline should produce G-code commands",
        )

    @unittest.skipUnless(_ocl_available, "OpenCamLib not available")
    def test21(self):
        """
        Executes the Waterline (Adaptive) strategy on a box.

        INPUT:
        - Function: ObjectSurface.opExecute()
        - Parameters: Strategy=Waterline on a 100x100x10mm box
        - Input data: Simple rectangular solid with 5mm endmill

        EXPECTED OUTPUT:
        - Operation should execute without errors
        - Should produce G-code commands (non-empty path)
        - Waterline (Adaptive) refines sampling where contour changes rapidly
        """
        job = self._createJobWithBox()

        op = FreeCAD.ActiveDocument.addObject("Path::FeaturePython", "PlanarSurface")
        proxy = PathPlanarSurface.ObjectSurface(op, "PlanarSurface")
        proxy.initOperation(op)

        op.Strategy = "Waterline"
        op.AdaptiveSampling = True
        op.SampleInterval = 2.0
        op.MinSampleInterval = 1.0

        # Set the Base geometry to the job's model
        op.Base = job.Model.Group

        job.Operations.addObject(op)
        self.doc.recompute()

        op.Proxy.execute(op)

        self.assertTrue(
            len(op.Path.Commands) > 0,
            "AdaptiveWaterline should produce G-code commands",
        )

    # -- Z-Level Hybrid execution tests --

    def test30(self):
        """
        Executes the Z-Level Hybrid strategy on a sphere.

        INPUT:
        - Function: ObjectSurface.opExecute()
        - Parameters: Strategy=ZLevelHybrid on a 50mm radius sphere
        - Input data: Simple rectangular solid with 5mm endmill

        EXPECTED OUTPUT:
        - Operation should execute without errors
        - Should produce G-code commands (non-empty path)
        - Operation should execute using the configured strategy
        """
        job = self._createJobWithSphere()

        op = FreeCAD.ActiveDocument.addObject("Path::FeaturePython", "PlanarSurface")
        proxy = PathPlanarSurface.ObjectSurface(op, "PlanarSurface")
        proxy.initOperation(op)

        op.Strategy = "ZLevelHybrid"

        # Set the Base geometry to the job's model
        op.Base = job.Model.Group

        job.Operations.addObject(op)
        self.doc.recompute()

        op.Proxy.execute(op)

        self.assertTrue(
            len(op.Path.Commands) > 0,
            "Z-Level Hybrid should produce G-code commands",
        )

    # -- Property visibility tests --

    def test40(self):
        """
        Verifies that SurfaceScan strategy shows scan pattern properties.

        INPUT:
        - Function: ObjectSurface.setEditorProperties()
        - Parameters: Strategy=SurfaceScan
        - Input data: Surface operation with SurfaceScan strategy selected

        EXPECTED OUTPUT:
        - CutPattern should be visible (editor mode 0)
        - ClearPlanarOnly, IgnoreOuter, SamplingAccuracy, CutPatternZLevel and StockToLeave
          should be hidden (editor mode 2) since it's Z-Level-specific
        - Ensures UI shows relevant properties for the selected strategy
        """
        self._createJobWithBox()

        op = FreeCAD.ActiveDocument.addObject("Path::FeaturePython", "PlanarSurface")
        proxy = PathPlanarSurface.ObjectSurface(op, "PlanarSurface")
        proxy.initOperation(op)

        op.Strategy = "SurfaceScan"
        proxy.setEditorProperties(op)

        self.assertEqual(op.getEditorMode("CutPattern"), [])  # visible
        self.assertEqual(op.getEditorMode("ClearPlanarOnly"), ["Hidden"])  # hidden
        self.assertEqual(op.getEditorMode("IgnoreOuter"), ["Hidden"])  # hidden
        self.assertEqual(op.getEditorMode("SamplingAccuracy"), ["Hidden"])  # hidden
        self.assertEqual(op.getEditorMode("CutPatternZLevel"), ["Hidden"])  # hidden
        self.assertEqual(op.getEditorMode("StockToLeave"), ["Hidden"])  # hidden

    def test41(self):
        """
        Verifies that Waterline strategy hides scan pattern properties.

        INPUT:
        - Function: ObjectSurface.setEditorProperties()
        - Parameters: Strategy=Waterline
        - Input data: Surface operation with Waterline strategy selected

        EXPECTED OUTPUT:
        - CutPattern should be hidden (editor mode 2) since waterline doesn't use scan patterns
        - ClearPlanarOnly, IgnoreOuter, SamplingAccuracy, CutPatternZLevel and StockToLeave
          should be hidden (editor mode 2) since it's Z-Level-specific
        - Ensures UI adapts to the selected strategy
        """
        self._createJobWithBox()

        op = FreeCAD.ActiveDocument.addObject("Path::FeaturePython", "PlanarSurface")
        proxy = PathPlanarSurface.ObjectSurface(op, "PlanarSurface")
        proxy.initOperation(op)

        op.Strategy = "Waterline"
        proxy.setEditorProperties(op)

        self.assertEqual(op.getEditorMode("CutPattern"), ["Hidden"])  # hidden
        self.assertEqual(op.getEditorMode("ClearPlanarOnly"), ["Hidden"])  # hidden
        self.assertEqual(op.getEditorMode("IgnoreOuter"), ["Hidden"])  # hidden
        self.assertEqual(op.getEditorMode("SamplingAccuracy"), ["Hidden"])  # hidden
        self.assertEqual(op.getEditorMode("CutPatternZLevel"), ["Hidden"])  # hidden
        self.assertEqual(op.getEditorMode("StockToLeave"), ["Hidden"])  # hidden

    # -- Accuracy Preset Tests --

    def test42(self):
        """
        Verifies that accuracy presets are properly defined and accessible.

        INPUT:
        - Function: ObjectSurface.ACCURACY_PRESETS
        - Parameters: None (static class attribute access)
        - Input data: The ACCURACY_PRESETS dictionary

        EXPECTED OUTPUT:
        - All levels 1-7 should exist in the presets
        - Each preset should have required keys: name, angular_deflection, linear_deflection,
          mesh_simplification, sample_interval, description
        - Confirms the preset system is properly structured
        """
        presets = PathPlanarSurface.ObjectSurface.ACCURACY_PRESETS

        # Check all levels 1-7 exist
        for level in range(1, 8):
            self.assertIn(level, presets)

        # Check required preset keys
        for level, preset in presets.items():
            self.assertIn("name", preset)
            self.assertIn("angular_deflection", preset)
            self.assertIn("linear_deflection", preset)
            self.assertIn("mesh_simplification", preset)
            self.assertIn("sample_interval", preset)
            self.assertIn("description", preset)

    def test43(self):
        """
        Verifies that preset progression is logical from fast to high accuracy.

        INPUT:
        - Function: ACCURACY_PRESETS dictionary access
        - Parameters: None (static analysis)
        - Input data: The ACCURACY_PRESETS dictionary

        EXPECTED OUTPUT:
        - Higher accuracy levels should have smaller deflections
        - Mesh simplification should decrease with level (less reduction)
        - Sample interval should decrease with level (denser sampling)
        - Ensures logical parameter progression from 1=Fastest to 7=Ultra
        """
        presets = PathPlanarSurface.ObjectSurface.ACCURACY_PRESETS

        # Higher accuracy levels should have smaller deflections
        for level in range(1, 7):
            current = presets[level]
            next_level = presets[level + 1]

            # Angular deflection should decrease (higher accuracy = smaller value)
            self.assertGreaterEqual(current["angular_deflection"], next_level["angular_deflection"])

            # Linear deflection should decrease (higher accuracy = smaller value)
            self.assertGreaterEqual(current["linear_deflection"], next_level["linear_deflection"])

            # Mesh simplification should decrease (less reduction for higher accuracy)
            self.assertGreaterEqual(
                current["mesh_simplification"], next_level["mesh_simplification"]
            )

            # Sample interval should decrease (denser sampling for higher accuracy)
            self.assertGreaterEqual(current["sample_interval"], next_level["sample_interval"])

    def test44(self):
        """
        Verifies that extreme presets have appropriate values.

        INPUT:
        - Function: ACCURACY_PRESETS dictionary access
        - Parameters: None (static analysis of levels 1 and 7)
        - Input data: Fastest (level 1) and Ultra (level 7) presets

        EXPECTED OUTPUT:
        - Level 1 should have fastest settings (maximum simplification, coarsest deflections)
        - Level 7 should have highest quality settings (no simplification, finest deflections)
        - Confirms extreme presets are properly configured for inverted scale
        """
        presets = PathPlanarSurface.ObjectSurface.ACCURACY_PRESETS

        # Level 1 should be fastest
        fastest = presets[1]
        self.assertEqual(fastest["mesh_simplification"], 7)  # Max reduction
        self.assertEqual(fastest["angular_deflection"], 0.5)  # Coarsest
        self.assertEqual(fastest["linear_deflection"], 0.1)  # Least precise
        self.assertEqual(fastest["sample_interval"], 1.5)  # Coarsest
        self.assertEqual(fastest["min_sample_interval"], 0.3)  # Coarsest

        # Level 7 should be highest quality
        ultra = presets[7]
        self.assertEqual(ultra["mesh_simplification"], 1)  # No reduction
        self.assertEqual(ultra["angular_deflection"], 0.05)  # Finest
        self.assertEqual(ultra["linear_deflection"], 0.005)  # Most precise
        self.assertEqual(ultra["sample_interval"], 0.05)  # Densest
        self.assertEqual(ultra["min_sample_interval"], 0.05)  # Densest

    # -- 3+2 workplane rotation tests --

    def _createRotatedJob(self, shape):
        """Helper: job for *shape* on a machine with a single table A axis.

        With Workplane=(0,-1,0) the solver yields A=-90 and the geometry
        transform maps (x, y, z) -> (x, z, -y): the model's -Y side becomes
        the Z-up top.
        """
        from Machine.models.machine import Machine, RotaryAxis, AxisRole

        model = self.doc.addObject("Part::Feature", "TestModel")
        model.Shape = shape
        job = PathJob.Create("Job", [model])
        toolbit = self.assets.get("toolbit://5mm_Endmill")
        loaded_tool = toolbit.attach_to_doc(doc=self.doc)
        job.Tools.Group[0].Tool = loaded_tool

        machine = Machine(name="Test A-axis Machine")
        machine.rotary_axes["A"] = RotaryAxis(
            name="A",
            rotation_vector=FreeCAD.Vector(1, 0, 0),
            min_limit=-120,
            max_limit=120,
            role=AxisRole.TABLE_ROTARY,
            parent=None,
            sequence=0,
        )
        job.Proxy.getMachine = lambda: machine
        self.doc.recompute()
        return job

    def _createRotatedOp(self, job, strategy):
        """Helper: PlanarSurface op on *job* with a -Y workplane, not yet executed.

        Execution is left to a document recompute: calling Proxy.execute()
        directly re-enters execute() via the base class's obj.recompute(),
        and the nested run strips the 3+2 transform state before the outer
        run emits its path.
        """
        op = FreeCAD.ActiveDocument.addObject("Path::FeaturePython", "PlanarSurface")
        proxy = PathPlanarSurface.ObjectSurface(op, "PlanarSurface")
        proxy.initOperation(op)
        op.Strategy = strategy
        op.Base = job.Model.Group
        # Workplane links a named plane on the Job; the test names it by tool axis.
        op.Workplane = PathWorkplane.createWorkplaneFromToolAxis(job, FreeCAD.Vector(0, -1, 0))
        job.Operations.addObject(op)
        return op

    @staticmethod
    def _rotaryMoves(op):
        """The rotary positioning the post would command for the op. The
        op's own path carries no rotary words - it is generated in its work
        plane's frame - so this solves, as the post does, from its Placement
        on the Job's machine."""
        import Path.Base.Generator.rotation as rotation
        import PathScripts.PathUtils as PathUtils

        machine = PathUtils.findParentJob(op).Proxy.getMachine()
        axis = op.Placement.Rotation.multVec(FreeCAD.Vector(0, 0, 1))
        result = rotation.solve_orientation(machine, axis)
        return [Path.Command("G0", dict(result.angles))] if result.success else []

    @staticmethod
    def _cutValues(op, axis):
        return [
            c.Parameters[axis] for c in op.Path.Commands if c.Name == "G1" and axis in c.Parameters
        ]

    @unittest.skipUnless(_ocl_available, "OpenCamLib not available")
    def test50(self):
        """
        Executes SurfaceScan with a non-Z-up Workplane and verifies the path is
        computed against the rotated geometry.

        INPUT:
        - Function: ObjectOp.execute() 3+2 setup -> ObjectSurface.opExecute()
        - Parameters: Workplane=(0,-1,0), Strategy=SurfaceScan, Line pattern
        - Input data: 50x40x30 box on a single-A-axis machine; rotated the box
          spans Y 0..30 and Z -40..0 with its top at Z=0

        EXPECTED OUTPUT:
        - The path contains the rotary positioning move G0 A-90
        - Cutting moves sit on the rotated top face (Z ~ 0), not the unrotated
          top (Z = 30)
        - Cutting moves stay within the rotated Y extent (0..30), never
          reaching the unrotated extent (up to 40)
        """
        job = self._createRotatedJob(Part.makeBox(50, 40, 30))
        op = self._createRotatedOp(job, "SurfaceScan")
        op.CutPattern = "Line"
        op.StepOver = 50.0
        op.SampleInterval = 5.0
        self.doc.recompute()

        rotary = self._rotaryMoves(op)
        self.assertTrue(rotary, "Path should contain a rotary positioning move")
        self.assertAlmostEqual(rotary[0].Parameters["A"], -90.0, places=3)

        cut_z = self._cutValues(op, "Z")
        self.assertTrue(cut_z, "SurfaceScan should produce cutting moves")
        self.assertLess(max(cut_z), 10.0, "Cuts must not sit on the unrotated top (Z=30)")
        self.assertGreater(min(cut_z), -10.0, "Cuts must stay near the rotated top (Z=0)")

        cut_y = self._cutValues(op, "Y")
        self.assertTrue(cut_y)
        self.assertLessEqual(max(cut_y), 30.0 + 0.1, "Cuts must respect the rotated Y extent")

    def test51(self):
        """
        Executes ZLevelHybrid with a non-Z-up Workplane and
        verifies depths and paths are in the rotated frame.

        INPUT:
        - Function: ObjectOp.execute() 3+2 setup -> ObjectSurface.opExecute()
        - Parameters: Workplane=(0,-1,0), Strategy=ZLevelHybrid, BoundBox=Stock
        - Input data: 15mm sphere centred at (25, 0, 15) on a single-A-axis
          machine; unrotated it spans Z 0..30, rotated it spans Z -15..15

        EXPECTED OUTPUT:
        - The path contains the rotary positioning move G0 A-90
        - OpFinalDepth is the rotated model floor (-15), not the unrotated one (0)
        - Cutting moves descend below Z=0, impossible on the unrotated model
        """
        job = self._createRotatedJob(Part.makeSphere(15, FreeCAD.Vector(25, 0, 15)))
        op = self._createRotatedOp(job, "ZLevelHybrid")
        op.BoundBox = "Stock"
        self.doc.recompute()

        rotary = self._rotaryMoves(op)
        self.assertTrue(rotary, "Path should contain a rotary positioning move")
        self.assertAlmostEqual(rotary[0].Parameters["A"], -90.0, places=3)

        self.assertAlmostEqual(op.OpFinalDepth.Value, -15.0, places=3)

        cut_z = self._cutValues(op, "Z")
        self.assertTrue(cut_z, "ZLevelHybrid should produce cutting moves")
        self.assertLess(max(cut_z), 16.0, "Cuts must not use the unrotated Z range (0..30)")
        self.assertLess(min(cut_z), -5.0, "Cuts must descend into the rotated Z range (-15..15)")

    # -- Secondary feature lists (AvoidGeometry, FillHoleFaces) --

    def _createOp(self, job, strategy):
        """Helper: PlanarSurface op added to *job*, not yet executed."""
        op = FreeCAD.ActiveDocument.addObject("Path::FeaturePython", "PlanarSurface")
        proxy = PathPlanarSurface.ObjectSurface(op, "PlanarSurface")
        proxy.initOperation(op)
        op.Strategy = strategy
        job.Operations.addObject(op)
        return op

    @staticmethod
    def _faceName(shape, predicate):
        for i, face in enumerate(shape.Faces):
            if predicate(face):
                return f"Face{i + 1}"
        return None

    @staticmethod
    def _cutPoints(op):
        """(x, y, z) of every G1 move, with modal X/Y/Z carried forward."""
        points, pos = [], {"X": None, "Y": None, "Z": None}
        for cmd in op.Path.Commands:
            for axis in pos:
                if axis in cmd.Parameters:
                    pos[axis] = cmd.Parameters[axis]
            if cmd.Name == "G1" and None not in pos.values():
                points.append((pos["X"], pos["Y"], pos["Z"]))
        return points

    @staticmethod
    def _cutSegments(op):
        """(start, end) of every horizontal G1 move, with modal X/Y/Z carried forward."""
        segments, pos, prev = [], {"X": None, "Y": None, "Z": None}, None
        for cmd in op.Path.Commands:
            for axis in pos:
                if axis in cmd.Parameters:
                    pos[axis] = cmd.Parameters[axis]
            if None in pos.values():
                continue
            cur = (pos["X"], pos["Y"], pos["Z"])
            if cmd.Name == "G1" and prev is not None and abs(prev[2] - cur[2]) < 1e-6:
                segments.append((prev, cur))
            prev = cur
        return segments

    @staticmethod
    def _arcPoints(op, step_deg=5.0):
        """(x, y, z) sampled along every G2 / G3 move (I / J relative to the start)."""
        points, pos = [], {"X": None, "Y": None, "Z": None}
        for cmd in op.Path.Commands:
            start = dict(pos)
            for axis in pos:
                if axis in cmd.Parameters:
                    pos[axis] = cmd.Parameters[axis]
            if cmd.Name not in ("G2", "G3") or None in start.values():
                continue
            cx = start["X"] + cmd.Parameters.get("I", 0.0)
            cy = start["Y"] + cmd.Parameters.get("J", 0.0)
            radius = math.hypot(start["X"] - cx, start["Y"] - cy)
            a0 = math.atan2(start["Y"] - cy, start["X"] - cx)
            a1 = math.atan2(pos["Y"] - cy, pos["X"] - cx)
            sweep = (a1 - a0) % (2 * math.pi)
            if cmd.Name == "G2":
                sweep -= 2 * math.pi  # clockwise
            n = max(1, int(abs(math.degrees(sweep)) / step_deg))
            for k in range(n + 1):
                ang = a0 + sweep * k / n
                points.append((cx + radius * math.cos(ang), cy + radius * math.sin(ang), pos["Z"]))
        return points

    def test60(self):
        """
        Verifies the face-list property schema.

        INPUT:
        - A new PlanarSurface operation, switched through all strategies.

        EXPECTED OUTPUT:
        - AvoidGeometry and FillHoleFaces exist as link-sub lists, empty by default.
        - The order-based AvoidLastX_Faces and the FillSelectedHoles flag are gone.
        - AvoidGeometry is shown for every strategy, FillHoleFaces for ZLevelHybrid and
          SurfaceScan.
        - AvoidGeometryOverlap is read-only while AvoidGeometry is empty.
        """
        job = self._createJobWithBox()
        op = self._createOp(job, "SurfaceScan")

        for prop in ("AvoidGeometry", "FillHoleFaces"):
            self.assertEqual(op.getTypeIdOfProperty(prop), "App::PropertyLinkSubListGlobal")
            self.assertEqual(len(getattr(op, prop)), 0)
        self.assertFalse(hasattr(op, "AvoidLastX_Faces"))
        self.assertFalse(hasattr(op, "FillSelectedHoles"))

        expected = {
            "SurfaceScan": (True, True),
            "Waterline": (True, False),
            "ZLevelHybrid": (True, True),
        }
        for strategy, (avoid_shown, fill_shown) in expected.items():
            op.Strategy = strategy
            self.assertEqual(
                "Hidden" not in op.getEditorMode("AvoidGeometry"), avoid_shown, strategy
            )
            self.assertEqual(
                "Hidden" not in op.getEditorMode("FillHoleFaces"), fill_shown, strategy
            )

        # Avoid Geometry Overlap is read-only until there is something to avoid
        op.Strategy = "SurfaceScan"
        self.assertIn("ReadOnly", op.getEditorMode("AvoidGeometryOverlap"))
        op.Proxy.addFeature(op, "AvoidGeometry", self.doc.getObject("TestBox"), "Face6")
        self.assertNotIn("ReadOnly", op.getEditorMode("AvoidGeometryOverlap"))
        self.assertNotIn("Hidden", op.getEditorMode("AvoidGeometryOverlap"))
        op.Strategy = "Waterline"
        self.assertNotIn("Hidden", op.getEditorMode("AvoidGeometryOverlap"))

    def test61(self):
        """
        Verifies addFeature() and featureShapes().

        INPUT:
        - Function: ObjectSurface.addFeature(), ObjectSurface.featureShapes()
        - A face picked on the user's body (not the job's model clone).

        EXPECTED OUTPUT:
        - The face is stored against the job's model clone, like addBase() does.
        - Adding the same face twice keeps a single entry; Base is untouched.
        - An unknown property name is rejected.
        - With a 3+2 transform active, featureShapes() yields transformed faces.
        """
        job = self._createJobWithBox()
        body = self.doc.getObject("TestBox")
        clone = job.Model.Group[0]
        op = self._createOp(job, "SurfaceScan")

        self.assertTrue(op.Proxy.addFeature(op, "AvoidGeometry", body, "Face6"))
        self.assertFalse(op.Proxy.addFeature(op, "AvoidGeometry", body, "Face6"))
        self.assertEqual(len(op.AvoidGeometry), 1)
        linked, subs = op.AvoidGeometry[0]
        self.assertEqual(linked, clone, "Selection must map to the job's model clone")
        self.assertEqual(tuple(subs), ("Face6",))
        self.assertEqual(len(op.Base), 0, "Base must not be modified")

        with self.assertRaises(ValueError):
            op.Proxy.addFeature(op, "Base", body, "Face1")

        top_z = clone.Shape.getElement("Face6").BoundBox.ZMax
        matrix = FreeCAD.Matrix()
        matrix.rotateX(3.141592653589793)  # flips Z
        op.Proxy._geom_transform_matrix = matrix
        try:
            ((proxy_obj, proxy_subs),) = op.Proxy.featureShapes(op, "AvoidGeometry")
            face = proxy_obj.Shape.getElement(proxy_subs[0])
            self.assertAlmostEqual(face.BoundBox.ZMax, -top_z, places=6)
        finally:
            op.Proxy._geom_transform_matrix = None

    @unittest.skipUnless(_ocl_available, "OpenCamLib not available")
    def test62(self):
        """
        Executes SurfaceScan with AvoidGeometry and an empty Base.

        INPUT:
        - 100x100x10 slab with a 30x30x10 boss at the origin corner.
        - Base empty (whole model); AvoidGeometry = the boss top face.

        EXPECTED OUTPUT:
        - Without AvoidGeometry the boss top is cut.
        - With AvoidGeometry no cutting move reaches the boss top or its interior.
        """
        shape = (
            Part.makeBox(100, 100, 10)
            .fuse(Part.makeBox(30, 30, 10, FreeCAD.Vector(0, 0, 10)))
            .removeSplitter()
        )
        model = self.doc.addObject("Part::Feature", "TestBoss")
        model.Shape = shape
        job = PathJob.Create("Job", [model])
        job.Tools.Group[0].Tool = self.assets.get("toolbit://5mm_Endmill").attach_to_doc(
            doc=self.doc
        )
        self.doc.recompute()
        boss_top = self._faceName(
            job.Model.Group[0].Shape, lambda f: abs(f.BoundBox.ZMin - 20) < 1e-6
        )
        self.assertIsNotNone(boss_top)

        def run(avoid):
            op = self._createOp(job, "SurfaceScan")
            op.CutPattern = "Line"
            op.SampleInterval = 2.0
            if avoid:
                op.Proxy.addFeature(op, "AvoidGeometry", model, boss_top)
            self.doc.recompute()
            return [p for p in self._cutPoints(op) if p[0] < 25 and p[1] < 25]

        self.assertTrue(run(avoid=False), "Sanity: the boss top is cut when not avoided")
        self.assertEqual(run(avoid=True), [], "No cuts may land on the avoided face")

    def test63(self):
        """
        Executes ZLevelHybrid with FillHoleFaces.

        INPUT:
        - 100x100x5 slab with a 40x40x15 boss; an 8 mm radius, 8 mm deep
          blind hole in the boss top. Offset pattern, Stock boundary.
        - FillHoleFaces = the hole wall; Base empty.

        EXPECTED OUTPUT:
        - The fill list changes the toolpath and removes cuts inside the hole.
        - The fill faces do not drive OpFinalDepth (only Base does).
        """
        boss = Part.makeBox(40, 40, 15, FreeCAD.Vector(30, 30, 5))
        shape = (
            Part.makeBox(100, 100, 5)
            .fuse(boss)
            .removeSplitter()
            .cut(Part.makeCylinder(8, 8, FreeCAD.Vector(50, 50, 12)))
        )
        model = self.doc.addObject("Part::Feature", "TestHole")
        model.Shape = shape
        job = PathJob.Create("Job", [model])
        job.Tools.Group[0].Tool = self.assets.get("toolbit://5mm_Endmill").attach_to_doc(
            doc=self.doc
        )
        self.doc.recompute()
        wall = self._faceName(
            job.Model.Group[0].Shape, lambda f: f.Surface.__class__.__name__ == "Cylinder"
        )
        self.assertIsNotNone(wall)

        def run(fill):
            op = self._createOp(job, "ZLevelHybrid")
            op.BoundBox = "Stock"
            op.CutPatternZLevel = "Offset"
            op.StepDown = 2
            if fill:
                op.Proxy.addFeature(op, "FillHoleFaces", model, wall)
            self.doc.recompute()
            inside = [p for p in self._cutPoints(op) if (p[0] - 50) ** 2 + (p[1] - 50) ** 2 < 6**2]
            return op, inside

        plain, plain_inside = run(fill=False)
        filled, filled_inside = run(fill=True)

        self.assertNotEqual(
            [c.toGCode() for c in plain.Path.Commands],
            [c.toGCode() for c in filled.Path.Commands],
            "FillHoleFaces must change the toolpath",
        )
        self.assertLess(len(filled_inside), len(plain_inside))
        self.assertAlmostEqual(plain.OpFinalDepth.Value, filled.OpFinalDepth.Value, places=6)

    def _createTwoPocketJob(self):
        """Helper: 100x60x20 block with two 30x40x10 pockets (A: x 10..40, B: x 60..90)
        and a 10x10x6 boss in the centre of pocket A."""
        shape = (
            Part.makeBox(100, 60, 20)
            .cut(Part.makeBox(30, 40, 10, FreeCAD.Vector(10, 10, 10)))
            .cut(Part.makeBox(30, 40, 10, FreeCAD.Vector(60, 10, 10)))
            .fuse(Part.makeBox(10, 10, 6, FreeCAD.Vector(20, 25, 10)))
            .removeSplitter()
        )
        model = self.doc.addObject("Part::Feature", "TestPockets")
        model.Shape = shape
        job = PathJob.Create("Job", [model])
        job.Tools.Group[0].Tool = self.assets.get("toolbit://5mm_Endmill").attach_to_doc(
            doc=self.doc
        )
        self.doc.recompute()
        return job, model

    def _runZLevel(self, job, base=(), avoid=()):
        op = self._createOp(job, "ZLevelHybrid")
        op.CutPatternZLevel = "Offset"
        op.BoundBox = "Stock"
        op.StepDown = 3
        clone = job.Model.Group[0]
        if base:
            op.Base = [(clone, tuple(base))]
        if avoid:
            op.AvoidGeometry = [(clone, tuple(avoid))]
        self.doc.recompute()
        return op, self._cutPoints(op)

    def test64(self):
        """
        ZLevelHybrid honours the Base Geometry selection.

        INPUT:
        - Two-pocket block; Base = the floor face of pocket A only.

        EXPECTED OUTPUT:
        - Without a selection both pockets are cut.
        - With pocket A's floor selected, every cutting move stays in pocket A.
        """
        job, _ = self._createTwoPocketJob()
        clone = job.Model.Group[0]
        floor_a = self._faceName(
            clone.Shape,
            lambda f: abs(f.BoundBox.ZMin - 10) < 1e-6
            and abs(f.BoundBox.ZMax - 10) < 1e-6
            and f.BoundBox.XMax < 50,
        )
        self.assertIsNotNone(floor_a)

        _, plain = self._runZLevel(job)
        self.assertTrue(
            any(p[0] > 55 for p in plain), "Sanity: pocket B is cut without a selection"
        )

        _, selected = self._runZLevel(job, base=[floor_a])
        self.assertTrue(selected, "Pocket A must still be machined")
        self.assertTrue(all(10 <= p[0] <= 40 and 10 <= p[1] <= 50 for p in selected))

    def test65(self):
        """
        ZLevelHybrid honours AvoidGeometry.

        INPUT:
        - Two-pocket block; AvoidGeometry = the boss top in pocket A; Base empty.

        EXPECTED OUTPUT:
        - Without AvoidGeometry the tool passes over the boss (Z16 clearing).
        - With it, no cutting move enters the boss footprint; both pockets are still cut.
        """
        job, _ = self._createTwoPocketJob()
        boss_top = self._faceName(
            job.Model.Group[0].Shape,
            lambda f: abs(f.BoundBox.ZMin - 16) < 1e-6 and abs(f.BoundBox.ZMax - 16) < 1e-6,
        )
        self.assertIsNotNone(boss_top)

        def over_boss(op):
            # Sample along each cutting move: loop corners alone can miss the boss.
            hits = []
            for a, b in self._cutSegments(op):
                n = max(1, int(((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5 / 0.5))
                for t in range(n + 1):
                    x, y = a[0] + (b[0] - a[0]) * t / n, a[1] + (b[1] - a[1]) * t / n
                    if 20.5 < x < 29.5 and 25.5 < y < 34.5:
                        hits.append((x, y, a[2]))
            return hits

        plain_op, _ = self._runZLevel(job)
        self.assertTrue(over_boss(plain_op), "Sanity: the boss top is passed over when not avoided")

        avoid_op, avoided = self._runZLevel(job, avoid=[boss_top])
        self.assertEqual(over_boss(avoid_op), [])
        self.assertTrue(any(p[0] > 55 for p in avoided), "Pocket B must still be machined")
        self.assertTrue(any(p[0] < 45 for p in avoided), "Pocket A must still be machined")

    @unittest.skipUnless(_ocl_available, "OpenCamLib not available")
    def test66(self):
        """
        An STL without triangles never reaches OpenCamLib.

        OCL aborts the whole application, with no Python exception, when one of
        its algorithms gets an empty mesh.

        INPUT:
        - surface_mesh.generate_stl() patched to return empty meshes.
        - Waterline and SurfaceScan operations on a simple box.

        EXPECTED OUTPUT:
        - Both operations finish (no crash) and produce no cutting moves.
        """
        import Path.Base.Generator.surface_common as surface_common
        import Path.Base.Generator.surface_mesh as surface_mesh

        job = self._createJobWithBox()
        ocl_module = surface_common.get_ocl()
        original = surface_mesh.generate_stl
        surface_mesh.generate_stl = lambda *a, **k: (ocl_module.STLSurf(), ocl_module.STLSurf())
        try:
            for strategy in ("Waterline", "SurfaceScan"):
                op = self._createOp(job, strategy)
                self.doc.recompute()
                self.assertFalse(
                    [c for c in op.Path.Commands if c.Name == "G1"], f"{strategy}: no cuts"
                )
        finally:
            surface_mesh.generate_stl = original

    # -- Waterline Base / Avoid selection --

    def _createWaterlineJob(self, shape):
        model = self.doc.addObject("Part::Feature", "TestWaterline")
        model.Shape = shape
        job = PathJob.Create("Job", [model])
        job.Tools.Group[0].Tool = self.assets.get("toolbit://5mm_Endmill").attach_to_doc(
            doc=self.doc
        )
        self.doc.recompute()
        return job

    def _runWaterline(self, job, base=(), avoid=(), leads=False):
        op = self._createOp(job, "Waterline")
        op.SampleInterval = 1.0
        op.StepDown = 3
        op.LeadInOut = leads
        clone = job.Model.Group[0]
        if base:
            op.Base = [(clone, tuple(base))]
        if avoid:
            op.AvoidGeometry = [(clone, tuple(avoid))]
        self.doc.recompute()
        return op

    @staticmethod
    def _sampled(segments, step=0.25):
        for a, b in segments:
            n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
            for t in range(n + 1):
                yield (a[0] + (b[0] - a[0]) * t / n, a[1] + (b[1] - a[1]) * t / n, a[2])

    @unittest.skipUnless(_ocl_available, "OpenCamLib not available")
    def test67(self):
        """
        Waterline honours the Base Geometry selection.

        INPUT:
        - 100x80x5 slab with a r12 cylinder boss at (30, 40) and a 20x30 box boss.
        - Base = the cylinder's faces.

        EXPECTED OUTPUT:
        - Without a selection the box boss and the slab outline are contoured.
        - With the cylinder selected only its contours remain: every cut within
          the cylinder radius + tool radius (+ tolerance), and the cylinder still cut.
        """
        shape = (
            Part.makeBox(100, 80, 5)
            .fuse(Part.makeCylinder(12, 15, FreeCAD.Vector(30, 40, 5)))
            .fuse(Part.makeBox(20, 30, 10, FreeCAD.Vector(60, 25, 5)))
            .removeSplitter()
        )
        job = self._createWaterlineJob(shape)
        clone = job.Model.Group[0]
        cylinder = [
            f"Face{i + 1}"
            for i, f in enumerate(clone.Shape.Faces)
            if f.BoundBox.XMax < 45 and f.BoundBox.ZMax > 6
        ]
        self.assertTrue(cylinder)

        plain = list(self._sampled(self._cutSegments(self._runWaterline(job))))
        self.assertTrue(any(p[0] > 55 for p in plain), "Sanity: the box boss is contoured")

        selected = list(self._sampled(self._cutSegments(self._runWaterline(job, base=cylinder))))
        self.assertTrue(selected, "The cylinder must still be contoured")
        radii = [math.hypot(p[0] - 30, p[1] - 40) for p in selected]
        self.assertLess(max(radii), 12 + 2.5 + 0.1, "Every cut stays around the cylinder")
        self.assertGreater(max(radii), 12 + 2.5 - 0.1, "The cylinder wall contour is kept")

    @unittest.skipUnless(_ocl_available, "OpenCamLib not available")
    def test68(self):
        """
        Waterline AvoidGeometry keep the tool off the avoided faces, not off their walls.

        INPUT:
        - 120x60x5 slab with two 20x30x10 boxes 5 mm apart (A: x 20..40, B: x 45..65).
        - AvoidGeometry = the top of box B. Its keep-out zone is B + r, less the
          contour margin (0.1 mm): contours that only touch B's walls stay.

        EXPECTED OUTPUT:
        - No cutting move (sampled along every move) comes closer to B than
          r - 0.1: nothing reaches over B's top.
        - B's own wall contour is kept, and so is A's contour in the 5 mm slot
          (it touches B's wall, a tool radius off it).
        - Box A is still contoured on its far side (x < 20), the slab outline too.
        """
        shape = (
            Part.makeBox(120, 60, 5)
            .fuse(Part.makeBox(20, 30, 10, FreeCAD.Vector(20, 15, 5)))
            .fuse(Part.makeBox(20, 30, 10, FreeCAD.Vector(45, 15, 5)))
            .removeSplitter()
        )
        job = self._createWaterlineJob(shape)
        top_b = self._faceName(
            job.Model.Group[0].Shape,
            lambda f: abs(f.BoundBox.ZMin - 15) < 1e-6 and f.BoundBox.XMin > 44,
        )
        self.assertIsNotNone(top_b)

        def dist_b(p):
            # Distance to box B's footprint (its zone has rounded corners)
            dx = max(45 - p[0], 0, p[0] - 65)
            dy = max(15 - p[1], 0, p[1] - 45)
            return math.hypot(dx, dy)

        avoided = list(self._sampled(self._cutSegments(self._runWaterline(job, avoid=[top_b]))))
        self.assertEqual([p for p in avoided if dist_b(p) < 2.5 - 0.1], [], "Nothing over B")
        self.assertTrue(
            any(abs(dist_b(p) - 2.5) < 0.1 and p[0] > 66 for p in avoided), "B's own contour"
        )
        self.assertTrue(any(abs(p[0] - 42.5) < 0.1 for p in avoided), "A's contour in the slot")
        self.assertTrue(any(17 < p[0] < 18 and 15 < p[1] < 45 for p in avoided), "A's far side")
        self.assertTrue(any(p[0] < 0 for p in avoided), "The slab outline is kept")

    @unittest.skipUnless(_ocl_available, "OpenCamLib not available")
    def test69(self):
        """
        Waterline trimming handles long straight contour segments.

        With Optimize Linear Paths, the slab's outer contour along its front edge
        is one long segment whose two ends lie far from the selected area.

        INPUT:
        - 120x60x5 slab with a 10x10x2 step on its front edge (x 50..60,
          y 0..10, top Z 7) and a 30x18x15 boss behind it (x 40..70, y 12..30).
        - Case A: AvoidGeometry = the step top. Above Z 7 the boss's front contour
          (y = 9.5) is one long segment running straight over the step, with
          both its ends outside the step's zone.
        - Case B: Base = the step top. The slab's front contour (y = -2.5) runs
          through the selected area with both its ends outside it.

        EXPECTED OUTPUT:
        - A: no cutting move on the boss's front contour over the step's zone
          (step + r - 0.1), the rest of that contour still cut.
        - B: the part of the front contour inside the area is cut.
        """
        shape = (
            Part.makeBox(120, 60, 5)
            .fuse(Part.makeBox(10, 10, 2, FreeCAD.Vector(50, 0, 5)))
            .fuse(Part.makeBox(30, 18, 15, FreeCAD.Vector(40, 12, 5)))
            .removeSplitter()
        )
        job = self._createWaterlineJob(shape)
        step_top = self._faceName(
            job.Model.Group[0].Shape,
            lambda f: abs(f.BoundBox.ZMin - 7) < 1e-6 and abs(f.BoundBox.ZMax - 7) < 1e-6,
        )
        self.assertIsNotNone(step_top)
        front = lambda pts: [p for p in pts if abs(p[1] + 2.5) < 0.05]
        boss_front = lambda pts: [p for p in pts if abs(p[1] - 9.5) < 0.05 and p[2] > 7.5]

        avoided = boss_front(
            self._sampled(self._cutSegments(self._runWaterline(job, avoid=[step_top])))
        )
        # The zone spans x 47.6..62.4 along y = 9.5 (inside the step's y range);
        # a cut ends on its edge, allowed a hair of rounding
        self.assertEqual([p for p in avoided if 47.62 < p[0] < 62.38], [], "Through the zone")
        self.assertTrue(any(p[0] < 45 for p in avoided) and any(p[0] > 65 for p in avoided))

        # Base does not raise Final Depth (see test70): the slab levels are cut
        selected = front(self._sampled(self._cutSegments(self._runWaterline(job, base=[step_top]))))
        self.assertTrue(any(50 < p[0] < 60 for p in selected), "Front contour inside the area")

    @unittest.skipUnless(_ocl_available, "OpenCamLib not available")
    def test70(self):
        """
        Waterline's Final Depth ignores the Base selection.

        INPUT:
        - 120x60x5 slab with a 10x10x2 step (top at Z 7).
        - Base = the step top, for Waterline and for Surface Scan.

        EXPECTED OUTPUT:
        - Waterline: Final Depth is the model bottom (0), as with nothing selected.
        - Surface Scan: Final Depth is the lowest selected face (7), unchanged.
        - Changing the Strategy to Waterline brings Final Depth back down.
        """
        shape = (
            Part.makeBox(120, 60, 5)
            .fuse(Part.makeBox(10, 10, 2, FreeCAD.Vector(50, 0, 5)))
            .removeSplitter()
        )
        job = self._createWaterlineJob(shape)
        step_top = self._faceName(
            job.Model.Group[0].Shape,
            lambda f: abs(f.BoundBox.ZMin - 7) < 1e-6 and abs(f.BoundBox.ZMax - 7) < 1e-6,
        )
        self.assertIsNotNone(step_top)

        waterline = self._runWaterline(job, base=[step_top])
        self.assertAlmostEqual(waterline.OpFinalDepth.Value, 0.0, places=6)
        self.assertAlmostEqual(waterline.FinalDepth.Value, 0.0, places=6)

        scan = self._createOp(job, "SurfaceScan")
        scan.Base = [(job.Model.Group[0], (step_top,))]
        self.doc.recompute()
        self.assertAlmostEqual(scan.OpFinalDepth.Value, 7.0, places=6)

        scan.Strategy = "Waterline"
        self.doc.recompute()
        self.assertAlmostEqual(scan.OpFinalDepth.Value, 0.0, places=6)

    @unittest.skipUnless(_ocl_available, "OpenCamLib not available")
    def test71(self):
        """
        Waterline lead arcs stay near the Base selection and out of Avoid zones.

        INPUT:
        - test67's model: slab, a r12 cylinder boss at (30, 40), a 20x30 box boss
          (x 60..80, y 25..55). 5 mm end mill (r = 2.5).
        - Lead-in/out on. Case A: Base = the cylinder. Case B:
          AvoidGeometry = the box boss top (no Base): contours around the box are
          cut open at its zone, and their leads attach there.

        EXPECTED OUTPUT:
        - Lead arcs are generated.
        - A: every arc point lies within the selection's tool-centre region
          (cylinder + r) plus the lead margin (1.5 * r). An arc of radius r
          attached inside the region reaches at most sqrt(2) * r beyond it.
        - B: no cutting move or arc point comes closer to the box boss than
          r - 0.1 (its zone, less the contour margin that keeps its own contour).
        """
        shape = (
            Part.makeBox(100, 80, 5)
            .fuse(Part.makeCylinder(12, 15, FreeCAD.Vector(30, 40, 5)))
            .fuse(Part.makeBox(20, 30, 10, FreeCAD.Vector(60, 25, 5)))
            .removeSplitter()
        )
        job = self._createWaterlineJob(shape)
        clone = job.Model.Group[0]
        cylinder = [
            f"Face{i + 1}"
            for i, f in enumerate(clone.Shape.Faces)
            if f.BoundBox.XMax < 45 and f.BoundBox.ZMax > 6
        ]
        box_top = self._faceName(
            clone.Shape,
            lambda f: abs(f.BoundBox.ZMin - 15) < 1e-6 and f.BoundBox.XMin > 59,
        )
        self.assertTrue(cylinder)
        self.assertIsNotNone(box_top)

        r = 2.5
        op = self._runWaterline(job, base=cylinder, leads=True)
        arcs = self._arcPoints(op)
        self.assertTrue(arcs, "Lead arcs are generated")
        reach = max(math.hypot(p[0] - 30, p[1] - 40) for p in arcs)
        self.assertLess(reach, 12 + r + 1.5 * r + 0.05, "Arcs within the lead margin")

        def in_box_zone(p, margin=0.1):
            dx = max(60 - p[0], 0, p[0] - 80)
            dy = max(25 - p[1], 0, p[1] - 55)
            return math.hypot(dx, dy) < r - margin

        op = self._runWaterline(job, avoid=[box_top], leads=True)
        moves = list(self._sampled(self._cutSegments(op))) + self._arcPoints(op)
        self.assertTrue(moves)
        self.assertEqual([p for p in moves if in_box_zone(p)], [], "Nothing in the Avoid zone")

    @staticmethod
    def _regionSig(shape):
        if shape is None:
            return None
        bb = shape.BoundBox
        return (
            round(shape.Area, 3),
            tuple(round(v, 3) for v in (bb.XMin, bb.XMax, bb.YMin, bb.YMax)),
        )

    def _executeBoundaries(self, op):
        """The Boundaries opExecute() built in the last recompute of *op*."""
        built = []
        proxy = op.Proxy
        original = proxy.buildBoundaries

        def capture(*args, **kwargs):
            result = original(*args, **kwargs)
            built.append(result)
            return result

        proxy.buildBoundaries = capture
        try:
            op.touch()
            self.doc.recompute()
        finally:
            del proxy.buildBoundaries
        self.assertTrue(built, "opExecute must build the boundaries")
        return built[-1]

    def test72(self):
        """
        The boundary preview builds the same regions as opExecute().

        INPUT:
        - Flat: 100x80x5 slab with a r12 cylinder boss and a 20x30x10 box boss.
          Z-Level (no OCL needed) with Base = the slab top and Avoid = the box
          top, then nothing selected.
        - 3+2: a 50x40x30 box on an A-axis machine, Workplane (0, -1, 0),
          Base = all its faces.

        EXPECTED OUTPUT:
        - previewRegions() draws the outline, the tool-centre region and the
          Avoid zones opExecute() built: same areas and bounds, in the same
          (work plane) frame. With nothing selected the outline is the region
          itself and is drawn once.
        - The op's frame state is left as it was.
        """
        V = FreeCAD.Vector
        shape = (
            Part.makeBox(100, 80, 5)
            .fuse(Part.makeCylinder(12, 15, V(30, 40, 5)))
            .fuse(Part.makeBox(20, 30, 10, V(60, 25, 5)))
            .removeSplitter()
        )
        job = self._createWaterlineJob(shape)
        clone = job.Model.Group[0]
        slab_top = self._faceName(
            clone.Shape,
            lambda f: abs(f.BoundBox.ZMin - 5) < 1e-6 and abs(f.BoundBox.ZMax - 5) < 1e-6,
        )
        box_top = self._faceName(
            clone.Shape, lambda f: abs(f.BoundBox.ZMin - 15) < 1e-6 and f.BoundBox.XMin > 59
        )

        def same(op):
            built = self._executeBoundaries(op)
            drawn = {role: shape for shape, role in op.Proxy.previewRegions(op)}
            expected = {"region": built.boundary_face, "keepout": built.avoid_boundary}
            if built.bb_face is not built.boundary_face:
                expected["outline"] = built.bb_face
            expected = {role: shape for role, shape in expected.items() if shape is not None}
            self.assertEqual(sorted(drawn), sorted(expected))
            for role, shape in expected.items():
                self.assertEqual(self._regionSig(drawn[role]), self._regionSig(shape), role)
            self.assertFalse(hasattr(op.Proxy, "_geom_transform_matrix"))
            return drawn

        op = self._createOp(job, "ZLevelHybrid")
        op.StepDown = 3
        op.Base = [(clone, (slab_top,))]
        op.AvoidGeometry = [(clone, (box_top,))]
        self.assertIn("keepout", same(op))
        op.Base = []
        op.AvoidGeometry = []
        self.assertNotIn("outline", same(op))

        # 3+2: the preview is in the work plane frame, like the run
        self.doc = FreeCAD.newDocument("test72_rotated")
        FreeCAD.setActiveDocument(self.doc.Name)
        rjob = self._createRotatedJob(Part.makeBox(50, 40, 30))
        rop = self._createRotatedOp(rjob, "ZLevelHybrid")
        rop.StepDown = 5
        rotated = same(rop)
        bb = rotated["region"].BoundBox
        # Rotated, the box spans Y 0..30 (its Z): never the unrotated 0..40
        self.assertLess(bb.YMax, 31.0)
        import Path.Base.Util as PathUtil

        self.assertFalse(PathUtil.workplaneForOp(rop).isIdentity())

    def test73(self):
        """
        BoundaryOutline: a Surface Scan property, off by default.

        INPUT:
        - A new PlanarSurface operation, switched through all strategies.

        EXPECTED OUTPUT:
        - BoundaryOutline exists, is a bool and is off.
        - It is shown for Surface Scan only.
        """
        job = self._createWaterlineJob(Part.makeBox(20, 20, 5))
        op = self._createOp(job, "SurfaceScan")
        self.assertIn("BoundaryOutline", op.PropertiesList)
        self.assertIs(op.BoundaryOutline, False)
        for strategy, shown in (
            ("SurfaceScan", True),
            ("ZLevelHybrid", False),
            ("Waterline", False),
        ):
            op.Strategy = strategy
            hidden = "Hidden" in op.getEditorMode("BoundaryOutline")
            self.assertEqual(not hidden, shown, strategy)

    def _edgeNames(self, shape, edges):
        """Edge names of *shape* for the given edges (matched with isSame)."""
        return tuple(
            f"Edge{i + 1}"
            for i, e in enumerate(shape.Edges)
            if any(e.isSame(other) for other in edges)
        )

    @unittest.skipUnless(_ocl_available, "OpenCamLib not available")
    def test74(self):
        """
        Edge loops in Base and Avoid: depth and a full Surface Scan run.

        INPUT:
        - 100x80x5 slab with a 20x30x10 box on top (box top at Z 15).
        - Base = the box top's rim edges; then Base = the slab top's outer rim
          edges with Avoid = the box top's rim edges.

        EXPECTED OUTPUT:
        - Edges do not set Final Depth: with only edges selected it is the
          model bottom (0), as with nothing selected; the box top face alone
          would give 15.
        - Base = box rim: every cut lies inside the box footprint.
        - Base = slab rim, Avoid = box rim: cuts are made, none enters the box
          footprint grown by the tool radius.
        """
        V = FreeCAD.Vector
        shape = (
            Part.makeBox(100, 80, 5).fuse(Part.makeBox(20, 30, 10, V(60, 25, 5))).removeSplitter()
        )
        job = self._createWaterlineJob(shape)
        clone = job.Model.Group[0]
        flat = lambda z: next(
            f
            for f in clone.Shape.Faces
            if abs(f.BoundBox.ZMin - z) < 1e-6 and abs(f.BoundBox.ZMax - z) < 1e-6
        )
        box_rim = self._edgeNames(clone.Shape, flat(15).OuterWire.Edges)
        slab_rim = self._edgeNames(clone.Shape, flat(5).OuterWire.Edges)
        box_top = self._faceName(clone.Shape, lambda f: f.isSame(flat(15)))
        self.assertEqual((len(box_rim), len(slab_rim)), (4, 4))

        op = self._createOp(job, "SurfaceScan")
        op.SampleInterval = 1.0
        op.Base = [(clone, box_top)]
        self.doc.recompute()
        self.assertAlmostEqual(op.OpFinalDepth.Value, 15.0, places=6)

        op.Base = [(clone, box_rim)]
        self.doc.recompute()
        self.assertAlmostEqual(op.OpFinalDepth.Value, 0.0, places=6)
        cuts = self._cutPoints(op)
        self.assertTrue(cuts, "The box footprint is machined")
        for x, y, _ in cuts:
            self.assertTrue(60 - 0.05 <= x <= 80 + 0.05 and 25 - 0.05 <= y <= 55 + 0.05)

        op.Base = [(clone, slab_rim)]
        op.AvoidGeometry = [(clone, box_rim)]
        self.doc.recompute()
        cuts = list(self._sampled(self._cutSegments(op)))
        self.assertTrue(cuts, "The slab is machined around the box")

        def in_zone(p, margin=0.05):
            dx = max(60 - p[0], 0, p[0] - 80)
            dy = max(25 - p[1], 0, p[1] - 55)
            return math.hypot(dx, dy) < 2.5 - margin

        self.assertEqual([p for p in cuts if in_zone(p)], [])

    @unittest.skipUnless(_ocl_available, "OpenCamLib not available")
    def test75(self):
        """
        Waterline with a single flat wall selected.

        INPUT:
        - 100x80x5 slab with a 20x30x10 box (x 60..80, y 25..55).
        - Base = the box's front wall only (y = 25, a flat vertical face).
        - Then Avoid = that wall, nothing in Base.

        EXPECTED OUTPUT:
        - Base: the wall's contour is cut (y = 22.5, along x 60..80), and every
          cut lies within the wall's strip: r (+ tolerance) of its footprint.
        - Avoid: the wall's contour is gone, the box's back contour (y = 57.5)
          and the slab outline stay.
        """
        V = FreeCAD.Vector
        shape = (
            Part.makeBox(100, 80, 5).fuse(Part.makeBox(20, 30, 10, V(60, 25, 5))).removeSplitter()
        )
        job = self._createWaterlineJob(shape)
        wall = self._faceName(
            job.Model.Group[0].Shape,
            lambda f: abs(f.BoundBox.YMin - 25) < 1e-6
            and abs(f.BoundBox.YMax - 25) < 1e-6
            and f.BoundBox.ZMax > 14,
        )
        self.assertIsNotNone(wall)

        def dist_wall(p):
            return math.hypot(max(60 - p[0], 0, p[0] - 80), p[1] - 25)

        selected = list(self._sampled(self._cutSegments(self._runWaterline(job, base=[wall]))))
        self.assertTrue(any(abs(p[1] - 22.5) < 0.1 and 61 < p[0] < 79 for p in selected))
        self.assertLess(max(dist_wall(p) for p in selected), 2.5 + 0.1)

        avoided = list(self._sampled(self._cutSegments(self._runWaterline(job, avoid=[wall]))))
        self.assertEqual([p for p in avoided if abs(p[1] - 22.5) < 0.1 and 61 < p[0] < 79], [])
        self.assertTrue(any(abs(p[1] - 57.5) < 0.1 and 61 < p[0] < 79 for p in avoided))
        self.assertTrue(any(p[0] < 0 for p in avoided), "The slab outline is kept")

    def _reopen(self):
        """Saves self.doc to a temporary file, closes it and opens it again."""
        path = os.path.join(tempfile.mkdtemp(), "planar_surface.FCStd")
        self.doc.saveAs(path)
        FreeCAD.closeDocument(self.doc.Name)
        self.doc = FreeCAD.openDocument(path)
        FreeCAD.setActiveDocument(self.doc.Name)
        return self.doc

    @staticmethod
    def _links(prop):
        return [(o.Name, tuple(subs)) for o, subs in prop]

    def _slabWithBoxJob(self):
        V = FreeCAD.Vector
        shape = (
            Part.makeBox(100, 80, 5).fuse(Part.makeBox(20, 30, 10, V(60, 25, 5))).removeSplitter()
        )
        job = self._createWaterlineJob(shape)
        clone = job.Model.Group[0]
        flat = lambda z: next(
            f
            for f in clone.Shape.Faces
            if abs(f.BoundBox.ZMin - z) < 1e-6 and abs(f.BoundBox.ZMax - z) < 1e-6
        )
        name = lambda face: self._faceName(clone.Shape, lambda f: f.isSame(face))
        walls = [
            f"Face{i + 1}"
            for i, f in enumerate(clone.Shape.Faces)
            if f.BoundBox.ZMin > 4.9 and f.BoundBox.ZMax > 14.9 and f.BoundBox.ZLength > 1
        ]
        box_rim = self._edgeNames(clone.Shape, flat(15).OuterWire.Edges)
        return job, clone, name(flat(5)), name(flat(15)), walls, box_rim

    def test76(self):
        """
        Every new setting survives saving and reopening the document.

        INPUT:
        - Z-Level on a slab with a box: Base = the slab top, Avoid Geometry =
          the box top's rim edges, Fill Holes = a box wall, and non-default
          Boundary Adjustment, Avoid Geometry Overlap, Boundary Outline,
          lead settings and Strategy.

        EXPECTED OUTPUT:
        - After saving, closing and opening: the same links and values, no
          legacy property, and the recomputed toolpath identical.
        """
        job, clone, slab_top, _, walls, box_rim = self._slabWithBoxJob()
        op = self._createOp(job, "ZLevelHybrid")
        op.StepDown = 3
        op.Base = [(clone, (slab_top,))]
        op.AvoidGeometry = [(clone, box_rim)]
        op.FillHoleFaces = [(clone, (walls[0],))]
        op.BoundaryAdjustment = 1.0
        op.AvoidGeometryOverlap = 0.5
        op.BoundaryOutline = True
        op.LeadInOut = True
        op.LeadLiftDistance = 2.0
        self.doc.recompute()
        values = {
            "Base": self._links(op.Base),
            "AvoidGeometry": self._links(op.AvoidGeometry),
            "FillHoleFaces": self._links(op.FillHoleFaces),
            "BoundaryAdjustment": op.BoundaryAdjustment.Value,
            "AvoidGeometryOverlap": op.AvoidGeometryOverlap.Value,
            "BoundaryOutline": op.BoundaryOutline,
            "LeadInOut": op.LeadInOut,
            "LeadLiftDistance": op.LeadLiftDistance.Value,
            "Strategy": op.Strategy,
        }
        path = [c.toGCode() for c in op.Path.Commands]
        self.assertTrue(any(c.startswith("G1") for c in path))
        name = op.Name

        doc = self._reopen()
        op = doc.getObject(name)
        restored = {
            "Base": self._links(op.Base),
            "AvoidGeometry": self._links(op.AvoidGeometry),
            "FillHoleFaces": self._links(op.FillHoleFaces),
            "BoundaryAdjustment": op.BoundaryAdjustment.Value,
            "AvoidGeometryOverlap": op.AvoidGeometryOverlap.Value,
            "BoundaryOutline": op.BoundaryOutline,
            "LeadInOut": op.LeadInOut,
            "LeadLiftDistance": op.LeadLiftDistance.Value,
            "Strategy": op.Strategy,
        }
        self.assertEqual(restored, values)
        for legacy in op.Proxy.LEGACY_PROPERTIES:
            self.assertNotIn(legacy, op.PropertiesList)
        op.touch()
        doc.recompute()
        self.assertEqual([c.toGCode() for c in op.Path.Commands], path)

    def test77(self):
        """
        A document from an earlier development build is converted on opening.

        INPUT:
        - Z-Level with Base = the slab top, the box top and a box wall, and the
          old properties: AvoidLastX_Faces = 1, FillSelectedHoles = True,
          EnforceGeofence = True. Saved and reopened.

        EXPECTED OUTPUT:
        - The last Base face (the wall) moved to Avoid Geometry; the other two
          stay in Base.
        - Fill Holes holds all three Base faces, as FillSelectedHoles used them.
        - The old properties are gone. A second reopen changes nothing more.
        """
        job, clone, slab_top, box_top, walls, _ = self._slabWithBoxJob()
        op = self._createOp(job, "ZLevelHybrid")
        op.Base = [(clone, (slab_top, box_top)), (clone, (walls[0],))]
        op.addProperty("App::PropertyInteger", "AvoidLastX_Faces", "Selected Geometry Settings")
        op.addProperty("App::PropertyBool", "FillSelectedHoles", "Clearing Options")
        op.addProperty("App::PropertyBool", "EnforceGeofence", "Clearing Options")
        op.AvoidLastX_Faces = 1
        op.FillSelectedHoles = True
        op.EnforceGeofence = True
        name = op.Name

        op = self._reopen().getObject(name)
        self.assertEqual(self._links(op.Base), [("Clone", (slab_top, box_top))])
        self.assertEqual(self._links(op.AvoidGeometry), [("Clone", (walls[0],))])
        self.assertEqual(self._links(op.FillHoleFaces), [("Clone", (slab_top, box_top, walls[0]))])
        for legacy in ("AvoidLastX_Faces", "FillSelectedHoles", "EnforceGeofence"):
            self.assertNotIn(legacy, op.PropertiesList)

        op = self._reopen().getObject(name)
        self.assertEqual(self._links(op.Base), [("Clone", (slab_top, box_top))])
        self.assertEqual(self._links(op.AvoidGeometry), [("Clone", (walls[0],))])

    def test78(self):
        """
        The preview shows the Fill Holes caps where the holes are.

        INPUT:
        - 100x80x20 block with a r8 hole (at x 30) and a 16x10 pocket (at
          x 60), both from the top (Z 20). Z-Level, Fill Holes = their walls.

        EXPECTED OUTPUT:
        - previewRegions() has "fill" entries: flat caps at Z 20 (the rim)
          covering both openings (area pi * 64 + 160).
        - Switched to Surface Scan, the same caps; to Waterline, no "fill" entry
          (Fill Holes is Z-Level's and Surface Scan's).
        """
        V = FreeCAD.Vector
        shape = (
            Part.makeBox(100, 80, 20)
            .cut(Part.makeCylinder(8, 12, V(30, 40, 8)))
            .cut(Part.makeBox(16, 10, 6, V(60, 30, 14)))
        )
        job = self._createWaterlineJob(shape)
        clone = job.Model.Group[0]
        walls = tuple(
            f"Face{i + 1}"
            for i, f in enumerate(clone.Shape.Faces)
            if f.BoundBox.ZLength > 1 and 10 < f.BoundBox.XMin and f.BoundBox.XMax < 90
        )
        op = self._createOp(job, "ZLevelHybrid")
        op.FillHoleFaces = [(clone, walls)]

        fills = [shape for shape, role in op.Proxy.previewRegions(op) if role == "fill"]
        self.assertTrue(fills)
        self.assertAlmostEqual(min(f.BoundBox.ZMin for f in fills), 20.0, places=6)
        self.assertAlmostEqual(max(f.BoundBox.ZMax for f in fills), 20.0, places=6)
        self.assertAlmostEqual(sum(f.Area for f in fills), math.pi * 64 + 160, delta=0.5)

        op.Strategy = "SurfaceScan"
        scan = [shape for shape, role in op.Proxy.previewRegions(op) if role == "fill"]
        self.assertAlmostEqual(sum(f.Area for f in scan), math.pi * 64 + 160, delta=0.5)

        op.Strategy = "Waterline"
        self.assertNotIn("fill", [role for _, role in op.Proxy.previewRegions(op)])

    def test79(self):
        """
        The preview keeps the prepared model between refreshes, and only that.

        INPUT:
        - A 60x40x10 block with a 20x20x5 boss, Surface Scan, nothing selected.
        - Refreshes after: an Avoid Geometry edit, a Boundary Adjustment change,
          a strategy switch, then an edit of the model itself.

        EXPECTED OUTPUT:
        - The model is prepared once for the first three refreshes, and again
          after the model edit.
        - At every step the drawn region is the region the run builds.
        """
        V = FreeCAD.Vector
        job = self._createWaterlineJob(
            Part.makeBox(60, 40, 10).fuse(Part.makeBox(20, 20, 5, V(10, 10, 10))).removeSplitter()
        )
        source = self.doc.getObject("TestWaterline")
        clone = job.Model.Group[0]
        op = self._createOp(job, "SurfaceScan")
        prepared = []
        proxy = op.Proxy
        original = proxy._prepare_geometry

        def count(*args, **kwargs):
            prepared.append(1)
            return original(*args, **kwargs)

        proxy._prepare_geometry = count
        top = self._faceName(clone.Shape, lambda f: abs(f.BoundBox.ZMin - 15) < 1e-6)

        def region():
            return {role: shape for shape, role in proxy.previewRegions(op)}["region"]

        def matches_run():
            built = self._executeBoundaries(op).boundary_face
            return self._regionSig(region()) == self._regionSig(built)

        try:
            self.assertTrue(matches_run())
            first = len(prepared)
            op.AvoidGeometry = [(clone, (top,))]
            self.assertTrue(matches_run())
            op.BoundaryAdjustment = 1.0
            self.assertTrue(matches_run())
            op.Strategy = "ZLevelHybrid"
            self.assertTrue(matches_run())
            # Only the runs prepared the model again; the preview reused its copy
            runs = 3
            self.assertEqual(len(prepared), first + runs)

            source.Shape = Part.makeBox(80, 40, 10)
            self.doc.recompute()
            before = len(prepared)
            region()
            self.assertEqual(len(prepared), before + 1, "A model edit prepares it again")
            # Block edge - tool radius + Boundary Adjustment + outline tolerance
            self.assertAlmostEqual(region().BoundBox.XMax, 80 - 2.5 + 1.0 + 0.01, delta=0.02)
        finally:
            del proxy._prepare_geometry

    @unittest.skipUnless(_ocl_available, "OpenCamLib not available")
    def test80(self):
        """
        Surface Scan rides over the Fill Holes caps instead of entering the hole.

        INPUT:
        - 60x60x20 block with a r8, 12 deep hole at (30, 30), 5 mm end mill.
          Surface Scan, without and then with Fill Holes = the hole wall.

        EXPECTED OUTPUT:
        - Without: cut points over the hole centre drop to its floor (Z 8).
        - With: every cut point stays on the top (Z 20), and a pass crosses
          the hole.
        """
        V = FreeCAD.Vector
        job = self._createWaterlineJob(
            Part.makeBox(60, 60, 20).cut(Part.makeCylinder(8, 12, V(30, 30, 8)))
        )
        clone = job.Model.Group[0]
        wall = self._faceName(clone.Shape, lambda f: f.Surface.TypeId == "Part::GeomCylinder")
        op = self._createOp(job, "SurfaceScan")
        op.SampleInterval = 1.0
        self.doc.recompute()
        self.assertAlmostEqual(min(p[2] for p in self._cutPoints(op)), 8.0, places=3)

        op.FillHoleFaces = [(clone, (wall,))]
        self.doc.recompute()
        self.assertAlmostEqual(min(p[2] for p in self._cutPoints(op)), 20.0, places=3)
        # A pass straight across the capped hole, on top
        self.assertTrue(
            any(
                abs(a[1] - 30) < 1 and min(a[0], b[0]) < 22 and max(a[0], b[0]) > 38
                for a, b in self._cutSegments(op)
            )
        )

    # ---- Finishing Passes: steep walls and fillets (Surface Scan) ----

    FINISHING = (
        "FinishSteepWalls",
        "SteepMaxDraftAngle",
        "SteepMinDraftAngle",
        "SteepStepDown",
        "SteepOrder",
        "FinishFillets",
        "FilletMaxRadius",
        "FilletIncludeConcave",
        "FilletStepOver",
        "FilletEdgeTrim",
    )

    def _createDraftedBossJob(self):
        """Helper: TestSurfaceFinishingGenerator's drafted_boss() (10 degree walls,
        r3 top fillets, boss Z 5..25 over x 30..70, y 20..60) in a job."""
        from CAMTests.TestSurfaceFinishingGenerator import drafted_boss

        return self._createWaterlineJob(drafted_boss())

    def _shown(self, op, prop):
        return "Hidden" not in op.getEditorMode(prop)

    def test81(self):
        """
        The Finishing Passes properties: defaults and when they are shown.

        INPUT:
        - A Surface Scan op, then switched through settings.

        EXPECTED OUTPUT:
        - Defaults: both passes off, 45 / 2 degree drafts, Level order,
          10 mm max fillet radius, 50 % edge trim, zero step distances.
        - The two switches are shown for Surface Scan Single-pass only; each
          switch's settings only while it is on.
        - Multi-pass with a switch on: no faces taken, one warning.
        """
        job = self._createDraftedBossJob()
        op = self._createOp(job, "SurfaceScan")
        for prop in self.FINISHING:
            self.assertIn(prop, op.PropertiesList)
        self.assertFalse(op.FinishSteepWalls or op.FinishFillets or op.FilletIncludeConcave)
        self.assertAlmostEqual(op.SteepMaxDraftAngle.Value, 45.0)
        self.assertAlmostEqual(op.SteepMinDraftAngle.Value, 2.0)
        self.assertEqual(op.SteepOrder, "Level")
        self.assertAlmostEqual(op.FilletMaxRadius.Value, 10.0)
        self.assertEqual(op.FilletEdgeTrim, 50)
        self.assertEqual((op.SteepStepDown.Value, op.FilletStepOver.Value), (0.0, 0.0))

        self.assertTrue(self._shown(op, "FinishSteepWalls"))
        self.assertFalse(self._shown(op, "SteepOrder"))
        op.FinishSteepWalls = True
        self.assertTrue(self._shown(op, "SteepOrder"))
        self.assertFalse(self._shown(op, "FilletMaxRadius"))
        op.FinishFillets = True
        self.assertTrue(self._shown(op, "FilletMaxRadius"))
        for strategy in ("Waterline", "ZLevelHybrid"):
            op.Strategy = strategy
            self.assertFalse(any(self._shown(op, p) for p in self.FINISHING), strategy)
        op.Strategy = "SurfaceScan"
        op.LayerMode = "Multi-pass"
        self.assertFalse(any(self._shown(op, p) for p in self.FINISHING))

        boundaries = self._executeBoundaries(op)
        self.assertEqual((boundaries.steep_faces, boundaries.fillet_faces), ([], []))

    @unittest.skipUnless(_ocl_available, "OpenCamLib not available")
    def test82(self):
        """
        Whole model: walls and fillets get their own passes, kept out of the
        pattern.

        INPUT:
        - The drafted boss, Surface Scan, nothing selected, 5 mm end mill.
          Finishing off, then steep walls and fillets on.

        EXPECTED OUTPUT:
        - On: the 4 walls and 4 fillets are taken, and the scan region loses
          the walls' footprint (a ring around the boss) while keeping the
          plate and the boss top.
        - On: constant-Z loops around the boss on the walls (a horizontal
          pass at one height reaching all four sides), which a Line
          pattern never makes.
        """
        job = self._createDraftedBossJob()
        op = self._createOp(job, "SurfaceScan")
        op.SampleInterval = 0.5

        def wall_loops():
            levels = {}
            for a, b in self._cutSegments(op):
                if 6.0 < a[2] < 21.0:
                    levels.setdefault(round(a[2], 3), []).extend((a, b))
            return [
                z
                for z, pts in levels.items()
                if min(p[0] for p in pts) < 31
                and max(p[0] for p in pts) > 69
                and min(p[1] for p in pts) < 21
                and max(p[1] for p in pts) > 59
            ]

        plain = self._executeBoundaries(op)
        self.assertEqual(wall_loops(), [])

        op.FinishSteepWalls = True
        op.FinishFillets = True
        finished = self._executeBoundaries(op)
        self.assertEqual((len(finished.steep_faces), len(finished.fillet_faces)), (4, 4))
        for x, y, kept in ((10, 10, True), (50, 40, True), (31.5, 40, False), (50, 21, False)):
            point = FreeCAD.Vector(x, y, 0)
            self.assertTrue(plain.boundary_face.isInside(point, 1e-6, True))
            self.assertEqual(finished.boundary_face.isInside(point, 1e-6, True), kept, (x, y))
        self.assertGreaterEqual(len(wall_loops()), 6)

    @unittest.skipUnless(_ocl_available, "OpenCamLib not available")
    def test83(self):
        """
        Only walls selected: their contours, and no pattern.

        INPUT:
        - The drafted boss, Surface Scan, Base = the four walls, steep walls
          on; Collectively, then Individually.

        EXPECTED OUTPUT:
        - The scan region is empty; the walls are the steep faces.
        - Every cut point is on the walls' height (above the plate, below
          the fillets): no pattern on the plate or the boss top.
        """
        job = self._createDraftedBossJob()
        clone = job.Model.Group[0]
        walls = tuple(
            f"Face{i + 1}"
            for i, f in enumerate(clone.Shape.Faces)
            if abs(f.BoundBox.ZMin - 5) < 1e-6 and f.BoundBox.ZLength > 10
        )
        self.assertEqual(len(walls), 4)
        op = self._createOp(job, "SurfaceScan")
        op.SampleInterval = 0.5
        op.Base = [(clone, walls)]
        op.FinishSteepWalls = True

        for handle in ("Collectively", "Individually"):
            op.HandleMultipleFeatures = handle
            boundaries = self._executeBoundaries(op)
            self.assertEqual(len(boundaries.steep_faces), 4, handle)
            self.assertEqual(boundaries.boundary_face.Faces, [], handle)
            points = self._cutPoints(op)
            self.assertTrue(points, handle)
            self.assertGreater(min(p[2] for p in points), 5.0, handle)
            self.assertLess(max(p[2] for p in points), 23.0, handle)

    @unittest.skipUnless(_ocl_available, "OpenCamLib not available")
    def test84(self):
        """
        Selected walls: the mesh reaches what their contours can hit.

        INPUT:
        - The drafted boss with an unselected 6x6x15 block 3 mm from its foot
          (x 21..27, y 37..43, top Z 20) and another far away (x 85..95,
          y 5..15). Base = the boss's four walls, steep walls on.

        EXPECTED OUTPUT:
        - The mesh's reach takes the near block and leaves the far one.
        - Wherever the tool is over the near block, it stays on its top
          (Z 20): the low contours don't cut into it.
        """
        from CAMTests.TestSurfaceFinishingGenerator import drafted_boss

        V = FreeCAD.Vector
        shape = drafted_boss().fuse(
            [Part.makeBox(6, 6, 15, V(21, 37, 5)), Part.makeBox(10, 10, 15, V(85, 5, 5))]
        )
        job = self._createWaterlineJob(shape.removeSplitter())
        clone = job.Model.Group[0]
        walls = tuple(
            f"Face{i + 1}"
            for i, f in enumerate(clone.Shape.Faces)
            if abs(f.BoundBox.ZMin - 5) < 1e-6
            and f.BoundBox.ZLength > 10
            and 28 < f.BoundBox.XMax < 71
        )
        self.assertEqual(len(walls), 4)
        op = self._createOp(job, "SurfaceScan")
        op.SampleInterval = 0.5
        op.Base = [(clone, walls)]
        op.FinishSteepWalls = True

        boundaries = self._executeBoundaries(op)
        clipper = op.Proxy._boundaryClipper(op, boundaries, 2.5, 0.0)
        near = Part.makePlane(6, 6, V(21, 37, 20)).BoundBox
        far = Part.makePlane(10, 10, V(85, 5, 20)).BoundBox
        self.assertTrue(clipper.touches(near))
        self.assertFalse(clipper.touches(far))

        def over_block(p):
            dx = max(21 - p[0], 0, p[0] - 27)
            dy = max(37 - p[1], 0, p[1] - 43)
            return math.hypot(dx, dy) < 2.45

        over = []
        for a, b in self._cutSegments(op):
            n = max(1, int(math.dist(a[:2], b[:2]) / 0.25))
            for k in range(n + 1):
                p = [a[i] + (b[i] - a[i]) * k / n for i in range(3)]
                if over_block(p):
                    over.append(p[2])
        self.assertTrue(over)
        self.assertAlmostEqual(min(over), 20.0, places=3)

    @unittest.skipUnless(_ocl_available, "OpenCamLib not available")
    def test85(self):
        """
        Volumetric Feed is off by default.

        INPUT:
        - The drafted boss, Surface Scan, tool controller at 600 mm/min
          horizontal feed; Volumetric Feed at its default, then 25%.

        EXPECTED OUTPUT:
        - A new op starts at 0%.
        - 25% raises the fastest feed (the user can still turn it on).
        """
        job = self._createDraftedBossJob()
        controller = job.Tools.Group[0]
        controller.HorizFeed = 600.0
        controller.VertFeed = 300.0
        op = self._createOp(job, "SurfaceScan")
        op.SampleInterval = 1.0
        self.assertEqual(op.VolumetricFeedPercent, 0)

        def fastest(percent):
            op.VolumetricFeedPercent = percent
            self.doc.recompute()
            return max(c.Parameters.get("F", 0.0) for c in op.Path.Commands if c.Name == "G1")

        self.assertGreater(fastest(25), fastest(0) * 1.05)

    @unittest.skipUnless(_ocl_available, "OpenCamLib not available")
    def test86(self):
        """
        Z-Level: Reverse changes the cut order, never the walls' milling direction.

        INPUT:
        - A 100x80x5 plate with a 40x30 boss up to Z 20. Z-Level, Offset
          pattern, Climb and Conventional, each with and without Reverse.

        EXPECTED OUTPUT:
        - The boss's wall loops run clockwise for Climb (the boss on the
          tool's right) and counter-clockwise for Conventional, Reverse or not.
        """
        V = FreeCAD.Vector
        shape = Part.makeBox(100, 80, 5).fuse(Part.makeBox(40, 30, 15, V(30, 25, 5)))
        job = self._createWaterlineJob(shape.removeSplitter())

        def island_loops(op):
            """Direction of each closed loop around the boss (+1 CCW, -1 CW)."""
            loops, run = [], []
            for c in op.Path.Commands:
                p = c.Parameters
                if c.Name in ("G1", "G2", "G3") and "X" in p and "Y" in p:
                    z = p.get("Z", run[-1][2] if run else 0.0)
                    if run and abs(z - run[-1][2]) > 1e-6:
                        loops.append(run)
                        run = []
                    run.append((p["X"], p["Y"], z))
                elif c.Name == "G0" and run:
                    loops.append(run)
                    run = []
            loops.append(run)
            signs = []
            for r in loops:
                if len(r) < 4 or math.dist(r[0][:2], r[-1][:2]) > 0.5:
                    continue
                if not 40 < max(q[0] for q in r) - min(q[0] for q in r) < 50:
                    continue  # not the loop around the boss
                area = sum(r[i][0] * r[i + 1][1] - r[i + 1][0] * r[i][1] for i in range(len(r) - 1))
                signs.append(1 if area > 0 else -1)
            return signs

        for mode, expected in (("Climb", -1), ("Conventional", 1)):
            for reverse in (False, True):
                op = self._createOp(job, "ZLevelHybrid")
                op.CutPattern = "Offset"
                op.CutMode = mode
                op.CutPatternReversed = reverse
                self.doc.recompute()
                signs = island_loops(op)
                self.assertTrue(signs, (mode, reverse))
                self.assertEqual(set(signs), {expected}, (mode, reverse))

    def test87(self):
        """
        Surface Scan: Reverse leaves the steep-wall passes alone.

        INPUT:
        - The drafted boss, Surface Scan with steep-wall passes, Climb;
          Reverse off, then on.

        EXPECTED OUTPUT:
        - The same steep-wall passes: Reverse only orders the cut pattern.
        """
        job = self._createDraftedBossJob()
        op = self._createOp(job, "SurfaceScan")
        op.FinishSteepWalls = True
        boundaries = self._executeBoundaries(op)
        normal = op.Proxy._finishingScanLines(op, 5.0, boundaries)
        op.CutPatternReversed = True
        self.assertTrue(normal)
        self.assertEqual(op.Proxy._finishingScanLines(op, 5.0, boundaries), normal)
