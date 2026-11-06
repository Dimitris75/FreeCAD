# SPDX-License-Identifier: LGPL-2.1-or-later
# SPDX-FileCopyrightText: 2025 Dimitrios Pana <dimitriospana75@gmail.com>
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

import Path
import unittest
import CAMTests.PathTestUtils as PathTestUtils

Path.Log.setLevel(Path.Log.Level.INFO, Path.Log.thisModule())
Path.Log.trackModule(Path.Log.thisModule())

# Check if OCL is available
_ocl_available = False
try:
    try:
        import ocl

        _ocl_available = True
    except ImportError:
        import opencamlib as ocl

        _ocl_available = True
except ImportError:
    pass


@unittest.skipUnless(_ocl_available, "OpenCamLib not available")
class TestSurfaceCommon(PathTestUtils.PathTestBase):
    """Tests for surface_common: OCL cutter creation and boundary generation utilities."""

    # -- make_ocl_cutter tests --

    def test00_create_endmill(self):
        """
        Creates a cylindrical end mill cutter from basic tool parameters.

        INPUT:
        - Function: make_ocl_cutter()
        - Parameters: tool_type="endmill", diameter=10.0, edge_height=20.0
        - Input data: Standard end mill specifications for a 10mm cutter

        EXPECTED OUTPUT:
        - Returns an OCL CylCutter object (not None)
        - Cutter diameter should be exactly 10.0mm
        - This is the most common cutter type used in CAM operations
        """
        from Path.Base.Generator.surface_common import make_ocl_cutter

        cutter = make_ocl_cutter("endmill", 10.0, edge_height=20.0)
        self.assertIsNotNone(cutter)
        self.assertAlmostEqual(cutter.getDiameter(), 10.0)

    def test01_create_ballend(self):
        """
        Creates a ball-nose cutter from ball end tool parameters.

        INPUT:
        - Function: make_ocl_cutter()
        - Parameters: tool_type="ballend", diameter=10.0
        - Input data: Ball nose cutter with radius equal to half diameter

        EXPECTED OUTPUT:
        - Returns an OCL BallCutter object (not None)
        - Cutter diameter should be exactly 10.0mm
        - Ball nose cutters are used for smooth 3D surface finishing
        """
        from Path.Base.Generator.surface_common import make_ocl_cutter

        cutter = make_ocl_cutter("ballend", 10.0)
        self.assertIsNotNone(cutter)
        self.assertAlmostEqual(cutter.getDiameter(), 10.0)

    def test02_create_bullnose(self):
        """
        Creates a bullnose cutter (end mill with corner radius) from tool parameters.

        INPUT:
        - Function: make_ocl_cutter()
        - Parameters: tool_type="bullnose", diameter=10.0, flat_radius=3.0, edge_height=20.0
        - Input data: Bullnose cutter with 2mm corner radius (10/2 - 3) on 10mm diameter

        EXPECTED OUTPUT:
        - Returns an OCL BullCutter object (not None)
        - Cutter diameter should be exactly 10.0mm
        - Bullnose cutters provide stronger corners than sharp end mills
        """
        from Path.Base.Generator.surface_common import make_ocl_cutter

        cutter = make_ocl_cutter("bullnose", 10.0, flat_radius=3.0, edge_height=20.0)
        self.assertIsNotNone(cutter)
        self.assertAlmostEqual(cutter.getDiameter(), 10.0)

    def test03_create_vbit(self):
        """
        Creates a cone-shaped V-bit cutter from engraving tool parameters.

        INPUT:
        - Function: make_ocl_cutter()
        - Parameters: tool_type="v-bit", diameter=10.0, edge_angle=90.0
        - Input data: 90-degree V-bit commonly used for engraving and chamfering

        EXPECTED OUTPUT:
        - Returns an OCL ConeCutter object (not None)
        - V-bit cutters create angled cuts for engraving letters and decorative edges
        """
        from Path.Base.Generator.surface_common import make_ocl_cutter

        cutter = make_ocl_cutter("v-bit", 10.0, edge_angle=90.0)
        self.assertIsNotNone(cutter)

    def test04_unsupported_tool_type(self):
        """
        Tests that unsupported tool types are handled gracefully by returning None.

        INPUT:
        - Function: make_ocl_cutter()
        - Parameters: tool_type="unsupported-tool", diameter=10.0
        - Input data: A tool type not mapped in _TOOL_TYPE_MAP

        EXPECTED OUTPUT:
        - Returns None (not a cutter object)
        - Function should fail gracefully for unsupported tool types
        - Prevents crashes when users specify invalid tool types
        """
        from Path.Base.Generator.surface_common import make_ocl_cutter

        cutter = make_ocl_cutter("unsupported-tool", 10.0)
        self.assertIsNone(cutter)

    def test05_invalid_diameter(self):
        """
        Tests that invalid cutter diameters (zero or negative) are rejected.

        INPUT:
        - Function: make_ocl_cutter()
        - Parameters: tool_type="endmill", diameter=0.0 and diameter=-5.0
        - Input data: Physically impossible cutter sizes

        EXPECTED OUTPUT:
        - Returns None for both zero and negative diameters
        - Prevents creation of invalid cutter objects
        - Ensures data validation catches impossible tool dimensions
        """
        from Path.Base.Generator.surface_common import make_ocl_cutter

        cutter_zero = make_ocl_cutter("endmill", 0.0)
        self.assertIsNone(cutter_zero)

        cutter_neg = make_ocl_cutter("endmill", -5.0)
        self.assertIsNone(cutter_neg)

    def test06_case_insensitivity(self):
        """
        Tests that tool type names are case-insensitive for user convenience.

        INPUT:
        - Function: make_ocl_cutter()
        - Parameters: tool_type="EndMill" and "BALLEND" (mixed case)
        - Input data: Same tool types with different capitalization

        EXPECTED OUTPUT:
        - Returns valid cutter objects for both cases
        - Users shouldn't have to worry about exact capitalization
        - Makes the function more user-friendly and forgiving
        """
        from Path.Base.Generator.surface_common import make_ocl_cutter

        cutter1 = make_ocl_cutter("EndMill", 10.0, edge_height=20.0)
        self.assertIsNotNone(cutter1)

        cutter2 = make_ocl_cutter("BALLEND", 10.0)
        self.assertIsNotNone(cutter2)

    def test07_safe_cutter(self):
        """
        Creates a safety oversized cutter for collision detection and clearance.

        INPUT:
        - Function: make_safe_cutter()
        - Parameters: tool_type="endmill", diameter=10.0, buffer_pct=0.25
        - Input data: Standard end mill specifications

        EXPECTED OUTPUT:
        - Safe cutter should have a diameter of 12.5mm (10.0 * 1.25)
        - Safe cutters are used to check for collisions without actual cutting
        """
        from Path.Base.Generator.surface_common import make_safe_cutter

        safe_cutter = make_safe_cutter("endmill", 10.0, buffer_pct=0.25)
        self.assertIsNotNone(safe_cutter)
        self.assertAlmostEqual(safe_cutter.getDiameter(), 12.5)

    # -- Boundary creation tests --

    # -- cut_avoid_boundary tests --

    # -- build_boundary_masks tests --

    # -- RegionClipper tests --
