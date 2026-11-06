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


import math

import FreeCAD
import Part
import Path
import CAMTests.PathTestUtils as PathTestUtils

Path.Log.setLevel(Path.Log.Level.INFO, Path.Log.thisModule())
Path.Log.trackModule(Path.Log.thisModule())


class TestSurfaceBoundary(PathTestUtils.PathTestBase):
    """Tests for surface_boundary: boundary faces, the machining region, Avoid Faces
    boundaries and BoundaryClipper. Pure geometry: no OpenCamLib needed."""

    def test00_separate_faces(self):
        """
        Tests the separation of touching vs. truly isolated faces.

        INPUT:
        - Function: _separate_touching_faces()
        - Input data: A list containing all faces from a box (touching group)
          and a single, distant, isolated face.

        EXPECTED OUTPUT:
        - All faces from the box should be in the 'touching_faces' list.
        - The single distant face should be in the 'isolated_faces' list.
        - This correctly tests the function's documented purpose.
        """
        from Path.Base.Generator.surface_boundary import _separate_touching_faces

        # A group of faces that all touch each other
        box1 = Part.makeBox(10, 10, 10, FreeCAD.Vector(0, 0, 0))

        # A truly isolated face that touches nothing else in the list
        p1 = FreeCAD.Vector(50, 50, 0)
        p2 = FreeCAD.Vector(60, 50, 0)
        p3 = FreeCAD.Vector(55, 60, 0)
        isolated_face = Part.makeFace(Part.makePolygon([p1, p2, p3, p1]))

        # The function expects a list of face groups (list of lists)
        input_list = [box1.Faces, [isolated_face]]

        touching, isolated = _separate_touching_faces(input_list)

        # touching is a list of groups — expect one group containing all 6 box faces
        self.assertEqual(
            len(touching),
            1,
            "Expected one touching group containing all box faces",
        )
        self.assertEqual(
            len(touching[0]),
            len(box1.Faces),
            "Expected all 6 faces from the box to be in the single touching group",
        )
        self.assertEqual(
            len(isolated),
            1,
            "Expected the single distant face to be in the isolated list",
        )

    def test01_create_boundary(self):
        """
        Tests creation of a 2D boundary face from a 3D shape's silhouette.

        INPUT:
        - Function: create_boundary_face()
        - Parameters: A 20x20x10 box, offset by +5mm.
        - Input data: A simple solid shape.

        EXPECTED OUTPUT:
        - Returns a valid Part.Shape object (Face or Compound).
        - The resulting shape should be larger than the original footprint.
        - The bounding box of the new shape should be approximately 30x30.
        """
        from Path.Base.Generator.surface_boundary import create_boundary_face

        box = Part.makeBox(20, 20, 10)
        boundary = create_boundary_face(box.Faces, offset=5.0)

        self.assertIsNotNone(boundary)
        # The result can be a Face or a Compound, so we check for a valid shape with faces
        self.assertTrue(
            hasattr(boundary, "Faces") and len(boundary.Faces) > 0,
            "Boundary should be a shape with faces",
        )
        self.assertAlmostEqual(boundary.BoundBox.XLength, 30.0, delta=0.1)
        self.assertAlmostEqual(boundary.BoundBox.YLength, 30.0, delta=0.1)

    def test02_generate_mask_with_avoidance(self):
        """
        Tests generation of a final toolpath mask with "keep-out" zones.

        INPUT:
        - Functions: build_machining_region(), then cut_avoid_boundary()
        - Input data: A large cutting area (50x50) and a single, already
          pre-built avoid boundary face (10x10) inside it.

        EXPECTED OUTPUT:
        - Returns a single Part.Face object.
        - The face should have a hole in the middle.
        - The area of the final mask should be less than the original cutting area.
        """
        from Path.Base.Generator.surface_boundary import (
            build_machining_region,
            cut_avoid_boundary,
        )

        # Main area to cut
        cutting_shape = Part.makeBox(50, 50, 1)

        # A pre-built avoid boundary
        avoid_boundary = Part.Face(
            Part.makePolygon(
                [
                    FreeCAD.Vector(20, 20, 0),
                    FreeCAD.Vector(30, 20, 0),
                    FreeCAD.Vector(30, 30, 0),
                    FreeCAD.Vector(20, 30, 0),
                    FreeCAD.Vector(20, 20, 0),
                ]
            )
        )

        top = [f for f in cutting_shape.Faces if f.BoundBox.ZMin > 0.5]
        region = build_machining_region(top, tool_radius=3.0, boundary_adj=0.0, tolerance=0.01)
        mask = cut_avoid_boundary(region, avoid_boundary)

        self.assertIsNotNone(mask)
        self.assertEqual(
            len(mask.Wires), 2, "Mask should have an outer wire and an inner (hole) wire"
        )
        self.assertLess(
            mask.Area, cutting_shape.Area, "Mask area should be smaller after cutting the hole"
        )

    def test03_build_avoid_boundary_caps_cylindrical_wall(self):
        """
        Tests that build_avoid_boundary() correctly caps a non-planar
        (cylindrical) wall face at its topmost rim, rather than passing
        it through unmodified or distorting it via a naive top-down
        projection.

        INPUT:
        - Function: build_avoid_boundary()
        - Input data: The lateral (curved) wall face only -- no top or
          bottom cap faces -- of a 10mm-radius, 20mm-deep cylindrical
          hole (e.g. a straight bored hole), with its top rim at Z=0 and
          bottom rim at Z=-20.

        EXPECTED OUTPUT:
        - Returns a valid, non-None boundary shape.
        - The boundary's footprint is a circle matching the cylinder's
          own diameter (Mask~25mm, STL~20mm), not empty or distorted the
          way a naive top-down projection of a vertical wall would produce.
        - The boundary sits at the wall's topmost Z (0), not its bottom
          (-20) -- confirming the true top rim was isolated and used,
          not an arbitrary wire.
        """
        from Path.Base.Generator.surface_boundary import build_avoid_boundary

        # A solid cylinder representing a straight bored hole: 10mm
        # radius, 20mm deep, top rim at Z=0, bottom rim at Z=-20.
        cylinder = Part.makeCylinder(
            10.0, 20.0, FreeCAD.Vector(0, 0, -20.0), FreeCAD.Vector(0, 0, 1)
        )

        # Isolate just the lateral (curved) wall face -- the actual
        # geometry a user would select for an Avoid Face on a hole wall
        # -- excluding the flat top/bottom cap faces.
        wall_faces = [f for f in cylinder.Faces if isinstance(f.Surface, Part.Cylinder)]
        self.assertEqual(len(wall_faces), 1, "Expected exactly one cylindrical wall face")

        avoid_boundary, avoid_boundary_stl = build_avoid_boundary(
            wall_faces, avoid_overlap=0.0, tool_radius=2.5, tolerance=0.01, needs_safe_stl=True
        )

        self.assertIsNotNone(avoid_boundary)
        self.assertIsNotNone(avoid_boundary_stl)

        # Mask: 20 mm footprint + tool_radius + epsilon on each side
        self.assertAlmostEqual(avoid_boundary.BoundBox.XLength, 25.02, delta=0.1)
        self.assertAlmostEqual(avoid_boundary.BoundBox.YLength, 25.02, delta=0.1)
        # STL pillar: 20 mm footprint + epsilon only (no tool_radius)
        self.assertAlmostEqual(avoid_boundary_stl.BoundBox.XLength, 20.02, delta=0.1)
        self.assertAlmostEqual(avoid_boundary_stl.BoundBox.YLength, 20.02, delta=0.1)
        # The two differ by exactly the tool diameter
        self.assertAlmostEqual(
            avoid_boundary.BoundBox.XLength - avoid_boundary_stl.BoundBox.XLength,
            5.0,
            delta=0.05,
        )

    def test04_pattern_mask_preserves_inner_holes(self):
        """
        The machining region keeps unselected areas inside the selection as holes.

        EXPECTED OUTPUT:
        - The mask has two wires (outer boundary and the island hole).
        - The hole is the 10 mm island grown by tool_radius + epsilon (~14.02 mm).
        """
        from Path.Base.Generator.surface_boundary import build_machining_region

        plate = Part.makeBox(60, 60, 5)
        island = Part.makeBox(10, 10, 10, FreeCAD.Vector(25, 25, 5))
        model = plate.fuse(island).removeSplitter()
        base_faces = [
            f
            for f in model.Faces
            if abs(f.normalAt(0, 0).z - 1) < 1e-6 and abs(f.CenterOfMass.z - 5) < 1e-6
        ]

        mask = build_machining_region(base_faces, 2.0, 0.0, 0.01)

        self.assertIsNotNone(mask)
        wires = [w for f in mask.Faces for w in f.Wires]
        self.assertEqual(len(wires), 2)
        hole = min(wires, key=lambda w: w.BoundBox.XLength)
        self.assertAlmostEqual(hole.BoundBox.XLength, 14.02, delta=0.1)

    @staticmethod
    def _rect(x0, y0, x1, y1):
        V = FreeCAD.Vector
        return Part.Face(
            Part.makePolygon([V(x0, y0, 0), V(x1, y0, 0), V(x1, y1, 0), V(x0, y1, 0), V(x0, y0, 0)])
        )

    def test05_cut_avoid_boundary(self):
        """
        Removes keep-out zones from a region in one cut.

        INPUT:
        - Function: cut_avoid_boundary()
        - A 40x40 region with a 10x10 avoid zone inside it; also None inputs.

        EXPECTED OUTPUT:
        - The zone becomes a hole: area 1600 - 100, two wires.
        - No avoid zone returns the region itself; a None region returns None.
        """
        from Path.Base.Generator.surface_boundary import cut_avoid_boundary

        region = self._rect(0, 0, 40, 40)
        result = cut_avoid_boundary(region, self._rect(15, 15, 25, 25))
        self.assertAlmostEqual(result.Area, 1500.0, places=3)
        self.assertEqual(len(result.Faces[0].Wires), 2)

        self.assertIs(cut_avoid_boundary(region, None), region)
        self.assertIsNone(cut_avoid_boundary(None, self._rect(0, 0, 1, 1)))

    def test06_cut_avoid_boundary_on_compound(self):
        """
        Cuts a compound of per-face regions once, as the "Individually" mode does.

        INPUT:
        - Function: cut_avoid_boundary()
        - A compound of three regions: A (0..40) untouched, B (50..90) crossed by a
          vertical avoid strip, C (100..110) fully inside an avoid zone.

        EXPECTED OUTPUT:
        - The regions stay separate faces (touching or not, nothing is fused).
        - B is split into 2 faces; C disappears: 3 faces in total.
        """
        from Path.Base.Generator.surface_boundary import cut_avoid_boundary

        a = self._rect(0, 0, 40, 40)
        b = self._rect(50, 0, 90, 40)
        c = self._rect(100, 0, 110, 10)
        avoid = Part.makeCompound([self._rect(65, -5, 75, 45), self._rect(98, -2, 112, 12)])

        result = cut_avoid_boundary(Part.makeCompound([a, b, c]), avoid)

        areas = sorted(round(f.Area, 3) for f in result.Faces)
        self.assertEqual(areas, [600.0, 600.0, 1600.0], "A whole, B split in two, C removed")
        self.assertFalse(any(f.BoundBox.XMin >= 100 - 1e-6 for f in result.Faces))

    @staticmethod
    def _plate_with_island():
        """60x60x5 plate with a 10x10x5 island in the middle; returns its Z=5 top faces."""
        plate = Part.makeBox(60, 60, 5)
        island = Part.makeBox(10, 10, 5, FreeCAD.Vector(25, 25, 5))
        model = plate.fuse(island).removeSplitter()
        return [
            f
            for f in model.Faces
            if abs(f.normalAt(0, 0).z - 1) < 1e-6 and abs(f.CenterOfMass.z - 5) < 1e-6
        ]

    def test07_boundary_masks_outline_modes(self):
        """
        build_boundary_masks() keeps or drops the holes of the selection.

        INPUT:
        - Function: build_boundary_masks()
        - The plate top around a 10x10 island (one face with a hole), tool radius 2.

        EXPECTED OUTPUT:
        - outline=False (Surface Scan): the island stays a hole in boundary_face.
        - outline=True (Z-Level): only the outer envelope remains, no hole.
        - Both shrink the outside by tool radius + inset: 60 - 2 * 2.01 wide.
        - No per-face masks unless per_face is requested.
        """
        from Path.Base.Generator.surface_boundary import build_boundary_masks

        faces = self._plate_with_island()

        holed, masks = build_boundary_masks(faces, None, 2.0, 0.0, 0.0001)
        self.assertIsNone(masks)
        self.assertEqual(sum(len(f.Wires) for f in holed.Faces), 2)

        envelope, _ = build_boundary_masks(faces, None, 2.0, 0.0, 0.0001, outline=True)
        self.assertEqual(sum(len(f.Wires) for f in envelope.Faces), 1)
        for region in (holed, envelope):
            self.assertAlmostEqual(region.BoundBox.XLength, 60 - 2 * 2.01, delta=0.05)

    def test08_boundary_masks_per_face_and_failures(self):
        """
        build_boundary_masks() per-face masks and failures.

        INPUT:
        - Two separate 20x20 faces and an avoid zone that covers the second one.

        EXPECTED OUTPUT:
        - per_face=True: one mask for the first face; the covered face gives none.
        - No selection, or faces too small for the tool: (None, None).
        """
        from Path.Base.Generator.surface_boundary import build_boundary_masks

        a, b = self._rect(0, 0, 20, 20), self._rect(40, 0, 60, 20)
        avoid = self._rect(35, -5, 65, 25)

        boundary, masks = build_boundary_masks([a, b], avoid, 2.0, 0.0, 0.0001, per_face=True)
        self.assertEqual(len(masks), 1)
        self.assertLess(masks[0].BoundBox.XMax, 20)
        self.assertIsNotNone(boundary)

        self.assertEqual(build_boundary_masks([], None, 2.0, 0.0, 0.0001), (None, None))
        tiny = self._rect(0, 0, 2, 2)
        self.assertEqual(build_boundary_masks([tiny], None, 2.0, 0.0, 0.0001), (None, None))

    def _ring(self):
        """40x40 square with a 10x10 hole in the middle."""
        return self._rect(0, 0, 40, 40).cut(self._rect(15, 15, 25, 25))

    def test09_boundary_clipper_contains_and_touches(self):
        """
        BoundaryClipper point tests and reach selection.

        INPUT:
        - Class: BoundaryClipper, a 40x40 region with a 10x10 hole, reach 2 mm.

        EXPECTED OUTPUT:
        - contains(): inside the ring True; in the hole and outside False.
        - touches(): a box in the middle of the hole (farther than reach from its
          edge) is rejected, a box just outside within reach is kept, a far box
          is rejected.
        """
        from Path.Base.Generator.surface_boundary import BoundaryClipper

        clip = BoundaryClipper(self._ring(), reach=2.0, tolerance=0.01)
        self.assertTrue(clip.contains(5, 5))
        self.assertFalse(clip.contains(20, 20), "Hole")
        self.assertFalse(clip.contains(50, 50), "Outside")

        box = lambda x0, y0, x1, y1: FreeCAD.BoundBox(x0, y0, 0, x1, y1, 1)
        self.assertFalse(clip.touches(box(19, 19, 21, 21)), "Deep in the hole")
        self.assertTrue(clip.touches(box(41, 10, 41.5, 12)), "Within reach outside")
        self.assertFalse(clip.touches(box(60, 60, 70, 70)), "Far away")

    def test10_boundary_clipper_trim(self):
        """
        BoundaryClipper trims polylines exactly, with Z interpolated at the cut.

        INPUT:
        - A line from (-10, 5, 0) to (50, 5, 6) across the 40x40 ring.
        - A closed loop around the hole, crossing nothing, and a closed loop that
          leaves the region and comes back (starting inside).

        EXPECTED OUTPUT:
        - The line gives one piece from x=0 to x=40 (to 0.005 mm), Z interpolated.
        - The inside loop stays one closed piece.
        - The crossing loop gives one open piece (its two ends rejoined at the seam).
        """
        from Path.Base.Generator.surface_boundary import BoundaryClipper

        clip = BoundaryClipper(self._ring(), tolerance=0.01)

        (piece,) = clip.trim_polylines([[(-10, 5, 0.0), (50, 5, 6.0)]])
        self.assertAlmostEqual(piece[0][0], 0.0, delta=0.006)
        self.assertAlmostEqual(piece[-1][0], 40.0, delta=0.006)
        self.assertAlmostEqual(piece[0][2], 1.0, delta=0.01)
        self.assertAlmostEqual(piece[-1][2], 5.0, delta=0.01)

        inside = [(10, 10, 1), (30, 10, 1), (30, 30, 1), (10, 30, 1)]
        (loop,) = clip.trim_polylines([inside], closed=True)
        self.assertEqual(loop[0], loop[-1])
        self.assertEqual(len(loop), len(inside) + 1)

        crossing = [(30, 5, 1), (50, 5, 1), (50, 10, 1), (30, 10, 1)]
        pieces = clip.trim_polylines([crossing], closed=True)
        self.assertEqual(len(pieces), 1, "Rejoined at the seam")
        self.assertNotEqual(pieces[0][0], pieces[0][-1])
        self.assertTrue(all(p[0] <= 40.0 + 0.006 for p in pieces[0]))

    def test11_boundary_clipper_long_segments(self):
        """
        Segments whose ends lie on the same side are still trimmed.

        Optimize Linear Paths merges collinear points, so a waterline contour
        can be one long segment that passes through the region, or across a
        hole of it, with both ends on the same side.

        INPUT:
        - A 40x40 region with a 10x10 hole at (15..25, 15..25).
        - Segment 1 from (-10, 20) to (50, 20): both ends outside.
        - Segment 2 from (5, 20) to (35, 20): both ends inside, crossing the hole.

        EXPECTED OUTPUT:
        - Segment 1: two pieces, 0..15 and 25..40 (nothing in the hole).
        - Segment 2: two pieces, 5..15 and 25..35 (the hole is never crossed).
        """
        from Path.Base.Generator.surface_boundary import BoundaryClipper

        clip = BoundaryClipper(self._ring(), tolerance=0.01)
        spans = lambda pieces: sorted((round(p[0][0], 3), round(p[-1][0], 3)) for p in pieces)

        self.assertEqual(
            spans(clip.trim_polylines([[(-10, 20, 0), (50, 20, 0)]])), [(0, 15), (25, 40)]
        )
        self.assertEqual(
            spans(clip.trim_polylines([[(5, 20, 0), (35, 20, 0)]])), [(5, 15), (25, 35)]
        )

    def test12_boundary_seals_seams_between_faces(self):
        """
        Hairline gaps between neighbouring selected faces are sealed.

        INPUT:
        - Function: build_boundary_masks(), tool radius 2, "inside".
        - Two 20x20 faces side by side, with a gap of 0, 0.005, 0.02 and 0.08 mm.

        EXPECTED OUTPUT:
        - Gaps up to 0.02 mm: one region with the same area as touching faces
          (the inward offset does not open a tool-wide slot along the seam).
        - 0.08 mm: a real slot, kept open (two regions).
        - A single face is unchanged by the sealing.
        """
        from Path.Base.Generator.surface_boundary import build_boundary_masks

        region = lambda faces: build_boundary_masks(faces, None, 2.0, 0.0, 0.0001)[0]
        pair = lambda gap: [self._rect(0, 0, 20, 20), self._rect(20 + gap, 0, 40 + gap, 20)]

        touching = region(pair(0.0))
        for gap in (0.005, 0.02):
            sealed = region(pair(gap))
            self.assertEqual(len(sealed.Faces), 1, f"gap {gap}")
            self.assertAlmostEqual(sealed.Area, touching.Area, delta=1.0)
        self.assertEqual(len(region(pair(0.08)).Faces), 2, "A real slot stays open")

        single = region([self._rect(0, 0, 20, 20)])
        self.assertAlmostEqual(single.BoundBox.XLength, 20 - 2 * 2.01, delta=0.01)

    def _slab_with_box(self):
        """100x80x5 slab with a 20x30x10 box on top: (model, slab top, box top)."""
        model = (
            Part.makeBox(100, 80, 5)
            .fuse(Part.makeBox(20, 30, 10, FreeCAD.Vector(60, 25, 5)))
            .removeSplitter()
        )
        flat = lambda z: next(
            f
            for f in model.Faces
            if abs(f.BoundBox.ZMin - z) < 1e-6 and abs(f.BoundBox.ZMax - z) < 1e-6
        )
        return model, flat(5), flat(15)

    def _operation(self, strategy, model, base=(), avoid=(), **kw):
        from Path.Base.Generator.surface_boundary import build_operation_boundaries

        args = dict(
            stock_shape=None,
            bound_box="BaseBoundBox",
            tool_radius=2.5,
            boundary_adjustment=0.0,
            avoid_overlap=0.0,
            linear_deflection=0.01,
        )
        args.update(kw)
        return build_operation_boundaries(
            strategy, list(base), list(avoid), model, model, False, **args
        )

    def test13_operation_boundaries_without_selection(self):
        """
        build_operation_boundaries() with nothing selected.

        INPUT:
        - 100x80x5 slab with a 20x30x10 box, tool radius 2.5, no Base / Avoid.

        EXPECTED OUTPUT:
        - Surface Scan / Z-Level: boundary_face is bb_face itself, the model
          outline offset by -r + OUTLINE_TOLERANCE (outward).
        - Stock: the stock outline instead.
        - Waterline: no regions at all (it is not limited).
        """
        from Path.Base.Generator.surface_boundary import OUTLINE_TOLERANCE

        model, _, _ = self._slab_with_box()
        r = 2.5
        for strategy in ("SurfaceScan", "ZLevelHybrid"):
            b = self._operation(strategy, model)
            self.assertIs(b.boundary_face, b.bb_face, strategy)
            bb = b.bb_face.BoundBox
            self.assertAlmostEqual(bb.XMin, r - OUTLINE_TOLERANCE, delta=0.01)
            self.assertAlmostEqual(bb.XMax, 100 - r + OUTLINE_TOLERANCE, delta=0.01)
            self.assertFalse(b.limits_waterline)

        stock = Part.makeBox(110, 90, 20, FreeCAD.Vector(-5, -5, 0))
        b = self._operation("SurfaceScan", model, stock_shape=stock, bound_box="Stock")
        self.assertAlmostEqual(
            b.boundary_face.BoundBox.XMin, -5 + r - OUTLINE_TOLERANCE, delta=0.01
        )

        w = self._operation("Waterline", model)
        self.assertIsNone(w.bb_face)
        self.assertIsNone(w.boundary_face)
        self.assertFalse(w.limits_waterline)

    def test14_operation_boundaries_with_selection(self):
        """
        build_operation_boundaries() with Base / Avoid selections.

        INPUT:
        - The slab and box of test13; Base = the slab top, Avoid = the box top.

        EXPECTED OUTPUT:
        - Surface Scan: bb_face is the selection's rectangle; boundary_face
          shrinks it by r and keeps the box as a hole (Avoid zone); the Avoid
          zones are returned too.
        - Waterline, Avoid only: bb_face is a frame around the model and the
          region is the frame less the Avoid zone; limits_waterline is set.
        - Individually: one face mask per selected face (Surface Scan only).
        - A selection too small for the tool: None (must not machine).
        """
        model, slab_top, box_top = self._slab_with_box()
        r = 2.5

        b = self._operation("SurfaceScan", model, [slab_top], [box_top])
        self.assertAlmostEqual(b.bb_face.BoundBox.XMin, r - 0.01, delta=0.01)
        self.assertIsNotNone(b.avoid_boundary)
        self.assertGreater(len(b.boundary_face.Wires), 1, "The Avoid zone is a hole")
        self.assertLess(b.boundary_face.Area, (100 - 2 * r) * (80 - 2 * r))
        self.assertIsNone(b.face_masks)

        w = self._operation("Waterline", model, avoid=[box_top])
        self.assertTrue(w.limits_waterline)
        self.assertLess(w.bb_face.BoundBox.XMin, -r)
        self.assertAlmostEqual(
            w.boundary_face.Area, w.bb_face.Area - w.avoid_boundary.Area, delta=1.0
        )

        masks = self._operation("SurfaceScan", model, [slab_top, box_top], per_face=True).face_masks
        self.assertEqual(len(masks), 2)
        self.assertIsNone(
            self._operation("ZLevelHybrid", model, [slab_top, box_top], per_face=True).face_masks
        )

        tiny = self._rect(0, 0, 2, 2)
        self.assertIsNone(self._operation("SurfaceScan", model, [tiny]))

    def test15_operation_boundaries_scan_outline(self):
        """
        build_operation_boundaries(): Surface Scan's Boundary Outline switch.

        INPUT:
        - The slab and box of test13; Base = the slab top, which has a hole
          where the box stands. No Avoid Faces.

        EXPECTED OUTPUT:
        - scan_outline off: the box footprint (+ r) stays a hole in the region.
        - scan_outline on: only the outer outline remains, no hole, larger area.
        - Z-Level and Waterline ignore the switch: always the outer outline.
        """
        model, slab_top, _ = self._slab_with_box()
        self.assertEqual(len(slab_top.Wires), 2, "Sanity: the slab top has a hole")

        holed = self._operation("SurfaceScan", model, [slab_top]).boundary_face
        outer = self._operation("SurfaceScan", model, [slab_top], scan_outline=True).boundary_face
        self.assertEqual(sum(len(f.Wires) for f in holed.Faces), 2)
        self.assertEqual(sum(len(f.Wires) for f in outer.Faces), 1)
        self.assertGreater(outer.Area, holed.Area)

        for strategy in ("ZLevelHybrid", "Waterline"):
            off = self._operation(strategy, model, [slab_top]).boundary_face
            on = self._operation(strategy, model, [slab_top], scan_outline=True).boundary_face
            self.assertEqual(sum(len(f.Wires) for f in off.Faces), 1, strategy)
            self.assertAlmostEqual(off.Area, on.Area, places=6)

    @staticmethod
    def _square_edges(x0, y0, x1, y1, z=0.0):
        V = FreeCAD.Vector
        pts = [V(x0, y0, z), V(x1, y0, z), V(x1, y1, z), V(x0, y1, z), V(x0, y0, z)]
        return Part.makePolygon(pts).Edges

    def test16_edge_loops(self):
        """
        Selected edges -> closed loops -> flat regions (edge_loop_regions).

        INPUT:
        - A square's four edges, shuffled and some reversed, at Z = 7.
        - The same square with a 0.05 mm gap, and with a 0.5 mm gap.
        - Two nested squares; the top rim of a cylinder (a 3D circle edge);
          a vertical edge.

        EXPECTED OUTPUT:
        - Shuffled / reversed edges: one closed loop, the square's area, at Z = 0.
        - A 0.05 mm gap is closed (below EDGE_GAP_TOLERANCE); 0.5 mm stays open
          and is counted, giving no region.
        - Nested squares: one face with a hole (outer area - inner area).
        - The rim: a disc of the cylinder's radius. The vertical edge: nothing.
        """
        from Path.Base.Generator.surface_boundary import edge_loop_regions

        edges = self._square_edges(0, 0, 20, 10, z=7.0)
        shuffled = [edges[2], edges[0].reversed(), edges[3], edges[1]]
        faces, open_count = edge_loop_regions(shuffled)
        self.assertEqual(open_count, 0)
        self.assertEqual(len(faces), 1)
        self.assertAlmostEqual(faces[0].Area, 200.0, places=6)
        self.assertAlmostEqual(faces[0].BoundBox.ZMax, 0.0, places=9)

        V = FreeCAD.Vector

        def gapped(gap):
            pts = [V(gap, 0, 0), V(20, 0, 0), V(20, 10, 0), V(0, 10, 0), V(0, 0, 0)]
            return Part.makePolygon(pts).Edges

        faces, open_count = edge_loop_regions(gapped(0.05))
        self.assertEqual((len(faces), open_count), (1, 0))
        self.assertAlmostEqual(faces[0].Area, 200.0, delta=0.5)
        faces, open_count = edge_loop_regions(gapped(0.5))
        self.assertEqual((faces, open_count), ([], 1))

        nested = self._square_edges(0, 0, 40, 40) + self._square_edges(10, 10, 30, 30)
        faces, _ = edge_loop_regions(nested)
        self.assertEqual(len(faces), 1)
        self.assertEqual(len(faces[0].Wires), 2)
        self.assertAlmostEqual(faces[0].Area, 1600.0 - 400.0, places=6)

        cylinder = Part.makeCylinder(6, 10, V(5, 5, 0))
        rim = [e for e in cylinder.Edges if e.BoundBox.ZMin > 9.9 and e.BoundBox.ZLength < 1e-6]
        faces, _ = edge_loop_regions(rim)
        self.assertAlmostEqual(sum(f.Area for f in faces), math.pi * 36, delta=0.1)

        vertical = Part.makeLine(V(0, 0, 0), V(0, 0, 10))
        self.assertEqual(edge_loop_regions([vertical]), ([], 0))

    def test17_operation_boundaries_from_edges(self):
        """
        build_operation_boundaries() with edge loops in Base and Avoid.

        INPUT:
        - The slab and box of test13. Base edges = the slab top's outer rim;
          Avoid edges = the box top's rim.

        EXPECTED OUTPUT:
        - A Base rim gives the region of the face it bounds, without its holes:
          the same as the slab top with the outline switch on (Surface Scan),
          and as the slab top itself for Z-Level and Waterline.
        - An Avoid rim gives the same keep-out zone as the face it bounds.
        - The loops are reported (base_loops, has_selection), and bb_face is
          the rectangle around them.
        - Base edges that form no loop, with nothing else selected: None, never
          the whole model. Open Avoid edges: ignored and counted.
        """
        model, slab_top, box_top = self._slab_with_box()
        outer_rim = slab_top.OuterWire.Edges
        box_rim = box_top.OuterWire.Edges
        area = lambda b: round(b.boundary_face.Area, 3)

        edges = self._operation("SurfaceScan", model, cutting_edges=outer_rim)
        face = self._operation("SurfaceScan", model, [slab_top], scan_outline=True)
        self.assertEqual(area(edges), area(face))
        self.assertEqual(len(edges.base_loops), 1)
        self.assertTrue(edges.has_selection)
        self.assertFalse(edges.cutting_faces)
        self.assertAlmostEqual(edges.bb_face.Area, face.bb_face.Area, places=6)

        for strategy in ("ZLevelHybrid", "Waterline"):
            e = self._operation(strategy, model, cutting_edges=outer_rim)
            f = self._operation(strategy, model, [slab_top])
            self.assertEqual(area(e), area(f), strategy)

        e = self._operation("ZLevelHybrid", model, avoid_edges=box_rim)
        f = self._operation("ZLevelHybrid", model, avoid=[box_top])
        self.assertAlmostEqual(e.avoid_boundary.Area, f.avoid_boundary.Area, places=3)
        self.assertEqual(area(e), area(f))

        open_edges = outer_rim[:2]
        self.assertIsNone(self._operation("SurfaceScan", model, cutting_edges=open_edges))
        b = self._operation("SurfaceScan", model, avoid_edges=open_edges)
        self.assertEqual(b.open_chains, 1)
        self.assertIsNone(b.avoid_boundary)

    def test18_wall_strips_and_waterline_avoid_margin(self):
        """
        Waterline: single flat walls (wall_strip) and the Avoid contour margin.

        INPUT:
        - The slab and box of test13 (box x 60..80, y 25..55, top Z 15).

        EXPECTED OUTPUT:
        - is_vertical_plane: true for the box's side walls, false for its top.
        - wall_strip: a thin strip along the wall's footprint (its length, at
          its Y), WALL_STRIP_HALF_WIDTH on each side.
        - Waterline, Base = one wall: a region around the wall (r + epsilon
          + the strip), where Surface Scan finds no region (None).
        - Waterline, Avoid = the box top: the zone is the box + r - margin
          (the box's own contour, at r, stays outside it); Surface Scan's zone
          is the box + r + epsilon.
        - Waterline, Avoid = one wall: the zone reaches past r, so the wall's
          own contour is removed.
        """
        from Path.Base.Generator.surface_boundary import (
            WALL_STRIP_HALF_WIDTH,
            is_vertical_plane,
            wall_strip,
        )

        model, _, box_top = self._slab_with_box()
        wall = next(
            f
            for f in model.Faces
            if abs(f.BoundBox.YMin - 25) < 1e-6 and abs(f.BoundBox.YMax - 25) < 1e-6
        )
        self.assertTrue(is_vertical_plane(wall))
        self.assertFalse(is_vertical_plane(box_top))

        strip = wall_strip(wall)
        bb = strip.BoundBox
        self.assertAlmostEqual(bb.XMin, 60, places=6)
        self.assertAlmostEqual(bb.XMax, 80, places=6)
        self.assertAlmostEqual(bb.YLength, 2 * WALL_STRIP_HALF_WIDTH, places=6)
        self.assertAlmostEqual(bb.ZMax, 0.0, places=9)

        r = 2.5
        w = self._operation("Waterline", model, [wall])
        bb = w.boundary_face.BoundBox
        self.assertAlmostEqual(bb.YMin, 25 - r - 0.011 - WALL_STRIP_HALF_WIDTH, delta=0.01)
        self.assertAlmostEqual(bb.XMax, 80 + r + 0.011, delta=0.02)
        self.assertIsNone(self._operation("SurfaceScan", model, [wall]))

        margin = 0.1
        avoid = self._operation("Waterline", model, avoid=[box_top], contour_margin=margin)
        self.assertAlmostEqual(avoid.avoid_boundary.BoundBox.XMax, 80 + r - margin, delta=0.01)
        scan = self._operation("SurfaceScan", model, avoid=[box_top])
        self.assertAlmostEqual(scan.avoid_boundary.BoundBox.XMax, 80 + r + 0.011, delta=0.01)

        # An avoided wall: its own contour (at r) must fall inside the zone
        wall_zone = self._operation("Waterline", model, avoid=[wall], contour_margin=margin)
        self.assertLess(wall_zone.avoid_boundary.BoundBox.YMin, 25 - r)
