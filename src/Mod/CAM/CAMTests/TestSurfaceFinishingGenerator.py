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

import itertools
import math

import FreeCAD
import Part
import Path
import CAMTests.PathTestUtils as PathTestUtils

Path.Log.setLevel(Path.Log.Level.INFO, Path.Log.thisModule())
Path.Log.trackModule(Path.Log.thisModule())

# Rounded box shared by several tests: 40 x 30 x 20 mm, every edge filleted.
_BOX = (40.0, 30.0, 20.0)
_ROUND = 3.0
_TOOL = 6.0  # Ball end mill diameter
_R = _TOOL / 2.0


def _rounded_box():
    """40x30x20 box with all 12 edges filleted at R3."""
    box = Part.makeBox(*_BOX)
    return box.makeFillet(_ROUND, box.Edges)


def _drafted_boss(r_bottom=10.0, r_top=8.0, height=20.0, x=0.0):
    """Conical boss (drafted round wall) standing on Z=0."""
    return Part.makeCone(r_bottom, r_top, height, FreeCAD.Vector(x, 0, 0))


def _cone_faces(shape):
    return [f for f in shape.Faces if "Cone" in f.Surface.TypeId]


def _filleted_boss():
    """Plate with a round boss; R2 roundover on the boss top, R2 fillet at its foot."""
    plate = Part.makeBox(60, 60, 5, FreeCAD.Vector(-30, -30, 0))
    boss = Part.makeCylinder(10, 15, FreeCAD.Vector(0, 0, 5))
    fused = plate.fuse(boss).removeSplitter()
    edges = [
        e
        for e in fused.Edges
        if hasattr(e.Curve, "Radius") and abs(e.Curve.Radius - 10.0) < 1e-6
    ]
    return fused.makeFillet(2.0, edges)


def _radius(point):
    return math.hypot(point[0], point[1])


def _entry(contact, n_xy, nz=0.5, tool_radius=_R):
    """Run entry (cl_x, cl_y, nx, ny, px, py, pz) for a contact point and normal."""
    length = math.hypot(*n_xy)
    h = math.sqrt(1.0 - nz * nz)
    nx, ny = n_xy[0] / length * h, n_xy[1] / length * h
    return (
        contact[0] + tool_radius * nx,
        contact[1] + tool_radius * ny,
        nx,
        ny,
        contact[0],
        contact[1],
        contact[2],
    )


class TestSurfaceFinishing(PathTestUtils.PathTestBase):
    """Tests for surface_finishing: detection, keep-out zone, steep and fillet passes."""

    def setUp(self):
        from Path.Base.Generator import surface_finishing

        self.fin = surface_finishing

    # -- steep wall detection --

    def test00_steep_box_walls(self):
        """
        Splits the faces of a plain box into steep walls and the rest.

        INPUT:
        - Function: split_steep_faces()
        - Parameters: default draft band (0-18 degrees)
        - Input data: 40x30x20 box (4 vertical walls, top and bottom)

        EXPECTED OUTPUT:
        - The 4 vertical walls are steep, top and bottom are not.
        """
        box = Part.makeBox(*_BOX)
        steep, other = self.fin.split_steep_faces(box.Faces)
        self.assertEqual(len(steep), 4)
        self.assertEqual(len(other), 2)
        for face in other:
            self.assertAlmostEqual(abs(face.normalAt(0, 0).z), 1.0, places=6)

    def test01_steep_draft_band(self):
        """
        Classifies a drafted (conical) wall by its draft angle.

        INPUT:
        - Function: split_steep_faces()
        - Parameters: max_draft_angle=18, then max_draft_angle=3
        - Input data: Cone boss, R10 to R8 over 20mm (about 5.7 degrees draft)

        EXPECTED OUTPUT:
        - Steep with an 18 degree limit, not steep with a 3 degree limit.
        """
        wall = _cone_faces(_drafted_boss())
        steep, _ = self.fin.split_steep_faces(wall, max_draft_angle=18.0)
        self.assertEqual(len(steep), 1)
        steep, _ = self.fin.split_steep_faces(wall, max_draft_angle=3.0)
        self.assertEqual(len(steep), 0)

    def test02_steep_min_draft_excludes_vertical(self):
        """
        Excludes perfectly vertical walls with a small minimum draft angle.

        INPUT:
        - Function: split_steep_faces()
        - Parameters: min_draft_angle=0.5
        - Input data: Box with vertical walls, and a 5.7 degree drafted cone

        EXPECTED OUTPUT:
        - No box wall is steep; the drafted cone still is.
        """
        steep, _ = self.fin.split_steep_faces(Part.makeBox(*_BOX).Faces, min_draft_angle=0.5)
        self.assertEqual(len(steep), 0)
        steep, _ = self.fin.split_steep_faces(_cone_faces(_drafted_boss()), min_draft_angle=0.5)
        self.assertEqual(len(steep), 1)

    # -- fillet detection --

    def test10_classify_rounded_box(self):
        """
        Splits a fully rounded box into steep walls, fillets and the rest.

        INPUT:
        - Function: split_finishing_faces()
        - Parameters: steep and fillets enabled, defaults otherwise
        - Input data: 40x30x20 box with all edges filleted at R3 (26 faces)

        EXPECTED OUTPUT:
        - Steep: 4 vertical planes + 4 vertical edge roundovers (vertical cylinders).
        - Fillets: 8 horizontal roundovers + 8 corner patches.
        - Rest: top and bottom planes.
        """
        steep, fillets, rest = self.fin.split_finishing_faces(_rounded_box().Faces)
        self.assertEqual(len(steep), 8)
        self.assertEqual(len(fillets), 16)
        self.assertEqual(len(rest), 2)
        for face in rest:
            self.assertIn("Plane", face.Surface.TypeId)

    def test11_vertical_cylinder_not_a_fillet(self):
        """
        Rejects a vertical cylinder (a bore or boss wall) as a fillet.

        INPUT:
        - Function: split_fillet_faces()
        - Input data: R10 x 20mm cylinder standing on Z=0

        EXPECTED OUTPUT:
        - No fillet faces.
        """
        fillets, _ = self.fin.split_fillet_faces(Part.makeCylinder(10, 20).Faces)
        self.assertEqual(fillets, [])

    def test12_concave_fillets_optional(self):
        """
        Separates convex roundovers from concave (inside) fillets.

        INPUT:
        - Function: split_fillet_faces(), _is_concave()
        - Parameters: include_concave=False, then True
        - Input data: Plate with a round boss, R2 roundover on the boss top
          (convex torus) and R2 fillet at its foot (concave torus)

        EXPECTED OUTPUT:
        - The top torus is convex, the foot torus concave.
        - Without concave fillets only the top torus is returned; with them both.
        """
        shape = _filleted_boss()
        tori = sorted(
            (f for f in shape.Faces if "Toroid" in f.Surface.TypeId),
            key=lambda f: f.BoundBox.ZMin,
        )
        self.assertEqual(len(tori), 2)
        foot, top = tori
        self.assertTrue(self.fin._is_concave(foot))
        self.assertFalse(self.fin._is_concave(top))

        fillets, _ = self.fin.split_fillet_faces(shape.Faces, include_concave=False)
        self.assertEqual([f.isSame(top) for f in fillets], [True])
        fillets, _ = self.fin.split_fillet_faces(shape.Faces, include_concave=True)
        self.assertEqual(len(fillets), 2)

    def test13_fillet_max_radius(self):
        """
        Ignores blends larger than the maximum fillet radius.

        INPUT:
        - Function: split_fillet_faces()
        - Parameters: max_radius=2
        - Input data: Box rounded at R3

        EXPECTED OUTPUT:
        - No fillets, and therefore no corner patches either.
        """
        fillets, _ = self.fin.split_fillet_faces(_rounded_box().Faces, max_radius=2.0)
        self.assertEqual(fillets, [])

    # -- keep-out zone --

    def test20_keep_out_steep_footprint(self):
        """
        Builds the main pattern's keep-out zone around a drafted boss wall.

        INPUT:
        - Function: build_feature_avoid_boundary()
        - Parameters: tool_radius=3
        - Input data: Cone boss wall, R10 at the bottom to R8 at the top

        EXPECTED OUTPUT:
        - The zone reaches one tool radius past the wall foot (diameter about 26).
        - The boss top stays outside the zone (the footprint keeps its hole),
          so the main pattern still finishes it.
        """
        zone = self.fin.build_feature_avoid_boundary(
            _cone_faces(_drafted_boss()), [], _R, 0.01
        )
        self.assertIsNotNone(zone)
        self.assertAlmostEqual(zone.BoundBox.XLength, 2.0 * (10.0 + _R), delta=0.1)
        center = Part.Vertex(FreeCAD.Vector(0, 0, zone.BoundBox.ZMin))
        self.assertGreater(center.distToShape(zone)[0], 4.0)

    # -- steep wall passes --

    def test30_steep_boss_contours(self):
        """
        Generates constant-Z contours around a drafted boss.

        INPUT:
        - Function: generate_steep_scan_lines()
        - Parameters: step_over=2, tool 6mm, sample_interval=0.5
        - Input data: Cone boss, R10 to R8 over 20mm

        EXPECTED OUTPUT:
        - One closed contour per slice height, flattened to Z=0.
        - Each contour lies one tool radius outside the wall at its height.
        """
        levels = self.fin._steep_levels(20.0, 0.0, 2.0)
        lines = self.fin.generate_steep_scan_lines(
            _cone_faces(_drafted_boss()), 2.0, _TOOL, 0.5
        )
        self.assertEqual(len(lines), len(levels))
        for line, z in zip(lines, levels):
            self.assertEqual(line[0], line[-1])
            expected = 10.0 - 2.0 * z / 20.0 + _R
            for point in line:
                self.assertEqual(point[2], 0.0)
                self.assertAlmostEqual(_radius(point), expected, delta=0.05)

    def test31_steep_cavity_contours(self):
        """
        Offsets contours inward for a cavity (drafted hole) wall.

        INPUT:
        - Function: generate_steep_scan_lines()
        - Input data: 60x60x20 block with a conical through-hole, R12 at the
          bottom to R14 at the top

        EXPECTED OUTPUT:
        - Each contour lies one tool radius inside the hole wall at its height.
        """
        block = Part.makeBox(60, 60, 20, FreeCAD.Vector(-30, -30, 0))
        shape = block.cut(Part.makeCone(12.0, 14.0, 20.0))
        levels = self.fin._steep_levels(20.0, 0.0, 2.0)
        lines = self.fin.generate_steep_scan_lines(_cone_faces(shape), 2.0, _TOOL, 0.5)
        self.assertEqual(len(lines), len(levels))
        for line, z in zip(lines, levels):
            expected = 12.0 + 2.0 * z / 20.0 - _R
            for point in line:
                self.assertAlmostEqual(_radius(point), expected, delta=0.05)

    def test32_steep_cut_direction(self):
        """
        Orients contours for climb and conventional milling.

        INPUT:
        - Function: generate_steep_scan_lines()
        - Parameters: cut_climb=True / False
        - Input data: Cone boss (material inside the loop) and cone hole
          (material outside the loop)

        EXPECTED OUTPUT:
        - Climb keeps the material on the right: clockwise around a boss,
          counter-clockwise inside a hole. Conventional is the opposite.
        """
        boss = _cone_faces(_drafted_boss())
        hole = _cone_faces(
            Part.makeBox(60, 60, 20, FreeCAD.Vector(-30, -30, 0)).cut(Part.makeCone(12, 14, 20))
        )
        for faces, climb, ccw in (
            (boss, True, False),
            (boss, False, True),
            (hole, True, True),
            (hole, False, False),
        ):
            lines = self.fin.generate_steep_scan_lines(faces, 4.0, _TOOL, 0.5, cut_climb=climb)
            for line in lines:
                area = self.fin._signed_area_xy([p[:2] for p in line[:-1]])
                self.assertEqual(area > 0.0, ccw)

    def test33_steep_order(self):
        """
        Orders contours by level or by wall.

        INPUT:
        - Function: generate_steep_scan_lines()
        - Parameters: order="Level", then order="Wall"
        - Input data: Two identical cone bosses 60mm apart

        EXPECTED OUTPUT:
        - Both orders produce the same number of contours.
        - "Wall" finishes one boss completely before the other (one switch);
          "Level" alternates between them.
        """
        faces = _cone_faces(_drafted_boss()) + _cone_faces(_drafted_boss(x=60.0))

        def switches(lines):
            owner = [line[0][0] > 30.0 for line in lines]
            return sum(1 for a, b in zip(owner, owner[1:]) if a != b)

        by_level = self.fin.generate_steep_scan_lines(faces, 4.0, _TOOL, 0.5, order="Level")
        by_wall = self.fin.generate_steep_scan_lines(faces, 4.0, _TOOL, 0.5, order="Wall")
        self.assertEqual(len(by_level), len(by_wall))
        self.assertEqual(switches(by_wall), 1)
        self.assertGreater(switches(by_level), 1)

    # -- fillet passes --

    def _front_top_roundover(self):
        """The top roundover of the rounded box along X, on the Y=0 side."""
        candidates = [
            f
            for f in _rounded_box().Faces
            if "Cylinder" in f.Surface.TypeId
            and abs(f.Surface.Axis.x) > 0.99
            and f.BoundBox.ZMin > _BOX[2] / 2.0
        ]
        return min(candidates, key=lambda f: f.BoundBox.YMin)

    def test40_fillet_single_face(self):
        """
        Generates flow lines on one roundover, trimmed at its free ends.

        INPUT:
        - Function: generate_fillet_scan_lines()
        - Parameters: step_over=1, tool 6mm, sample_interval=0.25
        - Input data: R3 top roundover along X (arc 4.71mm, from X=3 to X=37)

        EXPECTED OUTPUT:
        - 5 bands across the arc. The pass at the steep end (9 degrees from
          vertical) is below the 11 degree limit but above 6 degrees, so it is
          moved to exactly 11 degrees: still 5 passes.
        - Each pass runs along X and stops half a step over (0.5mm) short of
          both ends: X from 3.5 to 36.5.
        - The steepest pass sits at 11 degrees: its tool center is
          (3 + 3) * cos(11 deg) from the roundover axis (Y=3) in Y.
        """
        lines = self.fin.generate_fillet_scan_lines(
            [self._front_top_roundover()], 1.0, _TOOL, 0.25
        )
        self.assertEqual(len(lines), 5)
        steepest = min(line[0][1] for line in lines)
        self.assertAlmostEqual(
            steepest, _ROUND - (_ROUND + _R) * math.cos(math.radians(11.0)), delta=0.01
        )
        for line in lines:
            xs = [p[0] for p in line]
            self.assertAlmostEqual(min(xs), _ROUND + 0.5, delta=1e-3)
            self.assertAlmostEqual(max(xs), _BOX[0] - _ROUND - 0.5, delta=1e-3)
            for point in line:
                self.assertEqual(point[2], 0.0)

    def test41_fillet_ring_around_corners(self):
        """
        Joins roundover passes around the corner patches into closed loops.

        INPUT:
        - Function: generate_fillet_scan_lines()
        - Parameters: step_over=1, cut_climb=True
        - Input data: The 4 top roundovers and 4 top corner patches of the
          rounded box

        EXPECTED OUTPUT:
        - One closed loop per pass of a single roundover (5, see test40).
        - Each loop is continuous (no step longer than 2.5 sample intervals)
          and runs clockwise around the box (climb, material on the right).
        """
        steep, fillets, _ = self.fin.split_finishing_faces(_rounded_box().Faces)
        top = [f for f in fillets if f.BoundBox.ZMin > _BOX[2] / 2.0]
        self.assertEqual(len(top), 8)

        lines = self.fin.generate_fillet_scan_lines(top, 1.0, _TOOL, 0.25, cut_climb=True)
        self.assertEqual(len(lines), 5)
        for line in lines:
            self.assertEqual(line[0], line[-1])
            step = max(math.dist(a[:2], b[:2]) for a, b in zip(line, line[1:]))
            self.assertLessEqual(step, 2.5 * 0.25)
            self.assertLess(self.fin._signed_area_xy([p[:2] for p in line[:-1]]), 0.0)

    def test42_non_ball_tool_warning(self):
        """
        Warns when fillet passes are generated with a tool other than a ball.

        INPUT:
        - Function: _warn_if_not_ball()
        - Input data: Tool parameter dictionaries for several tool types

        EXPECTED OUTPUT:
        - No warning for a ball end, a tapered ball nose, or a bull nose whose
          corner radius equals the tool radius.
        - A warning for an end mill, a smaller-radius bull nose and an unknown tool.
        """
        warnings = []
        original = Path.Log.warning
        Path.Log.warning = lambda message, *args, **kwargs: warnings.append(message)
        try:
            for tool_type, corner, expect in (
                ("ballend", 0.0, False),
                ("TaperedBallNose", 0.0, False),
                ("bullnose", 3.0, False),
                ("endmill", 0.0, True),
                ("bullnose", 1.0, True),
                (None, 0.0, True),
            ):
                warnings.clear()
                self.fin._warn_if_not_ball(
                    {"tool_type": tool_type, "diameter": 6.0, "corner_radius": corner}, 6.0
                )
                self.assertEqual(bool(warnings), expect, tool_type)
        finally:
            Path.Log.warning = original

    # -- helpers --

    def test50_steep_levels(self):
        """
        Spaces slice heights evenly within the steep range.

        INPUT:
        - Function: _steep_levels()
        - Parameters: z_top=10, z_bottom=0, step_down=3

        EXPECTED OUTPUT:
        - First and last levels sit 0.01mm inside the range.
        - Equal spacing, never more than the step down.
        """
        levels = self.fin._steep_levels(10.0, 0.0, 3.0)
        self.assertAlmostEqual(levels[0], 9.99, places=9)
        self.assertAlmostEqual(levels[-1], 0.01, places=9)
        gaps = [a - b for a, b in zip(levels, levels[1:])]
        self.assertTrue(all(abs(g - gaps[0]) < 1e-9 for g in gaps))
        self.assertLessEqual(gaps[0], 3.0)

    def test51_trim_run_ends(self):
        """
        Shortens an open pass at its free ends only.

        INPUT:
        - Function: _trim_run_ends()
        - Input data: Straight 10mm pass, a closed ring

        EXPECTED OUTPUT:
        - A trim of 1.5 at both ends gives X from 1.5 to 8.5 (interpolated exactly).
        - With at_end=False the end stays at X=10.
        - A closed ring is returned unchanged.
        """
        run = [_entry((x, 0.0, 0.0), (0, -1)) for x in range(11)]
        trimmed = self.fin._trim_run_ends(run, 1.5)
        self.assertAlmostEqual(trimmed[0][4], 1.5, places=9)
        self.assertAlmostEqual(trimmed[-1][4], 8.5, places=9)
        kept_end = self.fin._trim_run_ends(run, 1.5, at_end=False)
        self.assertEqual(kept_end[-1], run[-1])
        ring = run + [run[0]]
        self.assertEqual(self.fin._trim_run_ends(ring, 1.5), ring)

    def test52_edge_arc(self):
        """
        Rounds a convex sharp edge with an arc and rejects implausible ones.

        INPUT:
        - Function: _edge_arc()
        - Input data: Two passes meeting at a convex 90 degree edge, the same
          edge seen as concave, and corrupted joins

        EXPECTED OUTPUT:
        - Convex edge: arc points all at the tool's horizontal offset from the
          edge point, spaced within the sample interval.
        - Concave edge, contacts apart, end beyond the tool radius, and a
          170 degree turn: no arc.
        """
        si = 0.25
        corner = (0.0, 0.0, 1.0)
        a = _entry(corner, (0, -1))
        before = (a[0] + si,) + a[1:]
        b = _entry(corner, (-1, 0))
        offset = math.hypot(a[0], a[1])

        arc = self.fin._edge_arc(before, a, b, si, _R)
        self.assertTrue(arc)
        for point in arc:
            self.assertAlmostEqual(math.hypot(point[0], point[1]), offset, places=6)
        path = [a] + arc + [b]
        self.assertLessEqual(max(math.dist(p[:2], q[:2]) for p, q in zip(path, path[1:])), si)

        concave_before = (a[0] - si,) + a[1:]  # Travelling the other way round
        self.assertEqual(self.fin._edge_arc(concave_before, a, b, si, _R), [])
        apart = _entry((0.3, -0.3, 1.0), (-1, 0))
        self.assertEqual(self.fin._edge_arc(before, a, apart, si, _R), [])
        far = (corner[0] - 1.6 * _R, corner[1]) + b[2:]
        self.assertEqual(self.fin._edge_arc(before, a, far, si, _R), [])
        turn = _entry(corner, (math.cos(math.radians(100)), math.sin(math.radians(100))))
        self.assertEqual(self.fin._edge_arc(before, a, turn, si, _R), [])

    def test53_join_closed_ring(self):
        """
        Joins four mitered sides into one closed loop with an arc at every corner.

        INPUT:
        - Function: _join_fillet_passes()
        - Parameters: tolerance=0.5, sample_interval=0.25, tool_radius=3
        - Input data: Four straight passes around a 20x20 square, travelling
          clockwise, each ending where the next begins (sharp 90 degree edges),
          supplied in every order

        EXPECTED OUTPUT:
        - One closed chain, whatever the order, including the corner where the
          loop closes.
        - No step between tool-center points longer than the sample interval.
        """
        si = 0.25
        corners = [(10, -10), (-10, -10), (-10, 10), (10, 10)]
        normals = [(0, -1), (-1, 0), (0, 1), (1, 0)]
        sides = []
        for k in range(4):
            (x0, y0), (x1, y1) = corners[k], corners[(k + 1) % 4]
            count = 81
            sides.append(
                [
                    _entry(
                        (x0 + (x1 - x0) * i / (count - 1), y0 + (y1 - y0) * i / (count - 1), 1.0),
                        normals[k],
                    )
                    for i in range(count)
                ]
            )
        for order in itertools.permutations(range(4)):
            passes = [sides[k] for k in order]
            chains, _ = self.fin._join_fillet_passes(passes, list(order), 0.5, si, _R)
            self.assertEqual(len(chains), 1)
            run, closed = chains[0][0], chains[0][1]
            self.assertTrue(closed)
            self.assertEqual(run[0], run[-1])
            self.assertLessEqual(
                max(math.dist(p[:2], q[:2]) for p, q in zip(run, run[1:])), si * 1.01
            )

    def test54_stack_by_wall(self):
        """
        Stacks contours per wall, keeping a boss apart from the pocket around it.

        INPUT:
        - Function: _stack_by_wall()
        - Input data: 5 levels, each with a boss loop inside a pocket loop
          (nested bounding boxes), both drifting slightly with depth

        EXPECTED OUTPUT:
        - Two stacks of 5 contours; each stack holds only one wall.
        """

        def circle(radius):
            pts = [
                (radius * math.cos(2 * math.pi * k / 48), radius * math.sin(2 * math.pi * k / 48))
                for k in range(48)
            ]
            return pts + [pts[0]]

        levels = [
            [
                self.fin._contour_record(circle(4.0 + 0.1 * i)),
                self.fin._contour_record(circle(20.0 - 0.1 * i)),
            ]
            for i in range(5)
        ]
        stacks = self.fin._stack_by_wall(levels, 3.0)
        self.assertEqual(len(stacks), 2)
        for stack in stacks:
            self.assertEqual(len(stack), 5)
            widths = [record[2][2] - record[2][0] for record in stack]
            self.assertTrue(all(w < 12.0 for w in widths) or all(w > 30.0 for w in widths))

    def test55_coverage_outcome(self):
        """
        Decides a sampled coverage test early only when the result is certain.

        INPUT:
        - Function: _coverage_outcome()
        - Input data: Every combination of hits, valid samples and remaining
          samples (up to 4), for coverages 1.0, 0.75 and 0.5

        EXPECTED OUTPUT:
        - True only if every possible completion passes; False only if none
          does; None otherwise (checked against brute-force enumeration).
        """
        for coverage in (1.0, 0.75, 0.5):
            for valid in range(7):
                for hits in range(valid + 1):
                    for remaining in range(5):
                        outcomes = set()
                        # Each remaining sample: qualifying, valid only, or invalid
                        for rest in itertools.product((0, 1, 2), repeat=remaining):
                            h = hits + rest.count(0)
                            v = valid + rest.count(0) + rest.count(1)
                            outcomes.add(h >= self.fin._needed_hits(v, coverage))
                        result = self.fin._coverage_outcome(hits, valid, remaining, coverage)
                        if result is True:
                            self.assertEqual(outcomes, {True})
                        elif result is False:
                            self.assertEqual(outcomes, {False})
                        else:
                            self.assertEqual(outcomes, {True, False})
