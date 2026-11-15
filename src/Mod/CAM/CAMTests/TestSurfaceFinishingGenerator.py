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


import math

import FreeCAD
import Part
import Path
import CAMTests.PathTestUtils as PathTestUtils
from Path.Base.Generator import surface_finishing

Path.Log.setLevel(Path.Log.Level.INFO, Path.Log.thisModule())
Path.Log.trackModule(Path.Log.thisModule())

V = FreeCAD.Vector
DRAFT = 10.0  # Boss wall draft, degrees from vertical


def drafted_boss(fillet=3.0, concave=0.0):
    """
    A 100x80x5 plate with a 40x40 boss, 20 high (Z 5..25), whose walls lean
    in by DRAFT degrees. fillet: radius of the boss's top edges (convex).
    concave: radius of its foot edges (inside fillets).
    """
    lean = 20 * math.tan(math.radians(DRAFT))
    foot = Part.makePolygon([V(30, 20, 5), V(70, 20, 5), V(70, 60, 5), V(30, 60, 5), V(30, 20, 5)])
    top = Part.makePolygon(
        [
            V(30 + lean, 20 + lean, 25),
            V(70 - lean, 20 + lean, 25),
            V(70 - lean, 60 - lean, 25),
            V(30 + lean, 60 - lean, 25),
            V(30 + lean, 20 + lean, 25),
        ]
    )
    boss = Part.makeLoft([foot, top], True)
    if fillet:
        edges = [
            e for e in boss.Edges if abs(e.BoundBox.ZMin - 25) < 1e-6 and e.BoundBox.ZLength < 1e-6
        ]
        boss = boss.makeFillet(fillet, edges)
    shape = Part.makeBox(100, 80, 5).fuse(boss).removeSplitter()
    if concave:
        edges = [
            e
            for e in shape.Edges
            if abs(e.BoundBox.ZMax - 5) < 1e-6
            and e.BoundBox.ZLength < 1e-6
            and 29 < e.BoundBox.XMin
            and e.BoundBox.XMax < 71
            and 19 < e.BoundBox.YMin
            and e.BoundBox.YMax < 61
        ]
        shape = shape.makeFillet(concave, edges)
    return shape


def split(shape, **kwargs):
    """split_finishing_faces with the operation's minimum draft (2 degrees)."""
    kwargs.setdefault("steep_min_draft_angle", 2.0)
    return surface_finishing.split_finishing_faces(shape.Faces, **kwargs)


def x_min(line):
    return min(p[0] for p in line)


class TestSurfaceFinishing(PathTestUtils.PathTestBase):
    """Tests for surface_finishing: classification, keep-out zone, steep-wall
    and fillet passes. Pure geometry: no OpenCamLib needed."""

    def test00_split(self):
        """
        Steep walls and fillets are told apart from the rest.

        INPUT:
        - drafted_boss(): four 10 degree walls, four r3 top fillets, the boss
          top, the plate (top, bottom, vertical sides).

        EXPECTED OUTPUT:
        - Defaults: the four boss walls are steep, the four fillets are
          fillets; the plate's vertical sides are below the 2 degree minimum.
        - Steep only: no fillets. Max draft 8 degrees: no steep walls.
        """
        shape = drafted_boss()
        steep, fillets, rest = split(shape)
        self.assertEqual(len(steep), 4)
        self.assertEqual(len(fillets), 4)
        self.assertEqual(len(steep) + len(fillets) + len(rest), len(shape.Faces))
        for face in steep:
            self.assertAlmostEqual(face.BoundBox.ZMin, 5.0, places=6)
        for face in fillets:
            self.assertGreater(face.BoundBox.ZMin, 20.0)

        steep, fillets, _ = split(shape, finish_fillets=False)
        self.assertEqual((len(steep), len(fillets)), (4, 0))
        steep, _, _ = split(shape, steep_max_draft_angle=8.0)
        self.assertEqual(len(steep), 0)

    def test01_concave(self):
        """
        Inside fillets are finished only on request.

        INPUT:
        - drafted_boss() with r2 fillets at the boss foot and none on top.

        EXPECTED OUTPUT:
        - Defaults: no fillets. FilletIncludeConcave: the four foot fillets
          (below Z 7).
        """
        shape = drafted_boss(fillet=0.0, concave=2.0)
        self.assertEqual(split(shape)[1], [])
        fillets = split(shape, fillet_include_concave=True)[1]
        self.assertEqual(len(fillets), 4)
        for face in fillets:
            self.assertLess(face.BoundBox.ZMax, 7.0)

    def test02_keep_out(self):
        """
        The main pattern's keep-out zone is the faces' XY footprint.

        INPUT:
        - drafted_boss()'s steep walls and fillets, tool radius 2.5.

        EXPECTED OUTPUT:
        - Steep walls: the boss foot (40x40) grown by the tool radius.
        - Fillets: their own footprint, a ring inside the boss foot.
        - Both: one region, the same as the steep walls' (the fillets lie
          inside it).
        """
        steep, fillets, _ = split(drafted_boss())
        walls = surface_finishing.build_feature_avoid_boundary(steep, [], 2.5, 0.025)
        self.assertAlmostEqual(walls.BoundBox.XMin, 30 - 2.5, delta=0.05)
        self.assertAlmostEqual(walls.BoundBox.XLength, 40 + 5, delta=0.1)

        ring = surface_finishing.build_feature_avoid_boundary([], fillets, 2.5, 0.025)
        self.assertGreater(ring.BoundBox.XMin, 30.0)
        self.assertLess(ring.BoundBox.XMax, 70.0)

        both = surface_finishing.build_feature_avoid_boundary(steep, fillets, 2.5, 0.025)
        self.assertEqual(len(both.Faces), 1)
        self.assertAlmostEqual(both.BoundBox.XLength, walls.BoundBox.XLength, delta=0.05)

    def test03_steep_contours(self):
        """
        Steep walls get constant-Z contours, a tool radius off the wall.

        INPUT:
        - drafted_boss()'s steep walls, 5 mm tool, 2 mm step down.

        EXPECTED OUTPUT:
        - Closed 2D loops (Z 0), one per level, about (wall height / step)
          of them.
        - Level to level the loop grows by about step * tan(draft): the wall
          leans out as the levels go down.
        - The lowest loop sits a tool radius outside the boss foot.
        """
        steep, _, _ = split(drafted_boss(), finish_fillets=False)
        lines = surface_finishing.generate_steep_scan_lines(steep, 2.0, 5.0, 0.5)
        self.assertGreaterEqual(len(lines), 8)
        self.assertLessEqual(len(lines), 11)
        for line in lines:
            self.assertEqual(line[0], line[-1])
            self.assertTrue(all(p[2] == 0.0 for p in line))

        edges = sorted(x_min(line) for line in lines)
        grow = 2.0 * math.tan(math.radians(DRAFT))
        for a, b in zip(edges, edges[1:]):
            self.assertAlmostEqual(b - a, grow, delta=0.05)
        self.assertAlmostEqual(edges[0], 30 - 2.5, delta=0.1)

    def test04_fillet_flow_lines(self):
        """
        Fillets get flow lines along the blend, spaced on the surface.

        INPUT:
        - drafted_boss()'s r3 top fillets (an 80 degree arc, 4.19 mm long),
          5 mm tool, 1 mm step over.

        EXPECTED OUTPUT:
        - The passes around the four fillets join into one loop per step:
          about 4.19 / 1 of them, each closed.
        - Each loop lies inside the boss foot and further out than the last.
        """
        _, fillets, _ = split(drafted_boss())
        lines = surface_finishing.generate_fillet_scan_lines(fillets, 1.0, 5.0, 0.5)
        self.assertIn(len(lines), (4, 5, 6))
        for line in lines:
            self.assertAlmostEqual(line[0][0], line[-1][0], delta=1e-6)
            self.assertAlmostEqual(line[0][1], line[-1][1], delta=1e-6)
            self.assertGreater(x_min(line), 30.0)
        edges = sorted((x_min(line) for line in lines), reverse=True)
        self.assertEqual(edges, sorted(set(edges), reverse=True))

    def test05_keep_out_around_seam(self):
        """
        Fillets around a cylinder (faces with a seam) still get their keep-out.

        INPUT:
        - A 60x60x10 plate with an r12 cylinder boss (Z 10..25): r3 fillet on
          its top edge (convex), r2 fillet at its foot (concave). Each fillet
          is a closed band with a seam edge, where Path.Area's projection
          returns no face.

        EXPECTED OUTPUT:
        - Both fillets taken (concave included on request).
        - The keep-out: two touching rings, r9..r12 and r12..r14, grown by
          epsilon (0.026 at a 0.025 tolerance) on both rims; never empty.
        """
        boss = Part.makeCylinder(12, 15, V(30, 30, 10))
        boss = boss.makeFillet(3, [e for e in boss.Edges if e.BoundBox.ZMin > 24])
        shape = Part.makeBox(60, 60, 10).fuse(boss).removeSplitter()
        foot = [
            e
            for e in shape.Edges
            if abs(e.BoundBox.ZMax - 10) < 1e-6 and e.Curve.TypeId == "Part::GeomCircle"
        ]
        shape = shape.makeFillet(2, foot)
        _, fillets, _ = split(shape, fillet_include_concave=True)
        self.assertEqual(len(fillets), 2)

        keep_out = surface_finishing.build_feature_avoid_boundary([], fillets, 2.5, 0.025)
        self.assertIsNotNone(keep_out)
        eps = 0.026
        self.assertAlmostEqual(
            keep_out.Area, math.pi * ((14 + eps) ** 2 - (9 - eps) ** 2), delta=0.5
        )
        self.assertAlmostEqual(keep_out.BoundBox.XMin, 30 - 14 - eps, delta=0.01)

    def test06_keep_out_corner_patches(self):
        """
        Fillet corner patches (with a degenerated edge) don't break the keep-out.

        INPUT:
        - A 40x30x15 block on a plate, its top edges filleted r3: four edge
          fillets and four corner patches where they meet. Each patch has a
          degenerated edge (a point, no curve).

        EXPECTED OUTPUT:
        - All 8 fillets taken; the keep-out is one band around the block top:
          its outline less the flat top inside the fillets (40x30 minus 34x24,
          less the rounded corners), grown by epsilon.
        """
        block = Part.makeBox(40, 30, 15, V(30, 25, 10))
        block = block.makeFillet(3, [e for e in block.Edges if e.BoundBox.ZMax > 24.9])
        shape = Part.makeBox(100, 80, 10).fuse(block).removeSplitter()
        _, fillets, _ = split(shape)
        self.assertEqual(len(fillets), 8)
        self.assertTrue(any(e.Degenerated for f in fillets for e in f.Edges))

        keep_out = surface_finishing.build_feature_avoid_boundary([], fillets, 2.5, 0.025)
        self.assertIsNotNone(keep_out)
        self.assertEqual(len(keep_out.Faces), 1)
        self.assertAlmostEqual(keep_out.Area, 40 * 30 - 34 * 24, delta=5.0)
        self.assertAlmostEqual(keep_out.BoundBox.XMin, 30.0, delta=0.05)
