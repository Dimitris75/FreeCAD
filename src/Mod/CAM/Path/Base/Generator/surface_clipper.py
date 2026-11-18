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


"""Clipping work to a boundary for the Planar Surface strategies.

BoundaryClipper limits meshes and polylines to a flat region after they have
been generated (mesh faces out of the cutter's reach, Waterline contours,
scan lines). The regions themselves come from surface_boundary.

Pure geometry: no FreeCAD document access.
"""

import Path

from Path.Base.Generator.surface_boundary import offset_region

__title__ = "Surface Boundary Clipper"
__url__ = "https://www.freecad.org"

if False:
    Path.Log.setLevel(Path.Log.Level.DEBUG, Path.Log.thisModule())
    Path.Log.trackModule(Path.Log.thisModule())
else:
    Path.Log.setLevel(Path.Log.Level.INFO, Path.Log.thisModule())


# ---------------------------------------------------------------------------
# Clipping to a boundary
# ---------------------------------------------------------------------------


class BoundaryClipper:
    """
    Limits work to a flat boundary after it has been generated.

    Some work cannot be confined to an area while it is being made: OCL traces
    Waterline contours on the whole mesh, and the mesh itself is built from
    model faces. BoundaryClipper does both jobs that follow from that, against
    the same boundary (normally ``boundary_face``, the tool-centre area):

    1. **Selection by reach** (``touches``): could a model face matter to
       the boundary? The boundary is grown by ``reach`` (tool
       radius + lead-in / Boundary Adjustment reach + a margin) and a face is
       dropped only when its XY bounds really miss the grown area. Keeping too
       much is always safe. Used to mesh only what the cutter can reach
       (Surface Scan and Waterline with a Base / Avoid selection). The grown
       boundary is built on the first call, so trimming alone never pays
       for it.
    2. **Exact trimming** (``trim_polylines``): which parts of a path lie in
       the boundary? Segments are split where they cross the boundary's edges
       and every part is kept or dropped by its midpoint. Used for Waterline
       contours.

    Together the two are exact: with a reach-clipped mesh, Waterline traces
    extra loops along the mesh's cut edges, but those run at least the margin
    outside the boundary, so trimming removes them, while every real contour
    inside is unchanged (all triangles within reach of it were kept).

    The boundary is read with the even-odd rule, as surface_pattern's C++ ray
    caster does: holes (Avoid Geometry zones) are outside, islands inside holes
    are inside again. Curved edges are discretized to straight segments with
    ``tolerance``, so results are exact to that polygon, not to the curve.

    Example::

        clipper = BoundaryClipper(boundary_face, reach=r + margin, tolerance=0.025)
        faces = [f for f in model_faces if clipper.touches(f.BoundBox)]
        pieces = clipper.trim_polylines(contours, closed=True)

    Args:
        region (Part.Shape): Flat boundary (faces at any Z, only XY is used).
        reach (float): Distance to grow the boundary by for selection by reach
            (mm). 0 selects against the boundary itself.
        tolerance (float): Discretization deflection for curved edges (mm),
            at least 1e-4.

    Attributes:
        region (Part.Shape): The boundary as given.
        reach (float): The reach distance used.
        reach_shape (Part.Shape): The boundary grown by reach, built on first
            use (the boundary itself when reach is 0 or the offset fails;
            selection is then tighter by the reach).
    """

    # -----------------------------------------------------------------------
    # Construction
    # -----------------------------------------------------------------------

    def __init__(self, region, reach=0.0, tolerance=0.01):
        self.region = region
        self.reach = max(float(reach), 0.0)
        self.tolerance = max(float(tolerance), 1e-4)

        # The boundary itself, as polygons: used for trimming
        self._polys = self._polygons(region)

        # The boundary grown by reach: used for selection only, built on first
        # use (see _reach)
        self._reach_shape = None
        self._reach_polys = None
        self._reach_bounds = None

        # Boundary edges for the crossing tests, built on first use (see _edges)
        self._edge_rows = None
        self._edge_array = None

    @property
    def reach_shape(self):
        """The boundary grown by reach (see the class Attributes)."""
        self._reach()
        return self._reach_shape

    def _reach(self):
        """
        Builds the grown boundary on first use.

        If the offset fails nothing is clipped (no polygons: touches() keeps
        everything), never the smaller boundary itself, which would leave
        faces within the cutter's reach out of the mesh.

        Returns:
            tuple: (polygons, bounds) of the grown boundary; bounds is None when
                there are no polygons.
        """
        if self._reach_polys is None:
            shape = self.region
            if self.reach > 0.0 and self.region is not None:
                shape = offset_region(self.region, self.reach)
                if shape is None:
                    Path.Log.warning(
                        "Could not grow the boundary by the cutter's reach: the mesh is not clipped."
                    )
            self._reach_shape = shape
            self._reach_polys = self._polygons(shape)
            self._reach_bounds = self._bounds(self._reach_polys)
        return self._reach_polys, self._reach_bounds

    def _polygons(self, shape):
        """
        Discretizes every wire of shape into an XY polygon.

        Args:
            shape (Part.Shape): A flat boundary, or None.

        Returns:
            list: One list of (x, y) tuples per wire (outer wires and holes
                alike; the even-odd rule tells them apart). Wires with fewer
                than three points are skipped. Empty for None or a null shape.
        """
        polys = []
        if shape is None or shape.isNull():
            return polys
        for wire in shape.Wires:
            try:
                pts = wire.discretize(Deflection=self.tolerance)
            except Exception:
                # Some wires refuse deflection-based discretization: fall back
                # to a fixed point count
                pts = wire.discretize(100)
            poly = [(p.x, p.y) for p in pts]
            if len(poly) > 2:
                polys.append(poly)
        return polys

    @staticmethod
    def _bounds(polys):
        """(xmin, xmax, ymin, ymax) of all polygons, or None when there are none."""
        if not polys:
            return None
        xs = [x for poly in polys for x, _ in poly]
        ys = [y for poly in polys for _, y in poly]
        return (min(xs), max(xs), min(ys), max(ys))

    # -----------------------------------------------------------------------
    # Point tests (even-odd rule)
    # -----------------------------------------------------------------------

    @staticmethod
    def _inside_scalar(x, y, polys):
        """
        Even-odd point-in-polygon test for one point.

        Casts a ray from (x, y) towards +X and counts the polygon edges it
        crosses: an odd count means inside. Holes and islands are handled by
        counting the edges of every polygon together.

        Args:
            x, y (float): The point.
            polys (list): Polygons as returned by _polygons().

        Returns:
            bool: True if the point is inside.
        """
        inside = False
        for poly in polys:
            j = len(poly) - 1
            for i in range(len(poly)):
                xi, yi = poly[i]
                xj, yj = poly[j]
                # The edge straddles the ray's height, and the crossing lies to
                # the right of the point
                if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
                    inside = not inside
                j = i
        return inside

    @staticmethod
    def _inside_many(xs, ys, polys):
        """
        Even-odd test (see _inside_scalar) for many points at once.

        Vectorized with numpy over the points, one polygon edge at a time;
        falls back to _inside_scalar per point when numpy is missing.

        Args:
            xs, ys (list): Point coordinates.
            polys (list): Polygons as returned by _polygons().

        Returns:
            list: One bool per point.
        """
        try:
            import numpy as np
        except ImportError:
            return [BoundaryClipper._inside_scalar(x, y, polys) for x, y in zip(xs, ys)]
        px = np.asarray(xs, dtype=float)
        py = np.asarray(ys, dtype=float)
        inside = np.zeros(px.shape, dtype=bool)
        for poly in polys:
            arr = np.asarray(poly, dtype=float)
            x1, y1 = arr[:, 0], arr[:, 1]
            x2, y2 = np.roll(x1, 1), np.roll(y1, 1)
            for k in range(len(arr)):
                cond = (y1[k] > py) != (y2[k] > py)
                if not cond.any():
                    continue
                # (A horizontal edge never straddles: the 1e-300 only avoids
                # dividing by zero in the masked-out lanes.)
                xc = (x2[k] - x1[k]) * (py - y1[k]) / ((y2[k] - y1[k]) or 1e-300) + x1[k]
                inside ^= cond & (px < xc)
        return inside.tolist()

    def contains(self, x, y):
        """True if (x, y) is inside the boundary (False when it is empty)."""
        return bool(self._polys) and self._inside_scalar(x, y, self._polys)

    # -----------------------------------------------------------------------
    # 1. Selection by reach
    # -----------------------------------------------------------------------

    def touches(self, bound_box):
        """
        Could anything inside bound_box be within reach of the boundary?

        Conservative, but exact for the box against the grown boundary (not
        just its bounding box): a box lying wholly in a hole of the boundary,
        or beside an L-shaped boundary, is rejected.

        The box touches the grown boundary when any of these holds:
        1. a boundary vertex lies in the box,
        2. a box corner lies inside the boundary (box overlapping or inside it),
        3. a boundary edge crosses the box.

        Args:
            bound_box (FreeCAD.BoundBox): XY bounds of the item (Z is ignored).

        Returns:
            bool: False only when the box certainly misses the grown boundary.
                True when there is no boundary to test against.
        """
        reach_polys, reach_bounds = self._reach()
        if not reach_polys:
            return True  # nothing to clip against: keep everything
        x0, x1, y0, y1 = reach_bounds
        bx0, bx1, by0, by1 = bound_box.XMin, bound_box.XMax, bound_box.YMin, bound_box.YMax

        # Quick reject: the box misses the grown boundary's bounding box
        if bx1 < x0 or bx0 > x1 or by1 < y0 or by0 > y1:
            return False
        # 1. A boundary vertex inside the box
        for poly in reach_polys:
            for x, y in poly:
                if bx0 <= x <= bx1 and by0 <= y <= by1:
                    return True
        # 2. A box corner inside the boundary
        corners = ((bx0, by0), (bx1, by0), (bx1, by1), (bx0, by1))
        if any(self._inside_scalar(x, y, reach_polys) for x, y in corners):
            return True
        # 3. A boundary edge crossing the box
        for poly in reach_polys:
            n = len(poly)
            for i in range(n):
                if _segment_hits_box(poly[i - 1], poly[i], bx0, bx1, by0, by1):
                    return True
        return False

    # -----------------------------------------------------------------------
    # 2. Exact trimming
    # -----------------------------------------------------------------------

    def _edges(self):
        """
        All boundary edges as (x1, y1, x2, y2) rows, built on first use.

        Every polygon contributes its closing edge too (last point back to the
        first).
        """
        if self._edge_rows is None:
            rows = []
            for poly in self._polys:
                for i in range(len(poly)):
                    (x1, y1), (x2, y2) = poly[i - 1], poly[i]
                    rows.append((x1, y1, x2, y2))
            self._edge_rows = rows
        return self._edge_rows

    def _crossings(self, a, b):
        """
        Where does segment a-b cross the boundary's edges? (one segment, no numpy)

        The segment is a + t * r (t in [0, 1], r = b - a) and each edge is
        q1 + u * s (u in [0, 1], s = q2 - q1). With the 2D cross product
        v x w = v.x * w.y - v.y * w.x and d = r x s:

            t = (q1 - a) x s / d        u = (q1 - a) x r / d

        and the two cross where 0 < t < 1 and 0 <= u <= 1. Parallel edges
        (d ~ 0) never count. The segment's own ends (t = 0, t = 1) are excluded:
        a crossing exactly at a polyline vertex needs no split, the segments
        already meet there.

        Args:
            a, b (tuple): Segment end points; only X and Y are used.

        Returns:
            list: The parameters t of the crossings, sorted, duplicates removed
                (rounded to 1e-12). Empty for a zero-length segment.
        """
        ax, ay = a[0], a[1]
        rx, ry = b[0] - ax, b[1] - ay
        if rx == 0.0 and ry == 0.0:
            return []
        out = set()
        for x1, y1, x2, y2 in self._edges():
            sx, sy = x2 - x1, y2 - y1
            denom = rx * sy - ry * sx
            if abs(denom) <= 1e-15:
                continue
            qx, qy = x1 - ax, y1 - ay
            t = (qx * sy - qy * sx) / denom
            u = (qx * ry - qy * rx) / denom
            if 1e-12 < t < 1.0 - 1e-12 and 0.0 <= u <= 1.0:
                out.add(round(t, 12))
        return sorted(out)

    # Largest (segments x edges) block computed at once by _crossings_many
    _CHUNK_CELLS = 2_000_000

    def _crossings_many(self, pts, np):
        """
        _crossings for every segment of a polyline at once (numpy).

        The same formula as _crossings, evaluated as a (segments x edges)
        array, in chunks of at most _CHUNK_CELLS cells to bound memory.

        Args:
            pts (list): The polyline's points (tuples); segment i is
                pts[i] -> pts[i + 1].
            np (module): numpy.

        Returns:
            dict: Segment index -> sorted crossing parameters, for the segments
                that cross at least one edge. Segments not in the dict cross
                nothing.
        """
        if self._edge_array is None:
            self._edge_array = np.asarray(self._edges(), dtype=float).reshape(-1, 4)
        e = self._edge_array
        sx, sy = e[:, 2] - e[:, 0], e[:, 3] - e[:, 1]

        xy = np.asarray([(p[0], p[1]) for p in pts], dtype=float)
        ax, ay = xy[:-1, 0], xy[:-1, 1]
        rx, ry = xy[1:, 0] - ax, xy[1:, 1] - ay

        out = {}
        rows = max(1, self._CHUNK_CELLS // max(1, len(e)))
        for start in range(0, len(ax), rows):
            sl = slice(start, start + rows)
            # Broadcast: one row per segment, one column per edge
            qx = e[None, :, 0] - ax[sl, None]
            qy = e[None, :, 1] - ay[sl, None]
            r_x, r_y = rx[sl, None], ry[sl, None]
            denom = r_x * sy - r_y * sx
            ok = np.abs(denom) > 1e-15
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (qx * sy - qy * sx) / denom
                u = (qx * r_y - qy * r_x) / denom
            hit = ok & (t > 1e-12) & (t < 1.0 - 1e-12) & (u >= 0.0) & (u <= 1.0)
            # Zero-length segments never cross (as in _crossings)
            hit &= ~((r_x == 0.0) & (r_y == 0.0))
            for i in np.nonzero(hit.any(axis=1))[0]:
                out[start + int(i)] = sorted(set(np.round(t[i][hit[i]], 12).tolist()))
        return out

    @staticmethod
    def _lerp(a, b, t):
        """The point at parameter t on segment a-b (every coordinate, Z included)."""
        return tuple(a[k] + (b[k] - a[k]) * t for k in range(len(a)))

    def _split_segments(self, pts):
        """
        Splits every segment of a polyline at its crossings with the boundary.

        Args:
            pts (list): The polyline's points (tuples).

        Returns:
            list: (start, end) sub-segments in order. A segment that crosses
                nothing is one sub-segment; one with k crossings gives k + 1.
                The points are the same values _lerp gives at t = 0 and t = 1.
        """
        try:
            import numpy as np
        except ImportError:
            np = None

        segments = list(zip(pts, pts[1:]))
        if np is None:
            crossings = {i: self._crossings(a, b) for i, (a, b) in enumerate(segments)}
        else:
            crossings = self._crossings_many(pts, np)

        subs = []
        for i, (a, b) in enumerate(segments):
            params = [0.0] + crossings.get(i, []) + [1.0]
            for t0, t1 in zip(params, params[1:]):
                subs.append((self._lerp(a, b, t0), self._lerp(a, b, t1)))
        return subs

    def trim_polylines(self, polylines, closed=False):
        """
        The pieces of polylines that lie inside the boundary.

        Every segment is split where it crosses a boundary edge
        (_split_segments), and each part is kept or dropped by testing its
        midpoint (even-odd, so holes count). A segment is therefore trimmed
        correctly even when both its ends lie on the same side: a long straight
        run passing through the boundary, or one crossing a hole, as Optimize
        Linear Paths produces.

        Z is interpolated at every cut. A loop cut open is rejoined at its
        seam, so a loop crossing the boundary twice gives one piece, not two.

        Args:
            polylines (list): Lists of (x, y) or (x, y, z) tuples.
            closed (bool): Treat every polyline as a loop (its last point
                connects back to its first).

        Returns:
            list: Pieces (lists of tuples) with at least two points. A closed
                loop wholly inside comes back whole, with its first point
                repeated at the end. Every polyline unchanged when the
                boundary is empty.
        """
        if not self._polys:
            return [list(p) for p in polylines]

        out = []
        for line in polylines:
            pts = [tuple(p) for p in line]
            if len(pts) < 2:
                continue
            # 1. Loops: add the closing segment
            if closed and (pts[0][0], pts[0][1]) != (pts[-1][0], pts[-1][1]):
                pts.append(pts[0])

            # 2. Split every segment at its crossings
            subs = self._split_segments(pts)
            if not subs:
                continue

            # 3. A sub-segment never crosses an edge: its midpoint tells for all of it
            mids = [((p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0) for p, q in subs]
            flags = self._inside_many([m[0] for m in mids], [m[1] for m in mids], self._polys)

            # 4. Chain consecutive inside sub-segments into pieces
            pieces, cur = [], []
            for (p, q), inside in zip(subs, flags):
                if inside:
                    if not cur:
                        cur = [p]
                    # Skip zero-length parts (a crossing exactly at a vertex)
                    if (q[0], q[1]) != (cur[-1][0], cur[-1][1]):
                        cur.append(q)
                elif cur:
                    pieces.append(cur)
                    cur = []
            if cur:
                pieces.append(cur)

            # 5. Loops: the piece running into the seam continues past it
            if closed and len(pieces) > 1 and flags[0] and flags[-1]:
                # The loop starts and ends inside: its last and first pieces are one
                first = pieces.pop(0)
                pieces[-1] = pieces[-1] + first[1:]
            out.extend(piece for piece in pieces if len(piece) >= 2)
        return out


def _segment_hits_box(a, b, x0, x1, y0, y1):
    """
    Does segment a-b touch the axis-aligned box [x0, x1] x [y0, y1]?

    Liang-Barsky clipping: the segment a + t * (b - a), t in [0, 1], is
    clipped against the box's four sides in turn; it touches the box when a
    non-empty t range survives.

    Args:
        a, b (tuple): Segment end points (x, y).
        x0, x1, y0, y1 (float): The box.

    Returns:
        bool: True if any part of the segment lies in the box (edges included).
    """
    dx, dy = b[0] - a[0], b[1] - a[1]
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, a[0] - x0), (dx, x1 - a[0]), (-dy, a[1] - y0), (dy, y1 - a[1])):
        if abs(p) < 1e-15:
            # Parallel to this side: outside it means no overlap at all
            if q < 0:
                return False
            continue
        t = q / p
        if p < 0:
            t0 = max(t0, t)  # entering
        else:
            t1 = min(t1, t)  # leaving
        if t0 > t1:
            return False
    return True
