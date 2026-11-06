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


"""Boundaries for the Planar Surface strategies.

Everything that defines *where* a strategy may cut, as flat 2D shapes:

- Boundary faces: the flat outline of 3D geometry (model, stock, faces).
- Fused boundaries: a boundary for a group of faces, touching faces fused.
- The machining region (``boundary_face``): the selected faces (or the
  model / stock boundary) offset by the tool radius, with the Avoid Faces cut
  out. Shared by Surface Scan, Z-Level Hybrid and Waterline.
- The Avoid Faces keep-out boundary.
- BoundaryClipper: limits meshes and polylines to a boundary afterwards
  (mesh faces out of the cutter's reach, Waterline contours).

Pure geometry: no FreeCAD document access. The operation passes the shapes
and tool parameters in.
"""

from dataclasses import dataclass, field
from typing import Any, Optional

import FreeCAD
import Part
import Path

__title__ = "Surface Boundary Utilities"
__url__ = "https://www.freecad.org"

if False:
    Path.Log.setLevel(Path.Log.Level.DEBUG, Path.Log.thisModule())
    Path.Log.trackModule(Path.Log.thisModule())
else:
    Path.Log.setLevel(Path.Log.Level.INFO, Path.Log.thisModule())


# ---------------------------------------------------------------------------
# Boundary faces from 3D geometry
# ---------------------------------------------------------------------------


def create_boundary_face(faces, offset=0.0, outline=True, compound=None, is_triangulated=False):
    """
    Creates a flat 2D boundary face from 3D faces using Path.Area's HLR
    projection (Outline mode) as primary method, falling back to
    TechDraw.findShapeOutline() if projection fails.

    Path.Area with Outline=True uses OCC's HLRBRep_Algo to project the
    3D shape silhouette onto the XY plane — more robust than TechDraw
    for complex curved and spiral faces where findShapeOutline() struggles.

    Args:
        faces (list): Part.Face objects to build the boundary from. Also
            used for mesh detection; pass the real list even when
            `compound` is given too, so mesh detection isn't skipped.
        offset (float): Offset to apply to the resulting boundary.
        outline (bool): Default 'True'. Keep only the outer envelope. 'False'
            preserves inner wires (holes), on a best-effort basis: the TechDraw
            fallback and mesh models always return the outline only.
        compound (Part.Shape, optional): A pre-built shape to use directly
            instead of rebuilding one from `faces` — pass this when the
            caller already has a cohesive shape (e.g. a fused multi-body
            model) to avoid reconstructing it from scratch.
        is_triangulated (bool): Pre-computed result of the mesh detection
            for `faces`.

    Returns:
        Part.Shape: The 2D boundary face, or None on failure.
    """
    if not faces and not compound:
        Path.Log.warning("No faces provided for boundary generation.")
        return None

    if faces and not compound:
        compound = faces[0] if len(faces) == 1 else Part.makeCompound(faces)

    if not is_triangulated:
        result = _boundary_via_area(compound, offset, outline)
        if result is not None:
            return result

    return _boundary_via_techdraw(compound, offset, outline)


def _boundary_via_area(compound, offset, outline):
    """
    Primary boundary engine: Path.Area projection/offset.

    Args:
        compound (Part.Shape): Shape to project — already built by the
            caller (create_boundary_face), not rebuilt here.
        offset (float): Offset to apply to the resulting boundary.
        outline (bool): Path.Area's Outline mode flag.

    Returns:
        Part.Shape: The resulting boundary, or None if Path.Area
        produced an empty/null shape or raised an exception.
    """
    try:
        wpc = Part.makeCircle(2)
        area = Path.Area()
        area.setPlane(wpc)
        area.add(compound)
        area.setParams(
            Outline=outline,
            Offset=offset,
            Coplanar=0,  # CoplanarNone — don't restrict to coplanar
            Fill=2,  # FillFace
        )
        result = area.getShape()

        if not result or result.isNull():
            Path.Log.debug(
                f"Path.Area returned an empty shape (offset={offset}, outline={outline})."
            )
            return None
        return result

    except Exception as e:
        Path.Log.debug(f"Path.Area projection failed: {e}")
        return None


def _boundary_via_techdraw(compound, offset, outline):
    """
    Secondary (fallback) boundary engine: TechDraw.findShapeOutline(),
    followed by a second Path.Area pass purely to apply `offset`.

    Note: findShapeOutline() only ever returns the outer envelope — unlike
    _boundary_via_area(), it cannot preserve inner wires (holes) when
    `outline` is False. If a caller needed holes preserved and lands
    here, that guarantee is lost, and a warning is logged.

    Args:
        compound (Part.Shape): Shape to project — already built by the
            caller (create_boundary_face), not rebuilt here.
        offset (float): Offset to apply to the resulting boundary.
        outline (bool): Path.Area's Outline mode flag.

    Returns:
        Part.Shape: The resulting boundary, or None on failure.
    """
    try:
        import TechDraw

        direction = FreeCAD.Vector(0, 0, 1)
        outline_shape = TechDraw.findShapeOutline(compound, 1.0, direction)

        if not outline_shape or outline_shape.isNull() or not outline_shape.Wires:
            Path.Log.debug("TechDraw returned an empty outline.")
            return None

        boundary = Part.makeFace(outline_shape.Wires, "Part::FaceMakerBullseye")
        if not boundary or boundary.isNull():
            Path.Log.debug("Failed to create a face from the TechDraw outline wires.")
            return None

        boundary.translate(FreeCAD.Vector(0, 0, -boundary.BoundBox.ZMin))

        offset_engine = Path.Area()
        offset_engine.add(boundary)
        offset_engine.setParams(Offset=offset)
        result = offset_engine.getShape()

        if not result or result.isNull():
            Path.Log.debug(f"TechDraw offset produced an empty shape (offset={offset}).")
            return None

        if not outline:
            Path.Log.warning(
                "Inner holes cannot be preserved for this geometry; only the outer outline is used."
            )

        return result

    except Exception as e:
        Path.Log.debug(f"TechDraw fallback failed: {e}")
        return None


# ---------------------------------------------------------------------------
# Fused boundaries of face groups
# ---------------------------------------------------------------------------


def build_optimized_boundary(faces, offset, outline=True):
    """
    Acts as a middleman to optimize boundary creation.

    Separates faces into connected groups and isolated faces. Each connected
    group is processed as a single batch — faces that touch transitively are
    guaranteed to be in the same batch, preventing TechDraw/ClipperLib
    artifacts from disjoint geometry. Isolated faces are processed one by one.

    Args:
        faces (list): List of Part.Face objects or nested list of faces.
        offset (float): Offset to apply to each boundary.
        outline (bool): Default 'True'. Keep only the outer envelope of each
            boundary. 'False' preserves inner wires (holes), on a best-effort
            basis (see create_boundary_face).

    Returns:
        Part.Shape: The combined boundary shape, or None on failure.
    """
    if not faces:
        return None

    touching_groups, isolated_faces = _separate_touching_faces(faces)

    Path.Log.debug(
        f"build_optimized_boundary: {len(touching_groups)} touching group(s), "
        f"{len(isolated_faces)} isolated face(s)."
    )

    generated_boundaries = []

    # Process each connected group as a single batch
    for group in touching_groups:
        bnd = create_boundary_face(group, offset, outline)
        if bnd and not bnd.isNull():
            generated_boundaries.append(bnd)

    # Process isolated faces one by one
    for face in isolated_faces:
        bnd = create_boundary_face([face], offset, outline)
        if bnd and not bnd.isNull():
            generated_boundaries.append(bnd)

    if not generated_boundaries:
        return None

    if len(generated_boundaries) == 1:
        return generated_boundaries[0]

    try:
        final_boundary = generated_boundaries[0].fuse(generated_boundaries[1:])
        if hasattr(final_boundary, "removeSplitter"):
            final_boundary = final_boundary.removeSplitter()
        return final_boundary
    except Exception as e:
        Path.Log.warning(
            f"build_optimized_boundary: Failed to fuse boundaries: {e}. "
            "Returning first boundary only."
        )
        return generated_boundaries[0]


def _separate_touching_faces(faces, tolerance=0.01):
    """
    Separates a list of faces into groups of touching faces and a list of
    isolated faces, based on XY bounding box overlap and physical distance.

    Uses a union-find (disjoint set) algorithm to correctly group transitively
    connected faces — if A touches B and B touches C, all three end up in the
    same group even if A and C don't directly touch.

    The bb_overlap pre-check tests both X and Y independently and only rejects
    when BOTH axes fail to overlap — a face touching only in Y is correctly
    identified as overlapping.

    Args:
        faces (list): A list of Part.Face objects or nested list of faces.
        tolerance (float): Maximum distance to be considered touching.

    Returns:
        tuple: (touching_groups, isolated_faces)
            touching_groups (list of lists): Each inner list is a group of
                mutually connected faces. Groups with a single face that
                touches another group are included here.
            isolated_faces (list): Faces that touch no other face.
    """
    if not faces:
        return [], []

    import math

    # Flatten input — handles both [Face, Face] and [[Face], [Face]]
    flat_faces = []
    for item in faces:
        if isinstance(item, list):
            flat_faces.extend(item)
        else:
            flat_faces.append(item)

    if not flat_faces:
        return [], []

    n = len(flat_faces)

    # XY-only bounding box overlap — Z deliberately excluded
    def bb_overlap(bb1, bb2, tol):
        if bb1.XMax < bb2.XMin - tol or bb1.XMin > bb2.XMax + tol:
            return False
        if bb1.YMax < bb2.YMin - tol or bb1.YMin > bb2.YMax + tol:
            return False
        return True  # Both axes overlap — faces may be touching

    # Union-Find implementation for transitive grouping
    parent = list(range(n))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]  # Path compression
            i = parent[i]
        return i

    def union(i, j):
        ri, rj = find(i), find(j)
        if ri != rj:
            parent[ri] = rj

    # Pre-compute bounding boxes once
    bboxes = [f.BoundBox for f in flat_faces]

    # Compare every pair — union touching faces into the same group
    for i in range(n):
        for j in range(i + 1, n):
            if not bb_overlap(bboxes[i], bboxes[j], tolerance):
                continue
            try:
                dist = flat_faces[i].distToShape(flat_faces[j])[0]
                if dist <= tolerance:
                    union(i, j)
                    continue
            except Exception as e:
                Path.Log.debug(
                    f"_separate_touching_faces: distToShape failed for " f"faces {i},{j}: {e}"
                )
            # Fallback: check if face centroids are within a larger
            # proximity threshold based on average face diagonal.
            try:
                bb_i = bboxes[i]
                bb_j = bboxes[j]
                cx_i = (bb_i.XMin + bb_i.XMax) / 2
                cy_i = (bb_i.YMin + bb_i.YMax) / 2
                cx_j = (bb_j.XMin + bb_j.XMax) / 2
                cy_j = (bb_j.YMin + bb_j.YMax) / 2
                centroid_dist = math.hypot(cx_i - cx_j, cy_i - cy_j)
                avg_diag = (
                    math.hypot(bb_i.XLength, bb_i.YLength) + math.hypot(bb_j.XLength, bb_j.YLength)
                ) / 2
                if centroid_dist < avg_diag * 0.75:
                    union(i, j)
            except Exception as e:
                Path.Log.debug(
                    f"_separatecreate_boundary_face_touching_faces: centroid check failed for "
                    f"faces {i},{j}: {e}"
                )

    # Collect groups by root
    from collections import defaultdict

    groups = defaultdict(list)
    for i in range(n):
        groups[find(i)].append(flat_faces[i])

    touching_groups = []
    isolated_faces = []

    for group in groups.values():
        if len(group) == 1:
            isolated_faces.append(group[0])
        else:
            touching_groups.append(group)

    return touching_groups, isolated_faces


def _offset_region(region, offset):
    """
    Grows (offset > 0) or shrinks a flat region with Path.Area.

    Same engine and settings as create_boundary_face (_boundary_via_area),
    keeping holes (no Outline).

    Args:
        region (Part.Shape): Flat faces (any Z).
        offset (float): Offset distance (mm).

    Returns:
        Part.Shape: The offset region, or None when it is empty or Path.Area
            fails.
    """
    result = _boundary_via_area(region, offset, outline=False)
    return result if result is not None and result.Faces else None


# ---------------------------------------------------------------------------
# Machining region (boundary_face)
# ---------------------------------------------------------------------------


# Gaps up to twice this between neighbouring selected faces are sealed (mm)
SEAM_GAP_OFFSET = 0.01


def build_machining_region(
    cutting_faces,
    tool_radius,
    boundary_adj,
    tolerance,
    outline=False,
    tool_side="inside",
):
    """
    Builds the 2D region the tool centre may visit within the selected faces,
    before Avoid Faces are removed.

    The outer boundary of 'cutting_faces' is shrunk inwards by the tool radius
    so the tool stays contained. Touching faces are fused first, so a shared
    edge is not shrunk. With outline=False, inner holes of the selected faces
    are preserved and expanded outward by the tool radius, so they don't need
    separate Avoid Faces; with outline=True only the outer envelope is kept.

    Args:
        cutting_faces (list): The selected Part.Face objects.
        tool_radius (float): The radius of the active cutter.
        boundary_adj (float): The Boundary Adjustment value.
        tolerance (float): The strategy's geometric tolerance (LinearDeflection
            for Surface Scan). Sets the small extra inset that avoids path
            spikes on vertical walls: max(0.01, tolerance + 0.001).
        outline (bool): Keep only the outer envelope of the selected faces
            (True), or also their inner holes (False, default).
        tool_side (str): Where the tool sits relative to the selected faces.
            "inside" (default): the whole tool stays inside them, offset
            -tool_radius, the small inset also inward. "outside": the tool
            edge may reach their outline from outside, offset +tool_radius,
            the inset outward. Waterline uses "outside": its contours run a
            tool radius outside the walls they finish.

    Returns:
        Part.Shape: The region (a face or a compound of faces), or None on failure.
    """
    if not cutting_faces:
        Path.Log.warning("No faces available to build the boundary mask.")
        return None

    # Add a small buffer to avoid "path spikes" on vertical walls. Outside,
    # it goes outward: a contour running exactly a tool radius off a wall
    # must not be clipped by the boundary taken from that wall.
    epsilon = max(0.01, tolerance + 0.001)
    if tool_side == "outside":
        offset = tool_radius + boundary_adj + epsilon
    else:
        offset = -tool_radius + boundary_adj - epsilon

    if len(cutting_faces) > 1:
        # Neighbouring faces often meet only within the model's tolerance
        # (imported models, blends): their union keeps hairline slits along
        # the seams, and an inward tool offset grows each one into a
        # tool-wide gap or cuts the region in two. Grow the union by
        # SEAM_GAP_OFFSET first (seals gaps up to twice that), then apply the
        # tool offset less that amount: the outline ends where it would have.
        sealed = build_optimized_boundary([cutting_faces], SEAM_GAP_OFFSET, outline=outline)
        region = _offset_region(sealed, offset - SEAM_GAP_OFFSET) if sealed else None
    else:
        region = build_optimized_boundary([cutting_faces], offset, outline=outline)

    if not region:
        Path.Log.warning(
            "Could not generate the boundary mask. The selected faces may be smaller than "
            "the tool diameter or contain invalid geometry; check the Boundary Adjustment."
        )
        return None
    return region


def cut_avoid_boundary(region, avoid_boundary):
    """
    Removes the pre-built Avoid Faces keep-out zones from a region in one cut.

    A compound of regions is cut as a whole and its faces stay separate; an
    avoid zone that crosses a face splits it into separate faces.

    Args:
        region (Part.Shape): A face, or a compound of faces.
        avoid_boundary (Part.Shape, optional): Keep-out zones from
            build_avoid_boundary(), already including the tool radius offset.

    Returns:
        Part.Shape: The region with the zones removed. If the cut fails the
        region is returned unchanged (the Avoid Faces are ignored, with a
        warning). None if 'region' is None.
    """
    if region is None or not avoid_boundary:
        return region
    try:
        result = region.cut(avoid_boundary)
        if result.isNull():
            Path.Log.warning(
                "Cutting the Avoid Faces from the boundary mask failed; they will be ignored."
            )
            return region
        return result
    except Exception as e:
        Path.Log.error(
            f"Failed to cut the Avoid Faces from the boundary mask: {e}; they will be ignored."
        )
        return region


def build_boundary_masks(
    cutting_faces,
    avoid_boundary,
    tool_radius,
    boundary_adj,
    tolerance,
    outline=False,
    per_face=False,
    tool_side="inside",
):
    """
    Builds the machining boundary of a Base Geometry selection.

    Without a selection the caller uses its outline (already offset) instead,
    with the Avoid zones cut out by cut_avoid_boundary().

    Args:
        cutting_faces (list): The selected Base Geometry faces.
        avoid_boundary (Part.Shape | None): The Avoid Faces keep-out zones from
            build_avoid_boundary(), already including the tool radius offset.
        tool_radius (float): The radius of the active tool.
        boundary_adj (float): The Boundary Adjustment value.
        tolerance (float): The strategy's geometric tolerance (see
            build_machining_region).
        outline (bool): Keep only the outer envelope of the selected faces.
        per_face (bool): Also build one mask per selected face (Surface Scan
            "Individually").
        tool_side (str): "inside" or "outside", see build_machining_region.

    Returns:
        tuple: (boundary_face, face_masks)
            boundary_face (Part.Shape | None): The selected faces (fused),
                offset by the tool radius, minus the avoid zones. None when no
                boundary can be built; the caller must not machine without it.
            face_masks (list | None): With per_face, the faces left after one
                cut of the avoid zones from all per-face regions: a face split
                by an avoid zone gives one mask per piece, a fully avoided face
                none. None otherwise.
    """
    region_args = (tool_radius, boundary_adj, tolerance, outline, tool_side)
    region = build_machining_region(cutting_faces, *region_args)
    if region is None:
        # build_machining_region() already logged why
        return None, None
    boundary_face = cut_avoid_boundary(region, avoid_boundary)

    if not per_face:
        return boundary_face, None

    # Each face is offset on its own (touching faces are not fused, so shared
    # edges are shrunk too), then all are cut in one go. A face too small for
    # the tool yields no region and is skipped.
    face_regions = []
    for face in cutting_faces:
        face_region = build_machining_region([face], *region_args)
        if face_region:
            face_regions.append(face_region)
    if not face_regions:
        return boundary_face, []
    masks = cut_avoid_boundary(Part.makeCompound(face_regions), avoid_boundary)
    return boundary_face, list(masks.Faces)


# ---------------------------------------------------------------------------
# Edge loops: selected edges -> flat regions
# ---------------------------------------------------------------------------

# Largest gap closed when chaining selected edges into a loop (mm)
EDGE_GAP_TOLERANCE = 0.1

# An edge whose Z span is below this is flat: moved to Z = 0 exactly
_FLAT_Z_TOL = 1e-6


def _dedupe(points, tol=1e-6):
    """Points without consecutive duplicates."""
    out = []
    for p in points:
        if not out or (p - out[-1]).Length > tol:
            out.append(p)
    return out


def _flatten_edge(edge, tolerance):
    """
    The edges lying on Z = 0 that follow *edge* in XY.

    Flat edges keep their exact geometry (an arc stays an arc), just moved to
    Z = 0. Anything else (3D rims, helices, B-splines) is discretized within
    *tolerance* and laid onto the plane. Vertical edges have no XY footprint
    and give nothing.

    Returns:
        list: Part.Edge objects at Z = 0 (may be empty).
    """
    bb = edge.BoundBox
    if bb.ZLength < _FLAT_Z_TOL:
        flat = edge.copy()
        flat.translate(FreeCAD.Vector(0, 0, -bb.ZMin))
        return [flat]

    try:
        pts = edge.discretize(Deflection=max(tolerance, 1e-4))
    except Part.OCCError:
        pts = edge.discretize(50)
    pts = _dedupe([FreeCAD.Vector(p.x, p.y, 0.0) for p in pts])
    if len(pts) < 2:
        return []
    return Part.makePolygon(pts).Edges


def _oriented_points(edges, tolerance):
    """
    The points of a sorted edge chain, each edge turned to continue from the
    previous one. Used where Part.Wire refuses a chain with small gaps.
    """
    polys = []
    for e in edges:
        try:
            polys.append(list(e.discretize(Deflection=max(tolerance, 1e-4))))
        except Part.OCCError:
            polys.append(list(e.discretize(20)))
    if not polys:
        return []

    # Orient the first edge toward the second, then follow the chain
    if len(polys) > 1:
        nxt = polys[1]
        d_end = min((polys[0][-1] - nxt[0]).Length, (polys[0][-1] - nxt[-1]).Length)
        d_start = min((polys[0][0] - nxt[0]).Length, (polys[0][0] - nxt[-1]).Length)
        if d_start < d_end:
            polys[0].reverse()

    pts = list(polys[0])
    for ep in polys[1:]:
        if (ep[-1] - pts[-1]).Length < (ep[0] - pts[-1]).Length:
            ep = list(reversed(ep))
        pts.extend(ep)
    return _dedupe(pts)


def _closed_polygon(pts, gap_tolerance):
    """A closed polygon wire through *pts*, closing a gap up to
    gap_tolerance; None when the gap is wider or there are too few points."""
    if len(pts) < 3:
        return None
    pts = list(pts)
    gap = (pts[0] - pts[-1]).Length
    if gap > gap_tolerance + 1e-7:
        return None
    if gap > 1e-7:
        pts.append(pts[0])
    else:
        pts[-1] = pts[0]
    try:
        poly = Part.makePolygon(pts)
    except Part.OCCError:
        return None
    return poly if poly.isClosed() else None


def _close_chain(edges, gap_tolerance, tolerance):
    """One sorted edge chain as a closed wire, keeping the exact geometry
    when it closes on its own; None if it is open by more than gap_tolerance."""
    try:
        wire = Part.Wire(edges)
        if wire.isClosed():
            return wire
    except Part.OCCError:
        pass
    return _closed_polygon(_oriented_points(edges, tolerance), gap_tolerance)


def _join_open_chains(chains, gap_tolerance):
    """
    Joins open point chains whose ends lie within gap_tolerance, nearest
    ends first, until no more join.

    Args:
        chains (list): Lists of FreeCAD.Vector, each an open polyline.

    Returns:
        list: The joined chains (some may still be open).
    """
    chains = [list(c) for c in chains if len(c) >= 2]
    joined = True
    while joined and len(chains) > 1:
        joined = False
        for i in range(len(chains)):
            a = chains[i]
            best = None
            for j in range(len(chains)):
                if i == j:
                    continue
                b = chains[j]
                for rev_b, pt in ((False, b[0]), (True, b[-1])):
                    d = (a[-1] - pt).Length
                    if d <= gap_tolerance + 1e-7 and (best is None or d < best[0]):
                        best = (d, j, rev_b)
            if best is not None:
                _, j, rev_b = best
                b = list(reversed(chains[j])) if rev_b else chains[j]
                chains[i] = _dedupe(a + b)
                del chains[j]
                joined = True
                break
    return chains


def wires_from_edges(edges, gap_tolerance=EDGE_GAP_TOLERANCE, tolerance=0.01):
    """
    Chains loose edges into closed wires on Z = 0.

    1. Every edge is flattened onto XY (_flatten_edge).
    2. Edges meeting within 0.001 mm are chained; a chain that closes on its
       own keeps its exact geometry.
    3. The remaining open chains are joined end to end, and closed, across
       gaps up to gap_tolerance.

    Args:
        edges (list): Part.Edge objects, in any order and direction.
        gap_tolerance (float): Largest gap closed (mm).
        tolerance (float): Discretization of non-flat edges (mm).

    Returns:
        tuple: (closed_wires, open_count): the closed wires, and how many
            chains stayed open (ignored).
    """
    flat = []
    for e in edges:
        flat.extend(_flatten_edge(e, tolerance))
    if not flat:
        return [], 0

    connect_tol = max(min(gap_tolerance, 0.001), 1e-7)
    try:
        groups = Part.sortEdges(flat, connect_tol)
    except (Part.OCCError, TypeError):
        groups = Part.sortEdges(flat)

    wires, open_chains = [], []
    for group in groups:
        wire = _close_chain(group, connect_tol, tolerance)
        if wire is None:
            open_chains.append(_oriented_points(group, tolerance))
        else:
            wires.append(wire)

    open_count = 0
    for chain in _join_open_chains(open_chains, gap_tolerance):
        wire = _closed_polygon(chain, gap_tolerance)
        if wire is None:
            open_count += 1
        else:
            wires.append(wire)
    return wires, open_count


def faces_from_wires(wires):
    """
    Closed wires on Z = 0 -> flat faces, nested loops as holes (even-odd).

    FaceMakerBullseye handles nested loops; loops that overlap or cross fall
    back to an exclusive-or of one face per loop through Path.Area.

    Returns:
        list: Part.Face objects (may be empty).
    """
    if not wires:
        return []
    try:
        face = Part.makeFace(wires, "Part::FaceMakerBullseye")
        if not face.isNull() and face.isValid() and face.Area > 1e-9:
            return face.Faces
    except Part.OCCError as e:
        Path.Log.debug(f"surface_boundary: Bullseye face making failed: {e}")

    singles = []
    for w in wires:
        try:
            singles.append(Part.Face(w))
        except Part.OCCError as e:
            Path.Log.debug(f"surface_boundary: skipping a loop that cannot be filled: {e}")
    if len(singles) < 2:
        return singles
    try:
        area = Path.Area()
        area.setPlane(Part.makeCircle(2))
        area.add(singles[0])
        for face in singles[1:]:
            area.add(face, op=3)  # exclusive-or
        area.setParams(Fill=2, Coplanar=0)
        result = area.getShape()
    except Exception as e:
        Path.Log.debug(f"surface_boundary: loop exclusive-or failed: {e}")
        return []
    return list(result.Faces) if result is not None and not result.isNull() else []


def edge_loop_regions(edges, gap_tolerance=EDGE_GAP_TOLERANCE, tolerance=0.01):
    """
    Selected edges -> the flat regions their closed loops enclose.

    Args:
        edges (list): Part.Edge objects (Base or Avoid selection).
        gap_tolerance (float): Largest gap closed when chaining (mm).
        tolerance (float): Discretization of non-flat edges (mm).

    Returns:
        tuple: (faces, open_count): the regions as faces at Z = 0, and how
            many chains stayed open (ignored).
    """
    if not edges:
        return [], 0
    wires, open_count = wires_from_edges(edges, gap_tolerance, tolerance)
    return faces_from_wires(wires), open_count


# ---------------------------------------------------------------------------
# Walls: vertical faces -> thin strips (Waterline)
# ---------------------------------------------------------------------------

# Half width of the strip standing in for a flat vertical face (mm). The
# strip is then offset like any selected face; this only gives it an area.
WALL_STRIP_HALF_WIDTH = 0.01


def is_vertical_plane(face):
    """True for a flat face standing upright: seen from above, a line."""
    try:
        if not isinstance(face.Surface, Part.Plane):
            return False
        u0, u1, v0, v1 = face.ParameterRange
        return abs(face.normalAt((u0 + u1) / 2.0, (v0 + v1) / 2.0).z) < 1e-6
    except Part.OCCError:
        return False


def wall_strip(face, tolerance=0.01, half_width=WALL_STRIP_HALF_WIDTH):
    """
    A flat vertical face as a thin strip along its footprint, at Z = 0.

    Seen from above the face is a straight segment and has no area, so no
    region can be built from it. The strip covers that segment (holes in the
    face, such as windows, are ignored: the footprint is the face's full
    length) and is half_width wide on each side.

    Args:
        face (Part.Face): A flat vertical face (see is_vertical_plane).
        tolerance (float): Discretization of curved edges (mm).
        half_width (float): Half the strip's width (mm).

    Returns:
        Part.Face: The strip, or None when the face has no length in XY.
    """
    pts = []
    for edge in face.Edges:
        for flat in _flatten_edge(edge, tolerance):
            pts.extend(v.Point for v in flat.Vertexes)
    if len(pts) < 2:
        return None
    u0, u1, v0, v1 = face.ParameterRange
    normal = face.normalAt((u0 + u1) / 2.0, (v0 + v1) / 2.0)
    n = FreeCAD.Vector(normal.x, normal.y, 0.0)
    if n.Length < 1e-9:
        return None
    n.normalize()
    d = FreeCAD.Vector(-n.y, n.x, 0.0)
    along = [p.dot(d) for p in pts]
    a, b = min(along), max(along)
    if b - a < 1e-6:
        return None
    c = sum(p.dot(n) for p in pts) / len(pts)
    p0, p1 = d * a + n * c, d * b + n * c
    w = n * half_width
    corners = [p0 - w, p1 - w, p1 + w, p0 + w, p0 - w]
    return Part.Face(Part.makePolygon(corners))


def _with_wall_strips(faces, tolerance, half_width=WALL_STRIP_HALF_WIDTH):
    """*faces* with every flat vertical face replaced by its wall_strip()
    (dropped when it has no length)."""
    out = []
    for face in faces:
        if is_vertical_plane(face):
            strip = wall_strip(face, tolerance, half_width)
            if strip is not None:
                out.append(strip)
        else:
            out.append(face)
    return out


# ---------------------------------------------------------------------------
# Operation boundaries (all regions of one run)
# ---------------------------------------------------------------------------


@dataclass
class Boundaries:
    """
    The 2D regions of one operation run, in the working frame.

    Built by build_operation_boundaries(), for the operation's run and for the
    task panel preview. All regions are flat (Z = 0).

    Attributes:
        cutting_faces (list): The Base Geometry faces, less any also in Avoid
            Faces. Empty: the whole model.
        avoid_faces (list): The Avoid Faces.
        bb_face (Part.Face | None): The outline, before any selection is
            applied, offset by Boundary Adjustment - tool radius:
              - Surface Scan, Z-Level: the stock outline (Boundary box = Stock),
                the bounding rectangle of the selected faces (BaseBoundBox with
                a selection) or the model outline (BaseBoundBox, nothing
                selected). Its BoundBox is also the rectangle Surface Scan lays
                its pattern out over, and the extent of the safe STL's pad.
              - Waterline with a selection: a frame around the model, standing
                in for "unbounded" (Waterline has no standard boundary).
              - Waterline without a selection: None.
        avoid_boundary (Part.Shape | None): The Avoid Faces keep-out zones,
            grown by the tool radius (less Avoid Faces Overlap).
        avoid_boundary_stl (Part.Shape | None): The same footprint without the
            tool radius, for the safe STL's keep-out lid. Built only when the
            run needs a safe STL.
        boundary_face (Part.Shape | None): The region the tool centre may
            visit: bb_face itself (nothing selected), bb_face less the Avoid
            zones (Avoid Faces only), or the selected faces offset by the tool
            radius less the Avoid zones. None for Waterline without a
            selection: it is not trimmed.
        face_masks (list | None): Surface Scan "Individually": one region per
            selected face, Avoid zones cut out. None otherwise.
        limits_waterline (bool): Waterline with a Base / Avoid selection: its
            contours are trimmed to boundary_face.
        base_loops (list): Flat faces enclosed by the closed loops of the
            Base edges: areas to machine, joined with cutting_faces.
        avoid_loops (list): The same for the Avoid edges: keep-out areas.
        open_chains (int): Selected edge chains that did not close (ignored).
    """

    cutting_faces: list = field(default_factory=list)
    avoid_faces: list = field(default_factory=list)
    bb_face: Optional[Any] = None
    avoid_boundary: Optional[Any] = None
    avoid_boundary_stl: Optional[Any] = None
    boundary_face: Optional[Any] = None
    face_masks: Optional[list] = None
    limits_waterline: bool = False
    base_loops: list = field(default_factory=list)
    avoid_loops: list = field(default_factory=list)
    open_chains: int = 0

    @property
    def has_selection(self):
        """A Base selection limits the area: faces or closed edge loops."""
        return bool(self.cutting_faces or self.base_loops)


# Small tolerance between the outline (model silhouette or stock) and the tool
# centre limit. It goes outward: nothing beyond the outline is taller than the
# surface, and a pass landing exactly on the boundary (Boundary Adjustment =
# tool radius, ...) is then kept on every side instead of being dropped by
# rounding.
OUTLINE_TOLERANCE = 0.01


def build_operation_boundaries(
    strategy,
    cutting_faces,
    avoid_faces,
    model_shape,
    optimized_shape,
    is_triangulated,
    stock_shape,
    bound_box,
    tool_radius,
    boundary_adjustment,
    avoid_overlap,
    linear_deflection,
    per_face=False,
    needs_safe_stl=False,
    scan_outline=False,
    cutting_edges=(),
    avoid_edges=(),
    contour_margin=0.1,
):
    """
    Builds all the 2D regions of one Planar Surface run (see Boundaries).

    Closed loops of selected edges (edge_loop_regions) count as flat faces:
    a Base loop is an area to machine, an Avoid loop a keep-out area.

    1. Avoid Faces: the keep-out zones (and their footprint for the safe STL).
    2. The outline (bb_face):
         - Surface Scan, Z-Level: the stock outline, the bounding rectangle of
           the selection, or the model outline, offset by Boundary Adjustment
           - tool radius + OUTLINE_TOLERANCE.
         - Waterline with a selection: a frame around the model.
    3. The tool-centre region (boundary_face): the outline itself, the outline
       less the Avoid zones, or the selected faces offset by the tool radius
       less the Avoid zones (build_boundary_masks). Per strategy:
         - Surface Scan: the selection's holes stay keep-out zones, unless
           scan_outline: then only its outer outline limits the scan.
         - Z-Level: only the selection's outer envelope limits the area.
         - Waterline: grown by the tool radius, as its contours run a tool
           radius outside the walls they finish.

    Args:
        strategy (str): "SurfaceScan", "ZLevelHybrid" or "Waterline".
        cutting_faces (list): The selected faces to machine (may be empty).
        avoid_faces (list): The Avoid Faces (may be empty).
        model_shape (Part.Shape): The fused model, in the working frame.
        optimized_shape (Part.Shape): The model without vertical faces, or
            model_shape.
        is_triangulated (bool): The model is mesh-derived.
        stock_shape (Part.Shape | None): The stock, in the working frame. Used
            with bound_box "Stock".
        bound_box (str): "BaseBoundBox" or "Stock" (Surface Scan, Z-Level).
        tool_radius (float): Cutter radius (mm).
        boundary_adjustment (float): Boundary Adjustment (mm).
        avoid_overlap (float): Avoid Faces Overlap (mm).
        linear_deflection (float): The run's linear deflection (mm).
        per_face (bool): Also build one region per selected face (Surface
            Scan "Individually").
        needs_safe_stl (bool): Also build the Avoid Faces footprint for the
            safe STL's keep-out lid.
        scan_outline (bool): Surface Scan: keep only the outer outline of the
            selected faces, so their holes are machined over (Boundary
            Outline). Z-Level and Waterline always do.
        cutting_edges (list): Selected Base edges (closed loops).
        avoid_edges (list): Selected Avoid edges (closed loops).
        contour_margin (float): Waterline: how far the Avoid zones shrink, so
            the avoided features' own contours (one tool radius off their
            walls) are kept (see build_avoid_boundary's tangent_margin).

    Waterline also takes single flat walls: a flat vertical face, which has
    no area seen from above, stands in as a thin strip along its footprint
    (wall_strip), in Base and in Avoid. Base: its contour is kept. Avoid: its
    contour is removed; the wall is what that contour touches, so its strip
    is widened by the contour margin (and epsilon) that otherwise keeps
    touching contours.

    Returns:
        Boundaries: The regions, or None when a boundary the strategy needs
            could not be built, or when the Base edges form no closed loop and
            nothing else is selected: the operation must not machine.
    """
    is_surface_scan = strategy == "SurfaceScan"
    is_waterline = strategy == "Waterline"
    is_zlevel = strategy == "ZLevelHybrid"
    result = Boundaries(cutting_faces=list(cutting_faces), avoid_faces=list(avoid_faces))

    # 0. Edge loops: flat faces joined with the selected faces
    result.base_loops, open_base = edge_loop_regions(cutting_edges, tolerance=linear_deflection)
    result.avoid_loops, open_avoid = edge_loop_regions(avoid_edges, tolerance=linear_deflection)
    result.open_chains = open_base + open_avoid
    if cutting_edges and not (result.base_loops or cutting_faces):
        # Edges meant to limit the area, but no loop: never the whole model
        return None
    cutting_faces = result.cutting_faces + result.base_loops
    avoid_faces = result.avoid_faces + result.avoid_loops
    if is_waterline:
        cutting_faces = _with_wall_strips(cutting_faces, linear_deflection)
        epsilon = max(0.01, linear_deflection + 0.001)
        avoid_faces = _with_wall_strips(
            avoid_faces, linear_deflection, WALL_STRIP_HALF_WIDTH + contour_margin + epsilon
        )

    # 1. Avoid Faces (and loops)
    if avoid_faces:
        result.avoid_boundary, result.avoid_boundary_stl = build_avoid_boundary(
            avoid_faces,
            avoid_overlap,
            tool_radius,
            linear_deflection,
            needs_safe_stl,
            tangent_margin=contour_margin if is_waterline else None,
        )

    # 2. Outline: Surface Scan and Z-Level
    needs_boundary = is_surface_scan or is_zlevel
    if needs_boundary:
        offset = boundary_adjustment - tool_radius + OUTLINE_TOLERANCE

        if bound_box == "Stock":
            result.bb_face = create_boundary_face(stock_shape.Faces, offset)
        elif cutting_faces:
            # The bounding rectangle of the selection
            bb = cutting_faces[0].BoundBox
            for face in cutting_faces[1:]:
                bb = bb.united(face.BoundBox)
            result.bb_face = _rectangle(
                bb.XMin - offset, bb.YMin - offset, bb.XMax + offset, bb.YMax + offset
            )
        else:
            # The model outline
            result.bb_face = create_boundary_face(
                model_shape.Faces,
                offset,
                outline=True,
                compound=optimized_shape,
                is_triangulated=is_triangulated,
            )

    # 2. Outline: Waterline, only to be limited by a selection. Its outer
    # contours run a tool radius outside the model: the frame clears them.
    result.limits_waterline = is_waterline and bool(cutting_faces or avoid_faces)
    if result.limits_waterline:
        mbb = model_shape.BoundBox
        grow = 2.0 * tool_radius + max(boundary_adjustment, 0.0) + 1.0
        result.bb_face = _rectangle(
            mbb.XMin - grow, mbb.YMin - grow, mbb.XMax + grow, mbb.YMax + grow
        )

    # 3. The tool-centre region
    result.boundary_face = result.bb_face
    if cutting_faces:
        tool_side = "inside"
        if is_zlevel:
            from .surface_zlevel import GEOMETRY_TOLERANCE

            tolerance, outline = GEOMETRY_TOLERANCE, True
        elif is_waterline:
            tolerance, outline, tool_side = linear_deflection, True, "outside"
        else:
            tolerance, outline = linear_deflection, bool(scan_outline)

        result.boundary_face, result.face_masks = build_boundary_masks(
            cutting_faces,
            result.avoid_boundary,
            tool_radius,
            boundary_adjustment,
            tolerance,
            outline=outline,
            per_face=per_face and is_surface_scan,
            tool_side=tool_side,
        )
    elif result.avoid_boundary is not None:
        result.boundary_face = cut_avoid_boundary(result.bb_face, result.avoid_boundary)

    # A boundary that failed must never fall back to machining everything
    if result.boundary_face is None and (needs_boundary or result.limits_waterline):
        return None
    return result


def _rectangle(x0, y0, x1, y1):
    """Flat rectangular face at Z = 0."""
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
    return Part.Face(Part.makePolygon([FreeCAD.Vector(x, y, 0) for x, y in corners]))


# ---------------------------------------------------------------------------
# Avoid Faces keep-out boundary
# ---------------------------------------------------------------------------


def build_avoid_boundary(
    avoid_faces,
    avoid_overlap,
    tool_radius,
    tolerance,
    needs_safe_stl=False,
    tangent_margin=None,
):
    """
    Builds the 2D "keep-out" geometry for user-selected Avoid Faces.

    Each raw face is first classified and, if needed, capped to a flat
    shape suitable for boundary generation:

        - Planar faces (flat or tilted, including ones with a genuine hole
          such as an annular/donut selection) are used as-is.
        - Non-planar faces (the cylindrical, conical, or otherwise curved
          wall of a hole or pocket) are reduced to a flat cap at their
          topmost rim, since a standard top-down projection would otherwise
          distort a tilted or cylindrical wall into an ellipse.
        - Any face that can't be resolved this way is grouped with the
          others like it and processed together as a fallback, rather than
          being left as raw, unprocessed geometry.

    Two shapes are then built from those prepared faces, because the two
    consumers need different geometry:
        - avoid_boundary: Cuts a matching hole out of the machining area.
          Offset by tool_radius - avoid_overlap plus a small buffer (epsilon)
          to avoid path spikes on vertical walls.
        - avoid_boundary_stl: Builds a collision-safety pillar around each
          Avoid Face. The same footprint without tool_radius (the OCL cutter
          already accounts for it): -avoid_overlap plus epsilon.

    With tangent_margin (Waterline) both shrink by that margin instead of
    growing by epsilon. A contour around an avoided feature runs exactly a
    tool radius off it, on the zone's edge: the margin keeps it, as it only
    touches the feature's walls, while anything reaching further in is
    removed. The margin must exceed the contour's own deviation from the
    exact offset (OCL samples it), or the contour is chopped where it wanders
    across the edge.

    Args:
        avoid_faces (list): Raw Part.Face objects selected by the user as
            Avoid Faces.
        avoid_overlap (float): How far the tool may cut into the Avoid
            Faces (0.0 when Avoid Faces Overlap is disabled).
        tool_radius (float): The tool radius. Only the machining-area
            boundary is expanded by it.
        tolerance (float): The deflection tolerance for discretizing
            curves smoothly.
        needs_safe_stl (bool): Build the extra boundary for the Safe STL
            pillar.
        tangent_margin (float, optional): Shrink the zones by this much
            instead of growing them by epsilon (see above). None: epsilon.

    Returns:
        tuple: (avoid_boundary, avoid_boundary_stl)
            avoid_boundary (Part.Shape | None): Boundary used to cut the
                hole in the machining area, or None if avoid_faces is
                empty or generation fails.
            avoid_boundary_stl (Part.Shape | None): The same footprint
                without the tool_radius offset, used for the Safe STL
                pillar. None if needs_safe_stl is False or the boundary
                collapsed.
    """
    if not avoid_faces:
        return None, None

    avoid_boundary_stl = None

    prepared_faces, fallback_faces = _classify_and_cap_faces(avoid_faces)

    if fallback_faces:
        # Hole walls that couldn't be capped: keep the default outline=True so the
        # hole interior is filled. Preserving inner wires would leave the hole open.
        secondary = build_optimized_boundary(fallback_faces, 0.0)
        if secondary is not None:
            prepared_faces.append(secondary)
        else:
            Path.Log.warning(
                f"Failed to build a fallback boundary for {len(fallback_faces)} unresolved "
                "avoid face(s); they will not be avoided."
            )

    if not prepared_faces:
        Path.Log.debug("build_avoid_boundary: Nothing left to build a boundary from.")
        return None, None

    # Small buffer to avoid "path spikes" on vertical walls (outward), or the
    # Waterline tangent margin (inward)
    epsilon = max(0.01, tolerance + 0.001)
    if tangent_margin is not None:
        epsilon = -tangent_margin
    avoid_offset = tool_radius - avoid_overlap

    avoid_boundary = build_optimized_boundary(
        prepared_faces,
        avoid_offset + epsilon,
        outline=False,
    )

    if not avoid_boundary or avoid_boundary.isNull():
        Path.Log.error(
            "Could not build the boundary for the Avoid Faces; they will not be avoided. "
            "Check the selected faces and the Avoid Faces Overlap value."
        )
        return None, None

    if needs_safe_stl:
        avoid_boundary_stl = build_optimized_boundary(
            prepared_faces,
            -avoid_overlap + epsilon,
            outline=False,
        )
        if avoid_boundary_stl is not None and avoid_boundary_stl.isNull():
            avoid_boundary_stl = None

    return avoid_boundary, avoid_boundary_stl


def _classify_and_cap_faces(raw_faces):
    """
    Sorts raw Avoid Faces into two buckets: faces already usable as-is,
    and faces that need further, batched processing.

    For each face:
      - Faces with no edges are set aside for the fallback batch.
      - Planar faces (flat or tilted, including ones with a genuine hole
        such as an annular/donut selection) are used as-is.
      - Non-planar faces (a cylindrical, conical, or otherwise curved
        wall) are capped at their topmost rim. If that succeeds, the cap
        is used; if not, the original face is set aside for the fallback
        batch instead.

    Args:
        raw_faces (list): Raw Part.Face objects selected by the user.

    Returns:
        tuple: (prepared_faces, fallback) — prepared_faces is a list of
        faces ready to use directly; fallback is a list of faces that
        couldn't be individually resolved and need further processing.
    """
    prepared_faces = []
    fallback_faces = []

    for raw_face in raw_faces:
        if not hasattr(raw_face, "Edges") or not raw_face.Edges:
            fallback_faces.append(raw_face)
            continue

        # Skip if face is planar
        try:
            is_planar = isinstance(raw_face.Surface, Part.Plane)
        except Exception:
            is_planar = False

        if is_planar:
            prepared_faces.append(raw_face)
            continue

        # Non-Planar 3D Walls (Sloped, Tapered, Curved)
        face_zmax = round(raw_face.BoundBox.ZMax, 4)

        top_wire = _extract_top_rim_wire(raw_face)
        if top_wire is None:
            fallback_faces.append(raw_face)
            continue

        cap_face = flatten_wire_to_cap(top_wire, face_zmax, translate_to_zero=False)
        if cap_face is not None:
            prepared_faces.append(cap_face)
        else:
            fallback_faces.append(raw_face)

    return prepared_faces, fallback_faces


def _extract_top_rim_wire(face, tolerance=1e-3):
    """
    Extracts just the top rim of a non-planar 3D wall face (sloped,
    tapered, or curved) as a single Part.Wire, skipping vertical seam
    edges that run the full height of the wall.

    Shared by _classify_and_cap_faces() here and surface_zlevel's
    _cap_3d_wall() — both need to find "the top edge loop of an
    arbitrary 3D wall" and previously duplicated this logic separately.

    Returns:
        Part.Wire or None: the top rim wire, or None if no top edges
        were found or the wire couldn't be assembled.
    """
    face_zmax = round(face.BoundBox.ZMax, 4)
    face_zmin = round(face.BoundBox.ZMin, 4)

    top_edges = []
    for edge in face.Edges:
        # Skip "seam" edges that run vertically down the full wall height
        is_seam = (round(edge.BoundBox.ZMin, 4) <= face_zmin + tolerance) and (
            round(edge.BoundBox.ZMax, 4) >= face_zmax - tolerance
        )
        if is_seam:
            continue
        # Keep only the edges that form the upper rim
        if round(edge.BoundBox.ZMax, 4) >= face_zmax - tolerance:
            top_edges.append(edge)

    if not top_edges:
        return None

    try:
        sorted_edges = Part.__sortEdges__(top_edges)
        return Part.Wire(sorted_edges)
    except Exception as e:
        Path.Log.debug(f"_extract_top_rim_wire: Failed to assemble top rim wire: {e}")
        return None


def flatten_wire_to_cap(wire, cap_z, discretize_distance=0.1, translate_to_zero=False):
    """
    Discretizes a wire, flattens all edges onto cap_z, and returns a
    closed Part.Face. Safely handles curved 3D edges (e.g. the rim of
    an angled hole) by crushing them to a flat 2D polygon rather than
    attempting an exact projection, which can crash OpenCASCADE for
    some inputs.

    Shared by _classify_and_cap_faces() here and surface_zlevel's
    _cap_3d_wall()/_cap_flat_face_holes() — all three previously
    duplicated this flatten-and-rebuild logic separately, with subtly
    different edge-case handling between copies.

    Args:
        wire (Part.Wire): the (possibly 3D/curved) wire to flatten.
        cap_z (float): the Z height to flatten edges onto.
        discretize_distance (float): point spacing used to discretize
            curved edges before flattening.
        translate_to_zero (bool): if True, translates the resulting
            face so its ZMin is 0 (for use as a Z=0 mask); if False,
            leaves it at cap_z.

    Returns:
        Part.Face or None: the flattened cap face, or None on failure.
    """
    flat_edges = []
    for edge in wire.Edges:
        try:
            points = edge.discretize(Distance=discretize_distance)
            flat_pts = [FreeCAD.Vector(p.x, p.y, cap_z) for p in points]
            flat_edge = Part.makePolygon(flat_pts)
            flat_edges.extend(flat_edge.Edges)
        except Exception as e:
            # A single bad edge shouldn't sink the whole cap — skip it
            # and keep going with the rest.
            Path.Log.debug(f"flatten_wire_to_cap: Edge flatten failed, skipping: {e}")
            continue

    if not flat_edges:
        return None

    try:
        sorted_edges = Part.__sortEdges__(flat_edges)
        flat_wire = Part.Wire(sorted_edges)
        cap_face = Part.Face(flat_wire)

        if not cap_face.isValid() or cap_face.isNull():
            return None

        if translate_to_zero:
            cap_face.translate(FreeCAD.Vector(0, 0, -cap_face.BoundBox.ZMin))

        return cap_face
    except Exception as e:
        Path.Log.debug(f"flatten_wire_to_cap: Failed to build cap face: {e}")
        return None


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
    caster does: holes (Avoid Faces zones) are outside, islands inside holes
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

        If the offset fails the boundary itself is used: selection is then
        tighter by the reach, and faces just outside the boundary may be left
        out of the mesh.

        Returns:
            tuple: (polygons, bounds) of the grown boundary; bounds is None when
                there are no polygons.
        """
        if self._reach_polys is None:
            shape = self.region
            if self.reach > 0.0 and self.region is not None:
                shape = _offset_region(self.region, self.reach) or self.region
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
