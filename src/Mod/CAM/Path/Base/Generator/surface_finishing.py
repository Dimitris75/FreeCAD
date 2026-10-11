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

"""
Steep-Wall and Fillet Finishing Passes for the 3D Surface Operation.

Surface Scan projects a 2D pattern onto the model, which spaces passes
evenly in XY. On steep walls and fillets that spacing stretches out on the
surface, so this module finishes them separately (Single-pass only):

1.  **Classification:** split_finishing_faces() separates steep walls
    (split_steep_faces) and fillets with their corner patches
    (split_fillet_faces) from the faces handled by the main pattern.
2.  **Keep-out zone:** build_feature_avoid_boundary() keeps the main pattern
    off those faces, using their true XY footprint.
3.  **Steep walls:** generate_steep_scan_lines() slices the walls at evenly
    spaced heights and offsets each section by the tool radius, away from
    the material.
4.  **Fillets:** generate_fillet_scan_lines() runs flow lines along each
    fillet at a constant step over on the surface, carries them around
    corner patches and joins them into continuous chains.

Like surface_pattern, the generators return lists of 2D (x, y, z=0)
coordinates, ready to be projected onto the model by the OCL drop-cutter.
"""

import math

import FreeCAD
import Part
import Path

from Path.Base.Generator.surface_common import build_optimized_boundary

__title__ = "Surface Scan Steep-Wall and Fillet Finishing"
__author__ = "sliptonic (Brad Collette)"
__url__ = "https://www.freecad.org"

if False:
    Path.Log.setLevel(Path.Log.Level.DEBUG, Path.Log.thisModule())
    Path.Log.trackModule(Path.Log.thisModule())
else:
    Path.Log.setLevel(Path.Log.Level.INFO, Path.Log.thisModule())


# ---------------------------------------------------------------------------
# Steep wall detection
# ---------------------------------------------------------------------------


def _face_normal_z(face, u, v):
    """Returns abs(normal.z) as a unit value at one parametric point.

    normalAt() is not guaranteed to return a unit vector for every surface
    type, so the result is normalized here. abs() is deliberate: a face whose
    normal is reported upside-down has the same inclination, and this
    classification does not care which side the material is on.
    """
    norm = face.normalAt(u, v)
    length = norm.Length

    if length < 1e-12:
        return None

    return abs(norm.z) / length


def _needed_hits(valid, coverage):
    """Qualifying samples required out of `valid` (same rule as the classifiers)."""
    return max(1, int(round(coverage * valid)))


def _coverage_outcome(hits, valid, remaining, coverage):
    """
    Decides a sampled coverage test early, when the remaining samples can no
    longer change the result. Exactly equivalent to evaluating every sample.

    Args:
        hits (int): Qualifying samples so far.
        valid (int): Valid samples so far.
        remaining (int): Samples not yet evaluated (each may be valid or not).
        coverage (float): Required fraction of valid samples.

    Returns:
        True if it passes whatever comes, False if it cannot pass, else None.
    """
    # Worst case: every remaining sample valid but not qualifying.
    if hits >= _needed_hits(valid + remaining, coverage):
        return True
    # Best case for some split of the remaining samples into qualifying and
    # invalid ones; if none of them reaches the requirement, it cannot pass.
    if all(hits + a < _needed_hits(valid + a, coverage) for a in range(remaining + 1)):
        return False
    return None


def _is_steep_by_sampling(face, lo, hi, samples, coverage):
    """
    Sampled steep test for curved faces, stopping as soon as the result is
    certain. Gives the same answer as sampling the whole grid.

    isPartOfDomain() is the expensive call (it builds a trimming classifier
    every time), so with full coverage it is only called where it can matter:
    for samples outside the band, and until one valid sample confirms the
    face has any valid area.
    """
    u1, u2, v1, v2 = face.ParameterRange
    grid = [
        (u1 + (u2 - u1) * (i + 0.5) / samples, v1 + (v2 - v1) * (j + 0.5) / samples)
        for i in range(samples)
        for j in range(samples)
    ]

    if coverage >= 1.0:
        have_valid = False
        for u, v in grid:
            try:
                nz = _face_normal_z(face, u, v)
                if nz is None:
                    continue  # Invalid sample, as in the full grid
                in_band = lo - 1e-9 <= nz <= hi + 1e-9
                if in_band and have_valid:
                    continue  # Cannot fail the face, valid or not
                if not face.isPartOfDomain(u, v):
                    continue
            except Exception:
                continue
            if not in_band:
                return False  # A valid sample outside the band
            have_valid = True
        return have_valid

    hits = valid = 0
    for index, (u, v) in enumerate(grid):
        try:
            if face.isPartOfDomain(u, v):
                nz = _face_normal_z(face, u, v)
                if nz is not None:
                    valid += 1
                    if lo - 1e-9 <= nz <= hi + 1e-9:
                        hits += 1
        except Exception:
            pass
        outcome = _coverage_outcome(hits, valid, len(grid) - index - 1, coverage)
        if outcome is not None:
            return outcome and valid > 0
    return valid > 0 and hits >= _needed_hits(valid, coverage)


def _steep_by_type(face, lo, hi):
    """
    Steep test for faces whose inclination is constant: planes, and
    cylinders or cones with a vertical axis. Returns None for anything else.
    """
    surf = face.Surface
    tid = getattr(surf, "TypeId", "")
    if "Plane" in tid:
        nz = _face_normal_z(face, *_mid_parameters(face))
    elif "Cylinder" in tid or "Cone" in tid:
        axis = surf.Axis
        if abs(axis.z) / axis.Length < 1.0 - 1e-9:
            return None  # Tilted axis: inclination varies around it
        nz = abs(math.sin(surf.SemiAngle)) if "Cone" in tid else 0.0
    else:
        return None
    return nz is not None and lo - 1e-9 <= nz <= hi + 1e-9


def _mid_parameters(face):
    u1, u2, v1, v2 = face.ParameterRange
    return (u1 + u2) / 2.0, (v1 + v2) / 2.0


def split_steep_faces(
    model_faces,
    max_draft_angle=18.0,
    min_draft_angle=0.0,
    samples=4,
    coverage=1.0,
    min_area=0.0,
):
    """
    Splits faces into "steep" (near-vertical, low draft angle) faces and the rest.

    The draft angle follows mold-making convention and is measured from the
    vertical pull direction: a perfectly vertical wall is 0 degrees, a flat
    floor is 90 degrees. For a unit normal n this gives sin(draft) = abs(n.z),
    so a face qualifies while abs(n.z) stays inside the requested band.

    These are the faces where a projected 2D pattern breaks down: the stepover
    measured along the surface grows as step_over / sin(draft), so a 3 degree
    wall receives its passes roughly 19 times further apart than intended.

    Curved faces count as steep only when `coverage` of their sampled normals
    fall inside the band, so a drafted wall that blends into a floor fillet is
    not classified from a single lucky sample.

    Cost order: planes and vertical-axis cylinders/cones are decided from
    their type; other faces are sampled with an early exit; the area (a
    surface integral) is only computed for faces that would otherwise
    qualify.

    Args:
        model_faces (list): Part.Face objects to classify.
        max_draft_angle (float): Upper bound of the draft band, in degrees.
        min_draft_angle (float): Lower bound, in degrees. Leave at 0.0 to
            include perfectly vertical walls.
        samples (int): Grid resolution per parametric direction for curved
            faces. Planar faces always use a single normal.
        coverage (float): Fraction of valid samples that must fall inside the
            band, 0.0 to 1.0. 1.0 requires the whole face to qualify.
        min_area (float): Faces smaller than this are never reported as steep,
            which keeps small fillets and chamfers out of the steep set.

    Returns:
        tuple: (steep_faces, other_faces)
    """
    if not model_faces:
        return [], []

    lo = math.sin(math.radians(max(0.0, min(90.0, min_draft_angle))))
    hi = math.sin(math.radians(max(0.0, min(90.0, max_draft_angle))))
    if hi < lo:
        lo, hi = hi, lo

    steep = []
    other = []

    for face in model_faces:
        try:
            is_steep = _steep_by_type(face, lo, hi)
            if is_steep is None:
                if samples < 2:
                    nz = _face_normal_z(face, *_mid_parameters(face))
                    is_steep = nz is not None and lo - 1e-9 <= nz <= hi + 1e-9
                else:
                    is_steep = _is_steep_by_sampling(face, lo, hi, samples, coverage)
            if is_steep and min_area > 0.0 and face.Area < min_area:
                is_steep = False
        except Exception as e:
            Path.Log.debug(f"split_steep_faces: skipping face ({e}).")
            is_steep = False

        (steep if is_steep else other).append(face)

    Path.Log.debug(
        f"split_steep_faces: {len(steep)} steep face(s) within "
        f"{min_draft_angle}-{max_draft_angle} deg draft, {len(other)} other."
    )

    return steep, other


# ---------------------------------------------------------------------------
# Fillet detection
# ---------------------------------------------------------------------------


def _is_fillet_by_curvature(face, min_curvature, flatness, samples, coverage):
    """
    Curvature test for faces without an analytic fillet type (B-splines,
    swept or variable-radius blends).

    A fillet is tightly curved in one direction (across the blend) and nearly
    straight in the other (along it): |k_max| >= min_curvature and
    |k_min| <= flatness * |k_max| at enough sample points.
    """
    def qualifies(k1, k2):
        k_max, k_min = max(k1, k2), min(k1, k2)
        return k_max >= min_curvature and k_min <= flatness * k_max

    return _curvature_coverage(face, qualifies, samples, coverage)


def _curvature_coverage(face, qualifies, samples, coverage):
    """
    Samples |principal curvatures| on a grid and applies the coverage rule,
    stopping as soon as the result is certain (same answer as sampling the
    whole grid). curvatureAt() is the costly call, so the domain test runs
    first and rejects trimmed-away points before it.
    """
    u1, u2, v1, v2 = face.ParameterRange
    total = samples * samples
    hits = valid = done = 0
    for i in range(samples):
        u = u1 + (u2 - u1) * (i + 0.5) / samples
        for j in range(samples):
            v = v1 + (v2 - v1) * (j + 0.5) / samples
            done += 1
            try:
                if face.isPartOfDomain(u, v):
                    k1, k2 = (abs(k) for k in face.curvatureAt(u, v))
                    valid += 1
                    if qualifies(k1, k2):
                        hits += 1
            except Exception:
                pass
            outcome = _coverage_outcome(hits, valid, total - done, coverage)
            if outcome is not None:
                return outcome and valid > 0
    return valid > 0 and hits >= _needed_hits(valid, coverage)


def _is_concave(face):
    """
    True when the face curves toward its open side (an inside blend).

    Uses the oriented normals, which point away from the material: moving
    across a concave surface they converge, across a convex one they spread
    apart. dot(dn, dp) / |dp|^2 estimates the signed curvature in each
    parametric direction.

    Only the curvature across the blend decides: the minor circle of a torus,
    the circle of a cylinder. That matters for saddles, such as a roundover
    turning around a pocket corner: convex across, concave around the corner.
    For other surfaces the two directions are summed, which is dominated by
    the tight curve across a fillet.
    """
    u1, u2, v1, v2 = face.ParameterRange
    um, vm = (u1 + u2) / 2.0, (v1 + v2) / 2.0
    du, dv = (u2 - u1) / 8.0, (v2 - v1) / 8.0

    def unit_normal(u, v):
        n = face.normalAt(u, v)
        return n * (1.0 / n.Length)

    def signed_curvature(a, b):
        dp = face.valueAt(*b) - face.valueAt(*a)
        dn = unit_normal(*b) - unit_normal(*a)
        length_sq = dp.x * dp.x + dp.y * dp.y + dp.z * dp.z
        if length_sq < 1e-18:
            return 0.0
        return (dn.x * dp.x + dn.y * dp.y + dn.z * dp.z) / length_sq

    k_u = signed_curvature((um - du, vm), (um + du, vm))
    k_v = signed_curvature((um, vm - dv), (um, vm + dv))

    tid = getattr(face.Surface, "TypeId", "")
    if "Toroid" in tid:
        return k_v < 0.0  # v runs around the minor circle
    if "Cylinder" in tid:
        return k_u < 0.0  # u runs around the circle
    return k_u + k_v < 0.0


def _is_corner_blend(face, fillet_edge_keys, min_curvature, samples, coverage):
    """
    A corner blend: the patch inserted where fillets meet at a corner, or
    where one turns around a corner (images: a spherical octant at a box
    corner, a saddle at a pocket rim corner).

    It shares an edge with a detected fillet and is tightly curved (by
    max_radius), but unlike a fillet it may curve in both directions, so the
    flatness test does not apply.
    """
    # Cheapest first: the surface type, then shared edges, then curvature
    # sampling. Planes (the bulk of most models) never qualify, and neither
    # do cylinders and cones: corner patches are spheres, tori or B-spline
    # vertex blends, while a cylinder touching a fillet is a wall (such as a
    # boss between its top roundover and its foot fillet) and a small tilted
    # cylinder is already a fillet in its own right.
    tid = getattr(face.Surface, "TypeId", "")
    if any(t in tid for t in ("Plane", "Cylinder", "Cone")):
        return False
    if not any(e.hashCode() in fillet_edge_keys for e in face.Edges):
        return False
    if "Sphere" in tid:
        return face.Surface.Radius <= 1.0 / min_curvature

    return _curvature_coverage(
        face, lambda k1, k2: max(k1, k2) >= min_curvature, samples, coverage
    )


def split_fillet_faces(
    faces, max_radius=10.0, samples=3, coverage=0.75, flatness=0.2, include_concave=True
):
    """
    Splits faces into fillets (blends, including their corner patches) and
    the rest.

    Run this after split_steep_faces(): vertical cylinders (small bores and
    bosses) are steep walls, not fillets, and are rejected here as well.

    Detection:
        Cylinder  radius <= max_radius and axis not vertical
        Toroid    minor radius <= max_radius (fillet around a curved edge)
        Other     curvature test (see _is_fillet_by_curvature); planes and
                  other analytic types never qualify.
        Corners   curved patches sharing an edge with a fillet (see
                  _is_corner_blend). They take the inside/outside
                  classification of the fillets they touch.

    Args:
        faces (list): Part.Face objects to classify.
        max_radius (float): Largest blend radius treated as a fillet.
        samples (int): Curvature grid resolution per parametric direction.
        coverage (float): Fraction of curvature samples that must qualify.
        flatness (float): Max ratio of the lower to the higher principal
            curvature for a sample to count as fillet-like.
        include_concave (bool): False leaves concave (inside) fillets and
            their corners in other_faces, so only convex roundovers get
            fillet passes.

    Returns:
        tuple: (fillet_faces, other_faces)
    """
    if not faces:
        return [], []

    min_curvature = 1.0 / max_radius
    fillets, other = [], []

    for face in faces:
        try:
            surf = face.Surface
            tid = getattr(surf, "TypeId", "")
            if "Cylinder" in tid:
                axis = surf.Axis
                is_fillet = surf.Radius <= max_radius and abs(axis.z) / axis.Length < 0.99
            elif "Toroid" in tid:
                is_fillet = surf.MinorRadius <= max_radius
            elif any(t in tid for t in ("BSpline", "Bezier", "Offset", "Sweep", "Extrusion")):
                is_fillet = _is_fillet_by_curvature(
                    face, min_curvature, flatness, samples, coverage
                )
            else:
                is_fillet = False
        except Exception as e:
            Path.Log.debug(f"split_fillet_faces: skipping face ({e}).")
            is_fillet = False

        (fillets if is_fillet else other).append(face)

    # Corner blends: curved patches touching a fillet.
    corners = []
    if fillets:
        edge_keys = {e.hashCode() for f in fillets for e in f.Edges}
        remaining = []
        for face in other:
            try:
                is_corner = _is_corner_blend(face, edge_keys, min_curvature, samples, coverage)
            except Exception as e:
                Path.Log.debug(f"split_fillet_faces: corner test failed ({e}).")
                is_corner = False
            (corners if is_corner else remaining).append(face)
        other = remaining

    if not include_concave:
        kept = []
        # Edge -> concave flags of the fillets using it, for the corner vote.
        edge_votes = {}
        for face in fillets:
            try:
                concave = _is_concave(face)
            except Exception:
                concave = False
            (other if concave else kept).append(face)
            for key in {e.hashCode() for e in face.Edges}:
                edge_votes.setdefault(key, []).append((id(face), concave))
        # A corner follows the fillets it touches (majority vote, one vote
        # per fillet even when it shares several edges with the corner).
        for face in corners:
            votes = {}
            for e in face.Edges:
                votes.update(edge_votes.get(e.hashCode(), ()))
            if votes and sum(votes.values()) * 2 > len(votes):
                other.append(face)
            else:
                kept.append(face)
        fillets = kept
    else:
        fillets = fillets + corners

    Path.Log.debug(
        f"split_fillet_faces: {len(fillets)} fillet face(s) up to R{max_radius} "
        f"(incl. {len(corners)} corner candidate(s)), {len(other)} other."
    )
    return fillets, other


# ---------------------------------------------------------------------------
# Classification entry point
# ---------------------------------------------------------------------------


# Faces lower than this (mm) are treated as flat by split_finishing_faces().
_FLAT_FACE_HEIGHT = 0.001


def split_finishing_faces(
    faces,
    finish_steep=True,
    finish_fillets=True,
    steep_max_draft_angle=18.0,
    steep_min_draft_angle=0.0,
    fillet_max_radius=10.0,
    fillet_include_concave=False,
):
    """
    Splits faces into steep walls, fillets and the rest, according to the
    Finishing Passes settings.

    Steep walls are separated first: vertical cylinders and cones (bores,
    bosses) are walls, not fillets. Fillets, with their corner patches, are
    then taken from what remains.

    Args:
        faces (list): Part.Face objects to classify.
        finish_steep (bool): Look for steep walls.
        finish_fillets (bool): Look for fillets.
        steep_max_draft_angle (float): Upper draft bound for steep walls, in
            degrees from vertical.
        steep_min_draft_angle (float): Lower draft bound, in degrees.
        fillet_max_radius (float): Largest blend radius treated as a fillet.
        fillet_include_concave (bool): Also take concave (inside) fillets.

    Returns:
        tuple: (steep_faces, fillet_faces, other_faces)
    """
    # Truly flat faces (less than _FLAT_FACE_HEIGHT tall) can be neither a
    # steep wall nor a fillet, so they go straight to the main pattern.
    # Planes are skipped here: the steep split decides them from one normal
    # and the fillet tests reject them on type, both cheaper than a bounding
    # box. The check pays off on flat B-spline faces (common in imported
    # models), which would otherwise go through curvature sampling.
    flat_ids = set()
    if finish_steep or finish_fillets:
        for face in faces:
            try:
                if (
                    "Plane" not in getattr(face.Surface, "TypeId", "")
                    and face.BoundBox.ZLength < _FLAT_FACE_HEIGHT
                ):
                    flat_ids.add(id(face))
            except Exception:
                pass

    steep, rest = [], [f for f in faces if id(f) not in flat_ids]
    if finish_steep:
        steep, rest = split_steep_faces(
            rest,
            max_draft_angle=steep_max_draft_angle,
            min_draft_angle=steep_min_draft_angle,
        )
    fillets = []
    if finish_fillets:
        fillets, rest = split_fillet_faces(
            rest,
            max_radius=fillet_max_radius,
            include_concave=fillet_include_concave,
        )
    if flat_ids:
        taken = {id(f) for f in steep} | {id(f) for f in fillets}
        rest = [f for f in faces if id(f) not in taken]  # Original order
    return steep, fillets, rest


# ---------------------------------------------------------------------------
# Main-pattern keep-out zone
# ---------------------------------------------------------------------------


def build_feature_avoid_boundary(steep_faces, fillet_faces, tool_radius, tolerance):
    """
    Builds the main pattern's keep-out zone for steep walls and fillets, which
    are finished by their own passes.

    Unlike build_avoid_boundary(), the faces are not capped at their top rim:
    capping is meant for hole walls, and would cover a whole boss top (or miss
    a fillet band) here. The true XY footprint is used instead, holes kept:

        - Steep walls grow by the tool radius. A ball touches a floor directly
          below its center, so a center one radius from the wall foot cuts
          the floor right up to the wall without climbing it.
        - Fillets grow by nothing. Main-pattern centers reach the fillet's
          tangent line, where the ball touches the floor or top face, and the
          fillet passes take over from there.

    These faces are part of the model, so no Safe STL pillar is built for them.

    Args:
        steep_faces (list): Part.Face objects classified as steep walls.
        fillet_faces (list): Part.Face objects classified as fillets.
        tool_radius (float): The tool radius.
        tolerance (float): The deflection tolerance.

    Returns:
        Part.Shape or None: The fused keep-out footprint.
    """
    epsilon = max(0.01, tolerance + 0.001)
    parts = []
    for faces, offset in ((steep_faces, tool_radius + epsilon), (fillet_faces, epsilon)):
        if not faces:
            continue
        boundary = build_optimized_boundary(faces, offset, outline=False)
        if boundary is None or boundary.isNull():
            Path.Log.warning(
                f"Could not build the keep-out zone for {len(faces)} steep/fillet face(s); "
                "the main pattern may overlap them."
            )
            continue
        parts.append(boundary)

    if not parts:
        return None
    if len(parts) == 1:
        return parts[0]
    try:
        fused = parts[0].fuse(parts[1:])
        return fused.removeSplitter() if hasattr(fused, "removeSplitter") else fused
    except Exception as e:
        Path.Log.warning(f"build_feature_avoid_boundary: fuse failed ({e}); using steep walls only.")
        return parts[0]


# ---------------------------------------------------------------------------
# Steep wall passes
# ---------------------------------------------------------------------------
#
# Steep walls are finished with constant-Z contours. Each contour is the
# wall's cross-section at one height, offset away from the material by the
# tool radius: the drop-cutter only lands the tool on the wall at that height
# when the cutter's edge, not its center, sits on the section line.

# Section planes stay this far inside the steep Z range, so the cut never
# lands exactly on the top/bottom boundary edges, where the section is
# tangential and tends to return degenerate edges.
_STEEP_EDGE_CLEARANCE = 0.01

# Finest chord deviation for turning a section chain into a polyline before
# offsetting (also the default). The operation passes its LinearDeflection,
# which is followed when coarser. Finer values would only add wall faces and
# slow makeOffsetShape down for a few microns on a fallback path.
_STEEP_MIN_DEFLECTION = 0.01

# Internal tolerance of makeOffsetShape. A solver setting, not an accuracy
# target, so it stays fixed: much smaller values make the offset fail more
# often, larger ones loosen the corner joins.
_STEEP_OFFSET_TOLERANCE = 0.01

# Most wall faces an open-chain offset may have. makeOffsetShape slows down
# sharply with the face count, so longer chains are simplified to this many
# segments first.
_STEEP_MAX_WALL_FACES = 200


def _contour_entry_dist_sq(contour, x, y):
    """
    Squared distance from (x, y) to where a contour would be entered: the
    start of an open contour, or the bounding box of a closed one (zero
    inside). contour is (points, closed, (xmin, ymin, xmax, ymax)).
    """
    pts, closed, (x0, y0, x1, y1) = contour
    if not closed:
        return (pts[0][0] - x) ** 2 + (pts[0][1] - y) ** 2
    dx = max(x0 - x, 0.0, x - x1)
    dy = max(y0 - y, 0.0, y - y1)
    return dx * dx + dy * dy


def _steep_levels(z_top, z_bottom, step_down, clearance=_STEEP_EDGE_CLEARANCE):
    """Evenly spaced Z levels, top to bottom, at most step_down apart."""
    zt, zb = z_top - clearance, z_bottom + clearance
    if zt <= zb:
        return [(z_top + z_bottom) / 2.0]
    count = max(1, int(math.ceil((zt - zb) / step_down - 1e-9)))
    dz = (zt - zb) / count
    return [zt - i * dz for i in range(count + 1)]


def _signed_area_xy(points):
    """Shoelace signed area in XY (positive = counter-clockwise)."""
    return 0.5 * sum(
        x1 * y2 - x2 * y1 for (x1, y1), (x2, y2) in zip(points, points[1:] + points[:1])
    )


def _rotate_closed_start(points, start_xy):
    """Rotates a closed polyline (first == last) to begin nearest start_xy."""
    ring = points[:-1]
    if not ring or start_xy is None:
        return points
    sx, sy = start_xy
    i = min(range(len(ring)), key=lambda k: (ring[k][0] - sx) ** 2 + (ring[k][1] - sy) ** 2)
    ring = ring[i:] + ring[:i]
    return ring + [ring[0]]


def _wall_normal_xy(p, faces):
    """
    Unit horizontal wall normal at point p, pointing away from the material,
    or None where the wall is horizontal.

    `faces` is a list of (face, enlarged_boundbox); only faces whose box holds
    p are measured, which keeps distToShape off most of the model. Face
    normalAt() honours face orientation, so for faces of a valid solid the
    normal points away from the material.
    """
    near = [f for f, bb in faces if bb.isInside(p)] or [f for f, _ in faces]
    face = min(near, key=lambda f: f.distToShape(Part.Vertex(p))[0])
    n = face.normalAt(*face.Surface.parameter(p))
    h = math.hypot(n.x, n.y)
    return None if h < 1e-9 else (n.x / h, n.y / h)


def _offset_loop(loop, z, faces, tool_radius, sample_interval, cut_climb):
    """
    Offsets one closed section loop away from the material with Path.Area.

    The wall normal at one point either points into the loop (a cavity, so
    shrink) or out of it (a boss or core, so grow).

    Returns:
        list: Closed (x, y) polylines oriented for the requested cut direction.
    """
    flat = loop.copy()
    flat.translate(FreeCAD.Vector(0, 0, -z))
    region = Part.Face(flat)

    edge = loop.Edges[0]
    p = edge.valueAt((edge.FirstParameter + edge.LastParameter) / 2.0)
    normal = _wall_normal_xy(p, faces)
    if normal is None:
        return []
    eps = max(1e-3, min(0.05, tool_radius * 0.1))
    probe = FreeCAD.Vector(p.x + normal[0] * eps, p.y + normal[1] * eps, 0.0)
    is_cavity = region.isInside(probe, 1e-4, True)

    engine = Path.Area()
    engine.setParams(Tolerance=0.01)
    engine.add(region)
    engine.setParams(Offset=-tool_radius if is_cavity else tool_radius)
    try:
        result = engine.getShape()
    except Exception as e:
        Path.Log.debug(f"generate_steep_scan_lines: loop offset failed at Z={z:.3f}: {e}")
        return []
    if not result or result.isNull():
        return []  # Cavity narrower than the tool

    contours = []
    for wire in result.Wires:
        pts = [(q.x, q.y) for q in wire.discretize(Distance=sample_interval)]
        if len(pts) < 3:
            continue
        if pts[0] != pts[-1]:
            pts.append(pts[0])
        # Climb (M3) = material on the right: clockwise around a core,
        # counter-clockwise inside a cavity.
        ccw = _signed_area_xy(pts[:-1]) > 0.0
        if ccw != (is_cavity == cut_climb):
            pts.reverse()
        contours.append(pts)
    return contours


def _offset_profile(profile, offset_left, tool_radius):
    """
    Offsets a flat open profile (at z = 0) to one side by tool_radius.

    The profile is extruded into a thin vertical wall and offset with
    makeOffsetShape, which offsets an open shell to one side only and joins
    the corners. The bottom edges of the offset shell are the tool-center
    path. The offset side is taken from the wall's own normal, compared with
    the requested side (left or right of the profile's travel direction).

    Returns:
        list: Bottom edges of the offset shell, or [] on failure.
    """
    wall = profile.extrude(FreeCAD.Vector(0, 0, 1))
    wf = wall.Faces[0]
    u1, u2, v1, v2 = wf.ParameterRange
    um, vm = (u1 + u2) / 2.0, (v1 + v2) / 2.0
    q, wn = wf.valueAt(um, vm), wf.normalAt(um, vm)

    # Travel direction of the profile where that wall face sits.
    pts = [FreeCAD.Vector(p.x, p.y, 0.0) for p in profile.discretize(Number=64)]
    k = min(range(len(pts) - 1), key=lambda i: (pts[i].x - q.x) ** 2 + (pts[i].y - q.y) ** 2)
    tx, ty = pts[k + 1].x - pts[k].x, pts[k + 1].y - pts[k].y
    wall_left = (-ty * wn.x + tx * wn.y) > 0.0
    sign = 1.0 if wall_left == offset_left else -1.0

    shell = wall.makeOffsetShape(sign * tool_radius, _STEEP_OFFSET_TOLERANCE, join=2)
    return [e for e in shell.Edges if e.BoundBox.ZLength < 1e-6 and abs(e.BoundBox.ZMax) < 1e-3]


def _offset_open(wire, z, faces, tool_radius, sample_interval, cut_climb, deflection):
    """
    Offsets an open section chain away from the material by the tool radius.

    The offset runs on the section's own edges (one wall face per edge), which
    keeps makeOffsetShape fast. Sections of tessellated models consist of many
    tiny edges, and the 3D offset slows down sharply with the face count, so
    above _STEEP_MAX_WALL_FACES edges, or if the edge-based offset fails, a
    polyline capped at that many segments is offset instead. The polyline
    follows the chain within `deflection` (the operation's LinearDeflection,
    at least _STEEP_MIN_DEFLECTION) unless the cap forces it coarser. Any leftover overlap at tight inside
    corners is harmless: the drop-cutter keeps the tool off the model.

    Returns:
        list: Open (x, y) polylines oriented for the requested cut direction.
    """
    flat = wire.copy()
    flat.translate(FreeCAD.Vector(0, 0, -z))
    pts = flat.discretize(Distance=sample_interval)
    if len(pts) < 3:
        pts = flat.discretize(Number=3)

    # Side check at a point on the section curve itself (not a chord), so it
    # lies on a steep face and the bounding-box lookup finds it.
    k = len(pts) // 2 - 1 if len(pts) > 2 else 0
    normal = _wall_normal_xy(FreeCAD.Vector(pts[k].x, pts[k].y, z), faces)
    if normal is None:
        return []
    nx, ny = normal
    tx, ty = pts[k + 1].x - pts[k].x, pts[k + 1].y - pts[k].y
    # The tool goes where the normal points; material is on the right of
    # travel when that is the left side.
    offset_left = (-ty * nx + tx * ny) > 0.0

    profiles = []
    if len(flat.Edges) <= _STEEP_MAX_WALL_FACES:
        profiles.append(flat)
    poly = [FreeCAD.Vector(p.x, p.y, 0.0) for p in flat.discretize(Deflection=deflection)]
    if len(poly) > _STEEP_MAX_WALL_FACES + 1:
        poly = [
            FreeCAD.Vector(p.x, p.y, 0.0)
            for p in flat.discretize(Number=_STEEP_MAX_WALL_FACES + 1)
        ]
    poly = [v for i, v in enumerate(poly) if i == 0 or (v - poly[i - 1]).Length > 1e-6]
    if len(poly) > 1:
        profiles.append(Part.makePolygon(poly))

    bottom = []
    for profile in profiles:
        try:
            bottom = _offset_profile(profile, offset_left, tool_radius)
        except Exception as e:
            Path.Log.debug(f"generate_steep_scan_lines: open offset failed at Z={z:.3f}: {e}")
            bottom = []
        if bottom:
            break
    if not bottom:
        return []

    start = pts[0]
    contours = []
    for group in Part.sortEdges(bottom):
        try:
            out = [(q.x, q.y) for q in Part.Wire(group).discretize(Distance=sample_interval)]
        except Exception:
            continue
        if len(out) < 2:
            continue
        # Follow the source chain's direction, then apply the cut mode.
        d0 = (out[0][0] - start.x) ** 2 + (out[0][1] - start.y) ** 2
        d1 = (out[-1][0] - start.x) ** 2 + (out[-1][1] - start.y) ** 2
        if d1 < d0:
            out.reverse()
        if offset_left != cut_climb:
            out.reverse()
        contours.append(out)
    return contours


def _contour_record(pts):
    """(points, closed, (xmin, ymin, xmax, ymax)) for ordering."""
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return (pts, pts[0] == pts[-1], (min(xs), min(ys), max(xs), max(ys)))


def _stack_by_wall(levels, side_tolerance):
    """
    Groups per-level contours into stacks, one per wall, top to bottom.

    A contour continues the stack whose contour one level up has the most
    similar bounding box: on a drafted wall the box shifts only by about the
    step down times the draft, while another wall's box differs by much more.
    All four box sides are compared, so a boss inside a pocket (nested
    boxes) is not mistaken for the pocket wall. Each stack takes at most one
    contour per level; a contour with no box within side_tolerance on every
    side starts a new stack.

    Args:
        levels (list): Per level, top to bottom, a list of contour records.
        side_tolerance (float): Largest shift of any box side between levels.

    Returns:
        list: Stacks, each a list of records in level order.
    """
    stacks = []  # [records, index of the last level]
    for index, records in enumerate(levels):
        open_stacks = [st for st in stacks if st[1] == index - 1]
        candidates = []
        for ci, record in enumerate(records):
            bb = record[2]
            for si, stack in enumerate(open_stacks):
                last = stack[0][-1][2]
                shift = max(abs(bb[k] - last[k]) for k in range(4))
                if shift <= side_tolerance:
                    candidates.append((shift, ci, si))
        candidates.sort()
        taken_c, taken_s = set(), set()
        for _shift, ci, si in candidates:
            if ci in taken_c or si in taken_s:
                continue
            taken_c.add(ci)
            taken_s.add(si)
            open_stacks[si][0].append(records[ci])
            open_stacks[si][1] = index
        for ci, record in enumerate(records):
            if ci not in taken_c:
                stacks.append([[record], index])
    return [st[0] for st in stacks]


def _order_steep_contours(per_level, order, step_down, sample_interval):
    """
    Orders the steep-wall contours and returns them as scan lines.

    Distances use only the open start / closed bounding box, so the cost does
    not grow with the point count of every contour. Closed contours enter at
    their point nearest the tool; open ones keep the direction set by the
    cut mode.

    Args:
        per_level (list): Per level, top to bottom, a list of contour records
            (see _contour_record).
        order (str): "Level" (all walls per height) or "Wall" (each wall top
            to bottom, see _stack_by_wall).
        step_down (float): Vertical distance between levels.
        sample_interval (float): Point spacing along the contours.

    Returns:
        list: Scan lines, each a list of (x, y, 0.0) tuples.
    """
    lines = []
    current = None

    def emit(record):
        nonlocal current
        pts, closed, _bb = record
        if closed:
            pts = _rotate_closed_start(pts, current)
        current = pts[-1]
        lines.append([(x, y, 0.0) for x, y in pts])

    if order == "Wall":
        stacks = _stack_by_wall(per_level, 3.0 * step_down + 2.0 * sample_interval)
        while stacks:
            if current is None:
                pick = stacks[0]
            else:
                pick = min(stacks, key=lambda st: _contour_entry_dist_sq(st[0], *current))
            stacks.remove(pick)
            for record in pick:
                emit(record)
    else:
        for records in per_level:
            pending = list(records)
            while pending:
                if current is None:
                    pick = pending[0]
                else:
                    pick = min(pending, key=lambda c: _contour_entry_dist_sq(c, *current))
                pending.remove(pick)
                emit(pick)
    return lines


def generate_steep_scan_lines(
    steep_faces,
    step_over,
    tool_diam,
    sample_interval,
    cut_climb=False,
    deflection=None,
    order="Level",
):
    """
    Generates constant-Z contour passes for steep (low-draft) walls.

    The steep faces are sliced at evenly spaced heights; every section wire is
    offset away from the material by the tool radius and flattened to XY,
    ready for drop-cutter projection like the other patterns here. Closed
    loops are offset with Path.Area, open chains (where a non-steep face
    breaks the loop) with a one-sided shell offset.

    On a perfectly vertical wall the tool only grazes the wall, so the
    drop-cutter lets it fall to whatever lies below: still safe, but the pass
    does not hold its height.

    Args:
        steep_faces (list): Part.Face objects classified as steep walls.
        step_over (float): Vertical distance between slice heights.
        tool_diam (float): Tool diameter.
        sample_interval (float): Point spacing along each contour.
        cut_climb (bool): True for climb milling (material on the tool's right).
        deflection (float, optional): Chord deviation for open chains that are
            turned into polylines; pass the operation's LinearDeflection. Never
            finer than _STEEP_MIN_DEFLECTION, which is also the default.
        order (str): "Level" finishes all walls at one height before stepping
            down; "Wall" finishes each wall top to bottom before moving to
            the nearest next one (fewer moves between separate walls).

    Returns:
        list: A nested list of scan lines, each a list of (x, y, 0.0) tuples.
    """
    import time

    if not steep_faces:
        return []
    if step_over <= 0.0 or sample_interval <= 0.0 or tool_diam <= 0.0:
        Path.Log.error(
            "generate_steep_scan_lines: step_over, sample_interval and tool_diam must be "
            f"positive (got {step_over}, {sample_interval}, {tool_diam})."
        )
        return []

    deflection = max(deflection or 0.0, _STEEP_MIN_DEFLECTION)

    steep_shape = steep_faces[0] if len(steep_faces) == 1 else Part.makeCompound(steep_faces)
    bb = steep_shape.BoundBox
    tool_radius = tool_diam / 2.0

    faces = []
    for f in steep_faces:
        fbb = FreeCAD.BoundBox(f.BoundBox)
        fbb.enlarge(0.01)
        faces.append((f, fbb))

    levels = _steep_levels(bb.ZMax, bb.ZMin, step_over)
    timings = {"section": 0.0, "closed": 0.0, "open": 0.0}
    counts = {"closed": 0, "open": 0}

    per_level = []

    for index, z in enumerate(levels):
        t0 = time.time()
        plane = Part.makePlane(
            bb.XLength + 2.0, bb.YLength + 2.0, FreeCAD.Vector(bb.XMin - 1.0, bb.YMin - 1.0, z)
        )
        edges = [e for e in steep_shape.section(plane).Edges if e.Length > 1e-6]
        groups = Part.sortEdges(edges) if edges else []
        timings["section"] += time.time() - t0

        wires = []
        for group in groups:
            try:
                wires.append(Part.Wire(group))
            except Exception:
                wires.extend(Part.Wire(e) for e in group)  # Unchainable: keep edges apart

        # Logged before the offsets, so a stall shows the level it is stuck on.
        open_wires = [w for w in wires if not w.isClosed()]
        Path.Log.debug(
            f"generate_steep_scan_lines: level {index + 1}/{len(levels)} Z={z:.3f}: "
            f"{len(wires) - len(open_wires)} closed, {len(open_wires)} open"
            + (
                f" (longest {max(len(w.Edges) for w in open_wires)} edges, "
                f"{max(w.Length for w in open_wires):.1f} mm)"
                if open_wires
                else ""
            )
        )

        contours = []
        for wire in wires:
            kind = "closed" if wire.isClosed() else "open"
            t0 = time.time()
            if kind == "closed":
                found = _offset_loop(wire, z, faces, tool_radius, sample_interval, cut_climb)
            else:
                found = _offset_open(
                    wire, z, faces, tool_radius, sample_interval, cut_climb, deflection
                )
            contours.extend(found)
            timings[kind] += time.time() - t0
            counts[kind] += 1

        per_level.append([_contour_record(pts) for pts in contours])

    steep_lines = _order_steep_contours(per_level, order, step_over, sample_interval)

    Path.Log.debug(
        f"generate_steep_scan_lines: {len(levels)} levels, {len(steep_faces)} faces | "
        f"section {timings['section']:.2f}s | "
        f"closed {counts['closed']} in {timings['closed']:.2f}s | "
        f"open {counts['open']} in {timings['open']:.2f}s"
    )
    return steep_lines


# ---------------------------------------------------------------------------
# Fillet passes
# ---------------------------------------------------------------------------
#
# Fillet passes on parts of the surface steeper than this (draft angle from
# vertical, degrees) are skipped. That is the steep end of each fillet: the
# bottom of a convex roundover, the top of a concave fillet. There the
# surface is nearly vertical, the tool's contact is marginal, and the
# tessellation error lets the drop-cutter slip it along the wall (a wavy
# pass). Fixed, independent of the steep-wall setting.
_FILLET_MAX_DRAFT_ANGLE = 11.0

# A skipped steep-end pass lying between this angle and the limit above is
# not dropped but moved to the limit, so the band at the steep end is not
# left 1.5 steps wide. Below this angle the bands are narrow enough that the
# next pass already sits close to the limit, and the pass is simply skipped.
_FILLET_RESCUE_MIN_ANGLE = 6.0

# Fillets are finished with flow lines: passes that run along the blend at
# equal arc-length steps across its curve, so the step over is constant on
# the surface itself. Each surface point p is moved out along its normal n
# by the tool radius and flattened, (p + R*n).xy, which is where a ball end
# mill's center sits when it touches p; the drop-cutter then lands the tool
# on the fillet at p.


def _face_point(face, across_is_u, a, b):
    """(u, v) for an (across, along) parameter pair."""
    return (a, b) if across_is_u else (b, a)


def _iso_samples(face, across_is_u, vary_across, fixed, lo, hi, count=48):
    """
    Samples an isoparametric line of the face.

    Returns:
        tuple: (params, cumulative_lengths) along the line, from lo to hi.
    """
    params = [lo + (hi - lo) * i / (count - 1) for i in range(count)]
    cum = [0.0]
    prev = None
    for t in params:
        a, b = (t, fixed) if vary_across else (fixed, t)
        p = face.valueAt(*_face_point(face, across_is_u, a, b))
        if prev is not None:
            cum.append(cum[-1] + (p - prev).Length)
        prev = p
    return params, cum


def _params_at_lengths(params, cum, targets):
    """Linear interpolation of parameters at the given arc lengths."""
    out = []
    k = 0
    for s in targets:
        while k < len(cum) - 2 and cum[k + 1] < s:
            k += 1
        span = cum[k + 1] - cum[k]
        f = 0.0 if span < 1e-12 else (s - cum[k]) / span
        out.append(params[k] + f * (params[k + 1] - params[k]))
    return out


def _normal_turning(face):
    """
    How fast the normal turns per unit length along u and along v, measured
    through the middle of the face: (turning_u, turning_v).
    """
    u1, u2, v1, v2 = face.ParameterRange
    um, vm = (u1 + u2) / 2.0, (v1 + v2) / 2.0

    def turning(points_normals):
        angle = length = 0.0
        for (p1, n1), (p2, n2) in zip(points_normals, points_normals[1:]):
            length += (p2 - p1).Length
            angle += n1.getAngle(n2)
        return angle / length if length > 1e-9 else 0.0

    steps = 16
    along_u = [
        (face.valueAt(u, vm), face.normalAt(u, vm))
        for u in (u1 + (u2 - u1) * i / steps for i in range(steps + 1))
    ]
    along_v = [
        (face.valueAt(um, v), face.normalAt(um, v))
        for v in (v1 + (v2 - v1) * i / steps for i in range(steps + 1))
    ]
    return turning(along_u), turning(along_v)


def _fillet_across_is_u(face):
    """
    True when the fillet's curve runs along the u parameter.

    Analytic types are known (cylinder: u is the angle; torus: v is the minor
    angle). Otherwise the direction in which the normal turns fastest per unit
    length is taken as the direction across the blend.
    """
    tid = getattr(face.Surface, "TypeId", "")
    if "Cylinder" in tid:
        return True
    if "Toroid" in tid:
        return False
    turning_u, turning_v = _normal_turning(face)
    return turning_u >= turning_v


def _is_corner_patch(face):
    """
    True for corner blends: patches curved about equally in both directions
    (a spherical octant, a vertex blend), which have no direction running
    along them. Cylinders and tori always have one.
    """
    return _classify_blend(face)[0]


def _classify_blend(face):
    """
    (is_corner_patch, across_is_u) for a fillet face, measuring the normal
    turning at most once (it is the costly part for B-spline faces).
    across_is_u is None for corner patches.
    """
    tid = getattr(face.Surface, "TypeId", "")
    if "Sphere" in tid:
        return True, None
    if "Cylinder" in tid:
        return False, True
    if "Toroid" in tid:
        return False, False
    turning_u, turning_v = _normal_turning(face)
    high = max(turning_u, turning_v)
    if high > 1e-9 and min(turning_u, turning_v) >= 0.5 * high:
        return True, None
    return False, turning_u >= turning_v


def _max_center_gap(run):
    """Largest XY step between consecutive tool-center points of a run."""
    return max(
        (math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(run, run[1:])), default=0.0
    )


def _touches(face, point, tol=0.01):
    """True when point (a FreeCAD.Vector) lies on the face, within tol."""
    bb = face.BoundBox
    if not (
        bb.XMin - tol <= point.x <= bb.XMax + tol
        and bb.YMin - tol <= point.y <= bb.YMax + tol
        and bb.ZMin - tol <= point.z <= bb.ZMax + tol
    ):
        return False
    return face.distToShape(Part.Vertex(point))[0] < tol


def _corner_levels(face, contacts, step_over):
    """
    Slice heights for a corner patch: the heights at which neighboring fillet
    passes end on the patch's edges, so each slice continues one of them.
    A patch no pass reaches gets evenly spaced heights instead.
    """
    bb = face.BoundBox
    zs = [c.z for c in contacts if _touches(face, c)]

    levels = []
    for z in sorted(zs):
        if not levels or z - levels[-1] > 1e-4:
            levels.append(z)
    if levels:
        return levels

    count = max(1, int(math.ceil(bb.ZLength / step_over - 1e-9)))
    return [bb.ZMin + (i + 0.5) * bb.ZLength / count for i in range(count)]


def _corner_patch_passes(face, levels, tool_radius, sample_interval, cut_climb):
    """
    Passes over a corner patch: horizontal slices at the given heights, with
    the same tool-center offset (p + R*n).xy and run format as
    _fillet_face_passes(), so the joining step links them to the fillets.
    """
    bb = face.BoundBox
    passes = []
    for z in levels:
        plane = Part.makePlane(
            bb.XLength + 2.0, bb.YLength + 2.0, FreeCAD.Vector(bb.XMin - 1.0, bb.YMin - 1.0, z)
        )
        try:
            edges = [e for e in face.section(plane).Edges if e.Length > 1e-6]
        except Exception as e:
            Path.Log.debug(f"generate_fillet_scan_lines: corner section failed at Z={z:.3f}: {e}")
            continue
        for group in Part.sortEdges(edges) if edges else []:
            try:
                wire = Part.Wire(group)
            except Exception:
                continue
            # On a convex corner the tool center travels further than the
            # contact point (by (r + R) / r), so sampling is refined until the
            # center spacing, not the contact spacing, is within the interval.
            spacing = sample_interval
            for _attempt in range(3):
                pts = wire.discretize(Distance=spacing)
                if len(pts) < 2:
                    pts = wire.discretize(Number=2)
                run = []
                for p in pts:
                    try:
                        n = face.normalAt(*face.Surface.parameter(p))
                    except Exception:
                        continue
                    if n.Length < 1e-12:
                        continue
                    n = n * (1.0 / n.Length)
                    run.append(
                        (p.x + tool_radius * n.x, p.y + tool_radius * n.y, n.x, n.y, p.x, p.y, p.z)
                    )
                gap = _max_center_gap(run)
                if len(run) < 2 or gap <= sample_interval * 1.001:
                    break
                spacing *= 0.98 * sample_interval / gap
            if len(run) < 2:
                continue
            vote = sum(
                (q[0] - p[0]) * p[3] - (q[1] - p[1]) * p[2] for p, q in zip(run, run[1:])
            )
            if (vote > 0.0) != cut_climb:
                run.reverse()
            passes.append(run)
    return passes


def _trim_run_ends(run, trim, at_start=True, at_end=True):
    """
    Shortens an open pass by `trim` at its ends (at_start / at_end select
    which), measured along the surface
    contact points (run entries: cl_x, cl_y, nx, ny, px, py, pz). New end
    points are interpolated, so the trim is exact rather than rounded to the
    sample spacing. Closed passes (a full ring) have no ends and are kept.

    Returns:
        list: The trimmed run, or [] if it is shorter than 2 * trim.
    """
    def contact_dist(a, b):
        return math.sqrt((a[4] - b[4]) ** 2 + (a[5] - b[5]) ** 2 + (a[6] - b[6]) ** 2)

    if trim <= 0.0 or contact_dist(run[0], run[-1]) < 1e-6:
        return run

    cum = [0.0]
    for a, b in zip(run, run[1:]):
        cum.append(cum[-1] + contact_dist(a, b))
    lo = trim if at_start else 0.0
    hi = cum[-1] - (trim if at_end else 0.0)
    if hi - lo < 1e-6:
        return []

    def at(s):
        k = 0
        while k < len(cum) - 2 and cum[k + 1] < s:
            k += 1
        span = cum[k + 1] - cum[k]
        f = 0.0 if span < 1e-12 else (s - cum[k]) / span
        return tuple(x + f * (y - x) for x, y in zip(run[k], run[k + 1]))

    inner = [r for r, c in zip(run, cum) if lo < c < hi]
    return [at(lo) if at_start else run[0]] + inner + [at(hi) if at_end else run[-1]]


def _face_pcurves(face, across_is_u, samples=64):
    """
    The face boundary in parameter space, as (across, along) polylines, one
    per edge (from face.curveOnSurface). Read once per face, it lets each
    pass find where it crosses the boundary directly instead of asking
    isPartOfDomain point by point. None if any edge has no 2D curve.
    """
    polys = []
    try:
        for edge in face.Edges:
            result = face.curveOnSurface(edge)
            if not result:
                return None
            curve, t0, t1 = result[0], result[1], result[2]
            poly = []
            for k in range(samples + 1):
                q = curve.value(t0 + (t1 - t0) * k / samples)
                poly.append((q.x, q.y) if across_is_u else (q.y, q.x))
            polys.append(poly)
    except Exception:
        return None
    return polys


def _domain_intervals(polys, a, b0, b1, inside, boundary, step):
    """
    Stretches of the pass line across = a, between b0 and b1, that lie on
    the face.

    Crossings with the boundary polylines split the line; one inside() test
    at the middle of each piece tells which pieces are on the face (so
    spurious crossings, such as a seam, merge away), and each real crossing
    is then located precisely by bisection. Costs a few isPartOfDomain calls
    per pass instead of one per sample point.

    Args:
        polys (list): Boundary polylines from _face_pcurves().
        inside (callable): b -> bool, the face domain test on this pass.
        boundary (callable): (b_in, b_out) -> b, bisection to the edge.
        step (float): Sample spacing along the pass, in parameter units.

    Returns:
        list: [[lo, hi], ...] along-parameter intervals on the face.
    """
    span = b1 - b0
    cuts = []
    for poly in polys:
        for (pa, pb), (qa, qb) in zip(poly, poly[1:]):
            if pa == qa or not (min(pa, qa) <= a < max(pa, qa)):
                continue
            b = pb + (a - pa) / (qa - pa) * (qb - pb)
            if b0 < b < b1:
                cuts.append(b)
    bounds = [b0]
    for b in sorted(cuts):
        if b - bounds[-1] > 1e-9 * span:
            bounds.append(b)
    if b1 - bounds[-1] > 1e-9 * span:
        bounds.append(b1)
    else:
        bounds[-1] = b1

    segments = []
    for lo, hi in zip(bounds, bounds[1:]):
        if inside((lo + hi) / 2.0):
            if segments and segments[-1][1] == lo:
                segments[-1][1] = hi  # A spurious cut inside the face
            else:
                segments.append([lo, hi])

    # Locate each real crossing precisely (the polylines only bracket it).
    for seg in segments:
        for end in (0, 1):
            e = seg[end]
            if e in (b0, b1):
                continue
            for d in (step, 4.0 * step):
                d = min(d, (seg[1] - seg[0]) / 2.0)
                inner = e + d if end == 0 else e - d
                outer = max(b0, min(b1, e - d if end == 0 else e + d))
                if inside(inner) and not inside(outer):
                    seg[end] = boundary(inner, outer)
                    break
    return segments


def _fillet_band_count(face, across_is_u, step_over):
    """
    Number of passes across a fillet face (its bands), as _fillet_face_passes()
    lays them out: before any are skipped or split at trimmed edges.
    """
    u1, u2, v1, v2 = face.ParameterRange
    (a0, a1), (b0, b1) = ((u1, u2), (v1, v2)) if across_is_u else ((v1, v2), (u1, u2))
    _params, cum = _iso_samples(face, across_is_u, True, (b0 + b1) / 2.0, a0, a1)
    if cum[-1] < 1e-6:
        return 0
    return max(1, int(math.ceil(cum[-1] / step_over - 1e-9)))


def _limit_steep_bands(face, across_is_u, across, a0, a1, b_mid, stats=None):
    """
    Applies the steep-end limit to the band positions across a fillet.

    Draft (degrees from vertical) is measured where each pass crosses the
    middle of the fillet's length. Passes at or above _FILLET_MAX_DRAFT_ANGLE
    are kept; between _FILLET_RESCUE_MIN_ANGLE and the limit they are moved
    to the limit, found by bisection toward the flatter neighboring pass (or
    the flatter end of the face); below they are dropped.

    Returns:
        list: The across parameters of the passes to generate, in order.
    """

    def draft(a):
        n = face.normalAt(*_face_point(face, across_is_u, a, b_mid))
        if n.Length < 1e-12:
            return 90.0
        return math.degrees(math.asin(min(1.0, abs(n.z) / n.Length)))

    drafts = [draft(a) for a in across]
    kept = []
    for i, (a, d) in enumerate(zip(across, drafts)):
        if d >= _FILLET_MAX_DRAFT_ANGLE:
            kept.append(a)
            continue
        target = None
        if d >= _FILLET_RESCUE_MIN_ANGLE:
            neighbors = [(drafts[j], across[j]) for j in (i - 1, i + 1) if 0 <= j < len(across)]
            neighbors.extend((draft(e), e) for e in (a0, a1))
            flatter = max(neighbors)
            if flatter[0] > _FILLET_MAX_DRAFT_ANGLE:
                steep_side, flat_side = a, flatter[1]
                for _ in range(30):
                    mid = (steep_side + flat_side) / 2.0
                    if draft(mid) < _FILLET_MAX_DRAFT_ANGLE:
                        steep_side = mid
                    else:
                        flat_side = mid
                target = flat_side
        if target is not None:
            kept.append(target)
            if stats is not None:
                stats["rescued"] = stats.get("rescued", 0) + 1
        elif stats is not None:
            stats["skipped"] = stats.get("skipped", 0) + 1
    return kept


def _fillet_face_passes(
    face, step_over, tool_radius, sample_interval, cut_climb, across_is_u=None, stats=None
):
    """
    Flow-line passes for one fillet face, untrimmed.

    Passes sit at the centers of equal-width bands across the blend, so the
    tangent edges shared with the neighboring wall and floor are left to
    those faces' own passes.

    Steep end: a pass where the surface is steeper than
    _FILLET_MAX_DRAFT_ANGLE (draft from vertical, measured at the middle of
    the pass) is moved to exactly that angle if it lies above
    _FILLET_RESCUE_MIN_ANGLE, and skipped otherwise. stats (a dict), when
    given, counts both under "rescued" and "skipped".

    Returns:
        list: Runs oriented for the cut direction. Each run is a list of
              (cl_x, cl_y, nx, ny, px, py, pz): the flattened tool-center
              point, the horizontal normal and the surface contact point.
    """
    if across_is_u is None:
        across_is_u = _fillet_across_is_u(face)
    u1, u2, v1, v2 = face.ParameterRange
    (a0, a1), (b0, b1) = ((u1, u2), (v1, v2)) if across_is_u else ((v1, v2), (u1, u2))

    params, cum = _iso_samples(face, across_is_u, True, (b0 + b1) / 2.0, a0, a1)
    width = cum[-1]
    if width < 1e-6:
        return []
    bands = max(1, int(math.ceil(width / step_over - 1e-9)))
    across = _params_at_lengths(params, cum, [(i + 0.5) * width / bands for i in range(bands)])
    across = _limit_steep_bands(face, across_is_u, across, a0, a1, (b0 + b1) / 2.0, stats)

    # Face boundary in parameter space, read once (None: per-point fallback).
    polys = _face_pcurves(face, across_is_u)

    passes = []
    for a in across:
        b_params, b_cum = _iso_samples(face, across_is_u, False, a, b0, b1)
        length = b_cum[-1]
        if length < 1e-6:
            continue

        # Trimmed faces: a pass breaks wherever it leaves the face domain.
        # Crossings are located by bisection and become the run's ends, so
        # passes reach a trimmed edge, such as a miter between two fillets,
        # instead of stopping up to one sample short.
        def entry_at(b):
            u, v = _face_point(face, across_is_u, a, b)
            p = face.valueAt(u, v)
            n = face.normalAt(u, v)
            if n.Length < 1e-12:
                return None
            n = n * (1.0 / n.Length)
            return (p.x + tool_radius * n.x, p.y + tool_radius * n.y, n.x, n.y, p.x, p.y, p.z)

        def inside(b):
            return face.isPartOfDomain(*_face_point(face, across_is_u, a, b))

        def boundary(b_in, b_out):
            # 12 halvings locate the edge to 1/4096 of the bracket; each
            # costs an isPartOfDomain call, so no more than needed.
            for _ in range(12):
                mid = (b_in + b_out) / 2.0
                if inside(mid):
                    b_in = mid
                else:
                    b_out = mid
            return b_in

        # On convex rings the tool center travels further than the contact
        # point, so sampling is refined until the center spacing, not the
        # contact spacing, is within the interval.
        count = max(2, int(math.ceil(length / sample_interval)) + 1)
        intervals = (
            _domain_intervals(polys, a, b0, b1, inside, boundary, (b1 - b0) / (count - 1))
            if polys is not None
            else None
        )
        for _attempt in range(3):
            along = _params_at_lengths(
                b_params, b_cum, [length * i / (count - 1) for i in range(count)]
            )

            if intervals is not None:
                runs = []
                for lo, hi in intervals:
                    run = [entry_at(lo)] + [entry_at(b) for b in along if lo < b < hi]
                    run = [e for e in run + [entry_at(hi)] if e is not None]
                    if len(run) > 1:
                        runs.append(run)
                gap = max((_max_center_gap(r) for r in runs), default=0.0)
                if gap <= sample_interval * 1.001:
                    break
                count = int(math.ceil((count - 1) * gap / sample_interval)) + 1
                continue

            # Per-point fallback when the boundary curves are unavailable.
            runs, run = [], []
            prev_b, prev_in = None, None
            for b in along:
                is_in = inside(b)
                if prev_b is not None and is_in != prev_in:
                    edge = entry_at(boundary(b, prev_b) if is_in else boundary(prev_b, b))
                    if is_in:
                        run = [edge] if edge is not None else []
                    else:
                        if edge is not None:
                            run.append(edge)
                        if len(run) > 1:
                            runs.append(run)
                        run = []
                prev_b, prev_in = b, is_in
                if not is_in:
                    continue
                entry = entry_at(b)
                if entry is not None:
                    run.append(entry)
            if len(run) > 1:
                runs.append(run)

            gap = max((_max_center_gap(r) for r in runs), default=0.0)
            if gap <= sample_interval * 1.001:
                break
            count = int(math.ceil((count - 1) * gap / sample_interval)) + 1

        for run in runs:
            # Material is on the right when the normal (pointing away from
            # it) is on the left of travel; weighted by the horizontal part
            # of the normal, so flat stretches barely vote.
            vote = sum(
                (q[0] - p[0]) * p[3] - (q[1] - p[1]) * p[2] for p, q in zip(run, run[1:])
            )
            if (vote > 0.0) != cut_climb:
                run.reverse()
            passes.append(run)

    return passes


def _contact_gap(a, b):
    """Distance between the surface contact points of two run entries."""
    return math.sqrt((a[4] - b[4]) ** 2 + (a[5] - b[5]) ** 2 + (a[6] - b[6]) ** 2)


# Largest turn accepted for an arc around a sharp edge (degrees). Real edges
# stay below it even at acute corners; anything more points at bad geometry.
_EDGE_ARC_MAX_TURN = 150.0


def _edge_arc(before, a, b, sample_interval, tool_radius=None):
    """
    Arc of the tool rolling around a convex sharp edge between two joined
    passes.

    Both passes reach the edge at (nearly) the same contact point, but the
    tool center sits one radius out along each fillet's own normal, so the
    centers on either side of the edge are apart. The arc is centered on the
    contact point (in XY), runs from one center to the other the short way
    round, and is sampled at sample_interval. At a concave edge the two
    offsets overlap instead of opening up; no arc is added there and the
    drop-cutter keeps the tool off the model.

    Plausibility checks, all cheap, keep defective geometry (a notch where
    two faces do not meet cleanly) from producing wild loops; when one
    fails, the passes are joined directly, as without arcs:
        - the two contact points meet (within half a sample interval);
        - the arc radius (the tool's horizontal offset) is within the tool
          radius;
        - both ends are about equally far from the edge (within 25%);
        - the turn around the edge is at most _EDGE_ARC_MAX_TURN.

    Args:
        before, a: The last two entries of the incoming pass (travel a-ward).
        b: The first entry of the outgoing pass.

    Returns:
        list: Entries strictly between a and b, or [] when none are needed.
    """
    if math.hypot(b[0] - a[0], b[1] - a[1]) <= sample_interval:
        return []
    gap = _contact_gap(a, b)
    if gap > 0.5 * sample_interval:
        return []  # Not one edge point
    cx, cy = (a[4] + b[4]) / 2.0, (a[5] + b[5]) / 2.0
    ax, ay = a[0] - cx, a[1] - cy
    bx, by = b[0] - cx, b[1] - cy
    ra, rb = math.hypot(ax, ay), math.hypot(bx, by)
    if ra < 1e-9 or rb < 1e-9:
        return []
    if tool_radius is not None and max(ra, rb) > tool_radius + gap + 1e-6:
        return []  # Further out than the tool can be
    if abs(ra - rb) > 0.25 * max(ra, rb):
        return []  # Ends not around the same edge
    theta = math.atan2(ax * by - ay * bx, ax * bx + ay * by)
    if abs(theta) > math.radians(_EDGE_ARC_MAX_TURN):
        return []

    # Convex when the offset side turns away from the travel's turn: the
    # normal (pointing away from the material) is left of travel and the
    # offsets rotate right, or the other way round.
    tx, ty = a[0] - before[0], a[1] - before[1]
    side = tx * a[3] - ty * a[2]
    if theta * side >= 0.0:
        return []

    start = math.atan2(ay, ax)
    steps = max(2, int(math.ceil(abs(theta) * max(ra, rb) / sample_interval)))
    arc = []
    for k in range(1, steps):
        f = k / steps
        angle = start + theta * f
        radius = ra + (rb - ra) * f
        arc.append(
            (cx + radius * math.cos(angle), cy + radius * math.sin(angle))
            + tuple(x + f * (y - x) for x, y in zip(a[2:], b[2:]))
        )
    return arc


def _join_fillet_passes(
    passes, owners, tolerance, sample_interval=None, tool_radius=None, band_counts=None
):
    """
    Joins passes end-to-start into chains across neighboring fillet faces.

    Each pass end is matched to the nearest pass start within `tolerance`
    (measured between surface contact points), closest pairs first, so every
    end and every start is used at most once. Only end-to-start joins are
    made, which keeps every pass in its cut direction.

    Args:
        passes (list): Oriented runs (see _fillet_face_passes).
        owners (list): Index of the face each pass came from.
        tolerance (float): Largest gap that is joined.
        sample_interval (float, optional): When given, a convex sharp edge
            between two joined passes gets an arc around it (see _edge_arc).
        tool_radius (float, optional): Enables the arc's radius check.
        band_counts (list, optional): Passes across the face each pass came
            from (None for corner-patch slices). Passes of faces with
            different counts are never joined: their passes do not
            correspond (a wider fillet next to a narrower one), so a join
            would link unrelated passes with a diagonal jump. Their ends stay
            at the shared edge for a lead-in/out to blend.

    Returns:
        tuple: (chains, zone_of_face)
            chains: [(run, is_closed, face_indices, first_pass, last_pass), ...]
                first_pass/last_pass index the passes at the chain's ends.
            zone_of_face: dict mapping each face index to a zone id; faces
                linked by any join share a zone.
    """
    n = len(passes)
    closed_alone = [_contact_gap(r[0], r[-1]) < 1e-6 for r in passes]

    # Starts indexed on a grid of `tolerance`-sized cells, so each end is
    # only compared with starts in its own and the neighboring cells instead
    # of with every pass. Finds exactly the same pairs.
    cell = max(tolerance, 1e-9)

    def cell_of(r):
        return (int(math.floor(r[4] / cell)), int(math.floor(r[5] / cell)), int(math.floor(r[6] / cell)))

    starts = {}
    for j in range(n):
        if not closed_alone[j]:
            starts.setdefault(cell_of(passes[j][0]), []).append(j)

    pairs = []
    for i in range(n):
        if closed_alone[i]:
            continue
        end = passes[i][-1]
        cx, cy, cz = cell_of(end)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    for j in starts.get((cx + dx, cy + dy, cz + dz), ()):
                        if j == i:
                            continue
                        if (
                            band_counts is not None
                            and band_counts[i] is not None
                            and band_counts[j] is not None
                            and band_counts[i] != band_counts[j]
                        ):
                            continue
                        gap = _contact_gap(end, passes[j][0])
                        if gap <= tolerance:
                            pairs.append((gap, i, j))
    pairs.sort()

    successor, has_predecessor = {}, set()
    for _gap, i, j in pairs:
        if i not in successor and j not in has_predecessor:
            successor[i] = j
            has_predecessor.add(j)

    # Faces linked by a join belong to the same zone (union-find).
    parent = {}

    def find(k):
        parent.setdefault(k, k)
        while parent[k] != k:
            parent[k] = parent[parent[k]]
            k = parent[k]
        return k

    for i, j in successor.items():
        parent[find(owners[i])] = find(owners[j])
    zone_of_face = {f: find(f) for f in set(owners)}

    def build(first):
        run, faces, k, last = list(passes[first]), {owners[first]}, successor.get(first), first
        visited.add(first)
        while k is not None and k not in visited:
            visited.add(k)
            nxt = passes[k]
            arc = (
                _edge_arc(run[-2], run[-1], nxt[0], sample_interval, tool_radius)
                if sample_interval and len(run) > 1
                else []
            )
            if arc:
                run.extend(arc)
                run.extend(nxt)
            else:
                run.extend(nxt[1:] if _contact_gap(run[-1], nxt[0]) < 1e-9 else nxt)
            faces.add(owners[k])
            last = k
            k = successor.get(k)
        return run, faces, last

    chains, visited = [], set()
    for i in range(n):
        if closed_alone[i]:
            visited.add(i)
            chains.append((list(passes[i]), True, {owners[i]}, i, i))
    # Open chains start where nothing leads in; what remains are loops.
    for i in range(n):
        if i not in visited and i not in has_predecessor:
            run, faces, last = build(i)
            chains.append((run, False, faces, i, last))
    for i in range(n):
        if i not in visited:
            run, faces, last = build(i)
            # Close the loop through the same edge check as every other join,
            # so the closing corner gets its arc too, and end exactly on the
            # first point (the closed-loop handling relies on it).
            arc = (
                _edge_arc(run[-2], run[-1], run[0], sample_interval, tool_radius)
                if sample_interval and len(run) > 1
                else []
            )
            if arc:
                run.extend(arc)
                run.append(run[0])
            elif _contact_gap(run[-1], run[0]) < 1e-9:
                run[-1] = run[0]
            else:
                run.append(run[0])
            chains.append((run, True, faces, i, last))

    return chains, zone_of_face


def warn_if_coarse_accuracy(linear_deflection, reference):
    """
    Warns when the linear deflection is coarser than the reference preset.

    The mesh deflection is what limits the finishing passes: a coarse mesh
    lets the drop-cutter miss the fine detail of narrow fillets and corner
    arcs. Every setting can be changed by hand besides the accuracy slider,
    so the value itself is checked, not the slider level.

    Args:
        linear_deflection (float): The operation's linear deflection.
        reference (dict): The preset to compare with, holding
            "linear_deflection" (accuracy level 5).

    Returns:
        bool: True if the warning was logged.
    """
    limit = reference["linear_deflection"]
    if linear_deflection <= limit + 1e-9:
        return False
    Path.Log.warning(
        "Steep-wall and fillet finishing give their best results at accuracy level 5 "
        f"or higher, or with a linear deflection of {limit:g} or finer "
        f"(current: {linear_deflection:g})."
    )
    return True


def _warn_if_not_ball(tool_params, tool_diam):
    """
    Warns when the tool is not a ball end mill: the fillet offset (p + R*n)
    is exact only for a ball. A bull nose whose corner radius equals the
    tool radius is geometrically a ball and passes.
    """
    tool_type = (tool_params.get("tool_type") or "").lower()
    diameter = tool_params.get("diameter") or tool_diam
    corner_radius = tool_params.get("corner_radius") or 0.0
    if tool_type in ("ballend", "taperedballnose"):
        return
    if diameter > 0.0 and corner_radius >= diameter / 2.0 - 1e-6:
        return
    Path.Log.warning(
        "Fillet finishing passes are exact only with a ball end mill. "
        f"With this tool ({tool_type or 'unknown'}) they stay gouge-free "
        "but touch the fillets off the intended lines."
    )


def generate_fillet_scan_lines(
    fillet_faces,
    step_over,
    tool_diam,
    sample_interval,
    cut_climb=False,
    edge_trim=None,
    tool_params=None,
):
    """
    Generates flow-line finishing passes for fillet faces.

    Each fillet gets passes along its length, spaced step_over apart measured
    across its curve on the surface, offset by the tool radius along the
    surface normal and flattened to XY for drop-cutter projection. The offset
    is exact for a ball end mill; other cutters stay gouge-free through the
    drop-cutter but touch slightly off the target line.

    Passes on neighboring fillet faces are joined end-to-start into
    continuous chains (a fillet run around a boss becomes one loop per
    step); a convex sharp edge between two fillets is rounded by an arc of
    the tool around the edge. Only free chain ends,
    not ends stopping on another fillet or corner face, are trimmed by
    edge_trim.

    Passes lying where the surface is steeper than _FILLET_MAX_DRAFT_ANGLE
    (the bottom of a convex roundover, the top of a concave fillet) are
    moved to that angle, or skipped when they lie below
    _FILLET_RESCUE_MIN_ANGLE (see _limit_steep_bands).
    Chains are grouped into zones of linked faces; zones are visited
    nearest-neighbor, and within a zone chains run from the highest to the
    lowest.

    Args:
        fillet_faces (list): Part.Face objects classified as fillets.
        step_over (float): Distance between passes, measured on the surface.
        tool_diam (float): Tool diameter.
        sample_interval (float): Point spacing along each pass.
        cut_climb (bool): True for climb milling (material on the tool's right).
        edge_trim (float, optional): How far open chains stop short of their
            free ends, measured on the surface. Defaults to half the step
            over, matching the margin left at the tangent edges.
        tool_params (dict, optional): The operation's tool parameters
            (tool_type, diameter, corner_radius). When given, a warning is
            logged for tools other than a ball end mill, for which the
            fillet offset is not exact.

    Returns:
        list: A nested list of scan lines, each a list of (x, y, 0.0) tuples.
    """
    if not fillet_faces:
        return []
    if step_over <= 0.0 or sample_interval <= 0.0 or tool_diam <= 0.0:
        Path.Log.error(
            "generate_fillet_scan_lines: step_over, sample_interval and tool_diam must be "
            f"positive (got {step_over}, {sample_interval}, {tool_diam})."
        )
        return []

    import time

    started = time.perf_counter()
    tool_radius = tool_diam / 2.0
    if edge_trim is None:
        edge_trim = step_over / 2.0

    if tool_params is not None:
        _warn_if_not_ball(tool_params, tool_diam)

    passes, owners, corners, band_counts = [], [], [], []
    steep_stats = {}
    for index, face in enumerate(fillet_faces):
        try:
            is_corner, across_is_u = _classify_blend(face)
            if is_corner:
                corners.append(index)  # Needs the fillet passes first
                continue
            face_started = time.perf_counter()
            face_passes = _fillet_face_passes(
                face, step_over, tool_radius, sample_interval, cut_climb, across_is_u, steep_stats
            )
            bands = _fillet_band_count(face, across_is_u, step_over)
            Path.Log.debug(
                f"generate_fillet_scan_lines: face {index} "
                f"({getattr(face.Surface, 'TypeId', '?')}, {len(getattr(face, 'Edges', ()))} edges): "
                f"{len(face_passes)} pass(es) in {time.perf_counter() - face_started:.3f}s"
            )
        except Exception as e:
            Path.Log.debug(f"generate_fillet_scan_lines: skipping face ({e}).")
            continue
        for run in face_passes:
            passes.append(run)
            owners.append(index)
            band_counts.append(bands)

    # Corner patches: sliced at the heights where fillet passes reach them,
    # so each slice continues a pass around the corner.
    contacts = [FreeCAD.Vector(*r[i][4:7]) for r in passes for i in (0, -1)]
    for index in corners:
        face = fillet_faces[index]
        try:
            levels = _corner_levels(face, contacts, step_over)
            face_passes = _corner_patch_passes(
                face, levels, tool_radius, sample_interval, cut_climb
            )
        except Exception as e:
            Path.Log.debug(f"generate_fillet_scan_lines: skipping corner ({e}).")
            continue
        passes.extend(face_passes)
        owners.extend([index] * len(face_passes))
        band_counts.extend([None] * len(face_passes))

    if not passes:
        return []
    evaluated = time.perf_counter()

    # Half a step over: the matching pass on the next face, never the
    # neighboring pass one step away.
    chains, zone_of_face = _join_fillet_passes(
        passes, owners, 0.5 * step_over, sample_interval, tool_radius, band_counts
    )

    def ends_on_other_face(entry, owner):
        point = FreeCAD.Vector(*entry[4:7])
        return any(
            _touches(f, point) for j, f in enumerate(fillet_faces) if j != owner
        )

    zones = {}
    for run, closed, faces, first, last in chains:
        if not closed:
            # Free ends only: an end stopping on another fillet or corner
            # face is an internal edge and keeps its full length.
            run = _trim_run_ends(
                run,
                edge_trim,
                at_start=not ends_on_other_face(run[0], owners[first]),
                at_end=not ends_on_other_face(run[-1], owners[last]),
            )
            if len(run) < 2:
                continue
        mean_z = sum(r[6] for r in run) / len(run)
        pts = [(r[0], r[1]) for r in run]
        zone = zone_of_face[next(iter(faces))]
        zones.setdefault(zone, []).append((pts, closed, mean_z))

    zone_list = [sorted(z, key=lambda c: -c[2]) for z in zones.values()]

    fillet_lines = []
    current = None
    while zone_list:
        if current is not None:
            cx, cy = current
            zone_list.sort(key=lambda z: (z[0][0][0][0] - cx) ** 2 + (z[0][0][0][1] - cy) ** 2)
        for pts, closed, _z in zone_list.pop(0):
            if closed:
                pts = _rotate_closed_start(pts, current)
            fillet_lines.append([(x, y, 0.0) for x, y in pts])
            current = pts[-1]

    joined = len(passes) - len(chains)
    Path.Log.debug(
        f"generate_fillet_scan_lines: {len(passes)} pass(es) on {len(fillet_faces)} fillet(s) "
        f"({len(corners)} corner patch(es)), {steep_stats.get('rescued', 0)} steep pass(es) moved to {_FILLET_MAX_DRAFT_ANGLE:g} deg, "
        f"{steep_stats.get('skipped', 0)} skipped, "
        f"{joined} join(s), {len(fillet_lines)} chain(s) in {len(zones)} zone(s) | "
        f"evaluation {evaluated - started:.3f}s, "
        f"joining/ordering {time.perf_counter() - evaluated:.3f}s"
    )
    return fillet_lines
