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

"""Waterline (constant-Z contour) generation using OpenCamLib.

Wraps OCL's Waterline (push-cutter + Weave) and AdaptiveWaterline
algorithms.  Also provides a fallback slice-based waterline using
FreeCAD's shape.slice() for when OCL is not available.
"""

import Path
import time
import math

__title__ = "Surface Waterline Generator"
__author__ = "sliptonic (Brad Collette)"
__url__ = "https://www.freecad.org"

if False:
    Path.Log.setLevel(Path.Log.Level.DEBUG, Path.Log.thisModule())
    Path.Log.trackModule(Path.Log.thisModule())
else:
    Path.Log.setLevel(Path.Log.Level.INFO, Path.Log.thisModule())


from Path.Base.Generator.surface_common import _get_ocl

# ---------------------------------------------------------------------------
# OCL Waterline (push-cutter + Weave)
# ---------------------------------------------------------------------------


def waterline(stl, cutter, sampling, z, threads=None, timer=None):
    """Run OCL Waterline (push-cutter + Weave) at a single Z-height.

    This is the primary waterline algorithm.  It uses BatchPushCutter
    internally to push the cutter along X and Y fibers, then builds
    a Weave graph and extracts loops via face-traversal.

    This replaces the ~400-line custom topo-map code in the old
    Waterline.py "OCL Dropcutter" mode.

    Args:
        stl: ``ocl.STLSurf`` mesh.
        cutter: OCL cutter object.
        sampling: Fiber spacing (mm).
        z: Z-height for this waterline slice.
        threads: Number of OpenMP threads (*None* = auto-detect).
        timer: Optional ``timer(stage_name, elapsed_seconds)`` callback.

    Returns:
        List of loops.  Each loop is a list of ``(x, y, z)`` tuples.
    """
    ocl = _get_ocl()

    wl = ocl.Waterline()
    wl.setSTL(stl)
    wl.setCutter(cutter)
    wl.setSampling(sampling)
    wl.setZ(z)

    if threads is not None and threads > 0:
        wl.setThreads(threads)

    t0 = time.time()
    wl.run()
    t1 = time.time()

    if timer:
        timer("waterline_z{:.3f}".format(z), t1 - t0)

    raw_loops = wl.getLoops()
    loops = []
    for raw_loop in raw_loops:
        loop = [(pt.x, pt.y, pt.z) for pt in raw_loop]
        loops.append(loop)

    Path.Log.debug("waterline z={:.3f}: {} loops in {:.3f}s".format(z, len(loops), t1 - t0))

    return loops


# ---------------------------------------------------------------------------
# OCL AdaptiveWaterline
# ---------------------------------------------------------------------------


def adaptive_waterline(stl, cutter, sampling, min_sampling, z, threads=None, timer=None):
    """Run OCL AdaptiveWaterline at a single Z-height.

    Adaptive-sampling variant that refines fiber density where the
    contour changes rapidly.  Better quality than fixed-sampling
    Waterline but potentially slower.

    Args:
        stl: ``ocl.STLSurf`` mesh.
        cutter: OCL cutter object.
        sampling: Base fiber spacing (mm).
        z: Z-height for this waterline slice.
        min_sampling: Minimum sampling interval for adaptive refinement.
        threads: Number of OpenMP threads.
        timer: Optional callback.

    Returns:
        List of loops.  Each loop is a list of ``(x, y, z)`` tuples.
    """
    ocl = _get_ocl()

    awl = ocl.AdaptiveWaterline()
    awl.setSTL(stl)
    awl.setCutter(cutter)
    awl.setSampling(sampling)
    awl.setMinSampling(min_sampling)
    awl.setZ(z)

    if threads is not None and threads > 0:
        awl.setThreads(threads)

    t0 = time.time()
    awl.run()
    t1 = time.time()

    if timer:
        timer("adaptive_waterline_z{:.3f}".format(z), t1 - t0)

    raw_loops = awl.getLoops()
    loops = []
    for raw_loop in raw_loops:
        loop = [(pt.x, pt.y, pt.z) for pt in raw_loop]
        loops.append(loop)

    Path.Log.debug(
        "adaptive_waterline z={:.3f}: {} loops in {:.3f}s".format(z, len(loops), t1 - t0)
    )

    return loops


# ---------------------------------------------------------------------------
# Multi-height waterline stack
# ---------------------------------------------------------------------------


def waterline_stack(
    stl,
    cutter,
    sampling,
    min_sampling,
    min_z,
    max_z,
    step_down,
    adaptive=False,
    depth_offset=0.0,
    threads=None,
    timer=None,
):
    """Generate waterline contours at multiple Z-heights.

    Convenience function that calls :func:`waterline` or
    :func:`adaptive_waterline` at each Z-level from *max_z* down to
    *min_z* in *step_down* increments.

    Args:
        stl: ``ocl.STLSurf`` mesh.
        cutter: OCL cutter object.
        sampling: Fiber spacing (mm).
        min_sampling: Minimum sampling for adaptive mode.
        min_z: Final depth (lowest Z).
        max_z: Start depth (highest Z).
        step_down: Step-down increment between layers.
        adaptive: If True, use AdaptiveWaterline.
        threads: OpenMP thread count.
        depth_offset: Z offset applied to all output points.
        timer: Optional callback.

    Returns:
        Ordered dict mapping ``z_height`` -> list of loops.
        Each loop is a list of ``(x, y, z)`` tuples.
    """
    from collections import OrderedDict

    # Add a small epsilon to the slicing Z-level to avoid Z-fighting
    # with simplified planar meshes.
    epsilon = 0.001

    # Compute Z-heights from max_z down to min_z
    z_heights = []
    z = max_z
    while z >= min_z - 1e-6:
        z_heights.append(z)
        z -= step_down

    # Ensure min_z is included
    if z_heights and abs(z_heights[-1] - min_z) > 1e-6:
        z_heights.append(min_z)

    result = OrderedDict()

    t0 = time.time()

    for z in z_heights:
        zh = z + epsilon
        if adaptive:
            loops = adaptive_waterline(
                stl,
                cutter,
                sampling,
                min_sampling,
                zh,
                threads=threads,
                timer=timer,
            )
        else:
            loops = waterline(
                stl,
                cutter,
                sampling,
                zh,
                threads=threads,
                timer=timer,
            )

        if not loops:
            Path.Log.debug("waterline_stack: no loops at z={:.3f}, skipping".format(zh))
            continue

        # Apply depth offset
        if abs(depth_offset) > 1e-9:
            offset_loops = []
            for loop in loops:
                offset_loops.append([(pt[0], pt[1], pt[2] + depth_offset) for pt in loop])
            result[zh] = offset_loops
        else:
            result[zh] = loops

    t1 = time.time()
    if timer:
        timer("waterline_stack_total", t1 - t0)

    Path.Log.debug(
        "waterline_stack: {} heights, {} total loops in {:.3f}s".format(
            len(z_heights),
            sum(len(v) for v in result.values()),
            t1 - t0,
        )
    )

    return result


# ---------------------------------------------------------------------------
# G-code generation from waterline data
# ---------------------------------------------------------------------------

# Tolerance (mm) to decide that the next contour starts where the last one ended
STAY_DOWN_TOLERANCE = 0.01


def _reorient_loop_start(loop, start_pt):
    """
    Rotates a closed loop so that it starts at the point nearest to start_pt.

    Used to align a loop's seam with the tool's current position, so the
    transit to the loop is as short as possible. The points are cycled, never
    reordered, so the cutting direction is kept.

    Args:
        loop (list): (x, y, z) tuples of a closed loop, without the closing
            point (the first point is not repeated at the end).
        start_pt (tuple): (x, y) of the tool, or None.

    Returns:
        list: The same points, starting at the one nearest to start_pt (by XY
            distance; the first one found wins a tie). The loop itself when
            it is empty, when start_pt is None, or when it already starts there.
    """
    if not loop or not start_pt:
        return loop

    min_dist = float("inf")
    closest_idx = 0

    for i, pt in enumerate(loop):
        dist = math.hypot(pt[0] - start_pt[0], pt[1] - start_pt[1])
        if dist < min_dist:
            min_dist = dist
            closest_idx = i

    if closest_idx == 0:
        return loop

    # Cycle the list
    return loop[closest_idx:] + loop[:closest_idx]


def _extend_overlap(pts, overlap):
    """
    Continues a closed loop past its seam, so the seam leaves no witness mark.

    The loop is followed again from its start for ``overlap`` mm (XY length);
    the last added point is interpolated (X, Y and Z) on the segment where that
    length is reached.

    Args:
        pts (list): (x, y, z) tuples of a closed loop whose first point is
            repeated at the end (pts[0] == pts[-1]).
        overlap (float): Distance to run past the seam, in mm.

    Returns:
        list: pts followed by the overlap points. pts unchanged (same object)
            when overlap <= 0 or the loop has fewer than three points. When the
            loop is shorter than overlap, it is run once more in full.
    """
    if overlap <= 0.0 or len(pts) < 3:
        return pts
    out, walked = list(pts), 0.0
    for a, b in zip(pts, pts[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if seg <= 1e-9:
            continue
        if walked + seg >= overlap:
            t = (overlap - walked) / seg
            out.append(tuple(a[k] + (b[k] - a[k]) * t for k in range(3)))
            return out
        out.append(b)
        walked += seg
    return out


def _short_tail(pts, distance):
    """
    Returns the last ``distance`` mm of a path as a two-point segment.

    The lead arcs only use the direction of the path's last segment, but the
    lead-out fallback (a straight retreat) goes back to the point before the
    last. OCL leaves long straight stretches as single segments, so that point
    can be far away; this short synthetic tail keeps the retreat local while
    giving the arcs the same direction.

    Args:
        pts (list): (x, y, z) tuples of the path just cut (at least two).
        distance (float): Length of the tail along the path (XY), in mm.

    Returns:
        list: [point distance mm back along the path, last point]. When the
            path is shorter than distance: [first point, last point].
    """
    end = pts[-1]
    walked = 0.0
    for a, b in zip(reversed(pts[:-1]), reversed(pts[1:])):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if seg <= 1e-9:
            continue
        if walked + seg >= distance:
            t = (distance - walked) / seg
            back = tuple(b[k] + (a[k] - b[k]) * t for k in range(3))
            return [back, end]
        walked += seg
    return [pts[0], end]


def _contour_heights(waterline_data, open_data):
    """
    Returns the Z heights to process, in cutting order.

    Args:
        waterline_data (dict): z -> closed loops, in the stack's order (top-down).
        open_data (dict): z -> open chains (may be empty).

    Returns:
        list: waterline_data's heights in their own order when there are no
            open chains. Otherwise every height of either dict, sorted
            top-down like the stack (heights with only open chains included).
    """
    heights = list(waterline_data.keys())
    if open_data:
        heights = sorted(set(heights) | set(open_data), reverse=True)
    return heights


def _pop_nearest_contour(remaining, tool_xy):
    """
    Removes and returns the contour the tool should cut next.

    Nearest-neighbour choice (a greedy travelling-salesman step): the contour
    with the entry point nearest to the tool wins. A closed loop can be
    entered at any of its points (its seam is moved there afterwards); an
    open chain only at its first point.

    Args:
        remaining (list): (points, is_closed) tuples still to cut. Modified:
            the chosen contour is removed.
        tool_xy (tuple): (x, y) of the tool, or None before the first cut, in
            which case the first contour in the list is taken.

    Returns:
        tuple: (points, is_closed) of the chosen contour. On equal distances
            the contour listed first wins.
    """
    if not tool_xy:
        return remaining.pop(0)

    best_idx = 0
    best_dist = float("inf")
    for i, (loop, closed) in enumerate(remaining):
        for pt in loop if closed else loop[:1]:
            d = math.hypot(pt[0] - tool_xy[0], pt[1] - tool_xy[1])
            if d < best_dist:
                best_dist = d
                best_idx = i
    return remaining.pop(best_idx)


def _close_and_orient(loop, tool_xy, cut_climb):
    """
    Prepares a closed loop for cutting: seam, closing point and direction.

    Args:
        loop (list): (x, y, z) tuples of a closed loop as traced by OCL.
        tool_xy (tuple): (x, y) of the tool, or None (seam left where it is).
        cut_climb (bool): Reverse the loop for climb milling.

    Returns:
        list: The loop starting at the point nearest to the tool, with its
            first point repeated at the end (unless it already was, within
            1e-5 mm), reversed for climb. Reversing keeps the start point.
    """
    if tool_xy:
        loop = _reorient_loop_start(loop, tool_xy)

    pts = list(loop)
    if math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) > 1e-5:
        pts.append(pts[0])

    if cut_climb:
        pts.reverse()
    return pts


def _order_contours(waterline_data, open_data, cut_climb):
    """
    Puts every closed loop and open chain in cutting order, ready to cut.

    Heights are processed top-down (see _contour_heights). Within a height,
    contours are chosen nearest-first from where the previous one ended (see
    _pop_nearest_contour), so the tool keeps moving to the closest work.

    Closed loops get their seam at the point nearest to the tool, their
    closing point and their climb direction (_close_and_orient). Open chains
    keep their points: they can only be entered at their start, so they are
    reversed for climb before ordering and are never closed or re-seamed.

    Args:
        waterline_data (dict): z -> closed loops ((x, y, z) tuples).
        open_data (dict): z -> open chains, or None.
        cut_climb (bool): Climb milling (reversed contours).

    Returns:
        list: (points, is_closed) tuples in cutting order.
    """
    open_data = open_data or {}
    ordered = []
    tool_xy = None

    for zh in _contour_heights(waterline_data, open_data):
        remaining = [(list(loop), True) for loop in waterline_data.get(zh, [])]
        for chain in open_data.get(zh, []):
            chain = list(chain)
            if cut_climb:
                chain.reverse()
            remaining.append((chain, False))

        while remaining:
            pts, closed = _pop_nearest_contour(remaining, tool_xy)
            if closed:
                pts = _close_and_orient(pts, tool_xy, cut_climb)
            ordered.append((pts, closed))
            tool_xy = (pts[-1][0], pts[-1][1])

    return ordered


def _stays_down(prev_pts, next_pts):
    """
    Smart Retraction test: does the next contour start where the last one ended?

    When it does (stacked loops on a vertical wall, chained open pieces), the
    tool stays in the material and simply moves down to the next level: no
    retract, no lead-out and no lead-in between the two.

    Args:
        prev_pts (list): Points of the contour just cut, or None.
        next_pts (list): Points of the contour to cut next.

    Returns:
        bool: True when the XY distance from the end of prev_pts to the start
            of next_pts is within STAY_DOWN_TOLERANCE.
    """
    return (
        prev_pts is not None
        and math.hypot(next_pts[0][0] - prev_pts[-1][0], next_pts[0][1] - prev_pts[-1][1])
        <= STAY_DOWN_TOLERANCE
    )


def _make_lead_attempt(leads):
    """
    Builds the lead-arc function for the Smart Lead-In/Lead-Out settings.

    The contours carry the Depth Offset already (waterline_stack), but the
    leads must be probed in the model frame, against the safe STL. The
    returned function takes the offset out of the points, hands it to
    surface_postprocess._attempt_lead_arc (which adds it back to the moves it
    emits) and puts it back on the returned end point.

    Args:
        leads (dict): Lead settings (see waterline_to_gcode), or None.

    Returns:
        callable or None: f(points, is_lead_in) -> (commands, point), where
            point is the lead's far end (the start of a lead-in, the end of a
            lead-out) or None when no arc fits. None when leads is falsy.
    """
    if not leads:
        return None

    from .surface_postprocess import _attempt_lead_arc

    lead_dz = leads.get("depth_offset", 0.0)

    def attempt(pts, is_lead_in):
        base = [(p[0], p[1], p[2] - lead_dz) for p in pts]
        cmds, point = _attempt_lead_arc(
            base,
            leads["safe_pdc"],
            leads["cutter"],
            leads["lead_feed"],
            leads["lift_lead_z"],
            is_lead_in=is_lead_in,
            depth_offset=lead_dz,
        )
        if point is not None:
            point = (point[0], point[1], point[2] + lead_dz)
        return cmds, point

    return attempt


class _WaterlineWriter:
    """
    Writes the moves of a waterline toolpath and tracks the tool between them.

    Keeps the Path.Command list, the tool's last XY position and whether the
    tool is retracted, so each contour only has to say where it starts and
    what it cuts.

    Args:
        horiz_feed, vert_feed (float): Cutting feeds (mm/min).
        horiz_rapid, vert_rapid (float): Rapid feeds (mm/min).
        safe_z (float): Height for transit moves between contours.
        clearance_z (float): Height of the first move.
    """

    def __init__(self, horiz_feed, vert_feed, horiz_rapid, vert_rapid, safe_z, clearance_z):
        self.horiz_feed = horiz_feed
        self.vert_feed = vert_feed
        self.horiz_rapid = horiz_rapid
        self.vert_rapid = vert_rapid
        self.safe_z = safe_z
        self.commands = [Path.Command("G0", {"Z": clearance_z, "F": vert_rapid})]
        self.tool_xy = None  # XY of the last emitted move
        self.is_retracted = True  # The tool is above the material

    def travel_to(self, entry):
        """
        Moves the tool over the entry point of the next contour.

        Nothing is written when the tool is already there (within
        STAY_DOWN_TOLERANCE): it stays down. Otherwise the tool retracts to
        safe_z if it is down, then rapids to the entry XY.

        Args:
            entry (tuple): (x, y, ...) of the contour's (or lead's) start.
        """
        if self.tool_xy:
            dist = math.hypot(entry[0] - self.tool_xy[0], entry[1] - self.tool_xy[1])
        else:
            dist = float("inf")
        if dist > STAY_DOWN_TOLERANCE:
            self.retract()
            self.commands.append(
                Path.Command("G0", {"X": entry[0], "Y": entry[1], "F": self.horiz_rapid})
            )

    def plunge_to(self, point):
        """Feeds straight down (vertical feed) to point (x, y, z)."""
        self.commands.append(
            Path.Command("G1", {"X": point[0], "Y": point[1], "Z": point[2], "F": self.vert_feed})
        )

    def cut(self, points, plunge_first):
        """
        Cuts through points with G1 moves; the tool is then down at the last one.

        Args:
            points (list): (x, y, z) tuples to visit (at least one).
            plunge_first (bool): The first move is the plunge onto the contour
                (vertical feed), e.g. after a transit or when staying down to
                the next level. False after a lead-in, which already brought
                the tool onto the contour.
        """
        for i, pt in enumerate(points):
            feed = self.vert_feed if (i == 0 and plunge_first) else self.horiz_feed
            self.commands.append(
                Path.Command("G1", {"X": pt[0], "Y": pt[1], "Z": pt[2], "F": feed})
            )
        self.tool_xy = (points[-1][0], points[-1][1])
        self.is_retracted = False

    def add(self, commands, end=None):
        """
        Appends ready-made moves (lead arcs).

        Args:
            commands (list): Path.Command objects.
            end (tuple): (x, y, ...) where they leave the tool, if it moved.
        """
        self.commands.extend(commands)
        if end is not None:
            self.tool_xy = (end[0], end[1])

    def retract(self):
        """Retracts to safe_z, unless the tool is already retracted."""
        if not self.is_retracted:
            self.commands.append(Path.Command("G0", {"Z": self.safe_z, "F": self.vert_rapid}))
            self.is_retracted = True


def waterline_to_gcode(
    waterline_data,
    horiz_feed,
    vert_feed,
    horiz_rapid,
    vert_rapid,
    safe_z,
    clearance_z,
    cut_climb=False,
    open_data=None,
    leads=None,
):
    """
    Converts waterline contours into a list of Path.Command.

    1. Order: every closed loop and open chain is put in cutting order,
       top-down and nearest-first, with seams and climb direction set
       (_order_contours).
    2. Write: each contour is reached, optionally entered on a lead-in arc,
       cut, optionally run past its seam, and left on a lead-out arc.
       Smart Retraction: when the next contour starts where this one ends
       (_stays_down), the tool stays in the material, with no retract and no
       leads in between.

    Args:
        waterline_data (dict): Ordered dict from waterline_stack(): z_height ->
            list of closed loops, each a list of (x, y, z) tuples.
        horiz_feed (float): Horizontal cutting feed (mm/min).
        vert_feed (float): Vertical (plunge) feed (mm/min).
        horiz_rapid (float): Horizontal rapid feed (mm/min).
        vert_rapid (float): Vertical rapid feed (mm/min).
        safe_z (float): Safe height for transit moves.
        clearance_z (float): Clearance height (first and only move above safe_z).
        cut_climb (bool): Reverse the contours for climb cutting.
        open_data (dict): Optional z_height -> list of *open* chains, e.g. loops
            trimmed to a boundary. An open chain is cut from its start to its
            end, in loop order (so climb/conventional is kept), is never closed
            and never re-seamed; the tool retracts between chains unless the
            next one starts where the last one ended.
        leads (dict): Optional Smart Lead-In/Lead-Out settings (see
            surface_postprocess._attempt_lead_arc): ``safe_pdc``, ``cutter``,
            ``lead_feed``, ``lift_lead_z``, ``overlap`` and ``depth_offset``.
            Every contour is entered and left on a tangent arc when the safe
            STL allows it (straight plunge / retreat otherwise). Closed loops
            run ``overlap`` mm past their seam before the lead-out, so the
            seam leaves no witness mark.

    Returns:
        list: Path.Command objects.
    """
    contours = _order_contours(waterline_data, open_data, cut_climb)
    attempt_lead = _make_lead_attempt(leads)
    writer = _WaterlineWriter(horiz_feed, vert_feed, horiz_rapid, vert_rapid, safe_z, clearance_z)

    for idx, (pts, closed) in enumerate(contours):
        prev_pts = contours[idx - 1][0] if idx > 0 else None
        next_pts = contours[idx + 1][0] if idx + 1 < len(contours) else None
        arrives_down = _stays_down(prev_pts, pts)
        # The tool leaves this contour (retract and lead-out) unless the next
        # one starts right here
        leaving = not (next_pts and _stays_down(pts, next_pts))

        # Lead-in arc, unless the tool stays down from the previous contour
        lead_in_cmds, lead_start = [], None
        if attempt_lead and not arrives_down and len(pts) >= 2:
            lead_in_cmds, lead_start = attempt_lead(pts, is_lead_in=True)

        writer.travel_to(lead_start if lead_start else pts[0])
        if lead_start:
            # Down to the arc start, then the arc onto the contour
            writer.plunge_to(lead_start)
            writer.add(lead_in_cmds)
            cut_pts = pts[1:]
        else:
            cut_pts = pts

        # A closed loop the tool leaves runs past its seam; one the tool stays
        # down after keeps its seam, so the next level starts right below it.
        if leads and closed and leaving:
            extended = _extend_overlap(pts, leads.get("overlap", 0.0))
            cut_pts = cut_pts + extended[len(pts) :]
            pts = extended

        writer.cut(cut_pts, plunge_first=not lead_start)

        # Lead-out arc, unless the next contour starts right here
        if attempt_lead and len(pts) >= 2 and leaving:
            tail = _short_tail(pts, leads["cutter"].getDiameter() / 2.0)
            lead_out_cmds, lead_end = attempt_lead(tail, is_lead_in=False)
            if lead_end:
                writer.add(lead_out_cmds, end=lead_end)

    writer.retract()
    return writer.commands
