# SPDX-License-Identifier: LGPL-2.1-or-later

# ***************************************************************************
# *   Copyright (c) 2026 Dimitris75 <dimitriospana75@gmail.com>             *
# *                                                                         *
# *   This program is free software; you can redistribute it and/or modify  *
# *   it under the terms of the GNU Lesser General Public License (LGPL)    *
# *   as published by the Free Software Foundation; either version 2 of     *
# *   the License, or (at your option) any later version.                   *
# *   for detail see the LICENCE text file.                                 *
# *                                                                         *
# *   This program is distributed in the hope that it will be useful,       *
# *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
# *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
# *   GNU Library General Public License for more details.                  *
# *                                                                         *
# *   You should have received a copy of the GNU Library General Public     *
# *   License along with this program; if not, write to the Free Software   *
# *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  *
# *   USA                                                                   *
# *                                                                         *
# ***************************************************************************

"""Adaptive clearing toolpath generator.

Wraps the area.Adaptive2d algorithm to produce per-layer adaptive clearing
toolpaths. Designed for use by any CAM operation that needs adaptive clearing
on pre-computed 2D boundary shapes.

Usage:
    from Path.Base.Generator import adaptive_common

    cmds = adaptive_common.generate(
        adaptive_params = {
            "op_type"            : "ClearingInside",
            "adaptive_accuracy"  : 0.1,
            "stock_to_leave"     : 0.0,
            "force_insideout"    : False,
            "finishing_profile"  : True,
            "lift_distance"      : 0.5,
            "keep_tool_down"     : 3.0,
            "helix_angle"        : 3.0,
            "helix_diameter"     : 75.0,
            "helix_min_diameter" : 10.0,
        },
        feed_params     = feed_params,
        radius          = tool_radius,
        step_over       = step_over,
        z_target        = z_target,
        safe_z          = safe_hght,
        prev_z          = prev_z,
        cut_area        = cut_area,
        bb_face         = border_face,
        cut_area_offset = 0.0,
        bb_face_offset  = 0.0,
    )
"""

import math
import FreeCAD
import Part
import Path

__title__  = "Adaptive Clearing Generator"
__author__  = "Dimitrios Pana"
__url__     = "https://www.freecad.org"


if False:
    Path.Log.setLevel(Path.Log.Level.DEBUG, Path.Log.thisModule())
    Path.Log.trackModule(Path.Log.thisModule())
else:
    Path.Log.setLevel(Path.Log.Level.INFO, Path.Log.thisModule())


# ---------------------------------------------------------------------------
# Offset areas helper
# ---------------------------------------------------------------------------


# Offset cutting area or boundary face to match Adaptive2d standards if needed
def _offset_area(area, area_offset):
    try:
        offset_engine = Path.Area()
        offset_engine.add(area)
        offset_engine.setParams(Offset=area_offset)
        offset_area = offset_engine.getShape()
        if (offset_area and not offset_area.isNull() and offset_area.Wires):
            return offset_area
        else:
            Path.Log.warning(
                f"adaptive_common.generate: Area offset failed."
            )
            return None
    except Exception as e:
        Path.Log.warning(
            f"adaptive_common.generate: Area offset failed: {e} ."
        )
        return None


# ---------------------------------------------------------------------------
# Wire discretization helper
# ---------------------------------------------------------------------------


def _wire_to_2d(wire, deflection=0.01):
    """Discretize a wire to a list of [x, y] points."""
    pts = []
    for edge in wire.Edges:
        for pt in edge.discretize(Deflection=deflection):
            pts.append([pt.x, pt.y])
    return pts


# ---------------------------------------------------------------------------
# Helix entry generator
# ---------------------------------------------------------------------------


def _generate_helix_entry(
    region,
    z_target,
    prev_z,
    safe_z,
    radius,
    feed_params,
    helix_min_diameter,
    helix_angle,
):
    """
    Generates a helix ramp entry move for a single Adaptive2d result region.

    Uses `region.HelixCenterPoint` and `region.StartPoint` (provided by
    Adaptive2d) to compute the helix geometry, then delegates to
    Path.Base.Generator.helix for G-code production.

    The helix descends from `prev_z` (previous layer cut depth) rather than
    `safe_z` — minimising air travel on deep multi-pass operations.

    Falls back to a straight plunge if:
    - The helix radius is smaller than helix_min_diameter / 2
    - helix.generate() raises an exception

    Args:
        region:               A single Adaptive2d result region object.
        z_target (float):     Z depth for this layer (helix bottom).
        prev_z (float):       Previous layer Z depth (helix top).
        safe_z (float):    Safe Z height — used only for the initial retract.
        radius (float):       Tool radius in mm.
        feed_params (dict):   Feed/rapid rates.
        helix_min_diameter (float): Minimum allowable helix diameter in mm.
        helix_angle (float):  Helix ramp angle in degrees.

    Returns:
        list: Path.Command objects for the helix entry (or straight plunge).
    """
    from . import helix as helix_gen

    v_feed  = feed_params.get("vertFeed",   0.0)
    h_rapid = feed_params.get("horizRapid", 0.0)
    v_rapid = feed_params.get("vertRapid",  0.0)

    p1 = region.HelixCenterPoint   # (x, y) helix center
    p2 = region.StartPoint         # (x, y) helix radius endpoint / cut start

    helix_radius  = math.dist(p1[:2], p2[:2])
    dir_angle_rad = math.atan2(p2[1] - p1[1], p2[0] - p1[0])

    commands = []

    # Retract to safe_z before moving to helix entry position
    commands.append(Path.Command("G0", {"Z": safe_z, "F": v_rapid}))

    if helix_radius > helix_min_diameter / 2.0:
        r              = helix_radius - 0.01   # tiny margin from wall
        ramp_angle_rad = math.radians(max(helix_angle, 0.5))
        pitch          = max(2.0 * math.pi * r * math.tan(ramp_angle_rad), 0.001)

        center_top    = FreeCAD.Vector(p1[0], p1[1], prev_z)
        center_bottom = FreeCAD.Vector(p1[0], p1[1], z_target)
        edge = Part.makeLine(center_top, center_bottom)

        try:
            helix_cmds = helix_gen.generate(
                edge                = edge,
                outer_radius        = r,
                pitch               = pitch,
                retract_height      = prev_z,
                direction           = "CCW",
                startAt             = "Inside",
                finish_circle       = True,
                cone_angle_rad      = 0.0,
                dir_angle_rad       = dir_angle_rad,
                ramp_angle_rad      = ramp_angle_rad,
            )

            # helix_gen.generate() prepends G0 Z retract [0] and appends
            # a retract [-1] — strip both since we manage retracts here.
            if len(helix_cmds) > 2:
                commands.extend(helix_cmds[1:-1])
                Path.Log.debug(
                    f"adaptive_common._generate_helix_entry: "
                    f"Z={round(z_target, 3)}, r={round(r, 3)}mm, "
                    f"angle={round(helix_angle, 1)}°, pitch={round(pitch, 3)}mm."
                )
                return commands

        except Exception as e:
            Path.Log.warning(
                f"adaptive_common._generate_helix_entry: helix.generate() failed "
                f"at Z={round(z_target, 3)}: {e} — falling back to straight plunge."
            )
    else:
        Path.Log.debug(
            f"adaptive_common._generate_helix_entry: helix radius too small "
            f"({round(helix_radius, 4)}mm < {round(helix_min_diameter / 2.0, 4)}mm) "
            f"— using straight plunge."
        )

    # Fallback: rapid to start point and straight plunge
    commands.append(Path.Command("G0", {"X": p2[0], "Y": p2[1], "F": h_rapid}))
    commands.append(Path.Command("G1", {"Z": z_target, "F": v_feed}))
    return commands


# ---------------------------------------------------------------------------
# Main adaptive pattern generator
# ---------------------------------------------------------------------------


def generate(
    adaptive_params,
    feed_params,
    radius,
    step_over,
    z_target,
    safe_z,
    prev_z,
    cut_area,
    bb_face,
    cut_area_offset=0.0,
    bb_face_offset=0.0,
):
    """
    Generates an adaptive clearing toolpath for a single Z-Level layer.

    `cut_area` shape at 'z_target'.

    `bb_face` provides the stock boundary.

    Results are converted point-by-point to G-code following the model
    'one command per point', with Z moves emitted only when the height
    changes (lz tracking).

    Args:
        adaptive_params (dict): Dict with keys: op_type, adaptive_accuracy, stock_to_leave,
                                force_insideout, finishing_profile, lift_distance,
                                keep_tool_down, helix_angle, helix_diameter,
                                helix_min_diameter.
        feed_params (dict):     Feed/rapid rates.
        radius (float):         Tool radius in mm.
        step_over (float):      Stepover distance in mm.
        z_target (float):       Z depth for this layer.
        safe_z (float):         Safe Z height for full retracts.
        prev_z (float):         Previous layer Z depth (helix entry start).
        cut_area (Part.Shape):  2D cutting boundary face for this layer.
        bb_face (Part.Shape):   2D stock boundary face.
        cut_area_offset (float):Offset value for cutting area or 0.0
        bb_face_offset (float): Offset value for boundary face or 0.0

    Returns:
        list: Path.Command objects for this layer's adaptive pattern,
              or [] on failure.
    """
    if not cut_area or cut_area.isNull():
        Path.Log.warning(
            f"adaptive_common.generate: No valid cutting area "
            f"at Z={round(z_target, 3)} — skipping."
        )
        return []

    try:
        import area as _area
    except ImportError:
        Path.Log.error(
            "adaptive_common.generate: libarea not available — "
            "cannot run adaptive pattern."
        )
        return []

    # -- Unpack parameters --
    tool_diam           = radius * 2.0
    op_type             = adaptive_params.get("op_type", "ClearingInside")
    adaptive_accuracy   = adaptive_params.get("adaptive_accuracy",   0.1)
    stock_to_leave      = adaptive_params.get("stock_to_leave",      0.0)
    force_insideout     = adaptive_params.get("force_insideout",     True)
    finishing_profile   = adaptive_params.get("finishing_profile",   True)
    lift_distance       = adaptive_params.get("lift_distance",       0.05)
    keep_tool_down      = adaptive_params.get("keep_tool_down",      3.0)
    helix_angle         = adaptive_params.get("helix_angle",         3.0)
    helix_diam_pct      = adaptive_params.get("helix_diameter",      75.0)
    helix_min_diam_pct  = adaptive_params.get("helix_min_diameter",  10.0)

    helix_diameter      = tool_diam * helix_diam_pct     / 100.0
    helix_min_diameter  = tool_diam * helix_min_diam_pct / 100.0

    h_feed  = feed_params.get("horizFeed",  0.0)
    v_feed  = feed_params.get("vertFeed",   0.0)
    v_rapid = feed_params.get("vertRapid",  0.0)
    h_rapid = feed_params.get("horizRapid", 0.0)

    # Map string to enum
    op_type_map = {
        "ClearingInside"   : _area.AdaptiveOperationType.ClearingInside,
        "ClearingOutside"  : _area.AdaptiveOperationType.ClearingOutside,
        "ProfilingInside"  : _area.AdaptiveOperationType.ProfilingInside,
        "ProfilingOutside" : _area.AdaptiveOperationType.ProfilingOutside,
    }

    # Apply cutting area offset
    if cut_area_offset != 0.0:
        cut_area = _offset_area(cut_area, cut_area_offset)
        if not cut_area:
            return []

    # Apply boundary face offset
    if bb_face_offset != 0.0:
        bb_face = _offset_area(bb_face, bb_face_offset)
        if not bb_face:
            return []

    # -- Build 2D path lists --
    path2d  = []
    stock2d = []

    for wire in cut_area.Wires:
        if not wire.isClosed():
            continue
        pts = _wire_to_2d(wire)
        if pts:
            path2d.append(pts)

    for wire in bb_face.Wires:
        if not wire.isClosed():
            continue
        pts = _wire_to_2d(wire)
        if pts:
            stock2d.append(pts)

    if not path2d:
        Path.Log.warning(
            f"adaptive_common.generate: No valid closed wires "
            f"at Z={round(z_target, 3)} — skipping."
        )
        return []

    # -- Configure Adaptive2d --
    a2d = _area.Adaptive2d()
    a2d.toolDiameter            = float(tool_diam)
    a2d.stepOverFactor          = min(step_over / tool_diam, 1.0)
    a2d.stockToLeave            = float(stock_to_leave)
    a2d.tolerance               = float(max(float(adaptive_accuracy), 0.01))  # Adaptive2d minimum
    a2d.forceInsideOut          = bool(force_insideout)
    a2d.finishingProfile        = bool(finishing_profile)
    a2d.opType                  = op_type_map.get(op_type, _area.AdaptiveOperationType.ClearingInside)
    a2d.helixRampTargetDiameter = float(helix_diameter)
    a2d.helixRampMinDiameter    = float(helix_min_diameter)
    a2d.keepToolDownDistRatio   = float(keep_tool_down)

    # -- Execute --
    try:
        results = a2d.Execute(stock2d, path2d, [], lambda tpaths: False)
    except Exception as e:
        Path.Log.error(
            f"adaptive_common.generate: Adaptive2d.Execute failed "
            f"at Z={round(z_target, 3)}: {e}"
        )
        return []

    if not results:
        Path.Log.debug(
            f"adaptive_common.generate: Adaptive2d returned no results "
            f"at Z={round(z_target, 3)}."
        )
        return []

    # -- Convert results to G-code --
    commands = []

    for result in results:
        if not result.AdaptivePaths:
            continue

        # Helix ramp entry for this region
        commands.extend(
            _generate_helix_entry(
                region             = result,
                z_target           = z_target,
                prev_z             = prev_z,
                safe_z             = safe_z,
                radius             = radius,
                feed_params        = feed_params,
                helix_min_diameter = helix_min_diameter,
                helix_angle        = helix_angle,
            )
        )

        lz = prev_z

        for idx, (motion_type, points) in enumerate(result.AdaptivePaths):
            if not points:
                continue

            for pt in points:
                x, y = pt[0], pt[1]

                if motion_type == _area.AdaptiveMotionType.Cutting:
                    z = z_target
                    if z != lz:
                        commands.append(Path.Command("G1", {"Z": z, "F": v_feed}))
                    commands.append(Path.Command("G1", {"X": x, "Y": y, "F": h_feed}))

                elif motion_type == _area.AdaptiveMotionType.LinkClear:
                    # LinkClear: lift by lift_distance above cut depth.
                    # Note: Adaptive2d's LinkClear classification can be
                    # unreliable near convex corners. lift_distance provides
                    # a safety margin — increase it if collisions occur.
                    z = z_target + float(lift_distance)
                    if z != lz:
                        commands.append(Path.Command("G0", {"Z": z, "F": v_rapid}))
                    commands.append(Path.Command("G0", {"X": x, "Y": y, "F": h_rapid}))

                elif motion_type == _area.AdaptiveMotionType.LinkNotClear:
                    z = safe_z
                    if z != lz:
                        commands.append(Path.Command("G0", {"Z": z, "F": v_rapid}))
                    commands.append(Path.Command("G0", {"X": x, "Y": y, "F": h_rapid}))

                lz = z

    return commands
