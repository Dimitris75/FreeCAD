# SPDX-License-Identifier: LGPL-2.1-or-later

# ***************************************************************************
# *   Copyright (c) 2026 Dimitris75 <dimitriospana75@gmail.com>               *
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

"""Z-Level Hybrid (constant-Z contour) generation using native geometry.

Implements a high-performance, geometric-only alternative to OCL-based operations.
Utilizes FreeCAD's native slicing kernel combined with the Path.Area (ClipperLib) 
C++ engine for precise tool radius compensation, linear radius sub-sampling, 
and robust layer-wise masking. Automatically detects and reconciles CAD floors 
to provide a complete hybrid finishing strategy for both steep walls and flat areas.
"""

import math
import FreeCAD
import Part
import Path


# ---------------------------------------------------------------------------
# Depth categorization
# ---------------------------------------------------------------------------


def categorize_floor_steps(
    shape,
    start_z,
    final_z,
    step_down
):
    """Reconciles physical model floors with calculated step-down heights.

    This function generates a top-down list of Z-depths starting from start_z 
    to final_z. It then analyzes the model geometry to find horizontal faces 
    (floors) and categorizes each depth as 'Pure' (standard step), 'Mixed' 
    (step lands on a floor), or 'Extra' (floor exists between standard steps).

    Args:
        shape: The manifold Part.Shape of the model to analyze.
        start_z: The absolute Z-height where machining begins (mm).
        final_z: The absolute target Z-depth (mm).
        step_down: The desired vertical distance between passes (mm).

    Returns:
        A list of tuples: (z_height, status, floor_geometry_at_Z0).
        Statuses are strings: "Pure", "Mixed", or "Extra".
    """
    # 1. Generate standard Z-heights list top-down
    z_heights = []
    curr_z = start_z
    while curr_z > (final_z + 0.0001):
        z_heights.append(round(curr_z, 5))
        curr_z -= step_down
    z_heights.append(round(final_z, 5))

    # 2. Get physical floors from model geometry
    fused_geometry = _get_fused_floor_geometry(shape, start_z, final_z)

    final_depth_logic = []
    accounted_floors = set()

    # 3. Match standard steps to physical floors
    for z_std in z_heights:
        match_z = None
        for floor_z in fused_geometry.keys():
            if abs(floor_z - z_std) < 0.0005:
                match_z = floor_z
                break

        if match_z is not None:
            final_depth_logic.append((z_std, "Mixed", fused_geometry[match_z]))
            accounted_floors.add(match_z)
        else:
            final_depth_logic.append((z_std, "Pure", None))

    # 4. Add intermediate floors as 'Extra' steps
    for z_f, geo in fused_geometry.items():
        if z_f not in accounted_floors:
            final_depth_logic.append((z_f, "Extra", geo))

    final_depth_logic.sort(key=lambda x: x[0], reverse=True)
    return final_depth_logic

def _get_fused_floor_geometry(
    shape,
    start_z,
    final_z,
    tolerance=0.001
):
    """Identifies and fuses upward-facing horizontal faces within the machining range.

    Iterates through all faces of the shape, filtering for planar surfaces 
    whose normal vector points strictly toward the tool (+Z). It performs 
    an accessibility check to ensure the floor is not occluded by geometry 
    above it and fuses coincident faces at the same height into single regions.

    Args:
        shape: The Part.Shape to analyze.
        start_z: Upper vertical bound for floor detection (mm).
        final_z: Lower vertical bound for floor detection (mm).
        tolerance: Distance threshold for considering faces coplanar (mm).

    Returns:
        A dictionary: {z_height: fused_face_at_Z0}.
    """

    def is_upward(face):
        if not (hasattr(face.Surface, "TypeId") and "Plane" in face.Surface.TypeId):
            return False
        u1, u2, v1, v2 = face.ParameterRange
        norm = face.normalAt((u1 + u2) / 2.0, (v1 + v2) / 2.0)
        if face.Orientation == "Reversed":
            norm = norm.multiply(-1)
        return norm.z > 0.99

    def isAccessibleFromTop(face, shape, abs_top):
        """Accessibility Check: Solid Projection (Shadow Test)."""
        try:
            z = face.Vertexes[0].Z
            extrude_h = (abs_top - z) + 5.0
            test_face = face.copy()
            test_face.translate(FreeCAD.Vector(0, 0, 0.001))  # Nudge above floor
            projection = test_face.extrude(FreeCAD.Vector(0, 0, extrude_h))

            # If the intersection with the model is empty, path is clear
            return not shape.common(projection).Vertexes
        except:
            return False

    floor_accumulator = {}
    abs_top = shape.BoundBox.ZMax
    z_min, z_max = min(start_z, final_z), max(start_z, final_z)

    for face in shape.Faces:
        if is_upward(face):
            if isAccessibleFromTop(face, shape, abs_top):
                z = round(face.Vertexes[0].Z, 5)
                if (z >= z_min - tolerance) and (z <= z_max + tolerance):
                    f_copy = face.copy()
                    f_copy.translate(FreeCAD.Vector(0, 0, -f_copy.BoundBox.ZMin))
    
                    if z not in floor_accumulator:
                        floor_accumulator[z] = []
                    floor_accumulator[z].append(f_copy)

    fused = {}
    for z, faces in floor_accumulator.items():
        res = faces[0]
        if len(faces) > 1:
            for i in range(1, len(faces)):
                res = res.fuse(faces[i])
        if hasattr(res, "removeSplitter"):
            res = res.removeSplitter()
        fused[z] = res

    return fused


# ---------------------------------------------------------------------------
# Z-Level Hybrid layer generation
# ---------------------------------------------------------------------------


def zlevel_hybrid_stack(
    shape,
    categorizedSteps,
    borderFace,
    trimFace,
    tool_params,
    stock_to_leave,
    accuracy_val,
    z_offset,
    step_down,
    wpc
):
    """Calculates a stack of 2D clearing areas using geometric slicing and Clipper Booleans.

    This function processes the 3D model layer-by-layer. For each layer, it generates
    a composite silhouette by sub-sampling the model curvature, applies tool radius compensation,
    and resolves the final machining area using a persistent C++ masking engine.
    Uses a dual Squeeze-and-Snap strategy: Pack samples at the tool tip to handle
    high-curvature contact, and snap samples to model floors for precise transitions.
    Linear radius sampling is performed equator-first to enable geometric 
    caching on vertical walls.

    Args:
        shape: The source Part.Shape to be machined.
        categorizedSteps: List of tuples (z_target, status, floor_geo) from categorization.
        borderFace: A Part.Face representing the stock or boundary footprint.
        trimFace: A Part.Face representing the 'Outside World' ( forbidden zone).
        tool_params: Dict containing 'radius', 'c_rad', 'profile', 'is_threeD'.
        stock_to_leave: Horizontal (XY) distance to keep from the model (mm).
        accuracy_val: Integer or string representing the number of sub-slices.
        z_offset: Vertical (Axial) distance to shift the final paths (mm).
        step_down: The vertical step distance (mm).
        wpc: The Part.Circle workplane defining the 2D calculation plane.

    Returns:
        A list of tuples: (z_target, cutAreaShape, status).
    """
    Path.Log.debug("Z-Level Hybrid: Starting geometric stack generation.")

    # Helpers
    def _get_h_from_r(r_target):
        """Inverse Math: Finds height h for a specific radius r."""
        if not is_3d or r_target <= 1e-7:
            return 0.0
        if profile == "Ballend":
            return R - math.sqrt(max(0, R**2 - r_target**2))
        if profile == "Bullnose":
            if r_target <= (R - c_rad): return 0.0
            return c_rad - math.sqrt(max(0, c_rad**2 - (r_target - (R - c_rad))**2))
        return 0.0

    def _get_r_from_h(h_target):
        """Forward Math: Finds radius r for a specific height h."""
        if not is_3d:
            return R
        if profile == "Ballend":
            return math.sqrt(max(0, R**2 - (R - h_target)**2))
        if profile == "Bullnose":
            if h_target < c_rad:
                return (R - c_rad) + math.sqrt(max(0, c_rad**2 - (c_rad - h_target)**2))
            return R
        return R

    # 1. Initialization
    stack = []
    allPrevComp = None
    tol = 0.001

    R, c_rad = tool_params["radius"], tool_params["c_rad"]
    profile, is_3d = tool_params["profile"], tool_params["is_threeD"]
    num_slices = int(accuracy_val) if is_3d else 1

    # Cache state
    prev_Area = None
    cached_cut_area = None
    cache_is_safe = False if step_down > (c_rad + 0.01) and is_3d else True

    # 2. Pre-load C++ engine
    area_engine = Path.Area()
    area_engine.setPlane(wpc)

    # Ensure we work on a clean copy
    proc_shape = shape.copy()
    area_engine.add(proc_shape)

    # Configure C++ engine parameters
    params = area_engine.getParams()
    params["SectionTolerance"] = 0.0001

    # 3. Identify critical snapping depths (Top and floors)
    modelBottom, modelTop = proc_shape.BoundBox.ZMin, proc_shape.BoundBox.ZMax
    critical_heights = {round(h, 6) for h, status, _ in categorizedSteps if status in ["Mixed", "Extra"]}
    critical_heights.add(round(modelTop, 6))

    # Progress Indicator
    indicator = FreeCAD.Base.ProgressIndicator()
    indicator.start("Z-Level Hybrid: Processing Geometry...", len(categorizedSteps))

    # 4. Main layer loop
    for z_target, status, floor_geo in categorizedSteps:
        if z_target > (modelTop - tol):
            indicator.next()
            continue

        # A. Squeeze logic: Calculate sampling window
        dist_submerged = max(0, modelTop - z_target)
        # Nudge slice height based on whether we are clearing a floor or a wall
        # Standard layers nudge up (+); Floors nudge down (-) to stay inside material
        slice_bias = 0.0002 if status in ["Mixed", "Extra"] else -0.0002

        # Widest radius reachable in current contact zone
        max_r_reachable = _get_r_from_h(dist_submerged) if dist_submerged < c_rad else R
        h_ceiling = min(c_rad, dist_submerged - tol) if is_3d else 0.0

        # B. Generate sampling plan (Height, Radius pairs)
        # Pre-calculating pairs avoids redundant math inside the heavy C++ loop
        sampling_plan = []

        # Linear Radius Steps (Squeeze sampling on Top of the model)
        for i in range(num_slices):
            r_theo = (max_r_reachable / (num_slices - 1)) * i if num_slices > 1 else max_r_reachable
            sampling_plan.append((_get_h_from_r(r_theo), r_theo))

        # Geometric Feature Snapping (Snap of an extra sample on edges)
        for ch in critical_heights:
            rel_h = ch - z_target
            if 0.0001 < rel_h < (h_ceiling - 0.0001):
                sampling_plan.append((rel_h, _get_r_from_h(rel_h)))

        # Clean, Unique, and Sort steps from Tip (h=0) to Equator
        # We sort by h to keep the slicing sequence logical
        unique_steps = sorted({(round(h, 6), round(r, 6)) for h, r in sampling_plan})
        unique_steps.reverse()

        # C. Geometric Fusion with Lazy Cache Validation
        fusion = Path.Area()
        fusion.setPlane(wpc)
        hit_cache = False

        for idx, (h, r_theo) in enumerate(unique_steps):
            r_comp = r_theo + stock_to_leave

            # Synchronized Slicing
            slice_z = max(modelBottom + 1e-5, min(z_target + h + slice_bias, modelTop - 1e-5))

            # Trigger C++ Slicing with dynamic offset
            params["Offset"] = r_comp
            area_engine.setParams(**params)
            sections = area_engine.makeSections(mode=0, project=False, heights=[slice_z])

            if sections:
                sub_face = sections[0].getShape()
                if sub_face and not sub_face.isNull():
                    # Cache check: We check the Area of the very first sub-sample
                    if idx == 0 and cache_is_safe and not status in ["Mixed", "Extra"]:
                        current_Area = sub_face.Area
                        if current_Area == prev_Area and cached_cut_area is not None:
                            hit_cache = True
                            break
                        prev_Area = current_Area
                    # Move results to machine plane for dissolved fusion
                    sub_face.translate(FreeCAD.Vector(0, 0, -sub_face.BoundBox.ZMin))
                    fusion.add(sub_face)

        if not sub_face:
            indicator.next()
            continue

        # D. Boolean resolution
        total_shift = z_target + z_offset

        if hit_cache:
            # Rapid Path: Reuse previous clearing result
            final_cut = cached_cut_area.copy()
            final_cut.translate(FreeCAD.Vector(0, 0, total_shift))

            # Store target G-code depth, calculated geometry, and metadata
            stack.append((total_shift, final_cut, status))
        else:
            # currentSilhouette is the union of all 3D contact points at this depth
            currentSilhouette = fusion.getShape()
            if hasattr(currentSilhouette, "removeSplitter"):
                currentSilhouette = currentSilhouette.removeSplitter()

            # Clearing engine (Clipper Booleans)
            layer_engine = Path.Area()
            layer_engine.setPlane(wpc)

            if status == "Extra":
                # Surgical Floor Mode: Only clear the floor geometry, ignore stock boundary
                if floor_geo:
                    layer_engine.add(floor_geo)
                    layer_engine.add(currentSilhouette, op=1)  # Subtract model

                if trimFace:
                    layer_engine.add(trimFace, op=1)

            else:
                # Standard Mode: Material = (Stock - Model) - TrimMask
                layer_engine.add(borderFace)
                layer_engine.add(currentSilhouette, op=1)

                if trimFace:
                    layer_engine.add(trimFace, op=1)

                # Rest Machining: subtract material cleared in layers above
                if allPrevComp:
                    layer_engine.add(allPrevComp, op=1)

            cutArea = layer_engine.getShape()

            # Reconciliation & Translation
            if cutArea:
                # Cache cutArea
                if cache_is_safe:
                    cached_cut_area = cutArea.copy()

                final_cut = cutArea.copy()
                final_cut.translate(FreeCAD.Vector(0, 0, total_shift))

                # Store target G-code depth, calculated geometry, and metadata
                stack.append((total_shift, final_cut, status))

                # Update Persistent Mask (strictly model silhouette to keep pockets open)
                mask_engine = Path.Area()
                mask_engine.setPlane(wpc)
                # Start with the mask from layers above
                if allPrevComp:
                    mask_engine.add(allPrevComp)
                # Add the current model silhouette
                mask_engine.add(currentSilhouette)
                # Add the physical floors (Mixed or Extra)
                if (status == "Mixed" or status == "Extra") and floor_geo:
                    mask_engine.add(floor_geo)
                # Extract the new 'Watertight' mask for the next iteration
                allPrevComp = mask_engine.getShape()

        indicator.next()

    indicator.stop()
    return stack


# ---------------------------------------------------------------------------
# G-code generation
# ---------------------------------------------------------------------------


def zlevel_hybrid_to_gcode(
    stack,
    feed_params,
    height_params,
    pattern_options,
    ignore_outer,
    clear_planar_only,
    step_over,
    radius
):
    """Converts the geometry stack into G-code Path Commands.

    This function iterates through a pre-calculated stack of geometric slices,
    generating perimeter (Waterline) paths and optional floor-clearing patterns.
    It manages tool engagement directions, safety transitions to safe heights,
    and progress reporting.

    Args:
        stack: A list of tuples (z_target, cutArea, status) representing layers.
        feed_params: Dict containing 'horizFeed', 'vertFeed', 'horizRapid', 'vertRapid'.
        height_params: Dict containing 'safe_hght' and 'clearance_hght'.
        pattern_options: Dict containing 'cut_climb', 'cut_pattern', 'pattern_angle',
            'reverse_pattern'.
        ignore_outer: Boolean. If True, skips the outermost boundary (stock edge).
        clear_planar_only: Boolean. If True, only clears floors detected as
            Mixed or Extra.
        step_over: The horizontal step-over distance for clearing patterns (mm).
        radius: The tool radius (mm).

    Returns:
        A list of Path.Command objects (G-code).
    """
    Path.Log.debug("Z-Level Hybrid: Starting G-code generation.")

    # 1. Initialization
    commands = []

    # Extract feeds and speeds
    h_feed = feed_params.get("horizFeed", 0.0)
    v_feed = feed_params.get("vertFeed", 0.0)
    h_rapid = feed_params.get("horizRapid", 0.0)
    v_rapid = feed_params.get("vertRapid", 0.0)

    # Extract heights
    safe_z = height_params.get("safe_hght", 3.0)
    clear_z = height_params.get("clearance_hght", 5.0)

    # Extract pattern logic
    cut_climb = pattern_options.get("cut_climb", True)
    pattern_name = pattern_options.get("cut_pattern", "None")
    pattern_angle = pattern_options.get("pattern_angle", 0.0)
    reverse_pattern = pattern_options.get("reverse_pattern", False)

    # Progress Indicator setup
    stLen = len(stack)
    indicator = FreeCAD.Base.ProgressIndicator()
    indicator.start("Z-Level Hybrid: Generating G-Code...", stLen)

    # 2. Main Layer Processing
    for z_target, cutArea, status in stack:

        if not cutArea or cutArea.isNull() or not cutArea.Wires:
            indicator.next()
            continue

        # Cut pattern reversed
        working_area = cutArea.reversed() if reverse_pattern else cutArea

        # Determine start index (0 = machine stock edge, 1 = ignore stock edge)
        start_w_idx = 1 if ignore_outer else 0

        # A: Perimeters (Waterline Walls)
        if start_w_idx < len(working_area.Wires):
            for w_idx in range(start_w_idx, len(working_area.Wires)):
                wire = working_area.Wires[w_idx]
                if not wire.isClosed():
                    continue

                # Geometry cleanup
                if hasattr(wire, "removeSplitter"):
                    wire = wire.removeSplitter()
                wire.fix(1e-6, 1e-6, 1e-4)

                # Determine start point coordinates
                V = wire.Vertexes
                lv = len(V) - 1
                # Start at the end vertex for Climb to move backward through CCW wire
                start_p = FreeCAD.Vector(V[lv].X, V[lv].Y, V[lv].Z) if cut_climb else FreeCAD.Vector(V[0].X, V[0].Y, V[0].Z)

                # Safety transition: Rapid to SafeHeight, then to start position
                commands.append(Path.Command("G0", {"Z": safe_z, "F": v_rapid}))
                commands.append(Path.Command("G0", {"X": start_p.x, "Y": start_p.y, "F": h_rapid}))
                # Move to depth (plunge)
                commands.append(Path.Command("G1", {"Z": z_target, "F": v_feed}))

                # Generate the wire-following path
                path_params = {
                    "shapes": [wire],
                    "feedrate": h_feed,
                    "start": start_p,
                    "preamble": False,
                    "retraction": safe_z,
                    "resume_height": safe_z
                }

                try:
                    pp = Path.fromShapes(**path_params)
                    commands.extend(pp.Commands)
                except Exception as e:
                    Path.Log.error(f"Z-Level Hybrid: Path generation failed at Z={z_target}: {str(e)}")

        # B: Cut pattern
        should_clear = False
        if pattern_name != "None":
            if clear_planar_only:
                # Targeted mode: only clear physical model floors
                if status in ["Mixed", "Extra"]:
                    should_clear = True
            else:
                # Global mode: clear every depth level
                should_clear = True

        if should_clear:
            # Ensure tool is at a safe level before moving into the pattern
            commands.append(Path.Command("G0", {"Z": safe_z, "F": v_rapid}))

            # Dispatch to the high-speed Path.Area pattern engine
            pattern_cmds = _generatePattern(
                cutArea,
                pattern_name,
                pattern_angle,
                cut_climb,
                reverse_pattern,
                z_target,
                step_over,
                radius,
                feed_params,
                safe_z
            )
            commands.extend(pattern_cmds)

        indicator.next()

    # 3. Finalize Operation
    indicator.stop()

    # Return to clearance height
    commands.append(Path.Command("G0", {"Z": clear_z, "F": v_rapid}))

    Path.Log.info(f"Z-Level Hybrid: G-code generation complete. {len(commands)} commands.")
    return commands

def _generatePattern(
    cutArea,
    cut_pattern,
    pattern_angle,
    cut_climb,
    reverse_pattern,
    z_target,
    step_over,
    radius,
    feed_params,
    safe_hght
):
    """Generates high-speed infill patterns using the native C++ Path.Area engine.

    This function utilizes the Clipper-based C++ kernel to calculate 2D clearing
    patterns (ZigZag, Offset, Line, Grid) within a provided boundary. It handles
    tool radius compensation, pattern rotation, and machining sequence.

    Args:
        cutArea: A Part.Face or Part.Shape representing the boundary to clear.
        cut_pattern: String identifier for the pattern (ZigZag, Offset, Line, Grid).
        pattern_angle: Float representing the yaw angle (degrees) for scanline patterns.
        cut_climb: Boolean. If True, uses Climb milling; otherwise, Conventional.
        reverse_pattern: Boolean. If True, reverses the clearing order (e.g., Inside-Out).
        z_target: The target Z-coordinate for the toolpath (machining depth).
        step_over: The horizontal distance between consecutive passes (mm).
        radius: The tool radius used for extra offset calculation (mm).
        feed_params: Dictionary containing 'horizFeed' and 'vertFeed' values.
        safe_hght: The Z-height for rapid transitions between segments.

    Returns:
        A list of Path.Command objects representing the clearing G-code.
    """
    Path.Log.debug(f"Z-Level Hybrid: Generating {cut_pattern} pattern at Z={z_target}")
    cmds = []
    should_reverse = True

    # 1. Validation Guards
    if not cutArea or cutArea.isNull():
        Path.Log.warning("Z-Level Hybrid: Pattern generation skipped - Null cutArea.")
        return []

    if cutArea.Area < 1e-7:
        return []

    # 2. Engine Setup
    horiz_feed = feed_params.get("horizFeed", 0.0)
    vert_feed = feed_params.get("vertFeed", 0.0)

    engine = Path.Area()
    engine.add(cutArea)

    # 3. Map UI Strategy to C++ PocketMode
    if cut_pattern == "ZigZag":
        pattern_mode = 1
    elif cut_pattern == "Offset":
        pattern_mode = 2
        # Specific logic for Offset: Climb vs Reverse
        if cut_climb != reverse_pattern:
            should_reverse = False
    elif cut_pattern == "Line":
        pattern_mode = 5
    elif cut_pattern == "Grid":
        pattern_mode = 6
    else:
        Path.Log.error(f"Z-Level Hybrid: Unsupported pattern type '{cut_pattern}'")
        return []

    # 4. Configure C++ Solver Parameters
    extra_offset = radius - step_over
    params = engine.getParams()
    params['PocketMode'] = pattern_mode
    params['PocketStepover'] = step_over
    params['PocketExtraOffset'] = -extra_offset
    params["Angle"] = float(pattern_angle)
    params["ToolRadius"] = radius
    params["FromCenter"] = reverse_pattern

    engine.setParams(**params)  

    # 5. Execute Native Solver
    try:
        engine.makePocket()
        res_area = engine.getShape()
    except Exception as e:
        Path.Log.error(f"Z-Level Hybrid: Pattern G-code generation failed: {str(e)}")
        return []

    if not res_area or res_area.isNull():
        return []

    # Apply topological reversal for Climb milling on Offset rings if needed
    if should_reverse:
        res_area = res_area.reversed()

    # 6. G-Code Generation Loop
    for wire in res_area.Wires:
        if not wire.isClosed() and pattern_mode == 2:  # Offsets should be closed
            continue

        # A. Lead-In: Transition at Safe Height, then plunge
        start_p = wire.Vertexes[0].Point
        cmds.append(Path.Command("G0", {"Z": safe_hght}))
        cmds.append(Path.Command("G0", {"X": start_p.x, "Y": start_p.y}))
        cmds.append(Path.Command("G1", {"Z": z_target, "F": vert_feed}))

        # B. Generate XY Path
        path_params = {
            "shapes": [wire],
            "feedrate": horiz_feed,
            "start": start_p,
            "preamble": False,
            'verbose': True,
            "retraction": safe_hght,
            "resume_height": safe_hght
        }
        pp = Path.fromShapes(**path_params)
        # Extend Commands list
        cmds.extend(pp.Commands)

        # C. Safety Retract after each segment (island or ring)
        cmds.append(Path.Command("G0", {"Z": safe_hght}))

    return cmds
