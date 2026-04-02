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
    wpc
):
    """Calculates a stack of 2D clearing areas using geometric slicing and Clipper Booleans.

    This function processes the 3D model layer-by-layer. For each layer, it generates 
    a composite silhouette by sub-sampling the model curvature (Linear Radius Squeeze), 
    applies tool radius compensation, and resolves the final machining area 
    using a persistent C++ masking engine.

    Args:
        shape: The source Part.Shape to be machined.
        categorizedSteps: List of tuples (z_target, status, floor_geo) from categorization.
        borderFace: A Part.Face representing the stock or boundary footprint.
        trimFace: A Part.Face representing the 'Outside World' ( forbidden zone).
        tool_params: Dict containing 'radius', 'c_rad', 'profile', 'is_threeD'.
        stock_to_leave: Horizontal (XY) distance to keep from the model (mm).
        accuracy_val: Integer or string representing the number of sub-slices.
        z_offset: Vertical (Axial) distance to shift the final paths (mm).
        wpc: The Part.Circle workplane defining the 2D calculation plane.

    Returns:
        A list of tuples: (z_target, cutAreaShape, status).
    """
    Path.Log.debug("Z-Level Hybrid: Starting geometric stack generation.")

    stack = []
    allPrevComp = None  # Persistent C++ mask engine (Tracks model silhouette)

    # 1. Initialization and Pre-loading
    area_engine = Path.Area()
    area_engine.setPlane(wpc)

    # Ensure we work on a clean copy
    proc_shape = shape.copy()
    area_engine.add(proc_shape)

    # Configure C++ engine parameters
    params = area_engine.getParams()
    params["SectionTolerance"] = 0.0001

    # Extract tool geometry
    radius = tool_params["radius"]
    c_rad = tool_params["c_rad"]
    profile = tool_params["profile"]
    is_3d = tool_params["is_threeD"]

    # Sampling strategy
    num_slices = int(accuracy_val) if is_3d else 1

    modelBottom, modelTop = proc_shape.BoundBox.ZMin, proc_shape.BoundBox.ZMax

    # Progress Indicator
    indicator = FreeCAD.Base.ProgressIndicator()
    total_layers = len(categorizedSteps)
    indicator.start("Z-Level Hybrid: Processing Geometry...", total_layers)

    # 2. Main Machining Loop
    for idx, (z_target, status, floor_geo) in enumerate(categorizedSteps):

        if z_target > (modelTop - 0.001):
            indicator.next()
            continue

        # A: Multi-Slice composite silhouette
        # Calculate vertical contact window
        dist_submerged = max(0, modelTop - z_target)
        is_at_top = dist_submerged < c_rad

        # Apply 2.2% safety bias ONLY when tool tip is just entering the model top
        r_bias = 0.978 if (is_at_top and is_3d) else 1.0

        max_h = min(c_rad, dist_submerged - 0.001)
        if max_h < 0: max_h = 0

        # Calculate maximum radius reachable for this submerged depth
        max_r_reachable = math.sqrt(max(0, radius**2 - (radius - max_h)**2)) if is_3d else radius

        # Engine to fuse sub-samples into one clean silhouette
        fusion = Path.Area()
        fusion.setPlane(wpc)

        sections = None
        for i in range(num_slices):
            # Divide radius linearly to ensure steps are spread equally
            div = (num_slices - 1) if num_slices > 1 else 1
            r_theo = (max_r_reachable / div) * i if num_slices > 1 else max_r_reachable

            # Inverse math: find height h that corresponds to this horizontal radius
            h = 0.0
            if is_3d and r_theo > 1e-7:
                if profile == "Ballend":
                    h = radius - math.sqrt(max(0, radius**2 - r_theo**2))
                elif profile == "Bullnose":
                    if r_theo > (radius - c_rad):
                        h = c_rad - math.sqrt(max(0, c_rad**2 - (r_theo - (radius - c_rad))**2))

            # Final offset distance including Stock to Leave and top-corner bias
            r_comp = (r_theo * r_bias) + stock_to_leave
            
            # Slicing height: ensure it stays within model boundaries
            slice_z = max(modelBottom + 1e-5, min(z_target + h + 0.0005, modelTop - 1e-5))

            # Trigger C++ Slicing with dynamic offset
            params["Offset"] = r_comp
            area_engine.setParams(**params)
            sections = area_engine.makeSections(mode=0, project=False, heights=[slice_z])

            if sections:
                sub_face = sections[0].getShape()
                if sub_face and not sub_face.isNull():
                    # Normalize to Z=0 for fusion with other heights
                    sub_face.translate(FreeCAD.Vector(0, 0, -sub_face.BoundBox.ZMin))
                    fusion.add(sub_face)

        if not sections:
            indicator.next()
            continue

        # currentSilhouette is the union of all 3D contact points at this depth
        currentSilhouette = fusion.getShape()
        if hasattr(currentSilhouette, "removeSplitter"):
            currentSilhouette = currentSilhouette.removeSplitter()

        # B: Clearing engine (Clipper Booleans)
        layer_engine = Path.Area()
        layer_engine.setPlane(wpc)

        if status == "Extra":
            # Surgical Floor Mode: Only clear the floor geometry, ignore stock boundary
            if floor_geo:
                layer_engine.add(floor_geo)
                layer_engine.add(currentSilhouette, op=1)  # Subtract model
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

        # C: Reconciliation & Translation
        if cutArea:
            # Apply final DepthOffset (Axial Stock to Leave)
            total_shift = z_target + z_offset

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

        # Winding adjustment for Climb vs Conventional milling
        working_area = cutArea.reversed() if cut_climb else cutArea

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
    should_reverse = False

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
        # Specific logic for Offset: if both Climb and Reverse are requested,
        if cut_climb and reverse_pattern:
            should_reverse = True
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
        res_area = res_area.reversed()  # --- Test Climb ---

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
            "retraction": z_target,
            "resume_height": z_target
        }
        pp = Path.fromShapes(**path_params)

        # Extract commands from Toolpath object
        for c in pp.Commands:
            # Enforce machining depth for all linear and circular moves
            if any(k in c.Parameters for k in ['X', 'Y']):
                c.Parameters['Z'] = z_target
            cmds.append(c)

        # C. Safety Retract after each segment (island or ring)
        cmds.append(Path.Command("G0", {"Z": safe_hght}))

    return cmds
