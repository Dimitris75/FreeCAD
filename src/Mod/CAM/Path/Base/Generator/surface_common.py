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

"""Shared utilities for 3D surface and waterline generators.

Provides OCL cutter creation and model preparation (mesh detection, vertical
face filtering). Boundaries (where a strategy may cut) live in
surface_boundary. These are pure functions with no FreeCAD document access —
tool parameters and geometry are passed in by the operation wrapper.
"""

import Path
import Part

__title__ = "Surface Common Utilities"
__author__ = "sliptonic (Brad Collette)"
__url__ = "https://www.freecad.org"

if False:
    Path.Log.setLevel(Path.Log.Level.DEBUG, Path.Log.thisModule())
    Path.Log.trackModule(Path.Log.thisModule())
else:
    Path.Log.setLevel(Path.Log.Level.INFO, Path.Log.thisModule())


# ---------------------------------------------------------------------------
# OCL import helper
# ---------------------------------------------------------------------------


_ocl = None


def _get_ocl():
    """Lazily import OCL, trying both package names."""
    global _ocl
    if _ocl is not None:
        return _ocl
    try:
        import ocl

        _ocl = ocl
    except ImportError:
        try:
            import opencamlib as ocl

            _ocl = ocl
        except ImportError:
            raise ImportError(
                "OpenCamLib (ocl) is required for 3D surface operations. "
                "Install it via your package manager or from "
                "https://github.com/aewallin/opencamlib"
            )
    return _ocl


# ---------------------------------------------------------------------------
# OCL Cutter creation
# ---------------------------------------------------------------------------


# Map of FreeCAD ToolBit shape types to OCL cutter factory names
_TOOL_TYPE_MAP = {
    "endmill": "CylCutter",
    "ballend": "BallCutter",
    "bullnose": "BullCutter",
    "taperedballnose": "BallCutter",
    "drill": "ConeCutter",
    "engraver": "ConeCutter",
    "v_bit": "ConeCutter",
    "v-bit": "ConeCutter",
    "vbit": "ConeCutter",
}


def make_ocl_cutter(
    tool_type,
    diameter,
    corner_radius=0.0,
    flat_radius=0.0,
    edge_height=0.0,
    edge_angle=0.0,
    length_offset=0.0,
):
    """Create an OCL cutter from tool parameters.

    Pure function — no FreeCAD document access.  Tool parameters are
    extracted by the operation wrapper before calling this.

    Args:
        tool_type: ToolBit shape type string (e.g. 'endmill', 'ballend',
                   'bullnose', 'drill', 'v-bit', etc.)
        diameter: Tool diameter in mm.
        corner_radius: Corner radius for bull-nose cutters.
        flat_radius: Flat radius at tip (derived from diameter and
                     corner_radius for bull-nose).
        edge_height: Cutting edge height in mm.
        edge_angle: Cutting edge full angle in degrees (for V-bits / drills).
        length_offset: Length offset in mm.

    Returns:
        An ``ocl`` cutter object, or *None* if the tool type is not
        supported.
    """
    ocl = _get_ocl()
    tool_type_lower = tool_type.lower()
    cutter_name = _TOOL_TYPE_MAP.get(tool_type_lower)

    if cutter_name is None:
        Path.Log.error("Unsupported tool type '{}' for OCL cutter creation.".format(tool_type))
        return None

    if diameter <= 0:
        Path.Log.error("Tool diameter must be positive, got {}".format(diameter))
        return None

    if cutter_name == "CylCutter":
        if edge_height <= 0:
            Path.Log.warning(
                f"The cutting edge height for this tool is set to {edge_height}. Using the tool's diameter as a fallback height."
            )
            edge_height = diameter
        return ocl.CylCutter(diameter, edge_height + length_offset)

    elif cutter_name == "BallCutter":
        if edge_height <= 0:
            edge_height = diameter / 2.0
        return ocl.BallCutter(diameter, edge_height + length_offset)

    elif cutter_name == "BullCutter":
        if edge_height <= 0:
            Path.Log.warning(
                f"The cutting edge height for this Bull-nose tool is set to {edge_height}. Using the tool's diameter as a fallback height."
            )
            edge_height = diameter
        # OCL BullCutter(diameter, minor_radius, length)
        # minor_radius = diameter/2 - flat_radius
        minor_radius = diameter / 2.0 - flat_radius
        if minor_radius < 0:
            minor_radius = 0.0
        return ocl.BullCutter(diameter, minor_radius, edge_height + length_offset)

    elif cutter_name == "ConeCutter":
        if edge_angle <= 0:
            Path.Log.error("ConeCutter requires a positive edge_angle, got {}".format(edge_angle))
            return None
        # OCL ConeCutter(diameter, half_angle, length)
        return ocl.ConeCutter(diameter, edge_angle / 2.0, length_offset)

    return None


def make_safe_cutter(
    tool_type,
    diameter,
    corner_radius=0.0,
    flat_radius=0.0,
    edge_height=0.0,
    edge_angle=0.0,
    length_offset=0.0,
    buffer_pct=0.25,
):
    """Create an oversized OCL cutter for safe-travel-height checks.

    Same interface as :func:`make_ocl_cutter` but inflates the diameter
    by *buffer_pct* (default 25 %).
    """
    safe_diam = diameter * (1.0 + buffer_pct)
    safe_flat = flat_radius * (1.0 + buffer_pct) if flat_radius > 0 else safe_diam * buffer_pct
    return make_ocl_cutter(
        tool_type,
        safe_diam,
        corner_radius=corner_radius,
        flat_radius=safe_flat,
        edge_height=edge_height,
        edge_angle=edge_angle,
        length_offset=length_offset,
    )


# ---------------------------------------------------------------------------
# Mesh detection
# ---------------------------------------------------------------------------


def _is_triangulated_mesh(faces, sample_size=75, threshold=0.50):
    """
    Heuristically detects whether `faces` come from a mesh-to-shape
    conversion, as opposed to a genuine CAD/BRep model.

    A bare "exactly 3 edges" test isn't enough — plenty of legitimate
    BRep faces on curved/circular parts (a conical wedge, a fillet
    blend, a trimmed cylindrical face) are also bounded by 3 edges while
    remaining curved. A genuine mesh-derived triangle is additionally
    *planar*.

    Args:
        faces (list): Part.Face objects to sample.
        sample_size (int): Maximum number of faces to sample.
        threshold (float): Fraction of sampled faces that must look like
            mesh triangles for the whole set to be flagged as triangulated.

    Returns:
        bool: True if the sampled faces look like a mesh-to-shape conversion.
    """
    if not faces:
        return False

    sample = faces[: min(sample_size, len(faces))]

    def _looks_like_mesh_triangle(face):
        if len(face.Edges) == 3:

            try:
                if isinstance(face.Surface, Part.Plane):
                    return True
            except Exception:
                return False

        return False

    triangle_count = sum(1 for f in sample if _looks_like_mesh_triangle(f))
    return len(sample) > 0 and (triangle_count / len(sample)) > threshold


# ---------------------------------------------------------------------------
# Filter vertical faces
# ---------------------------------------------------------------------------


def _horizontal_part(v):
    return (v.x * v.x + v.y * v.y) ** 0.5


def _is_vertical(face, tolerance=0.0005):
    """True for a face with no XY footprint (a wall parallel to the tool axis).

    Decided from the surface definition, not from one sampled normal: a
    curved face can have a horizontal normal at its middle (a fillet lying
    along X) and still have area in XY.

        Plane                       normal horizontal
        Cylinder                    axis vertical: every ruling runs along Z
        Extrusion                   extrusion direction vertical
        B-spline / Bezier           horizontal normals over a 3x3 UV grid
        Anything else               never a full wall (cone, sphere, torus...)
    """
    try:
        surf = face.Surface
    except Exception:
        return False
    if isinstance(surf, Part.Plane):
        return abs(surf.Axis.z) < tolerance
    if isinstance(surf, Part.Cylinder):
        return _horizontal_part(surf.Axis) < tolerance
    if isinstance(surf, Part.SurfaceOfExtrusion):
        return _horizontal_part(surf.Direction) < tolerance
    if isinstance(surf, (Part.BSplineSurface, Part.BezierSurface)):
        try:
            u1, u2, v1, v2 = face.ParameterRange
            for fu in (0.1, 0.5, 0.9):
                for fv in (0.1, 0.5, 0.9):
                    n = face.normalAt(u1 + (u2 - u1) * fu, v1 + (v2 - v1) * fv)
                    if abs(n.z) >= tolerance:
                        return False
            return True
        except Exception:
            return False
    return False


def _filter_vertical(model_faces, tolerance=0.0005):
    """Removes vertical faces (walls parallel to the tool axis) from a list of
    Part.Face objects. They have no XY footprint, so they add nothing to the
    mesh but triangles. See _is_vertical() for how a wall is recognised.

    Args:
        model_faces (list): A list of Part.Face objects to be filtered.
        tolerance (float, optional): How far from vertical a wall may be
            (Z component of its normal / horizontal component of its axis).
            Defaults to 0.0005.

    Returns:
        list: A new list of Part.Face objects with the vertical faces
            excluded. Returns the original list if filtering would result
            in an empty list.
    """
    filtered = [face for face in model_faces if not _is_vertical(face, tolerance)]
    return filtered if filtered else model_faces
