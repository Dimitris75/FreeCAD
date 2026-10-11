# SPDX-License-Identifier: LGPL-2.1-or-later

from __future__ import annotations

from typing import Any, Final

from Base.BaseClass import BaseClass
from Base.Metadata import export, no_args
from Base.Matrix import Matrix
from Base.Placement import Placement
from Part.App.TopoShape import TopoShape
from Mesh.App.Mesh import Mesh
from CAM.App.Command import Command

@export(
    FatherInclude="Base/BaseClassPy.h",
    Include="Mod/CAM/PathSimulator/App/PathSim.h",
    Namespace="PathSimulator",
    Constructor=True,
    Delete=True,
)
class PathSim(BaseClass):
    """
    FreeCAD python wrapper of PathSimulator

    PathSimulator.PathSim():

    Create a path simulator object

    Author: Shai Seger (shaise_at_g-mail)
    License: LGPL-2.1-or-later
    """

    def BeginSimulation(self, stock: TopoShape, resolution: float) -> None:
        """
        Start a simulation process on a box shape stock with given resolution
        """
        ...

    def ReseedFromMesh(
        self,
        mesh: Mesh,
        px: float,
        py: float,
        pz: float,
        lx: float,
        ly: float,
        lz: float,
        resolution: float,
        /,
    ) -> None:
        """
        Replace the current stock entirely with a fresh grid seeded from an
        already-tessellated mesh at the given bounding box. Used to re-seed
        mid-job from the current cut stock's own mesh, rotated into a new
        working frame, on a 3+2 workplane change between operations.

        Destroys the previous stock, so the buffers returned by
        GetHeightBuffer() are invalidated and must be fetched again.
        """
        ...

    def ReseedFromStock(
        self,
        matrix: Matrix,
        px: float,
        py: float,
        pz: float,
        lx: float,
        ly: float,
        lz: float,
        resolution: float,
        /,
    ) -> dict[str, int | float]:
        """
        Replace the current stock with a fresh grid at the given bounding
        box, holding the current stock's material carried through
        `matrix`, which maps the old working frame into the new one. Used
        on a 3+2 workplane change in place of ReseedFromMesh: no mesh is
        built, and a cavity the new frame cannot see into stays a cavity
        instead of being filled solid.

        `matrix` must be a rotation, optionally with a translation;
        anything else raises ValueError and leaves the stock untouched.

        Returns a report: faces, crossings, columns, imbalanced,
        extraSpans, area (of the source surface, in square units) and
        surfaceMs / assignMs. A non-zero `imbalanced` is a bug: that many
        columns met a gap in the source surface and were filled from
        their lowest crossing to their highest.

        Destroys the previous stock, so the buffers returned by
        GetHeightBuffer() and GetBottomBuffer() are invalidated and must
        be fetched again.
        """
        ...

    def GetSpanLayers(
        self,
        x0: int,
        y0: int,
        x1: int,
        y1: int,
        layers: int,
        tops: Any,
        bottoms: Any,
        /,
    ) -> None:
        """
        Write the spans BELOW each column's primary into `tops` and
        `bottoms`, for the inclusive cell rectangle (x0, y0)-(x1, y1),
        `layers` deep. Both must be writable contiguous buffers -- numpy
        arrays of float32 -- of at least layers * width * height floats.

        Layer 0 is the first span below the primary, layer 1 the one
        below that, and so on. Layout is layer-major then row-major over
        X, the same order as GetHeightBuffer:
            index = (k * width + (x - x0)) * height + (y - y0)
        A column with fewer spans than asked for gets the floor height in
        both arrays for the layers it lacks, which reads as no material.

        The primary span itself is NOT included; it is what
        GetHeightBuffer and GetBottomBuffer already carry. Cheap enough to
        call per frame over the dirty rectangle: the cost follows the
        number of columns holding extra spans, not the area requested.
        """
        ...

    def SetToolShape(self, tool: TopoShape, resolution: float, /) -> None:
        """
        Set the shape of the tool to be used for simulation
        """
        ...

    def GetResultMesh(self) -> tuple[Mesh, Mesh]:
        """
        Return the current mesh result of the simulation.
        """
        ...

    @no_args
    def MaterialVolume(self) -> float:
        """
        Volume of material the stock currently holds, in cubic units of
        the stock's own scale.

        Measured between the top and bottom surfaces, not from the top
        down to the floor, so it reflects the real extent of a stock that
        has been re-seeded into a rotated frame. Cells whose top has been
        cut below their bottom count as empty rather than negative.
        """
        ...

    @no_args
    def GetGridLength(self) -> tuple[float, float]:
        """
        Return the stock's size in X and Y. The grid covers exactly this:
        every cell is one resolution wide except the last column and row,
        which end at the stock's edge.
        """
        ...

    @no_args
    def BytesPerCell(self) -> int:
        """
        Bytes of simulation state per grid cell. Needs no stock: the
        figure is a property of the engine, and a fresh PathSim() can
        report it before any simulation exists.
        """
        ...

    def GetHeightBuffer(
        self,
    ) -> tuple[memoryview, memoryview, int, int, float, float, float, float, float]:
        """
        Return the raw height grid for direct viewport rendering, without
        building a Mesh.

        Returns (heights, originals, nx, ny, res, originX, originY,
        floorZ, planeZ). Both memoryviews are read-only float32, row-major
        over X: index = x * ny + y. The buffers are allocated once per
        simulation and never reallocated, so they may be wrapped once and
        kept for its lifetime -- but they are invalidated by anything that
        recreates the stock.
        """
        ...

    @no_args
    def GetBottomBuffer(self) -> memoryview:
        """
        Return the per-cell underside of the stock as a read-only float32
        memoryview, laid out exactly like the heights from
        GetHeightBuffer(): index = x * ny + y.

        Material exists between this and the height. A cell whose height
        is less than one cell size above its bottom holds nothing, even
        if it is well above the floor -- which is the normal state of a
        through-cut in a tilted frame after a 3+2 re-seed.

        The buffer lives as long as the stock, so it may be wrapped once,
        but read it live: a cut through the top span of a column holding
        several promotes the span below, bottom and all. Invalidated, like
        the heights, by anything that recreates the stock.
        """
        ...

    @no_args
    def InvalidateTessellation(self) -> None:
        """
        Does nothing. GetResultMesh() always tessellates the whole grid.

        Kept so that callers written for the incremental tessellator,
        which needed this before every complete mesh, keep working.
        """
        ...

    def ConsumeViewDirty(self) -> tuple[int, int, int, int] | None:
        """
        Return the cell rectangle (xMin, yMin, xMax, yMax, inclusive)
        touched since the last call, or None if nothing changed.
        """
        ...

    def ApplyCommand(self, placement: Placement, command: Command) -> Placement:
        """
        Apply a single path command on the stock starting from placement.
        """
        ...
    Tool: Final[Any]
    """Return current simulation tool."""
