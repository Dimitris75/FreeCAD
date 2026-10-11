// SPDX-License-Identifier: LGPL-2.1-or-later

/**************************************************************************
 *   Copyright (c) 2017 Shai Seger <shaise at gmail>                       *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************
 *   Volumetric Path simulation engine                                     *
 ***************************************************************************/

#pragma once

#include <algorithm>
#include <unordered_map>
#include <vector>

#include <Mod/Mesh/App/Mesh.h>
#include <Mod/CAM/App/Command.h>


#define SIM_EPSILON 0.00001

// Tolerance for "these two faces are at the same height", for the wall
// builders alone. A rounding epsilon, because they compare EMITTED face
// heights (m_topFace / m_botFace), which within one merged rectangle are
// identical rather than merely close.
//
// MUST stay far tighter than the rectangle merges' own m_res. Comparing
// per-cell m_stock at m_res leaves a cell-high gap along every step of a
// 45-degree slope, where adjacent cells differ by exactly one cell: 9%
// of the volume lost in tessellation, 17% more on the re-seed that reads
// the mesh back. Tightening the merges instead also fixes it, and costs
// one rectangle per cell on any slope -- two minutes for a high-accuracy
// re-seed.
#define SIM_HEIGHT_TOL SIM_EPSILON
#define SIM_TESSEL_TOP 1
#define SIM_TESSEL_BOT 2

// Tessellates an arbitrary BREP shape into outMesh. Used to seed a cStock
// from real geometry (see cStock::SeedFromMesh) instead of just its
// bounding box -- both for the initial stock, and for re-seeding a fresh
// cStock mid-job from the current cut stock's own mesh, rotated into a
// new working frame, on a 3+2 workplane change between operations.
void TessellateShapeToMesh(
    const TopoDS_Shape& shape,
    float linearDeflection,
    float angularDeflection,
    Mesh::MeshObject& outMesh
);

struct toolShapePoint
{
    float radiusPos;
    float heightPos;

    struct less_than
    {
        bool operator()(const toolShapePoint& a, const toolShapePoint& b)
        {
            return a.radiusPos < b.radiusPos;
        }
    };
};

struct Point3D
{
    Point3D()
        : x(0)
        , y(0)
        , z(0)
    {}
    Point3D(float x, float y, float z)
        : x(x)
        , y(y)
        , z(z)
    {}
    explicit Point3D(Base::Vector3d& vec)
        : x(vec[0])
        , y(vec[1])
        , z(vec[2])
    {}
    explicit Point3D(Base::Placement& pl)
        : x(pl.getPosition()[0])
        , y(pl.getPosition()[1])
        , z(pl.getPosition()[2])
    {}
    void UpdateCmd(Path::Command& cmd);
    float x, y, z;
};

class cSimTool
{
public:
    cSimTool(const TopoDS_Shape& toolShape, float res);
    ~cSimTool()
    {}

    float GetToolProfileAt(float pos);
    bool isInside(const TopoDS_Shape& toolShape, Base::Vector3d pnt, float res);

    /* m_toolShape has to be populated with linearly increased
       radiusPos to get the tool profile at given position */
    std::vector<toolShapePoint> m_toolShape;
    float radius;
    float length;
};

template<class T>
class Array2D
{
public:
    Array2D()
        : data(nullptr)
        , height(0)
    {}

    ~Array2D()
    {
        if (data) {
            delete[] data;
        }
    }

    void Init(int x, int y)
    {
        data = new T[x * y];
        height = y;
    }

    T* operator[](int i)
    {
        return data + i * height;
    }

    // Contiguous backing store, index = i * height + j. Exposed so the
    // viewport can wrap the height buffer once instead of copying it
    // cell by cell every tick.
    T* Data()
    {
        return data;
    }
    const T* Data() const
    {
        return data;
    }

private:
    T* data;
    int height;
};

// A run of material in one column of the grid, bottom to top.
struct cSpan
{
    float bottom;
    float top;
};

// What SeedFromStock found. `imbalanced` is the one that matters: a
// column whose crossing count went negative or did not return to zero
// means the source surface had a gap over it, and that column was
// filled min-to-max instead. Zero on any closed source.
struct cTransferReport
{
    size_t faces = 0;       // source surface rectangles that reached the grid
    size_t crossings = 0;   // ray-face crossings over the whole target grid
    size_t columns = 0;     // target columns left holding material
    size_t imbalanced = 0;  // columns that fell back to min/max
    double area = 0;        // the source surface's area, every face, in world units
    double surfaceMs = 0;   // both passes over the source surface
    double assignMs = 0;    // sorting crossings into spans
};

class cStock
{
public:
    cStock(float px, float py, float pz, float lx, float ly, float lz, float res);
    ~cStock();
    // The whole stock as a closed mesh, split into as-seeded (outer) and
    // cut (inner) faces. Every span of a column holding several is
    // emitted exactly; single-span columns are merged within m_res, the
    // tolerance they have always used.
    void Tessellate(Mesh::MeshObject& meshOuter, Mesh::MeshObject& meshInner);
    void SeedFromMesh(const Mesh::MeshObject& mesh);

    // Replaces this stock's contents with src's material, carried through
    // the rigid motion m: p_this = m * p_src, row-major 4x4, translation
    // in m[3], m[7], m[11] -- Base::Matrix4D's layout. For a 3+2 workplane
    // change, in place of Tessellate + transform + SeedFromMesh.
    //
    // Lossless in the way the min/max seed is not: every run of material
    // src holds reaches this grid as its own span, so a pocket that faces
    // sideways in the new frame stays a pocket instead of being filled.
    // See the definition for how.
    cTransferReport SeedFromStock(const cStock& src, const double m[16]);
    void ApplyLinearTool(Point3D& p1, Point3D& p2, cSimTool& tool);
    void ApplyCircularTool(Point3D& p1, Point3D& p2, Point3D& cent, cSimTool& tool, bool isCCW);

    // --- Direct height-buffer access for the viewport ---
    //
    // The viewport no longer needs Tessellate(). It draws a fixed grid
    // of cell tops and inter-cell walls, built once, whose Z is read per
    // vertex from this buffer in a vertex shader. Only the cells a tick
    // actually touched are re-uploaded, so per-tick cost is independent
    // of stock size and of how complex the cut surface has become.
    //
    // Tessellate() stays for producing a real Mesh on demand (export,
    // final result), just not on the per-frame path.
    int GridSizeX() const
    {
        return m_x;
    }
    int GridSizeY() const
    {
        return m_y;
    }
    float Resolution() const
    {
        return m_res;
    }
    float OriginX() const
    {
        return m_px;
    }
    float OriginY() const
    {
        return m_py;
    }
    // The stock's true size. The grid covers exactly this: its last
    // column and row are narrower than a cell wherever the size is not a
    // whole number of cells.
    float LengthX() const
    {
        return m_lx;
    }
    float LengthY() const
    {
        return m_ly;
    }
    float FloorZ() const
    {
        return m_pz;
    }
    float PlaneZ() const
    {
        return m_plane;
    }

    // Row-major over X: index = x * GridSizeY() + y, matching
    // Array2D::operator[]. Allocated once in the constructor and never
    // reallocated for the lifetime of the cStock, so a caller may wrap
    // it once and keep the pointer.
    const float* HeightData() const
    {
        return m_stock.Data();
    }
    // Per-cell height at seed time. A cell is a cut surface where its
    // height is below this -- the CutMaterial / CutMaterialIn split,
    // without needing two meshes.
    const float* OriginalData() const
    {
        return m_original.Data();
    }
    // Per-cell UNDERSIDE. See m_bottom.
    const float* BottomData() const
    {
        return m_bottom.Data();
    }

    // Material held, in cubic units of the stock's own scale, measured
    // BETWEEN the two surfaces rather than down to the floor. The one
    // figure that shows whether the underside is doing its job: a
    // re-seed onto a 45-degree workplane reports near zero change, and
    // reported +146.8% when the floor stood in for the bottom.
    double MaterialVolume() const;

    // --- Columns of several spans ---
    //
    // A column can hold more than one run of material: a rotated part
    // with a cavity in it, or an overhang with air beneath. The topmost
    // span -- the PRIMARY -- lives in the dense arrays (m_stock is its
    // top, m_bottom its bottom), so everything that reads only the top
    // surface is unchanged, and the common single-span column costs
    // nothing extra. Spans below it live in m_extra.
    //
    // Invariant: a column with spans in m_extra has a primary that holds
    // material (top > bottom). Cutting keeps it by promoting the next
    // span down when the tool passes through the primary.

    // Replaces column (x, y) with `spans`, given in any order. Overlapping
    // or touching spans merge; empty ones are dropped. Meant for seeding:
    // it also records the new top as the column's original.
    void SetColumn(int x, int y, std::vector<cSpan> spans);
    // The column's spans, topmost first. Empty if it holds nothing.
    void GetColumn(int x, int y, std::vector<cSpan>& out) const;
    // Spans held below a primary, over the whole grid.
    size_t ExtraSpanCount() const;

    // The spans BELOW the primary, as dense arrays, for the viewport --
    // which draws from flat buffers, not from the span map.
    //
    // Fills `tops` and `bottoms` for the inclusive cell rectangle, for
    // `layers` spans below each column's primary. Layout is layer-major
    // then row-major over x, matching the dense arrays:
    //     index = (k * width + (x - x0)) * height + (y - y0)
    // Layer 0 is the first span below the primary. A column with fewer
    // spans than that gets FloorZ() in both arrays for the layers it does
    // not have, which reads as no material to anything testing
    // top > bottom -- the same convention the height buffers use.
    //
    // Reads nothing and writes nothing else: safe to call per frame over
    // the dirty rectangle alone.
    void ReadSpanLayers(int x0, int y0, int x1, int y1, int layers, float* tops, float* bottoms) const;

    // Bytes of grid state per cell -- see the array declarations. Spans
    // below a primary come on top of this, 8 bytes each plus the map's
    // overhead per column that has any; ExtraSpanCount() says how many.
    static int BytesPerCell()
    {
        return 5 * (int)sizeof(float) + (int)sizeof(char);
    }

    // The cells the viewport has not yet re-uploaded, accumulated by
    // MarkDirty from every cut since the last call.
    bool ConsumeViewDirty(int& xMin, int& yMin, int& xMax, int& yMax);
    void MarkViewDirtyAll();

    // A no-op: Tessellate() is always a full pass, so there is nothing to
    // invalidate. Kept because the Python API exposes it, and a module
    // and a Python file one build apart must keep working.
    void InvalidateTessellation();
    // World XY to grid units: the stock's corner at 0, one unit per cell.
    // Cell i spans [i, i + 1), except the last, which ends at the stock's
    // edge (m_gx / m_gy).
    inline Point3D ToInner(Point3D& p)
    {
        return Point3D((p.x - m_px) / m_res, (p.y - m_py) / m_res, p.z);
    }

    // Width in grid units: 1, except the last cell.
    inline float CellWidthX(int x) const
    {
        return x < m_x - 1 ? 1.0f : m_gx - (float)(m_x - 1);
    }
    inline float CellWidthY(int y) const
    {
        return y < m_y - 1 ? 1.0f : m_gy - (float)(m_y - 1);
    }
    // Centre in grid units: the middle of the part of the cell that
    // exists. For a last cell narrower than half a cell, the nominal
    // centre x + 0.5 would lie outside the stock.
    inline float CellCentreX(int x) const
    {
        return (float)x + 0.5f * CellWidthX(x);
    }
    inline float CellCentreY(int y) const
    {
        return (float)y + 0.5f * CellWidthY(y);
    }

private:
    float FindRectTop(int& xp, int& yp, int& x_size, int& y_size, bool scanHoriz);
    // Returns the merged rectangle's bottom height. It used to return
    // nothing, because the bottom was always m_pz.
    float FindRectBot(int& xp, int& yp, int& x_size, int& y_size, bool scanHoriz);

    // The material interval of one column, [bot, top]. False when the
    // column holds nothing -- out of range, or thinner than one cell,
    // which is the same "no material" threshold the rest of this file
    // uses. The wall builders need to tell an empty column from a thin
    // one, and a degenerate interval at the floor cannot express that.
    bool ColumnSpan(int x, int y, float& bot, float& top) const;

    // One vertical quad on a wall line, spanning zA to zB. Winding
    // follows the sign of zA - zB, so the caller orders the two to put
    // the normal on the side the material is: for a wall between two
    // TOPS the material is on the higher one, for a wall between two
    // BOTTOMS it is on the lower, which is why those two calls pass
    // their arguments in opposite order.
    void AddWallQuadX(
        int x0,
        int x1,
        int yp,
        float zA,
        float zB,
        std::vector<MeshCore::MeshGeomFacet>& facets
    );
    void AddWallQuadY(
        int y0,
        int y1,
        int xp,
        float zA,
        float zB,
        std::vector<MeshCore::MeshGeomFacet>& facets
    );
    void SetFacetPoints(MeshCore::MeshGeomFacet& facet, Point3D& p1, Point3D& p2, Point3D& p3);
    void AddQuad(
        Point3D& p1,
        Point3D& p2,
        Point3D& p3,
        Point3D& p4,
        std::vector<MeshCore::MeshGeomFacet>& facets
    );
    int TesselTop(int x, int y);
    int TesselBot(int x, int y);
    // One wall line: between rows line-1 and line when alongX, between
    // columns line-1 and line otherwise.
    int TesselSides(bool alongX, int line);
    // The faces of every column holding more than one span, exactly.
    void TesselMultiSpan();
    // Whether column (x, y) holds spans below its primary.
    bool HasExtra(int x, int y) const;
    // The spans Tessellate emits for column (x, y), BOTTOM first: all of
    // them, exactly, for a column holding several; otherwise the primary
    // at its emitted face heights, or nothing if thinner than a cell
    // (ColumnSpan). The walls are set differences of these.
    void EmittedColumn(int x, int y, std::vector<cSpan>& out) const;

    // Unions a swept bbox into the viewport's dirty rectangle. Clipped to
    // the grid; a move entirely outside it marks nothing.
    void MarkDirty(int xMin, int yMin, int xMax, int yMax);

    // The viewport's dirty rectangle. See ConsumeViewDirty().
    bool m_viewHasDirty = false;
    int m_viewMinX = 0, m_viewMinY = 0, m_viewMaxX = 0, m_viewMaxY = 0;

    Array2D<float> m_stock;
    Array2D<float> m_original;  // per-cell height at seed time, never touched by cutting --
                                // used by TesselTop to tell "still original" from "cut" now
                                // that the seeded starting stock isn't necessarily flat.

    // Per-cell UNDERSIDE. Material exists BETWEEN this and m_stock.
    //
    // Without it, material runs from the floor up to the surface, which
    // is false for a tilted stock: a rotated block overhangs itself and
    // the space beneath is air. Seeding an uncut 152x77x27 box onto a
    // 45-degree workplane invented 147% extra material that way, at two
    // resolutions, so the error was geometric and not discretisation.
    //
    // Exact for a rotated convex stock; an approximation once cuts make
    // the solid non-convex, but far closer than a floor.
    //
    // Seeded with m_stock. A cut lowers the top, and a top below the
    // bottom is an empty cell -- MaterialVolume() clamps for exactly that.
    // The one time cutting writes here is when it goes through the whole
    // primary span of a column holding more: the next span down is
    // promoted, bottom and all (PromoteBelow).
    Array2D<float> m_bottom;

    // The height each cell's face was EMITTED at: written by TesselTop
    // and TesselBot as they mark a merged rectangle, read by the wall
    // builders through ColumnSpan.
    //
    // A merged rectangle is emitted at its seed cell's height, within
    // m_res of every cell it covers but equal to almost none. A wall
    // built from m_stock meets the face it is meant to close at the
    // wrong height. Recording what was emitted makes them agree by
    // construction and leaves the merge free to be as loose as it likes.
    //
    // Meaningful only where material exists: nothing writes them for
    // cells the sweeps skip, and ColumnSpan reports those empty from
    // m_stock and m_bottom before reading here.
    Array2D<float> m_topFace;
    Array2D<float> m_botFace;

    // Five float arrays and a char per cell: 21 bytes. See
    // cStock::BytesPerCell(), which the Accuracy label reports.
    Array2D<char> m_attr;
    float m_px, m_py, m_pz;  // stock zero position
    float m_lx, m_ly, m_lz;  // stock dimensions
    float m_gx, m_gy;        // stock dimensions in grid units: m_lx / m_res
    float m_res;             // resoulution
    float m_plane;           // stock plane height
    int m_x, m_y;            // stock array size
    std::vector<MeshCore::MeshGeomFacet> facetsOuter;
    std::vector<MeshCore::MeshGeomFacet> facetsInner;

    // Calls emit(axis, w, out, a0, a1, b0, b1) for every face of the
    // boundary of the material this grid holds: a rectangle on the plane
    // coordinate[axis] = w, outward normal along +axis when out > 0, in
    // world units. Built from the spans, merged only at exactly equal
    // height, so it is the grid's material to the bit. See SeedFromStock.
    template<typename Emit>
    void ForEachSurfaceFace(Emit&& emit) const;
    // World position of cell edge i / j: i * m_res from the corner, the
    // last clipped to the stock's edge.
    double EdgeX(int i) const;
    double EdgeY(int j) const;
    // Column (x, y)'s spans, BOTTOM first, into `out`; out-of-range
    // columns are empty. The ascending twin of GetColumn.
    void ColumnAscending(int x, int y, std::vector<cSpan>& out) const;
    // Writes a column from spans given bottom first, disjoint -- what
    // SeedFromStock produces. SetColumn without the sort, the merge, the
    // copy and the per-cell MarkDirty.
    void AssignAscending(int x, int y, const std::vector<cSpan>& asc);

    // Removes all material above z in column (x, y) -- a tool coming
    // down from above. Inline because every cut goes through it: with no
    // spans below the primary it is exactly "lower the top".
    inline void CutColumn(int x, int y, float z)
    {
        if (m_stock[x][y] > z) {
            m_stock[x][y] = z;
            // m_extra first: an empty map is one member read, while
            // m_bottom is a second array and costs a cache line per cut.
            if (!m_extra.empty() && z <= m_bottom[x][y]) {
                PromoteBelow(x, y, z);
            }
        }
    }
    // Cuts every cell whose centre the tool passes over, from a to b, in
    // grid units (z in world units). rad is the tool radius in grid units.
    void SweepSegment(const Point3D& a, const Point3D& b, cSimTool& tool, float rad);
    // The tool has gone through the primary span at (x, y): the next
    // span down that reaches above z becomes the primary, trimmed to z.
    void PromoteBelow(int x, int y, float z);
    inline long long ColumnKey(int x, int y) const
    {
        return (long long)x * m_y + y;
    }
    inline void ColumnFromKey(long long key, int& x, int& y) const
    {
        x = (int)(key / m_y);
        y = (int)(key % m_y);
    }
    // Spans below the primary, topmost first, keyed by ColumnKey. Sparse:
    // most columns hold one span even after several rotations.
    std::unordered_map<long long, std::vector<cSpan>> m_extra;
};
