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
 ***************************************************************************/

#include <algorithm>
#include <chrono>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <Base/Console.h>
#include <BRepBndLib.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include "VolSim.h"

using std::numbers::pi;

// Samples needed to cover [0, span] at `step` spacing, both ends included:
// the last sample, at (n - 1) * step, lands at or past the end. The
// obvious floor(span / step) + 1 -- which is also the count a
// "while (a < span)" loop runs -- stops up to a step short, and left a gap
// at the outer edge of every sweep that used it.
template<typename T>
static inline int SamplesToCover(T span, T step)
{
    return (int)std::ceil(span / step) + 1;
}

//************************************************************************************************************
// shape -> mesh tessellation
//************************************************************************************************************
void TessellateShapeToMesh(
    const TopoDS_Shape& shape,
    float linearDeflection,
    float angularDeflection,
    Mesh::MeshObject& outMesh
)
{
    // Mirrors the proven tessellation pattern already used for Path's
    // shape_tessellate_fast (surface_generator.cpp): mesh via
    // BRepMesh_IncrementalMesh, then walk each face's Poly_Triangulation.
    // No vertex deduplication is needed here, unlike that function --
    // MeshGeomFacet stores each triangle's 3 points directly rather than
    // by shared index, and Mesh::MeshObject::addFacets() builds its own
    // deduplicated internal topology from this "triangle soup" input, the
    // same way cStock::Tessellate() already produces its own output.
    BRepMesh_IncrementalMesh(shape, linearDeflection, false, angularDeflection, true);

    std::vector<MeshCore::MeshGeomFacet> facets;

    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
        const TopoDS_Face& face = TopoDS::Face(exp.Current());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull()) {
            continue;
        }

        gp_Trsf trsf = loc.Transformation();
        bool isReversed = (face.Orientation() == TopAbs_REVERSED);

        for (int i = 1; i <= tri->NbTriangles(); ++i) {
            int n1, n2, n3;
            tri->Triangle(i).Get(n1, n2, n3);
            gp_Pnt p1 = tri->Node(n1).Transformed(trsf);
            gp_Pnt p2 = tri->Node(n2).Transformed(trsf);
            gp_Pnt p3 = tri->Node(n3).Transformed(trsf);

            // Skip degenerate triangles (matches shape_tessellate_fast's own guard)
            const double tol = 1e-10;
            if (p1.Distance(p2) < tol || p1.Distance(p3) < tol || p2.Distance(p3) < tol) {
                continue;
            }

            // Maintain correct outward-facing winding
            gp_Pnt& q2 = isReversed ? p3 : p2;
            gp_Pnt& q3 = isReversed ? p2 : p3;

            MeshCore::MeshGeomFacet facet;
            facet._aclPoints[0][0] = (float)p1.X();
            facet._aclPoints[0][1] = (float)p1.Y();
            facet._aclPoints[0][2] = (float)p1.Z();
            facet._aclPoints[1][0] = (float)q2.X();
            facet._aclPoints[1][1] = (float)q2.Y();
            facet._aclPoints[1][2] = (float)q2.Z();
            facet._aclPoints[2][0] = (float)q3.X();
            facet._aclPoints[2][1] = (float)q3.Y();
            facet._aclPoints[2][2] = (float)q3.Z();
            facet.CalcNormal();

            facets.push_back(facet);
        }
    }

    outMesh.addFacets(facets);
}

//************************************************************************************************************
// stock
//************************************************************************************************************
cStock::cStock(float px, float py, float pz, float lx, float ly, float lz, float res)
    : m_px(px)
    , m_py(py)
    , m_pz(pz)
    , m_lx(lx)
    , m_ly(ly)
    , m_lz(lz)
    , m_res(res)
{
    // Exactly enough cells to cover the stock, the last one clipped to its
    // edge. (int)(l / res) + 1 always added a cell, so the grid overhung
    // the stock by up to a cell on +X and +Y and every volume carried the
    // overhang. The tolerance keeps float noise in l / res -- 50.00004 for
    // a true 50 -- from adding a sliver cell.
    m_gx = lx / res;
    m_gy = ly / res;
    m_x = std::max(1, (int)std::ceil(m_gx - 1e-4f));
    m_y = std::max(1, (int)std::ceil(m_gy - 1e-4f));
    m_stock.Init(m_x, m_y);
    m_original.Init(m_x, m_y);
    m_bottom.Init(m_x, m_y);
    m_topFace.Init(m_x, m_y);
    m_botFace.Init(m_x, m_y);
    m_attr.Init(m_x, m_y);
    m_plane = pz + lz;
    // x outer, y inner: Array2D puts (x,y) at data[x * m_y + y], so this
    // order walks contiguous memory. The reverse strides by m_y floats
    // per step -- 16 KB at a 4000-wide grid -- and measures ~33x slower.
    for (int x = 0; x < m_x; x++) {
        for (int y = 0; y < m_y; y++) {
            m_stock[x][y] = m_plane;
            m_original[x][y] = m_plane;
            // An unseeded stock is the full bounding box, so its
            // underside is the floor. SeedFromMesh overwrites this with
            // the real one.
            m_bottom[x][y] = m_pz;
            // Overwritten by the sweeps before any wall reads them; the
            // true surfaces are a sane starting value regardless.
            m_topFace[x][y] = m_plane;
            m_botFace[x][y] = m_pz;
            m_attr[x][y] = 0;
        }
    }
    // The viewport has never seen this stock, so everything is dirty.
    MarkViewDirtyAll();
}

cStock::~cStock()
{}

void cStock::MarkDirty(int xMin, int yMin, int xMax, int yMax)
{
    // Unions the bbox of whatever ApplyLinearTool/ApplyCircularTool just
    // swept into the viewport's dirty rectangle.
    xMin = std::max(0, xMin);
    yMin = std::max(0, yMin);
    xMax = std::min(m_x - 1, xMax);
    yMax = std::min(m_y - 1, yMax);
    if (xMin > xMax || yMin > yMax) {
        return;  // nothing valid to mark (e.g. move entirely outside the grid)
    }
    if (!m_viewHasDirty) {
        m_viewMinX = xMin;
        m_viewMinY = yMin;
        m_viewMaxX = xMax;
        m_viewMaxY = yMax;
        m_viewHasDirty = true;
    }
    else {
        m_viewMinX = std::min(m_viewMinX, xMin);
        m_viewMinY = std::min(m_viewMinY, yMin);
        m_viewMaxX = std::max(m_viewMaxX, xMax);
        m_viewMaxY = std::max(m_viewMaxY, yMax);
    }
}

bool cStock::ConsumeViewDirty(int& xMin, int& yMin, int& xMax, int& yMax)
{
    if (!m_viewHasDirty) {
        return false;
    }
    xMin = m_viewMinX;
    yMin = m_viewMinY;
    xMax = m_viewMaxX;
    yMax = m_viewMaxY;
    m_viewHasDirty = false;
    return true;
}

void cStock::InvalidateTessellation()
{
    // Nothing to do: Tessellate() is always a full pass. See the
    // declaration for why this still exists.
}

void cStock::MarkViewDirtyAll()
{
    m_viewMinX = 0;
    m_viewMinY = 0;
    m_viewMaxX = m_x - 1;
    m_viewMaxY = m_y - 1;
    m_viewHasDirty = true;
}

void cStock::SetColumn(int x, int y, std::vector<cSpan> spans)
{
    spans.erase(
        std::remove_if(spans.begin(), spans.end(), [](const cSpan& s) { return !(s.top > s.bottom); }),
        spans.end()
    );
    std::sort(spans.begin(), spans.end(), [](const cSpan& a, const cSpan& b) {
        return a.top > b.top;
    });
    std::vector<cSpan> merged;
    for (const cSpan& s : spans) {
        if (!merged.empty() && s.top >= merged.back().bottom) {
            merged.back().bottom = std::min(merged.back().bottom, s.bottom);
        }
        else {
            merged.push_back(s);
        }
    }

    const long long key = ColumnKey(x, y);
    if (merged.empty()) {
        // Empty, the way SeedFromMesh leaves a column no triangle covers.
        m_stock[x][y] = m_pz;
        m_bottom[x][y] = m_plane;
        m_extra.erase(key);
    }
    else {
        m_stock[x][y] = merged[0].top;
        m_bottom[x][y] = merged[0].bottom;
        if (merged.size() > 1) {
            m_extra[key].assign(merged.begin() + 1, merged.end());
        }
        else {
            m_extra.erase(key);
        }
    }
    m_original[x][y] = m_stock[x][y];
    MarkDirty(x, y, x, y);
}

void cStock::GetColumn(int x, int y, std::vector<cSpan>& out) const
{
    // Data() rather than operator[]: this is const, Array2D's subscript
    // is not.
    out.clear();
    const long long i = (long long)x * m_y + y;
    if (m_stock.Data()[i] > m_bottom.Data()[i]) {
        out.push_back({m_bottom.Data()[i], m_stock.Data()[i]});
    }
    auto it = m_extra.find(ColumnKey(x, y));
    if (it != m_extra.end()) {
        out.insert(out.end(), it->second.begin(), it->second.end());
    }
}

void cStock::ReadSpanLayers(int x0, int y0, int x1, int y1, int layers, float* tops, float* bottoms) const
{
    if (layers <= 0 || !tops || !bottoms) {
        return;
    }
    x0 = std::max(0, x0);
    y0 = std::max(0, y0);
    x1 = std::min(x1, m_x - 1);
    y1 = std::min(y1, m_y - 1);
    if (x1 < x0 || y1 < y0) {
        return;
    }
    const long long w = (long long)x1 - x0 + 1;
    const long long h = (long long)y1 - y0 + 1;

    // Floor everywhere first, so every cell the loop does not reach --
    // and every layer a column does not have -- already reads as empty.
    std::fill(tops, tops + (long long)layers * w * h, FloorZ());
    std::fill(bottoms, bottoms + (long long)layers * w * h, FloorZ());

    // Walk the span map, not the rectangle: columns with spans below a
    // primary are the exception, so this is proportional to how many
    // there are rather than to the area asked for.
    for (const auto& entry : m_extra) {
        int x = 0, y = 0;
        ColumnFromKey(entry.first, x, y);
        if (x < x0 || x > x1 || y < y0 || y > y1) {
            continue;
        }
        const long long cell = (x - x0) * h + (y - y0);
        const int n = std::min((int)entry.second.size(), layers);
        for (int k = 0; k < n; k++) {
            const long long i = (long long)k * w * h + cell;
            tops[i] = entry.second[k].top;
            bottoms[i] = entry.second[k].bottom;
        }
    }
}

size_t cStock::ExtraSpanCount() const
{
    size_t n = 0;
    for (const auto& entry : m_extra) {
        n += entry.second.size();
    }
    return n;
}

void cStock::PromoteBelow(int x, int y, float z)
{
    auto it = m_extra.find(ColumnKey(x, y));
    if (it == m_extra.end()) {
        return;  // nothing below: the column is simply empty, as it always was
    }
    std::vector<cSpan>& below = it->second;
    size_t i = 0;
    while (i < below.size() && z <= below[i].bottom) {
        ++i;  // the tool went clean through this one too
    }
    if (i < below.size()) {
        m_bottom[x][y] = below[i].bottom;
        m_stock[x][y] = std::min(below[i].top, z);
        below.erase(below.begin(), below.begin() + i + 1);
    }
    else {
        below.clear();  // all of it gone; the primary stays empty (top < bottom)
    }
    if (below.empty()) {
        m_extra.erase(it);
    }
}

double cStock::MaterialVolume() const
{
    // Clamped at zero per cell: cutting lowers the top and never raises
    // the bottom, so a cell milled through has a top below its bottom.
    // That is an empty cell, not negative material.
    // Data() rather than operator[]: Array2D's subscript is non-const
    // and this method is const. Flat walk, same order as the nested
    // loops elsewhere.
    const float* top = m_stock.Data();
    const float* bot = m_bottom.Data();
    const long long count = (long long)m_x * m_y;

    // Every cell is a whole cell except the last column and row, so sum
    // the interior at unit area and weight only those.
    double total = 0.0;
    for (long long i = 0; i < count; i++) {
        const int x = (int)(i / m_y);
        const int y = (int)(i % m_y);
        double h = (double)top[i] - (double)bot[i];
        if (h > 0.0) {
            total += h * (double)CellWidthX(x) * (double)CellWidthY(y);
        }
    }
    for (const auto& entry : m_extra) {
        const int x = (int)(entry.first / m_y);
        const int y = (int)(entry.first % m_y);
        const double area = (double)CellWidthX(x) * (double)CellWidthY(y);
        for (const cSpan& span : entry.second) {
            total += ((double)span.top - (double)span.bottom) * area;
        }
    }
    return total * (double)m_res * (double)m_res;
}

void cStock::SeedFromMesh(const Mesh::MeshObject& mesh)
{
    // Rasterises the mesh into the grid the way a Z-buffer does: at every
    // cell centre a triangle's XY footprint covers, interpolate its Z. The
    // highest Z over a column is the column's top and the lowest its
    // bottom, and material fills the interval between -- exact for a
    // convex solid, an over-estimate otherwise, since a cavity lying
    // between the two surfaces reads as solid.
    //
    // Seeds from the stock's real shape (BeginSimulation), and re-seeds
    // from the cut stock's own mesh, rotated into a new working frame, on
    // a 3+2 workplane change (ReseedFromMesh).
    //
    // Each surface starts at the value it can only move away from: the
    // top at the floor, the bottom at the plane. Started at the plane, the
    // top would stay there, since no triangle can rise above its own
    // bounding box. A column no triangle covers keeps top < bottom and so
    // holds nothing -- the corners around a cylindrical stock, say.
    m_extra.clear();                 // this rasterisation gives every column one span
    for (int x = 0; x < m_x; x++) {  // x outer: see the constructor
        for (int y = 0; y < m_y; y++) {
            m_stock[x][y] = m_pz;
            m_bottom[x][y] = m_plane;
        }
    }

    const MeshCore::MeshKernel& kernel = mesh.getKernel();
    const MeshCore::MeshPointArray& points = kernel.GetPoints();
    const MeshCore::MeshFacetArray& facets = kernel.GetFacets();

    unsigned long numFacets = kernel.CountFacets();

    for (unsigned long i = 0; i < numFacets; i++) {
        const MeshCore::MeshFacet& facet = facets[i];
        const MeshCore::MeshPoint& w0 = points[facet._aulPoints[0]];
        const MeshCore::MeshPoint& w1 = points[facet._aulPoints[1]];
        const MeshCore::MeshPoint& w2 = points[facet._aulPoints[2]];

        // XY converted to grid-index space (same convention as ToInner());
        // Z stays in world units, since m_stock stores world-space heights.
        float gx0 = (w0.x - m_px) / m_res, gy0 = (w0.y - m_py) / m_res;
        float gx1 = (w1.x - m_px) / m_res, gy1 = (w1.y - m_py) / m_res;
        float gx2 = (w2.x - m_px) / m_res, gy2 = (w2.y - m_py) / m_res;

        int xMin = std::max(0, (int)std::floor(std::min({gx0, gx1, gx2})));
        int xMax = std::min(m_x - 1, (int)std::ceil(std::max({gx0, gx1, gx2})));
        int yMin = std::max(0, (int)std::floor(std::min({gy0, gy1, gy2})));
        int yMax = std::min(m_y - 1, (int)std::ceil(std::max({gy0, gy1, gy2})));

        if (xMin > xMax || yMin > yMax) {
            continue;  // triangle entirely outside the stock's footprint
        }

        float denom = (gy1 - gy2) * (gx0 - gx2) + (gx2 - gx1) * (gy0 - gy2);
        if (std::fabs(denom) < 1e-9f) {
            continue;  // degenerate (zero XY area) triangle -- e.g. a vertical wall
        }

        for (int y = yMin; y <= yMax; y++) {
            for (int x = xMin; x <= xMax; x++) {
                float px = CellCentreX(x);
                float py = CellCentreY(y);

                float w0b = ((gy1 - gy2) * (px - gx2) + (gx2 - gx1) * (py - gy2)) / denom;
                float w1b = ((gy2 - gy0) * (px - gx2) + (gx0 - gx2) * (py - gy2)) / denom;
                float w2b = 1.0f - w0b - w1b;

                const float eps = -1e-4f;
                if (w0b < eps || w1b < eps || w2b < eps) {
                    continue;  // cell center falls outside this triangle
                }

                float z = w0b * w0.z + w1b * w1.z + w2b * w2.z;
                if (z > m_stock[x][y]) {
                    m_stock[x][y] = z;
                }
                // Same rasterization, same interpolated Z, opposite
                // comparison. The lowest surface any triangle puts over
                // this column is the underside of the material.
                if (z < m_bottom[x][y]) {
                    m_bottom[x][y] = z;
                }
            }
        }
    }

    // Record this seeded state as "original" for every cell, so TesselTop
    // can later tell "still the starting geometry" from "actually cut"
    // without assuming the starting stock is flat (see the m_original
    // member declaration in VolSim.h for why this exists).
    for (int x = 0; x < m_x; x++) {  // x outer: see the constructor
        for (int y = 0; y < m_y; y++) {
            m_original[x][y] = m_stock[x][y];
        }
    }

    // Every cell has just been rewritten, so the viewport's incremental
    // path cannot be trusted here -- force a full re-upload.
    MarkViewDirtyAll();
}


//************************************************************************************************************
// crossing transfer: stock -> rotated stock without the min/max fill
//************************************************************************************************************
//
// The min/max seed gives each column one interval, from the lowest
// triangle over it to the highest, so every cavity the new frame cannot
// see into is filled solid: +20% on a pocketed block turned 90 degrees.
// This carries the material over run by run instead:
//
//   1. The source's boundary is generated from its grid -- tops and
//      bottoms of every span, and walls wherever a neighbour's spans do
//      not cover this column's (a set difference). Not from Tessellate():
//      that merges within m_res and emits at one cell's height, which
//      loses material. Here faces merge only at EXACTLY equal height, so the
//      surface is the grid's material, to the bit.
//   2. Each face is carried into this frame and rasterised onto this
//      grid's cell centres. Every centre it covers gets a crossing: the
//      height there, and +1 or -1 by whether the face looks down (the
//      ray enters material going up) or up (it leaves).
//   3. Per column, the crossings are sorted and the running count turned
//      into spans: material wherever it is above zero.
//
// Cost follows the source's surface area, not its volume.
//
// The count only balances if every point of the projected surface is
// claimed by exactly one of the faces over it. Every face is a rectangle
// on a grid plane, and every edge of one lies on a grid LINE: a height
// on a cell-edge plane, or a cell corner. So an edge is tested against
// its whole line -- a number that depends on the line alone, not on
// where the edge starts or stops (see "Lines" in SeedFromStock). Every
// face touching that line computes the same number, bit for bit, and a
// centre is on the same side of it for all of them. That makes
// T-junctions harmless -- a wall one cell long beside a top face ten
// cells long test the same line identically -- which is what lets faces
// merge at all. Testing against endpoints instead would need every edge
// split wherever another meets it; the first version did that, cell by
// cell in triangles, and measured 5 to 15 times slower.
//
// A centre exactly on a line is decided as if nudged by (d, d^2),
// d -> 0+. Every face uses the same nudge, so the tie goes to exactly one
// side.
//
// A column that still does not balance is reported and filled
// min-to-max, the old behaviour, rather than guessed at. None has, on
// any source tried (engine_tests/transfer.inc); the report says if one
// ever does.

namespace
{
struct cCrossing
{
    float z;
    float dir;  // +1 entering material going up, -1 leaving
};

// a \ b for span lists given bottom first and disjoint, into out.
void SpanDifference(const std::vector<cSpan>& a, const std::vector<cSpan>& b, std::vector<cSpan>& out)
{
    out.clear();
    for (const cSpan& s : a) {
        float lo = s.bottom;
        for (const cSpan& t : b) {
            if (t.top <= lo) {
                continue;
            }
            if (t.bottom >= s.top) {
                break;
            }
            if (t.bottom > lo) {
                out.push_back({lo, t.bottom});
            }
            lo = std::max(lo, t.top);
            if (lo >= s.top) {
                break;
            }
        }
        if (lo < s.top) {
            out.push_back({lo, s.top});
        }
    }
}

// A run of wall being extended along its line: material between lo and
// hi on the side `out` faces away from, starting at cell `from`.
struct cWallRun
{
    float lo, hi;
    float out;
    int from;
};
}  // namespace

void cStock::ColumnAscending(int x, int y, std::vector<cSpan>& out) const
{
    out.clear();
    if (x < 0 || x >= m_x || y < 0 || y >= m_y) {
        return;
    }
    const long long i = (long long)x * m_y + y;
    if (!m_extra.empty()) {
        auto it = m_extra.find(i);
        if (it != m_extra.end()) {
            out.assign(it->second.rbegin(), it->second.rend());
        }
    }
    if (m_stock.Data()[i] > m_bottom.Data()[i]) {
        out.push_back({m_bottom.Data()[i], m_stock.Data()[i]});
    }
}

double cStock::EdgeX(int i) const
{
    // i * m_res from the corner, except the far edge, which is the
    // stock's own (the last cell is clipped) -- the rule SetFacetPoints
    // uses. One formula for every use, so a shared edge is the same
    // double everywhere.
    return (double)m_px + (i >= m_x ? (double)m_lx : (double)i * (double)m_res);
}

double cStock::EdgeY(int j) const
{
    return (double)m_py + (j >= m_y ? (double)m_ly : (double)j * (double)m_res);
}

template<typename Emit>
void cStock::ForEachSurfaceFace(Emit&& emit) const
{
    // emit(axis, w, out, a0, a1, b0, b1): a rectangle on the plane
    // coordinate[axis] = w, its outward normal along +axis when out > 0.
    // (a, b) are the other two coordinates in order -- (x, y) for a
    // horizontal face, (y, z) on an X plane, (x, z) on a Y plane -- with
    // a0 < a1 and b0 < b1.

    // --- Tops and bottoms. Primary spans run along Y while the height is
    // exactly equal; spans below a primary are one cell each, being rare.
    for (int x = 0; x < m_x; x++) {
        const long long row = (long long)x * m_y;
        const float* top = m_stock.Data() + row;
        const float* bot = m_bottom.Data() + row;
        for (int face = 0; face < 2; face++) {
            const float* h = face ? bot : top;
            int y = 0;
            while (y < m_y) {
                if (!(top[y] > bot[y])) {
                    y++;
                    continue;
                }
                const float z = h[y];
                int y1 = y + 1;
                while (y1 < m_y && top[y1] > bot[y1] && h[y1] == z) {
                    y1++;
                }
                emit(2, (double)z, face ? -1.0 : 1.0, EdgeX(x), EdgeX(x + 1), EdgeY(y), EdgeY(y1));
                y = y1;
            }
        }
    }
    for (const auto& entry : m_extra) {
        const int x = (int)(entry.first / m_y);
        const int y = (int)(entry.first % m_y);
        for (const cSpan& s : entry.second) {
            emit(2, (double)s.top, 1.0, EdgeX(x), EdgeX(x + 1), EdgeY(y), EdgeY(y + 1));
            emit(2, (double)s.bottom, -1.0, EdgeX(x), EdgeX(x + 1), EdgeY(y), EdgeY(y + 1));
        }
    }

    // --- Walls, one line at a time: the plane x = EdgeX(i) between
    // columns (i - 1, j) and (i, j), swept along j, then the planes
    // y = EdgeY(j). A piece of wall is exposed wherever one side holds
    // material the other does not, and runs along the line while an
    // identical piece continues.
    std::vector<cSpan> colA, colB, diff;
    std::vector<cWallRun> open, next;

    auto sweep = [&](int axis, int line, int count) {
        const double w = axis == 0 ? EdgeX(line) : EdgeY(line);
        auto edge = [&](int k) {
            return axis == 0 ? EdgeY(k) : EdgeX(k);
        };
        open.clear();
        for (int k = 0; k <= count; k++) {
            next.clear();
            if (k < count) {
                const int bx = axis == 0 ? line : k;
                const int by = axis == 0 ? k : line;
                const int ax = axis == 0 ? bx - 1 : bx;
                const int ay = axis == 0 ? by : by - 1;
                // Fast path, most of the grid: no spans below any
                // primary, and both columns hold the same one or both
                // nothing -- no wall here and none open to close.
                if (m_extra.empty() && open.empty()) {
                    const bool inA = ax >= 0 && ay >= 0;
                    const bool inB = bx < m_x && by < m_y;
                    const long long ia = inA ? (long long)ax * m_y + ay : 0;
                    const long long ib = inB ? (long long)bx * m_y + by : 0;
                    const bool hasA = inA && m_stock.Data()[ia] > m_bottom.Data()[ia];
                    const bool hasB = inB && m_stock.Data()[ib] > m_bottom.Data()[ib];
                    if (!hasA && !hasB) {
                        continue;
                    }
                    if (hasA && hasB && m_stock.Data()[ia] == m_stock.Data()[ib]
                        && m_bottom.Data()[ia] == m_bottom.Data()[ib]) {
                        continue;
                    }
                }
                ColumnAscending(ax, ay, colA);
                ColumnAscending(bx, by, colB);
                // A is on the low side of the plane, so its wall faces
                // +axis; B's faces -axis.
                for (int side = 0; side < 2; side++) {
                    SpanDifference(side ? colB : colA, side ? colA : colB, diff);
                    const float out = side ? -1.0f : 1.0f;
                    for (const cSpan& p : diff) {
                        int from = k;
                        for (const cWallRun& r : open) {
                            if (r.lo == p.bottom && r.hi == p.top && r.out == out) {
                                from = r.from;
                                break;
                            }
                        }
                        next.push_back({p.bottom, p.top, out, from});
                    }
                }
            }
            // Whatever did not continue ends at this cell's edge.
            for (const cWallRun& r : open) {
                bool continues = false;
                for (const cWallRun& n : next) {
                    if (n.from == r.from && n.lo == r.lo && n.hi == r.hi && n.out == r.out) {
                        continues = true;
                        break;
                    }
                }
                if (!continues) {
                    emit(axis, w, (double)r.out, edge(r.from), edge(k), (double)r.lo, (double)r.hi);
                }
            }
            open.swap(next);
        }
    };
    for (int i = 0; i <= m_x; i++) {
        sweep(0, i, m_y);
    }
    for (int j = 0; j <= m_y; j++) {
        sweep(1, j, m_x);
    }
}

void cStock::AssignAscending(int x, int y, const std::vector<cSpan>& asc)
{
    const long long key = ColumnKey(x, y);
    if (asc.empty()) {
        // Empty, the way SeedFromMesh leaves a column no triangle covers.
        m_stock[x][y] = m_pz;
        m_bottom[x][y] = m_plane;
    }
    else {
        m_stock[x][y] = asc.back().top;
        m_bottom[x][y] = asc.back().bottom;
        if (asc.size() > 1) {
            // m_extra is topmost first.
            m_extra[key].assign(asc.rbegin() + 1, asc.rend());
        }
    }
    m_original[x][y] = m_stock[x][y];
}

cTransferReport cStock::SeedFromStock(const cStock& src, const double m[16])
{
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    cTransferReport report;

    const long long ncols = (long long)m_x * m_y;
    const double inv = 1.0 / (double)m_res;

    // --- Lines.
    //
    // Every edge lies on a line running along one source axis k. Seen
    // from above in this frame, all lines along k are parallel, with
    // direction (m[k], m[4 + k]). So which side of such a line a centre P
    // is on comes down to comparing one number that depends only on P,
    //     K_k(P) = m[k] * P.v - m[4 + k] * P.u        (grid units)
    // with the same number anywhere on the line, c_k. That is linear in
    // the line's two fixed source coordinates and blind to the third,
    //     c_k = A[k][j] q_j + A[k][l] q_l + B[k],     {j, l} = not k,
    // so a line's c is one fixed expression in the coordinates that name
    // it: the same double for every face that touches it, however long
    // that face's edge and wherever it starts. A centre's K is likewise
    // the same for every face over it. Consistent verdicts follow.
    double A[3][3], B[3];
    for (int k = 0; k < 3; k++) {
        for (int j = 0; j < 3; j++) {
            A[k][j] = (m[k] * m[4 + j] - m[4 + k] * m[j]) * inv;
        }
        B[k] = (m[k] * (m[7] - (double)m_py) - m[4 + k] * (m[3] - (double)m_px)) * inv;
    }
    // c for the line along source axis k through point p.
    auto lineC = [&](int k, const double* p) {
        const int j = k == 0 ? 1 : 0;
        const int l = k == 2 ? 1 : 2;
        return A[k][j] * p[j] + A[k][l] * p[l] + B[k];
    };
    // A centre exactly on a line: decided as if moved by (d, d^2), d ->
    // 0+, which moves K_k by -m[4 + k] d + m[k] d^2. The same for every
    // line along k, so every face sharing the line agrees.
    bool tieAbove[3];
    double invKv[3];  // 1 / (dK_k / dv), for narrowing a scan row; 0 if K_k ignores v
    for (int k = 0; k < 3; k++) {
        tieAbove[k] = m[4 + k] != 0.0 ? m[4 + k] < 0.0 : m[k] > 0.0;
        invKv[k] = m[k] != 0.0 ? 1.0 / m[k] : 0.0;
    }

    size_t faces = 0;
    auto raster =
        [&](int axis, double w, double out, double a0, double a1, double b0, double b1, auto&& hit) {
            // The face's normal seen in this frame: m's row 2 against the
            // source axis. Zero means the face stands vertical here and no
            // ray crosses it.
            const double nz = m[8 + axis] * out;
            if (nz == 0.0) {
                return;
            }
            const float dir = nz > 0.0 ? -1.0f : 1.0f;  // looks up: the ray leaves

            // In the face's plane `a` measures source axis da and `b` axis db.
            const int da = axis == 0 ? 1 : 0;
            const int db = axis == 2 ? 1 : 2;

            // Its box first, from one corner and the two edge vectors: most
            // faces of a cut surface are a cell or less and cover no centre.
            // Only the box comes from here; the lines decide coverage.
            double p[3];
            p[axis] = w;
            p[da] = a0;
            p[db] = b0;
            const double u0 = (m[0] * p[0] + m[1] * p[1] + m[2] * p[2] + m[3] - (double)m_px) * inv;
            const double v0 = (m[4] * p[0] + m[5] * p[1] + m[6] * p[2] + m[7] - (double)m_py) * inv;
            const double z0 = m[8] * p[0] + m[9] * p[1] + m[10] * p[2] + m[11];
            const double sa = a1 - a0, sb = b1 - b0;
            const double eau = m[da] * sa * inv, eav = m[4 + da] * sa * inv, eaz = m[8 + da] * sa;
            const double ebu = m[db] * sb * inv, ebv = m[4 + db] * sb * inv, ebz = m[8 + db] * sb;
            // Widened a hair so that rounding here cannot clip a centre the
            // lines accept.
            const double pad = 1e-9;
            const double umin = u0 + std::min(0.0, eau) + std::min(0.0, ebu) - pad;
            const double umax = u0 + std::max(0.0, eau) + std::max(0.0, ebu) + pad;
            const double vmin = v0 + std::min(0.0, eav) + std::min(0.0, ebv) - pad;
            const double vmax = v0 + std::max(0.0, eav) + std::max(0.0, ebv) + pad;
            const int xa = std::max(0, (int)std::floor(umin));
            const int xb = std::min(m_x - 1, (int)std::floor(umax));
            const int ya = std::max(0, (int)std::floor(vmin));
            const int yb = std::min(m_y - 1, (int)std::floor(vmax));
            if (xa > xb || ya > yb) {
                return;
            }
            // No centre can be inside a box that holds none, even if the
            // integer ranges above are not empty.
            if (CellCentreX(xa) > umax && xa == xb) {
                return;
            }
            if (CellCentreY(ya) > vmax && ya == yb) {
                return;
            }
            const double zlo = z0 + std::min(0.0, eaz) + std::min(0.0, ebz);
            const double zhi = z0 + std::max(0.0, eaz) + std::max(0.0, ebz);

            // The edges a = a0, a1 run along db; b = b0, b1 run along da. A
            // centre is on the face when it lies between each parallel pair.
            p[db] = 0.0;  // ignored by lineC along db
            const double ca0 = lineC(db, p);
            p[da] = a1;
            const double ca1 = lineC(db, p);
            p[da] = 0.0;  // ignored along da
            p[db] = b0;
            const double cb0 = lineC(da, p);
            p[db] = b1;
            const double cb1 = lineC(da, p);
            const bool tieA = tieAbove[db], tieB = tieAbove[da];
            const double kau = -m[4 + db], kav = m[db];  // K along db
            const double kbu = -m[4 + da], kbv = m[da];  // K along da
            faces++;

            // Height where the ray over (X, Y) meets the plane: the source
            // coordinate[axis] of this frame's point (X, Y, Z) is
            // m[axis] X + m[4 + axis] Y + m[8 + axis] Z - ta, m being a
            // rotation, so its inverse its transpose.
            const double ta = m[axis] * m[3] + m[4 + axis] * m[7] + m[8 + axis] * m[11];
            const double zden = m[8 + axis];

            auto above = [](double K, double c, bool tie) {
                return K > c || (K == c && tie);
            };
            // Down one column of centres K_a and K_b are linear in v, so the
            // stretch of v between each pair of lines is known outright. Scan
            // only that, a cell wider each way, and let the exact tests below
            // decide: a long thin wall at an angle covers a sliver of its box.
            auto narrow =
                [](double ku, double ikv, double c0, double c1, double pu, double& lo, double& hi) {
                    if (ikv == 0.0) {
                        return;  // this pair does not vary along v
                    }
                    double v0 = (c0 - ku * pu) * ikv;
                    double v1 = (c1 - ku * pu) * ikv;
                    if (v0 > v1) {
                        std::swap(v0, v1);
                    }
                    lo = std::max(lo, v0);
                    hi = std::min(hi, v1);
                };
            for (int x = xa; x <= xb; x++) {
                const double pu = CellCentreX(x);
                double vlo = -1e300, vhi = 1e300;
                narrow(kau, invKv[db], ca0, ca1, pu, vlo, vhi);
                narrow(kbu, invKv[da], cb0, cb1, pu, vlo, vhi);
                if (vlo > vhi + 2.0) {
                    continue;
                }
                const int y0 = std::max(ya, vlo > -1e299 ? (int)std::floor(vlo) - 1 : ya);
                const int y1 = std::min(yb, vhi < 1e299 ? (int)std::floor(vhi) + 1 : yb);
                for (int y = y0; y <= y1; y++) {
                    const double pv = CellCentreY(y);
                    const double Ka = kau * pu + kav * pv;
                    if (above(Ka, ca0, tieA) == above(Ka, ca1, tieA)) {
                        continue;
                    }
                    const double Kb = kbu * pu + kbv * pv;
                    if (above(Kb, cb0, tieB) == above(Kb, cb1, tieB)) {
                        continue;
                    }
                    const double X = (double)m_px + pu * (double)m_res;
                    const double Y = (double)m_py + pv * (double)m_res;
                    double Z = (w + ta - m[axis] * X - m[4 + axis] * Y) / zden;
                    Z = std::min(zhi, std::max(zlo, Z));
                    hit((long long)x * m_y + y, (float)Z, dir);
                }
            }
        };

    // Two passes over the source surface, counting then filling, so the
    // crossings sit in one flat array bucketed by column. Generating the
    // surface twice is cheaper than holding it at a fine grid.
    std::vector<uint32_t> first(ncols + 1, 0);
    src.ForEachSurfaceFace(
        [&](int axis, double w, double out, double a0, double a1, double b0, double b1) {
            // Every face, whether or not it reaches this grid: the caller
            // bounds what resampling may change by this area times a cell.
            report.area += (a1 - a0) * (b1 - b0);
            raster(axis, w, out, a0, a1, b0, b1, [&](long long col, float, float) {
                first[col + 1]++;
            });
        }
    );
    report.faces = faces;
    for (long long i = 0; i < ncols; i++) {
        first[i + 1] += first[i];
    }
    std::vector<cCrossing> xs(first[ncols]);
    {
        std::vector<uint32_t> cursor(first.begin(), first.end() - 1);
        src.ForEachSurfaceFace(
            [&](int axis, double w, double out, double a0, double a1, double b0, double b1) {
                raster(axis, w, out, a0, a1, b0, b1, [&](long long col, float zc, float dir) {
                    xs[cursor[col]++] = {zc, dir};
                });
            }
        );
    }
    report.crossings = xs.size();
    const auto t1 = Clock::now();

    // --- Per column: sort, count, span.
    m_extra.clear();
    std::vector<cSpan> asc;
    for (int x = 0; x < m_x; x++) {  // x outer: see the constructor
        for (int y = 0; y < m_y; y++) {
            const long long col = (long long)x * m_y + y;
            cCrossing* beg = xs.data() + first[col];
            cCrossing* end = xs.data() + first[col + 1];
            asc.clear();
            if (beg != end) {
                // Entries before exits at equal height: two runs that
                // touch then merge, and a coincident exit can never take
                // the count below zero ahead of its entry.
                std::sort(beg, end, [](const cCrossing& p, const cCrossing& q) {
                    return p.z < q.z || (p.z == q.z && p.dir > q.dir);
                });
                int count = 0;
                bool balanced = true;
                float start = 0.0f;
                for (cCrossing* it = beg; it != end; ++it) {
                    const int next = count + (it->dir > 0 ? 1 : -1);
                    if (next < 0) {
                        balanced = false;
                        break;
                    }
                    if (count == 0 && next > 0) {
                        start = it->z;
                    }
                    else if (count > 0 && next == 0 && it->z > start) {
                        // A gap of a few float steps is not air: the two
                        // faces met at one height that rounded apart, or
                        // the gap is narrower than a float can hold here.
                        if (!asc.empty()
                            && start - asc.back().top
                                <= 4.0f * FLT_EPSILON * std::max(1.0f, std::fabs(start))) {
                            asc.back().top = it->z;  // touches the run below
                        }
                        else {
                            asc.push_back({start, it->z});
                        }
                    }
                    count = next;
                }
                if (!balanced || count != 0) {
                    // A gap in the source surface over this column. Fill
                    // it the old way, lowest crossing to highest, and
                    // say so through the report.
                    report.imbalanced++;
                    asc.clear();
                    asc.push_back({beg->z, (end - 1)->z});
                }
            }
            AssignAscending(x, y, asc);
            if (!asc.empty()) {
                report.columns++;
            }
        }
    }
    MarkViewDirtyAll();
    const auto t2 = Clock::now();
    report.surfaceMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    report.assignMs = std::chrono::duration<double, std::milli>(t2 - t1).count();
    return report;
}

float cStock::FindRectTop(int& xp, int& yp, int& x_size, int& y_size, bool scanHoriz)
{
    float z = m_stock[xp][yp];
    bool xr_ok = true;
    bool xl_ok = scanHoriz;
    bool yu_ok = true;
    bool yd_ok = !scanHoriz;
    x_size = 1;
    y_size = 1;
    while (xr_ok || xl_ok || yu_ok || yd_ok) {
        // sweep right x direction
        if (xr_ok) {
            int tx = xp + x_size;
            if (tx >= m_x) {
                xr_ok = false;
            }
            else {
                for (int y = yp; y < yp + y_size; y++) {
                    if ((m_attr[tx][y] & SIM_TESSEL_TOP) != 0
                        || (m_stock[tx][y] - m_bottom[tx][y]) < m_res
                        || fabs(z - m_stock[tx][y]) > m_res) {
                        xr_ok = false;
                        break;
                    }
                }
                if (xr_ok) {
                    x_size++;
                }
            }
        }

        // sweep left x direction
        if (xl_ok) {
            int tx = xp - 1;
            if (tx < 0) {
                xl_ok = false;
            }
            else {
                for (int y = yp; y < yp + y_size; y++) {
                    if ((m_attr[tx][y] & SIM_TESSEL_TOP) != 0
                        || (m_stock[tx][y] - m_bottom[tx][y]) < m_res
                        || fabs(z - m_stock[tx][y]) > m_res) {
                        xl_ok = false;
                        break;
                    }
                }
                if (xl_ok) {
                    x_size++;
                    xp--;
                }
            }
        }

        // sweep up y direction
        if (yu_ok) {
            int ty = yp + y_size;
            if (ty >= m_y) {
                yu_ok = false;
            }
            else {
                for (int x = xp; x < xp + x_size; x++) {
                    if ((m_attr[x][ty] & SIM_TESSEL_TOP) != 0
                        || (m_stock[x][ty] - m_bottom[x][ty]) < m_res
                        || fabs(z - m_stock[x][ty]) > m_res) {
                        yu_ok = false;
                        break;
                    }
                }
                if (yu_ok) {
                    y_size++;
                }
            }
        }

        // sweep down y direction
        if (yd_ok) {
            int ty = yp - 1;
            if (ty < 0) {
                yd_ok = false;
            }
            else {
                for (int x = xp; x < xp + x_size; x++) {
                    if ((m_attr[x][ty] & SIM_TESSEL_TOP) != 0
                        || (m_stock[x][ty] - m_bottom[x][ty]) < m_res
                        || fabs(z - m_stock[x][ty]) > m_res) {
                        yd_ok = false;
                        break;
                    }
                }
                if (yd_ok) {
                    y_size++;
                    yp--;
                }
            }
        }
    }
    return z;
}

int cStock::TesselTop(int xp, int yp)
{
    int x_size, y_size;
    float z = FindRectTop(xp, yp, x_size, y_size, true);
    bool farRect = false;
    while (y_size / x_size > 5) {
        farRect = true;
        yp += x_size * 5;
        z = FindRectTop(xp, yp, x_size, y_size, true);
    }

    while (x_size / y_size > 5) {
        farRect = true;
        xp += y_size * 5;
        z = FindRectTop(xp, yp, x_size, y_size, false);
    }

    // mark all points inside, and record the height the whole
    // rectangle is emitted at so the walls close against the face that
    // is actually there rather than against each cell's own m_stock
    for (int y = yp; y < yp + y_size; y++) {
        for (int x = xp; x < xp + x_size; x++) {
            m_attr[x][y] |= SIM_TESSEL_TOP;
            m_topFace[x][y] = z;
        }
    }

    // "Holds material", not "is above the floor": a column can sit at
    // the floor and hold material, or well above it and hold none.
    if ((m_stock[xp][yp] - m_bottom[xp][yp]) >= m_res) {
        // generate 4 3d points
        Point3D pbl(xp, yp, z);
        Point3D pbr(xp + x_size, yp, z);
        Point3D ptl(xp, yp + y_size, z);
        Point3D ptr(xp + x_size, yp + y_size, z);
        // Compares against this cell's own original seeded height, not a
        // single global "the whole stock started flat here" constant --
        // that assumption broke once SeedFromMesh could seed non-flat
        // starting geometry (see m_original's declaration). xp,yp is used
        // as the representative cell for the whole merged rectangle, the
        // same way z itself is already just m_stock[xp][yp].
        if (fabs(m_original[xp][yp] - z) < SIM_EPSILON) {
            AddQuad(pbl, pbr, ptr, ptl, facetsOuter);
        }
        else {
            AddQuad(pbl, pbr, ptr, ptl, facetsInner);
        }
    }

    if (farRect) {
        return -1;
    }
    return std::max(0, x_size - 1);
    // return 0;
}


bool cStock::ColumnSpan(int x, int y, float& bot, float& top) const
{
    if (x < 0 || x >= m_x || y < 0 || y >= m_y) {
        return false;
    }
    // Data() rather than operator[]: this is const, Array2D's subscript
    // is not.
    const long long i = (long long)x * m_y + y;
    // Presence from the TRUE surfaces, heights from the EMITTED faces:
    // a wall exists because material does, but must close against the
    // quad that was generated.
    if ((m_stock.Data()[i] - m_bottom.Data()[i]) < m_res) {
        return false;
    }
    top = m_topFace.Data()[i];
    bot = m_botFace.Data()[i];
    return true;
}

void cStock::AddWallQuadX(
    int x0,
    int x1,
    int yp,
    float zA,
    float zB,
    std::vector<MeshCore::MeshGeomFacet>& facets
)
{
    Point3D pbl(x0, yp, zA);
    Point3D ptl(x0, yp, zB);
    Point3D ptr(x1, yp, zB);
    Point3D pbr(x1, yp, zA);
    AddQuad(pbl, ptl, ptr, pbr, facets);
}

void cStock::AddWallQuadY(
    int y0,
    int y1,
    int xp,
    float zA,
    float zB,
    std::vector<MeshCore::MeshGeomFacet>& facets
)
{
    Point3D pbr(xp, y0, zA);
    Point3D ptr(xp, y0, zB);
    Point3D ptl(xp, y1, zB);
    Point3D pbl(xp, y1, zA);
    AddQuad(pbl, ptl, ptr, pbr, facets);
}

float cStock::FindRectBot(int& xp, int& yp, int& x_size, int& y_size, bool scanHoriz)
{
    // Merges on EQUAL BOTTOM, the mirror of FindRectTop on equal top.
    // Merging on "has any material" alone is only right when every
    // cell's underside is the same m_pz.
    float z = m_bottom[xp][yp];
    bool xr_ok = true;
    bool xl_ok = scanHoriz;
    bool yu_ok = true;
    bool yd_ok = !scanHoriz;
    x_size = 1;
    y_size = 1;
    while (xr_ok || xl_ok || yu_ok || yd_ok) {
        // sweep right x direction
        if (xr_ok) {
            int tx = xp + x_size;
            if (tx >= m_x) {
                xr_ok = false;
            }
            else {
                for (int y = yp; y < yp + y_size; y++) {
                    if ((m_attr[tx][y] & SIM_TESSEL_BOT) != 0
                        || (m_stock[tx][y] - m_bottom[tx][y]) < m_res
                        || fabs(z - m_bottom[tx][y]) > m_res) {
                        xr_ok = false;
                        break;
                    }
                }
                if (xr_ok) {
                    x_size++;
                }
            }
        }

        // sweep left x direction
        if (xl_ok) {
            int tx = xp - 1;
            if (tx < 0) {
                xl_ok = false;
            }
            else {
                for (int y = yp; y < yp + y_size; y++) {
                    if ((m_attr[tx][y] & SIM_TESSEL_BOT) != 0
                        || (m_stock[tx][y] - m_bottom[tx][y]) < m_res
                        || fabs(z - m_bottom[tx][y]) > m_res) {
                        xl_ok = false;
                        break;
                    }
                }
                if (xl_ok) {
                    x_size++;
                    xp--;
                }
            }
        }

        // sweep up y direction
        if (yu_ok) {
            int ty = yp + y_size;
            if (ty >= m_y) {
                yu_ok = false;
            }
            else {
                for (int x = xp; x < xp + x_size; x++) {
                    if ((m_attr[x][ty] & SIM_TESSEL_BOT) != 0
                        || (m_stock[x][ty] - m_bottom[x][ty]) < m_res
                        || fabs(z - m_bottom[x][ty]) > m_res) {
                        yu_ok = false;
                        break;
                    }
                }
                if (yu_ok) {
                    y_size++;
                }
            }
        }

        // sweep down y direction
        if (yd_ok) {
            int ty = yp - 1;
            if (ty < 0) {
                yd_ok = false;
            }
            else {
                for (int x = xp; x < xp + x_size; x++) {
                    if ((m_attr[x][ty] & SIM_TESSEL_BOT) != 0
                        || (m_stock[x][ty] - m_bottom[x][ty]) < m_res
                        || fabs(z - m_bottom[x][ty]) > m_res) {
                        yd_ok = false;
                        break;
                    }
                }
                if (yd_ok) {
                    y_size++;
                    yp--;
                }
            }
        }
    }
    return z;
}

int cStock::TesselBot(int xp, int yp)
{
    int x_size, y_size;
    float z = FindRectBot(xp, yp, x_size, y_size, true);
    bool farRect = false;
    while (y_size / x_size > 5) {
        farRect = true;
        yp += x_size * 5;
        z = FindRectBot(xp, yp, x_size, y_size, true);
    }

    while (x_size / y_size > 5) {
        farRect = true;
        xp += y_size * 5;
        z = FindRectBot(xp, yp, x_size, y_size, false);
    }

    // mark all points inside, and record the emitted height -- see
    // TesselTop
    for (int y = yp; y < yp + y_size; y++) {
        for (int x = xp; x < xp + x_size; x++) {
            m_attr[x][y] |= SIM_TESSEL_BOT;
            m_botFace[x][y] = z;
        }
    }

    // At the merged rectangle's own bottom, not the box floor: a
    // tilted stock's underside is a real surface, and a re-seed reads
    // whatever this emits back as the underside.
    Point3D pbl(xp, yp, z);
    Point3D pbr(xp + x_size, yp, z);
    Point3D ptl(xp, yp + y_size, z);
    Point3D ptr(xp + x_size, yp + y_size, z);
    AddQuad(pbl, ptl, ptr, pbr, facetsOuter);

    if (farRect) {
        return -1;
    }
    return std::max(0, x_size - 1);
    // return 0;
}


bool cStock::HasExtra(int x, int y) const
{
    return !m_extra.empty() && x >= 0 && x < m_x && y >= 0 && y < m_y
        && m_extra.find(ColumnKey(x, y)) != m_extra.end();
}

void cStock::EmittedColumn(int x, int y, std::vector<cSpan>& out) const
{
    if (HasExtra(x, y)) {
        ColumnAscending(x, y, out);
        return;
    }
    out.clear();
    float bot, top;
    if (ColumnSpan(x, y, bot, top)) {
        out.push_back({bot, top});
    }
}

void cStock::TesselMultiSpan()
{
    // Every span of every column holding more than one, at its exact
    // heights. The sweeps were kept off these columns (see Tessellate),
    // so nothing else covers them. Walls come from TesselSides.
    //
    // Faces merge along Y while the height is EXACTLY equal, the face
    // faces the same way and falls in the same mesh: the union of the
    // cells' faces, not an approximation of it, so closure cannot
    // suffer. A quarter turn leaves these faces flat and a row of cells
    // becomes one rectangle: 282,592 facets fell to 29,472 on a pocketed
    // block turned 90 degrees at 0.1 mm (engine_tests tessellate.inc).
    // At other angles equal heights are rare and this costs nothing.
    //
    // Sorted, x then y, so rows arrive in order and the output does not
    // depend on the hash map's.
    std::vector<long long> keys;
    keys.reserve(m_extra.size());
    for (const auto& entry : m_extra) {
        keys.push_back(entry.first);
    }
    std::sort(keys.begin(), keys.end());

    struct Face
    {
        float z;
        int kind;  // 0 top, cut; 1 top, as seeded; 2 bottom
        int from;  // first cell of the run, along Y
    };
    std::vector<Face> open, next;
    int openX = -1, openY = -1;  // the cell the open runs last reached

    auto flush = [&](const Face& f, int x, int y1) {
        Point3D pbl(x, f.from, f.z);
        Point3D pbr(x + 1, f.from, f.z);
        Point3D ptl(x, y1, f.z);
        Point3D ptr(x + 1, y1, f.z);
        if (f.kind == 2) {
            AddQuad(pbl, ptl, ptr, pbr, facetsOuter);
        }
        else {
            AddQuad(pbl, pbr, ptr, ptl, f.kind == 0 ? facetsInner : facetsOuter);
        }
    };
    auto flushAll = [&]() {
        for (const Face& f : open) {
            flush(f, openX, openY + 1);
        }
        open.clear();
    };

    std::vector<cSpan> spans;
    for (long long key : keys) {
        const int x = (int)(key / m_y);
        const int y = (int)(key % m_y);
        if (x != openX || y != openY + 1) {
            flushAll();
        }
        ColumnAscending(x, y, spans);
        next.clear();
        for (size_t k = 0; k < spans.size(); k++) {
            // Only the primary (the last, ascending) is ever cut: the
            // spans below it are as the seed left them.
            const bool primary = (k + 1 == spans.size());
            const bool cut = primary && fabs(m_original[x][y] - spans[k].top) >= SIM_EPSILON;
            next.push_back({spans[k].top, cut ? 0 : 1, y});
            next.push_back({spans[k].bottom, 2, y});
        }
        // Carry on each open run this cell continues; close the rest.
        for (Face& f : next) {
            for (size_t i = 0; i < open.size(); i++) {
                if (open[i].z == f.z && open[i].kind == f.kind) {
                    f.from = open[i].from;
                    open[i] = open.back();
                    open.pop_back();
                    break;
                }
            }
        }
        flushAll();
        open.swap(next);
        openX = x;
        openY = y;
    }
    flushAll();
}

int cStock::TesselSides(bool alongX, int line)
{
    // One wall line, swept along it: between row line-1 and row line
    // when alongX, between column line-1 and column line otherwise.
    // Side 1 is the column at `line`, side 2 the one at `line - 1`.
    //
    // Mostly the original sweep. Each side is a material interval
    // [bot, top], and whatever the two intervals do not share is exposed
    // and needs a quad: both present, up to two (tops and bottoms); one
    // present, one spanning its whole interval; neither, nothing. That
    // holds only while the intervals overlap, so two cases go to an
    // exact path instead, one cell at a time (runs of identical cells
    // merged): a column holding more than one span, and two single
    // spans that do not overlap. There the wall is the set difference of
    // the two columns' emitted span lists, each way.
    const int n = alongX ? m_x : m_y;
    auto cx1 = [&](int i) {
        return alongX ? i : line;
    };
    auto cy1 = [&](int i) {
        return alongX ? line : i;
    };
    auto cx2 = [&](int i) {
        return alongX ? i : line - 1;
    };
    auto cy2 = [&](int i) {
        return alongX ? line - 1 : i;
    };
    auto original = [&](int x, int y) {
        return (x >= 0 && x < m_x && y >= 0 && y < m_y) ? m_original[x][y] : m_pz;
    };
    auto quad = [&](int i0, int i1, float zA, float zB, std::vector<MeshCore::MeshGeomFacet>& f) {
        if (alongX) {
            AddWallQuadX(i0, i1, line, zA, zB, f);
        }
        else {
            AddWallQuadY(i0, i1, line, zA, zB, f);
        }
    };
    // On the stock's TRUE outer boundary if either adjacent column was
    // never part of the seeded shape (m_original == m_pz). Not the grid's
    // edge: a cylinder's curved side is a true boundary sitting entirely
    // inside the grid.
    auto facetsAt = [&](int i) -> std::vector<MeshCore::MeshGeomFacet>& {
        bool isTrueBoundary = (fabs(original(cx1(i), cy1(i)) - m_pz) < m_res)
            || (fabs(original(cx2(i), cy2(i)) - m_pz) < m_res);
        return isTrueBoundary ? facetsOuter : facetsInner;
    };
    auto special = [&](int i, bool p1, float b1, float t1, bool p2, float b2, float t2) {
        if (i >= n) {
            return false;
        }
        if (p1 && p2 && (t1 < b2 || t2 < b1)) {
            return true;
        }
        return HasExtra(cx1(i), cy1(i)) || HasExtra(cx2(i), cy2(i));
    };

    // --- The exact path's run: cells [runFrom, runTo) with identical
    // span lists on both sides.
    int runFrom = -1;
    int runTo = -1;
    std::vector<cSpan> runA, runB, curA, curB, piece;
    auto same = [](const std::vector<cSpan>& a, const std::vector<cSpan>& b) {
        if (a.size() != b.size()) {
            return false;
        }
        for (size_t k = 0; k < a.size(); k++) {
            if (a[k].bottom != b[k].bottom || a[k].top != b[k].top) {
                return false;
            }
        }
        return true;
    };
    auto flushExact = [&]() {
        if (runFrom < 0) {
            return;
        }
        std::vector<MeshCore::MeshGeomFacet>& facets = facetsAt(runFrom);
        // Side 1's material that side 2 does not cover faces side 2, and
        // the reverse: the argument orders of the one-sided cases below.
        SpanDifference(runA, runB, piece);
        for (const cSpan& s : piece) {
            quad(runFrom, runTo, s.top, s.bottom, facets);
        }
        SpanDifference(runB, runA, piece);
        for (const cSpan& s : piece) {
            quad(runFrom, runTo, s.bottom, s.top, facets);
        }
        runFrom = -1;
    };
    auto exactCell = [&](int c) {
        EmittedColumn(cx1(c), cy1(c), curA);
        EmittedColumn(cx2(c), cy2(c), curB);
        if (runFrom >= 0 && runTo == c && same(curA, runA) && same(curB, runB)) {
            runTo = c + 1;
            return;
        }
        flushExact();
        runFrom = c;
        runTo = c + 1;
        runA.swap(curA);
        runB.swap(curB);
    };

    float lastBot1, lastTop1, lastBot2, lastTop2;
    bool last1 = ColumnSpan(cx1(0), cy1(0), lastBot1, lastTop1);
    bool last2 = ColumnSpan(cx2(0), cy2(0), lastBot2, lastTop2);
    bool lastSpecial = special(0, last1, lastBot1, lastTop1, last2, lastBot2, lastTop2);

    int lastpoint = 0;
    for (int i = 1; i <= n; i++) {
        float newBot1, newTop1, newBot2, newTop2;
        bool new1 = ColumnSpan(cx1(i), cy1(i), newBot1, newTop1);
        bool new2 = ColumnSpan(cx2(i), cy2(i), newBot2, newTop2);
        bool newSpecial = special(i, new1, newBot1, newTop1, new2, newBot2, newTop2);

        if (lastSpecial) {
            exactCell(lastpoint);
        }
        else {
            flushExact();

            bool topsDiffer = last1 && last2 && fabs(lastTop1 - lastTop2) > SIM_HEIGHT_TOL;
            bool botsDiffer = last1 && last2 && fabs(lastBot1 - lastBot2) > SIM_HEIGHT_TOL;
            bool oneSided = (last1 != last2);

            if (topsDiffer || botsDiffer || oneSided) {
                // Extend the run while nothing that shapes the wall
                // changes: four heights and both presence flags -- and
                // the next cell is not one for the exact path.
                bool unchanged = !newSpecial && (new1 == last1) && (new2 == last2)
                    && (!last1
                        || (fabs(newTop1 - lastTop1) < SIM_HEIGHT_TOL
                            && fabs(newBot1 - lastBot1) < SIM_HEIGHT_TOL))
                    && (!last2
                        || (fabs(newTop2 - lastTop2) < SIM_HEIGHT_TOL
                            && fabs(newBot2 - lastBot2) < SIM_HEIGHT_TOL));
                if (unchanged) {
                    continue;
                }

                std::vector<MeshCore::MeshGeomFacet>& facets = facetsAt(lastpoint);

                if (last1 && last2) {
                    if (topsDiffer) {
                        quad(lastpoint, i, lastTop1, lastTop2, facets);
                    }
                    if (botsDiffer) {
                        // Arguments REVERSED against the top case: winding
                        // follows the sign of zA - zB, and the material is
                        // on the higher side between two tops but the
                        // lower side between two bottoms. Same order would
                        // face this quad inward.
                        //
                        // Normals were computed for all ten wall cases on
                        // both axes and every one faces outward. Check that
                        // way if it changes: a back-facing underside bounds
                        // the same volume and re-seeds identically, so no
                        // volume check can see it.
                        quad(lastpoint, i, lastBot2, lastBot1, facets);
                    }
                }
                else if (last1) {
                    // Side 2 empty, so side 1's whole interval is exposed.
                    quad(lastpoint, i, lastTop1, lastBot1, facets);
                }
                else {
                    quad(lastpoint, i, lastBot2, lastTop2, facets);
                }
            }
        }

        last1 = new1;
        last2 = new2;
        lastTop1 = newTop1;
        lastBot1 = newBot1;
        lastTop2 = newTop2;
        lastBot2 = newBot2;
        lastSpecial = newSpecial;
        lastpoint = i;
    }
    flushExact();
    return 0;
}

void cStock::SetFacetPoints(MeshCore::MeshGeomFacet& facet, Point3D& p1, Point3D& p2, Point3D& p3)
{
    // Tessellated vertices lie on cell edges, 0..m_x in grid units. Every
    // edge is i * m_res from the corner except the far one, which is the
    // stock's edge: the last cell is clipped there.
    auto worldX = [this](float gx) {
        return (gx >= (float)m_x ? m_lx : gx * m_res) + m_px;
    };
    auto worldY = [this](float gy) {
        return (gy >= (float)m_y ? m_ly : gy * m_res) + m_py;
    };
    facet._aclPoints[0][0] = worldX(p1.x);
    facet._aclPoints[0][1] = worldY(p1.y);
    facet._aclPoints[0][2] = p1.z;
    facet._aclPoints[1][0] = worldX(p2.x);
    facet._aclPoints[1][1] = worldY(p2.y);
    facet._aclPoints[1][2] = p2.z;
    facet._aclPoints[2][0] = worldX(p3.x);
    facet._aclPoints[2][1] = worldY(p3.y);
    facet._aclPoints[2][2] = p3.z;
    facet.CalcNormal();
}

void cStock::AddQuad(
    Point3D& p1,
    Point3D& p2,
    Point3D& p3,
    Point3D& p4,
    std::vector<MeshCore::MeshGeomFacet>& facets
)
{
    MeshCore::MeshGeomFacet facet;
    SetFacetPoints(facet, p1, p2, p3);
    facets.push_back(facet);
    SetFacetPoints(facet, p1, p3, p4);
    facets.push_back(facet);
}

void cStock::Tessellate(Mesh::MeshObject& meshOuter, Mesh::MeshObject& meshInner)
{
    // Always a full pass over the whole grid. It runs on demand -- export,
    // and the mesh a 3+2 re-seed reads back -- never per tick. An
    // incremental version that re-merged only around cut cells was
    // removed: every caller forced a full pass anyway, and on its own it
    // left holes in the mesh.

    // The sweep flags record which cells a merged rectangle has claimed.
    for (int x = 0; x < m_x; x++) {  // x outer: see the constructor
        for (int y = 0; y < m_y; y++) {
            m_attr[x][y] = 0;
        }
    }
    facetsOuter.clear();
    facetsInner.clear();

    // Columns holding more than one span stay out of both merging sweeps:
    // a merge emits within m_res of the true height, and a bottom lowered
    // that far could pass through the span beneath it. They are emitted
    // exactly by TesselMultiSpan, and their recorded face heights are the
    // true ones.
    for (const auto& entry : m_extra) {
        const int x = (int)(entry.first / m_y);
        const int y = (int)(entry.first % m_y);
        m_attr[x][y] |= SIM_TESSEL_TOP | SIM_TESSEL_BOT;
        m_topFace[x][y] = m_stock[x][y];
        m_botFace[x][y] = m_bottom[x][y];
    }

    // --- Top surface
    for (int y = 0; y < m_y; y++) {
        for (int x = 0; x < m_x; x++) {
            // Milled clean through: top below bottom, holds nothing. The
            // bottom sweep and the wall builders skip these, and the top
            // sweep must agree -- a face with no bottom under it is a
            // hole, which corrupts the volume integral and leaves the
            // column unreadable on re-seed.
            if ((m_stock[x][y] - m_bottom[x][y]) < m_res) {
                m_attr[x][y] |= SIM_TESSEL_TOP;
            }
            if ((m_attr[x][y] & SIM_TESSEL_TOP) == 0) {
                x += TesselTop(x, y);
            }
        }
    }

    // --- Bottom surface
    for (int y = 0; y < m_y; y++) {
        for (int x = 0; x < m_x; x++) {
            if ((m_stock[x][y] - m_bottom[x][y]) < m_res) {
                m_attr[x][y] |= SIM_TESSEL_BOT;
            }
            if ((m_attr[x][y] & SIM_TESSEL_BOT) == 0) {
                x += TesselBot(x, y);
            }
        }
    }

    // --- Columns of several spans, every face at its exact height.
    TesselMultiSpan();

    // --- Walls. Must follow both sweeps: they close against the heights
    // TesselTop and TesselBot recorded in m_topFace / m_botFace. The
    // ranges are inclusive of m_y / m_x, which is the grid's far edge.
    for (int y = 0; y <= m_y; y++) {
        TesselSides(true, y);
    }
    for (int x = 0; x <= m_x; x++) {
        TesselSides(false, x);
    }

    meshOuter.addFacets(facetsOuter);
    meshInner.addFacets(facetsInner);

    // Released, not just cleared: at a fine grid these run to hundreds of
    // thousands of facets and nothing needs them until the next call.
    std::vector<MeshCore::MeshGeomFacet>().swap(facetsOuter);
    std::vector<MeshCore::MeshGeomFacet>().swap(facetsInner);
}


// The lowest point a tool's surface reaches over one cell centre during
// part of a move, the part being parameter range [lo, hi]. surface(t) is
// the surface's height over the centre at t; tc, in [lo, hi], is where the
// tool's axis passes closest; zFalls says whether the move descends as t
// grows; tEps is how finely to locate the minimum, in t. The tool's
// profile rises with distance from its axis, so:
//
//   flat tool      surface(t) is linear in t; the minimum is at an end
//   level move     the minimum is at the closest approach, tc
//   ramp, shaped   surface(t) is convex for ball, bull and V tools and
//   tool           has its minimum between tc and the lower end; a
//                  golden-section search finds it
template<typename F>
static float LowestSurface(
    F surface,
    float lo,
    float hi,
    float tc,
    bool flat,
    bool level,
    bool zFalls,
    float tEps
)
{
    if (flat) {
        return std::min(surface(lo), surface(hi));
    }
    if (level) {
        return surface(tc);
    }
    float a = zFalls ? tc : lo;
    float b = zFalls ? hi : tc;
    const float g = 0.6180340f;
    float m1 = b - g * (b - a);
    float m2 = a + g * (b - a);
    float f1 = surface(m1);
    float f2 = surface(m2);
    for (int i = 0; i < 40 && (b - a) > tEps; i++) {
        if (f1 < f2) {
            b = m2;
            m2 = m1;
            f2 = f1;
            m1 = b - g * (b - a);
            f1 = surface(m1);
        }
        else {
            a = m1;
            m1 = m2;
            f1 = f2;
            m2 = a + g * (b - a);
            f2 = surface(m2);
        }
    }
    // The ends are always candidates: the minimum can sit exactly on one.
    return std::min({f1, f2, surface(zFalls ? hi : lo), surface(tc)});
}

// Whether the tool's profile is the same everywhere across it.
static bool IsFlatTool(cSimTool& tool)
{
    const float p0 = tool.GetToolProfileAt(0.0f);
    return std::fabs(tool.GetToolProfileAt(1.0f) - p0) < 1e-6f
        && std::fabs(tool.GetToolProfileAt(0.5f) - p0) < 1e-6f;
}

void cStock::SweepSegment(const Point3D& a, const Point3D& b, cSimTool& tool, float rad)
{
    // Every cell whose CENTRE the tool passes over is cut to the lowest
    // point the tool's surface reaches above that centre during the move.
    // Grid units throughout, except z, which is in world units as m_stock
    // is. For each centre: the part of the move within reach, [tlo, thi],
    // from |w - t v| <= rad, then LowestSurface over it.
    const float vx = b.x - a.x;
    const float vy = b.y - a.y;
    const float vz = b.z - a.z;
    const float vv = vx * vx + vy * vy;
    const float r2 = rad * rad;
    const bool moving = vv > 1e-12f;
    const bool flat = IsFlatTool(tool);
    const bool level = std::fabs(vz) < 1e-6f;
    const float p0 = tool.GetToolProfileAt(0.0f);
    // How finely the search locates a minimum: a thousandth of a cell
    // along the move. Where the minimum is smooth this is far finer than
    // needed, but it can sit on a kink -- a bull nose's flat meeting its
    // radius, a ball near its rim, and a real tool's profile is a table of
    // straight segments, kinked everywhere -- and there the height error is
    // the slope times what is left of the bracket. At a twentieth of a cell
    // that reached 0.012 mm; at a thousandth, 600 random moves against a
    // brute-force reference showed none above 0.002 mm, for 44% more time
    // on ball surfacing.
    const float tEps = 1e-3f / std::max(1.0f, std::sqrt(vv));

    const int x0 = std::max(0, (int)std::floor(std::min(a.x, b.x) - rad));
    const int x1 = std::min(m_x - 1, (int)std::ceil(std::max(a.x, b.x) + rad));
    const int y0 = std::max(0, (int)std::floor(std::min(a.y, b.y) - rad));
    const int y1 = std::min(m_y - 1, (int)std::ceil(std::max(a.y, b.y) + rad));

    for (int x = x0; x <= x1; x++) {  // x outer: see the constructor
        const float cx = CellCentreX(x);
        const float wx = cx - a.x;

        // Only the rows this column can reach: the part of the move within
        // rad of it in X, widened by rad in Y. A long diagonal move would
        // otherwise walk its whole bounding box.
        int ys = y0;
        int ye = y1;
        if (std::fabs(vx) > 1e-9f) {
            float ta = (cx - rad - a.x) / vx;
            float tb = (cx + rad - a.x) / vx;
            if (ta > tb) {
                std::swap(ta, tb);
            }
            ta = std::max(ta, 0.0f);
            tb = std::min(tb, 1.0f);
            if (ta > tb) {
                continue;
            }
            const float ya = a.y + ta * vy;
            const float yb = a.y + tb * vy;
            ys = std::max(y0, (int)std::floor(std::min(ya, yb) - rad));
            ye = std::min(y1, (int)std::ceil(std::max(ya, yb) + rad));
        }

        for (int y = ys; y <= ye; y++) {
            const float wy = CellCentreY(y) - a.y;
            const float ww = wx * wx + wy * wy;
            float tlo = 0.0f;
            float thi = 1.0f;
            float tc = 0.0f;
            if (moving) {
                const float wv = wx * vx + wy * vy;
                const float disc = wv * wv - vv * (ww - r2);
                if (disc < 0.0f) {
                    continue;
                }
                const float sq = std::sqrt(disc);
                tlo = std::max(0.0f, (wv - sq) / vv);
                thi = std::min(1.0f, (wv + sq) / vv);
                if (tlo > thi) {
                    continue;
                }
                tc = std::min(thi, std::max(tlo, wv / vv));
            }
            else if (ww > r2) {
                continue;
            }
            auto surface = [&](float t) {
                const float dx = wx - t * vx;
                const float dy = wy - t * vy;
                const float d = std::sqrt(dx * dx + dy * dy);
                return a.z + t * vz + tool.GetToolProfileAt(std::min(1.0f, d / rad));
            };
            // Flat: the profile is the same everywhere, so no distance is
            // needed. A plunge has one distance throughout, with its lowest
            // point at the lower end.
            if (flat || !moving) {
                CutColumn(
                    x,
                    y,
                    flat ? a.z + std::min(tlo * vz, thi * vz) + p0
                         : std::min(surface(0.0f), surface(1.0f))
                );
                continue;
            }
            // Bounds before searching. No point of the move is closer than
            // tc and the profile only rises with distance, so the lowest z
            // in reach plus the profile at tc is a floor under the answer;
            // the surface at tc is a value actually reached, so a ceiling.
            // In surfacing every cell is within reach of a dozen passes and
            // only the nearest decides it: the floor alone usually shows a
            // move cannot cut the cell at all.
            const float atTc = surface(tc);
            const float floorZ = atTc - (a.z + tc * vz) + a.z + std::min(tlo * vz, thi * vz);
            if (floorZ >= m_stock[x][y]) {
                continue;
            }
            const float z = (level || atTc - floorZ < 1e-3f)
                ? atTc
                : LowestSurface(surface, tlo, thi, tc, flat, level, vz < 0.0f, tEps);
            CutColumn(x, y, z);
        }
    }
}

void cStock::ApplyLinearTool(Point3D& p1, Point3D& p2, cSimTool& tool)
{
    const Point3D a = ToInner(p1);
    const Point3D b = ToInner(p2);
    const float rad = tool.radius / m_res;
    MarkDirty(
        (int)std::floor(std::min(a.x, b.x) - rad),
        (int)std::floor(std::min(a.y, b.y) - rad),
        (int)std::ceil(std::max(a.x, b.x) + rad),
        (int)std::ceil(std::max(a.y, b.y) + rad)
    );
    SweepSegment(a, b, tool, rad);
}

void cStock::ApplyCircularTool(Point3D& p1, Point3D& p2, Point3D& cent, cSimTool& tool, bool isCCW)
{
    // An arc in the XY plane, Z moving linearly along it -- a helix when
    // the ends differ in Z -- cut the same way as a line: every cell
    // centre within reach, to the lowest point the tool's surface reaches
    // over it. Parameter s runs 0..1 along the arc; angle = a0 + s * sweep.
    const Point3D a = ToInner(p1);
    const Point3D b = ToInner(p2);
    const float rad = tool.radius / m_res;
    const float ccx = a.x + cent.x / m_res;
    const float ccy = a.y + cent.y / m_res;
    const float ra = std::sqrt((a.x - ccx) * (a.x - ccx) + (a.y - ccy) * (a.y - ccy));
    const float vz = b.z - a.z;
    const bool flat = IsFlatTool(tool);
    const bool level = std::fabs(vz) < 1e-6f;
    const float p0 = tool.GetToolProfileAt(0.0f);

    MarkDirty(
        (int)std::floor(ccx - ra - rad),
        (int)std::floor(ccy - ra - rad),
        (int)std::ceil(ccx + ra + rad),
        (int)std::ceil(ccy + ra + rad)
    );

    const double a0 = std::atan2(a.y - ccy, a.x - ccx);
    double sweep;
    if (std::hypot(b.x - a.x, b.y - a.y) < 1e-4) {
        // Coincident ends are a full circle in G-code, not an empty arc.
        sweep = isCCW ? 2 * pi : -2 * pi;
    }
    else {
        sweep = std::atan2(b.y - ccy, b.x - ccx) - a0;
        if (isCCW && sweep < 0) {
            sweep += 2 * pi;
        }
        if (!isCCW && sweep > 0) {
            sweep -= 2 * pi;
        }
    }
    const double span = std::fabs(sweep);
    const double dir = sweep < 0 ? -1.0 : 1.0;
    const float tEps = 1e-3f / std::max(1.0f, (float)(span * ra));  // see SweepSegment
    if (span < 1e-9 || ra < 1e-6f) {
        SweepSegment(a, b, tool, rad);  // no arc to speak of
        return;
    }

    const int x0 = std::max(0, (int)std::floor(ccx - ra - rad));
    const int x1 = std::min(m_x - 1, (int)std::ceil(ccx + ra + rad));
    const int y0 = std::max(0, (int)std::floor(ccy - ra - rad));
    const int y1 = std::min(m_y - 1, (int)std::ceil(ccy + ra + rad));

    for (int x = x0; x <= x1; x++) {  // x outer: see the constructor
        const float px = CellCentreX(x) - ccx;
        for (int y = y0; y <= y1; y++) {
            const float py = CellCentreY(y) - ccy;
            const float rho = std::sqrt(px * px + py * py);
            if (std::fabs(rho - ra) > rad) {
                continue;  // outside the ring the tool sweeps
            }
            // Angles within reach: |angle - phi| <= h, from the law of
            // cosines. At the ring's centre every angle is within reach.
            double h = pi;
            if (rho > 1e-6f) {
                const double k = ((double)rho * rho + (double)ra * ra - (double)rad * rad)
                    / (2.0 * rho * ra);
                h = std::acos(std::min(1.0, std::max(-1.0, k)));
            }
            // In "along the arc" angle u in [0, span]: the reachable window
            // is centred on u = (phi - a0) * dir, repeating every 2 pi.
            double uc = std::remainder((std::atan2(py, px) - a0) * dir, 2 * pi);
            if (uc < 0) {
                uc += 2 * pi;
            }
            auto surface = [&](float s) {
                const double ang = a0 + s * sweep;
                const float dx = px - ra * (float)std::cos(ang);
                const float dy = py - ra * (float)std::sin(ang);
                const float d = std::sqrt(dx * dx + dy * dy);
                return a.z + s * vz + tool.GetToolProfileAt(std::min(1.0f, d / rad));
            };
            float z = 1e30f;
            for (int k = -1; k <= 1; k++) {
                const double c = uc + k * 2 * pi;
                const double lo = std::max(0.0, c - h);
                const double hi = std::min(span, c + h);
                if (lo > hi) {
                    continue;
                }
                const float slo = (float)(lo / span);
                const float shi = (float)(hi / span);
                const float sc = (float)(std::min(hi, std::max(lo, c)) / span);
                z = std::min(
                    z,
                    flat ? a.z + std::min(slo * vz, shi * vz) + p0
                         : LowestSurface(surface, slo, shi, sc, flat, level, vz < 0.0f, tEps)
                );
            }
            if (z < 1e29f) {
                CutColumn(x, y, z);
            }
        }
    }
}

//************************************************************************************************************
// Point (or vector)
//************************************************************************************************************

void Point3D::UpdateCmd(Path::Command& cmd)
{
    if (cmd.has("X")) {
        x = cmd.getPlacement().getPosition()[0];
    }
    if (cmd.has("Y")) {
        y = cmd.getPlacement().getPosition()[1];
    }
    if (cmd.has("Z")) {
        z = cmd.getPlacement().getPosition()[2];
    }
}

//************************************************************************************************************
// Simulation tool
//************************************************************************************************************
cSimTool::cSimTool(const TopoDS_Shape& toolShape, float res)
{
    BRepCheck_Analyzer aChecker(toolShape);
    bool shapeIsValid = aChecker.IsValid() ? true : false;

    if (!shapeIsValid) {
        throw Base::RuntimeError("Path Simulation: Error in tool geometry");
    }

    Bnd_Box boundBox;
    BRepBndLib::Add(toolShape, boundBox);

    boundBox.SetGap(0.0);
    Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
    boundBox.Get(xMin, yMin, zMin, xMax, yMax, zMax);
    radius = (xMax - xMin) / 2;
    length = zMax - zMin;

    // Tool profile sample density scales with whatever `res` the caller
    // passes in (typically tied to the general accuracy setting), so a
    // coarser accuracy setting under-samples the tool's curvature even
    // though GetToolProfileAt() interpolates between samples - fewer, more
    // widely-spaced samples still track a curved profile (e.g. a ball nose)
    // less faithfully. Guarantee a minimum sample count across the radius
    // regardless of what was requested: profile extraction is a one-time
    // cost per operation, not a per-frame one, so affording extra samples
    // here is cheap.
    const int minProfileSamples = 200;
    if (radius > 0.0f) {
        res = std::min(res, radius / (float)(minProfileSamples - 1));
    }

    Base::Vector3d pnt;
    pnt.x = 0;
    pnt.y = 0;
    pnt.z = 0;

    // Out to the tool's own edge, so GetToolProfileAt has a sample there.
    int radValue = SamplesToCover<float>(radius, res);

    for (int x = 0; x < radValue; x++) {
        // find the face of the tool by checking z points across the
        // radius to see if the point is inside the shape
        pnt.x = static_cast<double>(x) * res;
        bool inside = isInside(toolShape, pnt, res);

        // move down until the point is outside the shape
        while (inside && std::abs(pnt.z) < length) {
            pnt.z -= res;
            inside = isInside(toolShape, pnt, res);
        }

        // move up until the point is first inside the shape and record the position
        while (!inside && pnt.z < length) {
            pnt.z += res;
            inside = isInside(toolShape, pnt, res);

            if (inside) {
                toolShapePoint shapePoint;
                shapePoint.radiusPos = pnt.x;
                shapePoint.heightPos = pnt.z;
                m_toolShape.push_back(shapePoint);
                break;
            }
        }
    }
}

float cSimTool::GetToolProfileAt(float pos)  // pos is -1..1 location along the radius of the tool
                                             // (0 is center)
{
    if (m_toolShape.empty()) {
        return 0.0f;
    }

    float queryRadius = std::abs(pos) * radius;

    toolShapePoint test;
    test.radiusPos = queryRadius;

    auto it
        = std::lower_bound(m_toolShape.begin(), m_toolShape.end(), test, toolShapePoint::less_than());

    if (it == m_toolShape.end()) {
        // Past the last sampled radius (can happen right at the tool's outer
        // edge due to integer truncation in how many samples were taken) -
        // hold the last known height instead of snapping to 0, which would
        // otherwise show up as an artificial cliff at the tool boundary.
        return m_toolShape.back().heightPos;
    }
    if (it == m_toolShape.begin()) {
        // At or before the first sample - nothing to interpolate from.
        return it->heightPos;
    }

    // Linearly interpolate between the bracketing samples instead of
    // snapping to whichever one is >= the query radius. Without this, the
    // returned height is a staircase function of the sample spacing (res),
    // no matter how fine that spacing is made.
    auto prev = it;
    --prev;
    float span = it->radiusPos - prev->radiusPos;
    float t = span > 0.0f ? (queryRadius - prev->radiusPos) / span : 0.0f;
    return prev->heightPos + t * (it->heightPos - prev->heightPos);
}

bool cSimTool::isInside(const TopoDS_Shape& toolShape, Base::Vector3d pnt, float res)
{
    bool checkFace = true;
    TopAbs_State stateIn = TopAbs_IN;

    try {
        BRepClass3d_SolidClassifier solidClassifier(toolShape);
        gp_Pnt vertex = gp_Pnt(pnt.x, pnt.y, pnt.z);
        solidClassifier.Perform(vertex, res);
        bool inside = (solidClassifier.State() == stateIn);
        if (checkFace && solidClassifier.IsOnAFace()) {
            inside = true;
        }
        return inside;
    }
    catch (...) {
        return false;
    }
}
