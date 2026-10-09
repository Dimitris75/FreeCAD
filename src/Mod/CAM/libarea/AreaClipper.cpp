// SPDX-License-Identifier: BSD-3-Clause

// AreaClipper.cpp

// implements CArea methods using Angus Johnson's "Clipper"

#include "Area.h"
#include "clipper2/clipper.h"
#include <Precision.hxx>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <map>
#include <optional>
#include <stdexcept>
#include <vector>

using namespace heeks;
using namespace Clipper2Lib;

bool CArea::HolesLinked()
{
    return false;
}

double CArea::m_clipper_scale = CArea::default_clipper_scale;

static const int min_arc_points = 4;

// Convert between PointD (double) and Point64 (int64) with scaling
static Point64 ToPoint64(const PointD& p)
{
    return Point64(
        (int64_t)(floor(p.x * CArea::m_clipper_scale + 0.5)),
        (int64_t)(floor(p.y * CArea::m_clipper_scale + 0.5)),
        p.z
    );
}

static PointD ToPointD(const Point64& p)
{
    return PointD((double)p.x / CArea::m_clipper_scale, (double)p.y / CArea::m_clipper_scale, p.z);
}


// Helper method for recentering an angle in the 2*PI range next to a reference angle
// type = 1 puts phi CCW of phi_ref; type = -1 puts it CW
// final bounds: result is between phi_ref (exclusive) and (phi_ref + 2*pi * type) (inclusive)
static double recenter(double phi, double phi_ref, int type)
{
    while (phi <= phi_ref && phi < phi_ref + 2 * M_PI * type) {
        phi += 2 * M_PI;
    }
    while (phi >= phi_ref && phi > phi_ref + 2 * M_PI * type) {
        phi -= 2 * M_PI;
    }
    return phi;
};

void CArea::Subtract(const CArea& a2)
{
    Clip(ClipType::Difference, a2);
}

void CArea::Intersect(const CArea& a2)
{
    Clip(ClipType::Intersection, a2);
}

void CArea::Union(const CArea& a2)
{
    Clip(ClipType::Union, a2);
}

void CArea::Xor(const CArea& a2)
{
    Clip(ClipType::Xor, a2);
}

void CArea::PopulateClipper(Clipper64& c, bool as_clip, ConversionMetadata& metadata) const
{
    Paths64 closed_paths;
    Paths64 open_paths;

    for (const CCurve& curve : m_curves) {
        bool is_closed = curve.IsClosed();

        if (!is_closed && as_clip) {
            throw std::logic_error("Open curves cannot be used as clip geometry");
        }

        Path64 p = MakePoly(curve, metadata);

        if (is_closed) {
            closed_paths.push_back(p);
        }
        else {
            open_paths.push_back(p);
        }
    }

    if (as_clip) {
        if (!closed_paths.empty()) {
            c.AddClip(closed_paths);
        }
    }
    else {
        if (!closed_paths.empty()) {
            c.AddSubject(closed_paths);
        }
        if (!open_paths.empty()) {
            c.AddOpenSubject(open_paths);
        }
    }
}

// Internal function to apply a clipping operation with clipper. Results (for edges tagged 1,
// or if there are no tags) are stored in `this`.
//
// op: the boolean clipping operation to perform (Union, Difference, etc)
// this: subject geometry
// clip_area: clipping geometry
// fillType: fill rule applied to determine inside/outside (Positive, EvenOdd, etc)
// reverseOpenPathContents: if true, reverse the point order within each open path result
// reverseOpenPathOrder: if true, reverse the ordering of open path results
// cNeg: if provided, edges tagged -1 (i.e. negative-offset segments from NaiveOffset) are
//       stored in cNeg instead of being dropped. Edges tagged 1always go into `this`. End caps
//       (tagged 0) are always dropped.
void CArea::_Clip(
    ClipType op,
    const CArea& clip_area,
    FillRule fillType,
    bool reverseOpenPathContents,
    bool reverseOpenPathOrder,
    std::optional<std::reference_wrapper<CArea>> cNeg
)
{
    // Initialize a clipper object and populate it with subject/clip geometry
    Clipper64 c;
    ConversionMetadata metadata;
    PopulateClipper(c, false, metadata);
    clip_area.PopulateClipper(c, true, metadata);

    // Set up a callback for clipper to log information about points created during
    // the clipper operation.
    c.SetZCallback([&metadata](
                       const Point64& e1bot,
                       const Point64& e1top,
                       const Point64& e2bot,
                       const Point64& e2top,
                       Point64& pt
                   ) {
        if (pt.x == e1bot.x && pt.y == e1bot.y) {
            pt.z = e1bot.z;
        }
        else if (pt.x == e1top.x && pt.y == e1top.y) {
            pt.z = e1top.z;
        }
        else if (pt.x == e2bot.x && pt.y == e2bot.y) {
            pt.z = e2bot.z;
        }
        else if (pt.x == e2top.x && pt.y == e2top.y) {
            pt.z = e2top.z;
        }
        else if (e1bot.z != 0 || e1top.z != 0 || e2bot.z != 0 || e2top.z != 0) {
            pt.z = metadata.z_next++;
            metadata.z_to_xy[pt.z] = {pt.x, pt.y};
        }

        if (pt.z != e1bot.z && pt.z != e1top.z) {
            metadata.edges[pt.z].push_back(e1bot.z);
            metadata.edges[e1bot.z].push_back(pt.z);
            metadata.edges[pt.z].push_back(e1top.z);
            metadata.edges[e1top.z].push_back(pt.z);
        }
        if (pt.z != e2bot.z && pt.z != e2top.z) {
            metadata.edges[pt.z].push_back(e2bot.z);
            metadata.edges[e2bot.z].push_back(pt.z);
            metadata.edges[pt.z].push_back(e2top.z);
            metadata.edges[e2top.z].push_back(pt.z);
        }

        const int64_t e1min = std::min(e1bot.z, e1top.z);
        const int64_t e1max = std::max(e1bot.z, e1top.z);
        const int64_t e2min = std::min(e2bot.z, e2top.z);
        const int64_t e2max = std::max(e2bot.z, e2top.z);
        metadata.intersections.insert({pt.z, std::make_tuple(e1min, e1max, e2min, e2max)});
    });

    // Execute the operation, potentially producing both closed and open path results
    Paths64 closedPaths, openPaths;
    c.Execute(op, fillType, closedPaths, openPaths);

    // Reverse open path contents if requested
    if (reverseOpenPathContents) {
        for (auto& path : openPaths) {
            std::reverse(path.begin(), path.end());
        }
    }

    // Reverse open path order if requested
    if (reverseOpenPathOrder) {
        std::reverse(openPaths.begin(), openPaths.end());
    }

    m_curves.clear();
    SetFromResult(closedPaths, /*is_closed=*/true, metadata, cNeg);
    SetFromResult(openPaths, /*is_closed=*/false, metadata, cNeg);
}

void CArea::Clip(ClipType op, const CArea& clip_area, FillRule fillType)
{
    _Clip(op, clip_area, fillType);
}

void CArea::ClipperNoop()
{
    ConversionMetadata metadata;
    Paths64 closed_paths;
    Paths64 open_paths;
    for (const CCurve& curve : m_curves) {
        bool is_closed = curve.IsClosed();
        Path64 p = MakePoly(curve, metadata);

        if (is_closed) {
            closed_paths.push_back(p);
        }
        else {
            open_paths.push_back(p);
        }
    }

    m_curves.clear();
    SetFromResult(closed_paths, /*is_closed=*/true, metadata);
    SetFromResult(open_paths, /*is_closed=*/false, metadata);
}

void CArea::Debug_IntersectOpenPathReversal(
    const CArea& clip_area,
    bool reverseOpenPathContents,
    bool reverseOpenPathOrder
)
{
    _Clip(
        ClipType::Intersection,
        clip_area,
        FillRule::EvenOdd,
        reverseOpenPathContents,
        reverseOpenPathOrder
    );
}

// Creates the naive offset of curves by offsetting each segment by +-offset.
//
// The "naive offset" is produced by offsetting each individual segment on its own and
// joining adjacent offset segments to produce a closed curve. Open curves are closed
// with round end caps. The final result of this operation is expected to be self intersecting,
// and should be post-processed with a union operation with positive fill rule.
//
// This function should always be called with positive offset. Also, closed
// input curves should be correctly oriented. These input requirements are
// necessary to ensure that the output has positive winding, so subsequent
// union operations (with positive fill type) behave as expected.
//
// This function operates natively on arcs and line segments/CVertex; no arc approximation/clipper.
//
// The output geometry is written to m_curves, with each curve's m_edgeTags holding one tag per
// edge. A tag of 0 indicates that the segment is an end cap. 1 indicates that it was produced by
// offsetting in the requested/positive direction, and -1 indicates that it was produced by
// offsetting in the opposite/negative direction.
void CArea::NaiveOffset(double offset)
{
    // Positive oriented curves should be CCW (positive area) but some callers use the
    // wrong convention. For backwards compatibility/to support them, we check if the
    // total area is negative and use that as a cue to reverse all closed curves
    if (GetArea() < 0) {
        for (CCurve& curve : m_curves) {
            if (curve.IsClosed()) {
                curve.Reverse();
            }
        }
    }

    std::list<CCurve> offset_curves;

    for (CCurve& curve : m_curves) {
        if (curve.m_vertices.empty()) {
            continue;
        }

        // Handle "curves" of a single point -- positive offset is the circle about that point,
        // and negative offset is empty.
        if (curve.m_vertices.size() == 1) {
            const Point& center = curve.m_vertices.front().m_p;
            const Point right(center.x + offset, center.y);
            const Point left(center.x - offset, center.y);

            // Construct output circle
            CCurve output_curve;
            output_curve.m_vertices.emplace_back(0, right, heeks::Point {0, 0});
            output_curve.m_vertices.emplace_back(1, left, center);
            output_curve.m_vertices.emplace_back(1, right, center);
            for (auto it = std::next(output_curve.m_vertices.begin());
                 it != output_curve.m_vertices.end();
                 ++it) {
                output_curve.m_edgeTags.push_back(1);
            }
            offset_curves.push_back(output_curve);

            continue;
        }

        // Loop over the segments, offsetting and joining
        //
        // cPos is the positive offset; cNeg is the negative offset. Note that both are built
        // forwards, so cNeg will need to be reversed later. End caps will be handled later
        CCurve cPos;
        CCurve cNeg;
        double startDirX = 0, startDirY = 0, startQex = 0;  // initialized below
        double prevDirX = 0, prevDirY = 0;                  // initialized below
        heeks::Point pPrev = curve.m_vertices.front().m_p;
        double enterQ = 0;

        // Utility for joining from the current endpoint to posTarget and negTarget.
        //
        // arcCenter: the un-offset vertex/center of joining arc
        // enterDirX/enterDirY: the tangent direction entering the join
        // exitDirX/exitDirY: the tangent exiting the join
        // enterQ/exitQ: the curvature of the enterance/exit segments
        auto addJoin = [&](const heeks::Point& posTarget,
                           const heeks::Point& negTarget,
                           const heeks::Point& arcCenter,
                           double enterDirX,
                           double enterDirY,
                           double exitDirX,
                           double exitDirY,
                           const double enterQ,
                           const double exitQ) {
            // Skip the join if the points already match in clipper coordinates
            const Point64 posTarget64 = ToPoint64(PointD(posTarget.x, posTarget.y, 0));
            const Point64 negTarget64 = ToPoint64(PointD(negTarget.x, negTarget.y, 0));
            const Point64 posBack64 = ToPoint64(
                PointD(cPos.m_vertices.back().m_p.x, cPos.m_vertices.back().m_p.y, 0)
            );
            const Point64 negBack64 = ToPoint64(
                PointD(cNeg.m_vertices.back().m_p.x, cNeg.m_vertices.back().m_p.y, 0)
            );
            if ((std::abs(posTarget64.x - posBack64.x) < 2 && std::abs(posTarget64.y - posBack64.y) < 2)
                || (std::abs(negTarget64.x - negBack64.x) < 2
                    && std::abs(negTarget64.y - negBack64.y) < 2)) {
                // Skip the join if the points are already equal, or nearly equal. I chose
                // `dx > 2 || dy > 2` to be sure that in segments we join, the process of rounding
                // to integers does't move the points enough to change which side should be joined
                // by an arc and which should be joined by lines to the center point. Without
                // allowing this much slack, the decision is sometimes incorrect and the final
                // output contains spikes to the arc center.
                return;
            }

            // Determine how the join is done. 0 = no join, 1 = positive arc, -1 = negative arc
            int joinType = 0;

            // Check the angle of the tangents. If not (anti-)parallel, it is easy to choose if the
            // round join belongs to the positive side or the negative side.
            const double cross = enterDirX * exitDirY - enterDirY * exitDirX;
            if (cross > 0) {
                joinType = 1;
            }
            else if (cross < 0) {
                joinType = -1;
            }
            else {
                // Check which it is, parallel or anti-parallel. If parallel, no join is required.
                // In principle parallel tangents should already be filtered out by the check at the
                // top for skipping joins, but I'm not so confident in the numerics that it isn't
                // worth checking.
                const double dot = enterDirX * exitDirX + enterDirY * exitDirY;
                if (dot > 0) {
                    // Parallel --> the ends of the previous/current offset segments already align,
                    // no join required
                    return;
                }
                else {
                    // Anti parallel --> make the decision based on curvature
                    if (enterQ < -exitQ) {
                        joinType = 1;
                    }
                    else {
                        joinType = -1;
                    }
                }
            }

            // Join methodology has been determined; now construct the joining segments
            if (joinType == 1) {
                cPos.m_vertices.emplace_back(1, posTarget, arcCenter);
                cNeg.m_vertices.emplace_back(0, arcCenter, heeks::Point {0, 0});
                cNeg.m_vertices.emplace_back(0, negTarget, heeks::Point {0, 0});
            }
            else if (joinType == -1) {
                cPos.m_vertices.emplace_back(0, arcCenter, heeks::Point {0, 0});
                cPos.m_vertices.emplace_back(0, posTarget, heeks::Point {0, 0});
                cNeg.m_vertices.emplace_back(-1, negTarget, arcCenter);
            }
        };

        for (auto it = std::next(curve.m_vertices.begin()); it != curve.m_vertices.end(); ++it) {
            const CVertex& v = *it;

            // Compute segment start and end normal and tangent directions. Normalize length to offset
            double sTanX, sTanY;    // tangent vector at the start point
            double sNormX, sNormY;  // normal vector at the start point
            double eTanX, eTanY;    // tangent vector at the end point
            double eNormX, eNormY;  // normal vector at the end point
            double sRadius = 0;
            if (v.m_type == 0) {
                const double dx = v.m_p.x - pPrev.x;
                const double dy = v.m_p.y - pPrev.y;
                const double len = std::hypot(dx, dy);
                if (len == 0) {
                    continue;
                }
                std::tie(sTanX, sTanY) = std::make_pair(dx / len * offset, dy / len * offset);
                std::tie(sNormX, sNormY) = std::make_pair(sTanY, -sTanX);
                std::tie(eTanX, eTanY) = std::make_pair(sTanX, sTanY);
                std::tie(eNormX, eNormY) = std::make_pair(sNormX, sNormY);
            }
            else {
                assert(v.m_type == 1 || v.m_type == -1);

                // Compute dx and dy at start and end points, p - center
                const double sdx = v.m_type * (pPrev.x - v.m_c.x);
                const double sdy = v.m_type * (pPrev.y - v.m_c.y);
                const double edx = v.m_type * (v.m_p.x - v.m_c.x);
                const double edy = v.m_type * (v.m_p.y - v.m_c.y);

                // Compute the radius at the start and end points.
                // These are nominally equal, but they can differ a little due to precision issues,
                // and it is important to normalize each vector by the appropriate length
                sRadius = std::hypot(sdx, sdy);
                const double eRadius = std::hypot(edx, edy);
                if (sRadius == 0 || eRadius == 0) {
                    continue;
                }

                // Rescale the start and end normal vectors to length = offset
                sNormX = sdx / sRadius * offset;
                sNormY = sdy / sRadius * offset;
                eNormX = edx / eRadius * offset;
                eNormY = edy / eRadius * offset;

                sTanX = -sNormY;
                sTanY = sNormX;
                eTanX = -eNormY;
                eTanY = eNormX;
            }

            // Compute the start and end points of the offset segment
            const heeks::Point pPosS(pPrev.x + sNormX, pPrev.y + sNormY);
            const heeks::Point pNegS(pPrev.x - sNormX, pPrev.y - sNormY);
            const heeks::Point pPosE(v.m_p.x + eNormX, v.m_p.y + eNormY);
            const heeks::Point pNegE(v.m_p.x - eNormX, v.m_p.y - eNormY);

            // If the output curves are empty, intialize the start point and direction
            const bool hasPrev = !cPos.m_vertices.empty();
            double exitQ = v.m_type == 0 ? 0 : v.m_type / sRadius;
            if (!hasPrev) {
                cPos.m_vertices.emplace_back(0, pPosS, Point(0, 0));
                cNeg.m_vertices.emplace_back(0, pNegS, Point(0, 0));
                std::tie(startDirX, startDirY) = std::make_pair(sTanX, sTanY);
                startQex = exitQ;
            }

            // Join from the previous segment, if there is one
            if (hasPrev) {
                addJoin(pPosS, pNegS, pPrev, prevDirX, prevDirY, sTanX, sTanY, enterQ, exitQ);
            }
            enterQ = v.m_type == 0 ? 0 : v.m_type / sRadius;

            // Generate the positive and negative offset segments connecting pPosS to pPosE and
            // pNegS to pNegE
            if (v.m_type == 0) {
                cPos.m_vertices.emplace_back(0, pPosE, heeks::Point {0, 0});
                cNeg.m_vertices.emplace_back(0, pNegE, heeks::Point {0, 0});
            }
            else {
                // Check if the offset causes the arc to collapse; if so, it needs special handling.
                // Offsetting by -R reduces the arc to a point at the arc's center. Further
                // offsetting should have the same effect as offsetting that point (connecting
                // segments back to the point).
                assert(v.m_type == 1 || v.m_type == -1);

                const bool posCollapse = sRadius + (offset * v.m_type) <= 0;
                if (posCollapse) {
                    cPos.m_vertices.emplace_back(0, v.m_c, heeks::Point {0, 0});
                    cPos.m_vertices.emplace_back(0, pPosE, heeks::Point {0, 0});
                }
                else {
                    cPos.m_vertices.emplace_back(v.m_type, pPosE, v.m_c);
                }

                const bool negCollapse = sRadius - (offset * v.m_type) <= 0;
                if (negCollapse) {
                    cNeg.m_vertices.emplace_back(0, v.m_c, heeks::Point {0, 0});
                    cNeg.m_vertices.emplace_back(0, pNegE, heeks::Point {0, 0});
                }
                else {
                    cNeg.m_vertices.emplace_back(v.m_type, pNegE, v.m_c);
                }
            }

            // Update state variables
            pPrev = v.m_p;
            std::tie(prevDirX, prevDirY) = std::make_pair(eTanX, eTanY);
        }

        // Post processing
        if (curve.IsClosed()) {
            // Add the final join back to the start
            const heeks::Point pPosStart = cPos.m_vertices.front().m_p;
            const heeks::Point pNegStart = cNeg.m_vertices.front().m_p;
            addJoin(pPosStart, pNegStart, pPrev, prevDirX, prevDirY, startDirX, startDirY, enterQ, startQex);

            // curve.IsClosed() allows for start/end mismatch by some tolerance, but we really want
            // to produce a curve that is actually closed here. Coerce the start point to match the
            // end point (moving it by at most that tolerance).
            cPos.m_vertices.front().m_p = cPos.m_vertices.back().m_p;
            cNeg.m_vertices.front().m_p = cNeg.m_vertices.back().m_p;

            // Reverse the negative path so together cPos and cNeg enclose the area within `offset`
            // of the original curve
            cNeg.Reverse();

            // Save positive offset to the output list
            for (auto it = std::next(cPos.m_vertices.begin()); it != cPos.m_vertices.end(); ++it) {
                cPos.m_edgeTags.push_back(1);
            }
            offset_curves.push_back(cPos);

            // Save the negative offset to the output list
            for (auto it = std::next(cNeg.m_vertices.begin()); it != cNeg.m_vertices.end(); ++it) {
                cNeg.m_edgeTags.push_back(-1);
            }
            offset_curves.push_back(cNeg);
        }
        else {
            // Reverse cNeg so it's correctly oriented to join with cPos and the end caps
            cNeg.Reverse();

            // Tag positive offset edges
            while (cPos.m_edgeTags.size() < cPos.m_vertices.size() - 1) {
                cPos.m_edgeTags.push_back(1);
            }

            // Append the first end cap
            cPos.m_vertices.emplace_back(1, cNeg.m_vertices.begin()->m_p, curve.m_vertices.back().m_p);
            cPos.m_edgeTags.push_back(0);

            // Concatenate cNeg
            for (auto it = std::next(cNeg.m_vertices.begin()); it != cNeg.m_vertices.end(); ++it) {
                cPos.m_vertices.push_back(*it);
                cPos.m_edgeTags.push_back(-1);
            }

            // Append the final end cap
            cPos.m_vertices.emplace_back(1, cPos.m_vertices.begin()->m_p, curve.m_vertices.front().m_p);
            cPos.m_edgeTags.push_back(0);

            // Save results to the output list
            offset_curves.push_back(cPos);
        }
    }

    m_curves = std::move(offset_curves);
}

// Convert the input CCurve to clipper, populating metadata.
//
// Edge tags are read from curve.m_edgeTags. If that list is empty, all edges
// are treated as if they were tagged 1 (positive offset edge).
Path64 CArea::MakePoly(const CCurve& curve, ConversionMetadata& metadata) const
{
    if (!curve.m_vertices.size()) {
        return {};
    }

    Path64 result;
    const int curveIndex = metadata.nextCurveIndex++;

    // Helper function to convert to clipper units, handling z caching
    auto getPoint64 = [&](double x, double y) -> Point64 {
        const Point64 p64 = ToPoint64(PointD(x, y, 0));
        const auto key = std::make_pair(p64.x, p64.y);
        auto it = metadata.xy_to_z.find(key);
        if (it != metadata.xy_to_z.end()) {
            return Point64(p64.x, p64.y, it->second);
        }
        const int64_t z = metadata.z_next++;
        metadata.xy_to_z[key] = z;
        metadata.z_to_xy[z] = key;
        return Point64(p64.x, p64.y, z);
    };

    // Init the start of the curve
    Point64 pPrev = getPoint64(curve.m_vertices.front().m_p.x, curve.m_vertices.front().m_p.y);
    result.push_back(pPrev);
    heeks::Point ptPrev = curve.m_vertices.front().m_p;
    assert(curve.m_edgeTags.empty() || curve.m_edgeTags.size() == curve.m_vertices.size() - 1);
    auto tagIt = curve.m_edgeTags.cbegin();
    int vertexIndex = 0;

    // Iterate through edges
    for (auto vIt = std::next(curve.m_vertices.cbegin()); vIt != curve.m_vertices.cend(); vIt++) {
        CVertex vertex = *vIt;
        const bool isLoop = std::next(vIt) == curve.m_vertices.end() && curve.IsClosed();
        if (isLoop) {
            // IsClosed() uses tolerance-based heeks::Point equality. If the last vertex doesn't
            // exactly match the first, processing that vertex unmodified will create a new/unique
            // z coordinate to "close" the curve, and fail to correctly record metadata for the
            // actual edge back to the start point. To fix this, we coerce the end point to exactly
            // equal the start point.
            vertex.m_p = curve.m_vertices.front().m_p;
        }
        const int edgeTag = tagIt != curve.m_edgeTags.cend() ? *tagIt : 1;

        if (vertex.m_type == 0) {
            // The current edge is a line segment; add a single point to clipper

            Point64 newPt = getPoint64(vertex.m_p.x, vertex.m_p.y);
            if (!isLoop) {
                // Clipper paths are implicitly closed, so only explicitly add non-loop edges
                result.push_back(newPt);
            }

            // Save metadata for the new segment
            const auto key = std::make_pair(std::min(pPrev.z, newPt.z), std::max(pPrev.z, newPt.z));
            metadata.edgeData[key] = SegmentData {vertex, edgeTag, curveIndex, vertexIndex};
            metadata.edges[pPrev.z].push_back(newPt.z);
            metadata.edges[newPt.z].push_back(pPrev.z);
            pPrev = newPt;
        }
        else if (!vertex.m_p.exactlyEquals(ptPrev)) {
            // The current edge is an arc; interpolate many lines in clipper
            assert(vertex.m_type == 1 || vertex.m_type == -1);

            // Compute start and end angles
            const double phi0 = atan2(ptPrev.y - vertex.m_c.y, ptPrev.x - vertex.m_c.x);
            double phi1 = atan2(vertex.m_p.y - vertex.m_c.y, vertex.m_p.x - vertex.m_c.x);

            if (vertex.m_type == -1 && phi1 > phi0) {
                // fix to make it clockwise
                phi1 -= 2 * M_PI;
            }
            else if (vertex.m_type == 1 && phi1 < phi0) {
                // fix to make it counterclockwise
                phi1 += 2 * M_PI;
            }

            // Compute the maximum angular step to achieve the required accuracy
            const double dx = ptPrev.x - vertex.m_c.x;
            const double dy = ptPrev.y - vertex.m_c.y;
            const double radius = sqrt(dx * dx + dy * dy);
            const double max_dphi = 2 * acos((radius - CArea::m_accuracy) / radius);

            // Determine the number of segments
            const int num_segments
                = std::max(min_arc_points, (int)ceil(std::abs(phi1 - phi0) / max_dphi));
            const double dphi = (phi1 - phi0) / num_segments;

            // Generate arc points
            for (int i = 1; i <= num_segments; i++) {
                Point64 newPt;
                if (i == num_segments) {
                    // Final segment is special; use the specified endpoint instead of recomputing it
                    newPt = getPoint64(vertex.m_p.x, vertex.m_p.y);
                    if (newPt == pPrev) {
                        continue;
                    }
                    if (!isLoop) {
                        // Clipper paths are implicitly closed, so only explicitly add non-loop edges
                        result.push_back(newPt);
                    }
                }
                else {
                    // Compute the interpoalted point
                    const double px = vertex.m_c.x + radius * cos(phi0 + dphi * i);
                    const double py = vertex.m_c.y + radius * sin(phi0 + dphi * i);
                    newPt = getPoint64(px, py);
                    if (newPt == pPrev) {
                        continue;
                    }
                    result.push_back(newPt);
                }

                const auto key = std::make_pair(std::min(pPrev.z, newPt.z), std::max(pPrev.z, newPt.z));
                metadata.edgeData[key] = SegmentData {vertex, edgeTag, curveIndex, vertexIndex};
                metadata.edges[pPrev.z].push_back(newPt.z);
                metadata.edges[newPt.z].push_back(pPrev.z);
                pPrev = newPt;
            }
        }

        ptPrev = vertex.m_p;
        vertexIndex++;
        if (tagIt != curve.m_edgeTags.cend()) {
            tagIt++;
        }
    }

    return result;
}


// In getParentMetadataFallback, we may need to reconstruct a fake parent node for unknown parent
// edges (fall back to connecting with a line). We use this tag sentinel value in that case.
const int tagSentinel = -2;

// Convert the provided clipper paths back to CArea/CCurve data, using metadata to correctly
// infer edge type (arc/line) and arc center information. Only edges tagged 1 (i.e. positive offset
// segments from NaiveOffset) are kept in `this` CArea. If cNeg is provided, edges tagged -1 are
// kept there. Edges tagged 0 (end caps from NaiveOffset) are always dropped.
//
// Parameter isClosed specifies if the clipper paths represent open or closed curves. If the curves
// are open, they will be reoriented/ordered using metadata to preserve the original ordering and
// orientation.
void CArea::SetFromResult(
    Paths64& paths,
    bool isClosed,
    ConversionMetadata& metadata,
    std::optional<std::reference_wrapper<CArea>> cNeg
)
{
    // Reorder/reorient open paths
    if (!isClosed) {
        ReorderOpenPaths(paths, metadata);
    }

    // Convert each path back to a CCurve
    for (const Path64& path : paths) {
        if (!path.size()) {
            continue;
        }

        // Preserve single vertex paths. This requires special handling because the code below
        // expects/processes edges, not vertices
        if (path.size() == 1) {
            const PointD pt = ToPointD(path[0]);
            CCurve c;
            c.m_vertices.emplace_back(heeks::Point {pt.x, pt.y});
            m_curves.push_back(c);
            continue;
        }

        // Initialize state variables: the current curve and its tag, and (for final joining of
        // closed curves) the first curve and its tag.
        CCurve c;
        int tag = tagSentinel;
        CCurve* firstCurve = nullptr;
        std::optional<int> firstTag;

        // Helper function to save the current curve to the appropriate CArea when done with it,
        // and update firstTag/firstCurve variables
        auto saveCurve = [&]() {
            if (!c.m_vertices.empty()) {
                CCurve* added = nullptr;

                if (tag == 1 || tag == tagSentinel) {
                    m_curves.push_back(c);
                    added = &m_curves.back();
                }
                else if (tag == -1 && cNeg) {
                    cNeg->get().m_curves.push_back(c);
                    added = &cNeg->get().m_curves.back();
                }

                if (!firstTag) {
                    firstTag = tag;
                    firstCurve = added;
                }
            }
        };

        // For closed paths, start at the smallest z-value of a segment that won't be skipped
        size_t startVertex = 0;
        const int skipDx = 2;
        const int skipDy = skipDx;
        if (isClosed) {
            bool bestSkip = true;
            for (size_t i = 0; i < path.size(); i++) {
                const Point64& v0 = path[i];
                const Point64& v1 = path[(i + 1) % path.size()];

                bool isSkip = std::abs(v1.x - v0.x) < skipDx && std::abs(v1.y - v0.y) < skipDy;
                if (isSkip < bestSkip || (isSkip <= bestSkip && path[i].z <= path[startVertex].z)) {
                    startVertex = i;
                    bestSkip = isSkip;
                }
            }
        }

        // Loop through clipper edges, converting to CVertex and building up the current CCurve
        for (size_t edgeNum = 0; edgeNum < (isClosed ? path.size() : path.size() - 1); edgeNum++) {
            // Current edge
            const size_t iEdge = (startVertex + edgeNum) % path.size();
            const Point64& v0 = path[iEdge];
            const Point64& v1 = path[(iEdge + 1) % path.size()];
            const PointD endD = ToPointD(v1);
            const heeks::Point end = {endD.x, endD.y};

            // If length is tiny, skip the edge. This is important because clipper sometimes
            // silently merges points that are only 1 unit away from each other (Clipper2 issue
            // #1111), and this can result in incorrect tags on segments that short. Fortunately, it
            // is acceptable to skip such short segments because it changes the output very little.
            //
            // When skipping the edge, amend the previous vertex to end at the new end location to
            // keep the curve closed. This process may require extra handling if the adjustment
            // terminates the segment back at its start point.
            if (std::abs(v1.x - v0.x) < skipDx && std::abs(v1.y - v0.y) < skipDy) {
                if (c.m_vertices.size()) {
                    const bool fullLoop = c.m_vertices.size() >= 2
                        && std::prev(c.m_vertices.end(), 2)->m_p.exactlyEquals(end);
                    if (!fullLoop) {
                        c.m_vertices.back().m_p = end;
                    }
                    else if (c.m_vertices.back().m_type == 0) {
                        // Collapsed a line to its start point -- delete the vertex
                        c.m_vertices.pop_back();
                    }
                    else {
                        // Completed an arc -- change the representation to two semi-circles
                        CVertex& prev = c.m_vertices.back();
                        const heeks::Point mid {2 * prev.m_c.x - end.x, 2 * prev.m_c.y - end.y};
                        prev.m_p = mid;
                        c.m_vertices.emplace_back(prev.m_type, end, prev.m_c);
                    }
                }
                continue;
            }

            // Look up the segment data of the parent edge. Check for and handle the tag sentinel
            // value. The sentinel value is provided only when the parent edge lookup fails.
            // We handle this by assuming the tag is unchanged.
            // If the full curve is completed without any non-sentinel tags, it is treated as tag 1
            SegmentData parentData = getParentMetadata(v0, v1, metadata);
            if (parentData.edgeTag == tagSentinel) {
                parentData.edgeTag = tag;
            }
            if (tag == tagSentinel) {
                tag = parentData.edgeTag;
            }


            // Check if the tag changed. If it did, end the curve and start a new one
            if (parentData.edgeTag != tag) {
                saveCurve();
                c.m_vertices.clear();
            }
            tag = parentData.edgeTag;

            // If the curve is empty, initialize it with the start point
            if (c.m_vertices.empty()) {
                const PointD start = ToPointD(v0);
                c.m_vertices.emplace_back(heeks::Point {start.x, start.y});
            }

            // Construct the edge to be added based on the end point and the parent's type
            CVertex edge(parentData.orig.m_type, {end.x, end.y}, parentData.orig.m_c);
            if (!CArea::m_fit_arcs) {
                edge.m_type = 0;
                edge.m_c = {0, 0};
            }
            CVertex& prev = c.m_vertices.back();

            // Determine if the edge is reversed from the parent (arc), and update type accordingly
            if (edge.m_type == 1 || edge.m_type == -1) {
                const Point64 mc64 = ToPoint64(PointD(parentData.orig.m_c.x, parentData.orig.m_c.y, 0));
                const double phi1 = atan2(v0.y - mc64.y, v0.x - mc64.x);
                const double phi2 = recenter(atan2(v1.y - mc64.y, v1.x - mc64.x), phi1 - M_PI, 1);
                if (phi2 * edge.m_type < phi1 * edge.m_type) {
                    edge.m_type = -edge.m_type;
                }
            }

            // Check if the edge is a continuation of an existing arc
            if (edge.m_type != 0 && edge.m_type == prev.m_type && edge.m_c == prev.m_c) {
                // It is. If the edge does not complete a circle, we should extend the existing
                // CVertex instead of adding a new one.
                const bool fullLoop = std::prev(c.m_vertices.end(), 2)->m_p.exactlyEquals(edge.m_p);
                if (!fullLoop) {
                    prev.m_p = edge.m_p;
                }
                else {
                    // The edge cannot be extended, because it would complete a circle and CVertex
                    // arcs are supposed to be less than a full circle. Instead, represent the full
                    // circle as 2 semi circles
                    const heeks::Point mid {2 * edge.m_c.x - edge.m_p.x, 2 * edge.m_c.y - edge.m_p.y};
                    prev.m_p = mid;
                    c.m_vertices.push_back(edge);
                }
            }
            else {
                // The edge is not an extension of the previous CVertex; just add it
                c.m_vertices.push_back(edge);
            }
        }

        // Save the final curve
        if (isClosed && firstCurve && firstTag && tag == *firstTag) {
            // Save the curve by joining it with the (distinct!) first curve

            // Remove the first curve's (now redundant) start point
            firstCurve->m_vertices.pop_front();

            // Check if the first CVertex of the first curve is an extension of the last CVertex of
            // the current curve, and if so deduplicate them
            CVertex& edge = firstCurve->m_vertices.front();
            CVertex& prev = c.m_vertices.back();
            if (edge.m_type != 0 && edge.m_type == prev.m_type && edge.m_c == prev.m_c) {
                // It is an extension
                const bool fullLoop = std::prev(c.m_vertices.end(), 2)->m_p.exactlyEquals(edge.m_p);
                if (!fullLoop) {
                    prev.m_p = edge.m_p;
                    firstCurve->m_vertices.pop_front();
                }
                else {
                    // Full circle; represent it as 2 semi circles
                    const heeks::Point mid {2 * edge.m_c.x - edge.m_p.x, 2 * edge.m_c.y - edge.m_p.y};
                    prev.m_p = mid;
                }
            }

            // ...and finally concatenate them
            firstCurve->m_vertices
                .insert(firstCurve->m_vertices.begin(), c.m_vertices.begin(), c.m_vertices.end());
        }
        else if (!firstTag && isClosed && c.m_vertices.size() >= 3) {
            // Same as above, but the first curve has not been saved yet because the current curve
            // *is* the first curve. Merging the curve to itself requires some special handling

            // First check if the first CVertex of the curve can extend the last CVertex
            CVertex& first = *std::next(c.m_vertices.begin());
            CVertex& last = c.m_vertices.back();
            if (last.m_type != 0 && last.m_type == first.m_type && last.m_c == first.m_c) {
                // It is an extension
                const bool fullLoop = std::prev(c.m_vertices.end(), 2)->m_p.exactlyEquals(first.m_p);
                if (!fullLoop) {
                    c.m_vertices.front().m_p = std::prev(c.m_vertices.end(), 2)->m_p;
                    c.m_vertices.pop_back();
                }
                else {
                    // Break it into 2 semi circles
                    const heeks::Point mid {
                        2 * first.m_c.x - first.m_p.x,
                        2 * first.m_c.y - first.m_p.y
                    };
                    last.m_p = mid;
                    c.m_vertices.front().m_p = mid;
                }
            }

            // Save it as a new curve
            saveCurve();
        }
        else {
            // Save it as a new curve
            saveCurve();
        }
    }

    CorrectArcCenters();
}

void CArea::CorrectArcCenters()
{
    for (CCurve& curve : m_curves) {
        auto it = curve.m_vertices.begin();
        if (it == curve.m_vertices.end()) {
            continue;
        }

        // Initialize prevPt for our loop over edges
        Point prevPt = it->m_p;
        ++it;

        // Loop over edges
        for (; it != curve.m_vertices.end(); ++it) {
            CVertex& v = *it;
            if (v.m_type != 0) {
                // Arc: start=prevPt, end=v.m_p, center=v.m_c

                // Check if the arc is out of tolerance
                const double r1 = std::hypot(prevPt.x - v.m_c.x, prevPt.y - v.m_c.y);
                const double r2 = std::hypot(v.m_p.x - v.m_c.x, v.m_p.y - v.m_c.y);
                const double d = std::hypot(v.m_p.x - prevPt.x, v.m_p.y - prevPt.y);
                if (d < Precision::Confusion() || fabs(r2 - r1) < Precision::Confusion()) {
                    prevPt = v.m_p;
                    continue;
                }

                // Compute the chord perpendicular bisector
                const double nx = (prevPt.y - v.m_p.y) / d;
                const double ny = (v.m_p.x - prevPt.x) / d;
                const double mx = (prevPt.x + v.m_p.x) * 0.5;
                const double my = (prevPt.y + v.m_p.y) * 0.5;

                // Project the arc center onto the perpendicular bisector
                const double dot = (v.m_c.x - mx) * nx + (v.m_c.y - my) * ny;
                const double new_cx = mx + dot * nx;
                const double new_cy = my + dot * ny;

                v.m_c.x = new_cx;
                v.m_c.y = new_cy;
            }

            prevPt = v.m_p;
        }
    }
}


// ---------------------------------------------------------------------------------------------
// Snap-based closed offset
//
// Geometry is computed by Clipper2's native polygon offset (ClipperOffset, round joins) on a
// discretized copy of the input. No metadata is carried through Clipper: Clipper is free to
// merge, drop or create vertices, and the result is always a valid region.
//
// Arcs are then recovered by snapping the output to the only circles that can legitimately
// appear in an offset result:
//   - every input arc (center c, radius r) offset to radii r + |d| and r - |d|
//   - the round join around every input vertex (center = vertex, radius |d|)
// A run of consecutive output edges whose vertices all lie on one candidate circle (within
// m_accuracy), turning consistently in one direction, becomes a single arc with that circle's
// exact center and radius. Anything that does not snap stays as line segments.
//
// Failure mode: a missed snap yields line segments (still within tolerance of the true offset),
// never an invalid or wrong-sized region. A wrong snap is bounded too, because every snapped edge
// must be a chord of the candidate circle with sagitta <= m_accuracy.
// ---------------------------------------------------------------------------------------------

namespace
{

// Angular sector of a candidate circle: start angle and signed sweep (CCW positive), plus a slack
// angle on both ends for discretization. An offset arc can only appear within the sector of the
// input arc it comes from, and a round join only within the turn at its corner. Without this, two
// nearly identical circles (e.g. tangent input arcs of similar radius) could both match long
// stretches of output, and a run could follow the wrong circle past its end.
struct SnapSector
{
    double a0, sweep, slack;
};

bool inSector(double ang, const SnapSector& sec)
{
    if (std::fabs(sec.sweep) + 2 * sec.slack >= 2 * M_PI) {
        return true;
    }
    double t = sec.sweep >= 0 ? ang - sec.a0 : sec.a0 - ang;
    t = std::fmod(t + sec.slack, 2 * M_PI);
    if (t < 0) {
        t += 2 * M_PI;
    }
    return t - sec.slack <= std::fabs(sec.sweep) + sec.slack;
}

bool inAnySector(double x, double y, double cx, double cy, const std::vector<SnapSector>& secs)
{
    if (secs.empty()) {
        return true;
    }
    const double ang = std::atan2(y - cy, x - cx);
    for (const SnapSector& sec : secs) {
        if (inSector(ang, sec)) {
            return true;
        }
    }
    return false;
}

struct SnapCircle
{
    double cx, cy, r;
    double xmin, ymin, xmax, ymax;  // bbox inflated by tolerance, for cheap rejection
    std::vector<SnapSector> sectors;
};

class SnapIndex
{
public:
    SnapIndex(double tol)
        : m_tol(tol)
    {}

    // Arc candidates: few, arbitrary radii. Input arcs on the same circle share one candidate
    // with several sectors, so a run can continue across them.
    void addCircle(double cx, double cy, double r, const SnapSector& sector)
    {
        if (r <= 2 * m_tol) {
            return;
        }
        for (SnapCircle& c : m_circles) {
            if (std::fabs(c.cx - cx) < 1e-9 && std::fabs(c.cy - cy) < 1e-9
                && std::fabs(c.r - r) < 1e-9) {
                c.sectors.push_back(sector);
                return;
            }
        }
        m_circles.push_back(
            {cx, cy, r, cx - r - m_tol, cy - r - m_tol, cx + r + m_tol, cy + r + m_tol, {sector}}
        );
    }

    // Join candidates: many, all the same radius. Stored in a uniform grid.
    void setJoinRadius(double r)
    {
        m_joinR = r;
        m_cell = r + m_tol;
    }
    // corner: the vertex joins two long input edges (a real corner rather than a vertex of a
    // densely sampled polyline); its round join is kept as an arc however small it is
    // sectors: where the join can appear (empty: anywhere)
    void addJoinCenter(double cx, double cy, bool corner, std::vector<SnapSector> sectors)
    {
        if (m_joinR <= 2 * m_tol) {
            return;
        }
        m_joins.push_back({cx, cy});
        m_corner.push_back(corner);
        m_joinSectors.push_back(std::move(sectors));
        m_grid[cellKey(cx, cy)].push_back(static_cast<int>(m_joins.size() - 1));
    }

    size_t numCircles() const
    {
        return m_circles.size();
    }

    bool isCornerJoin(int id) const
    {
        return id >= static_cast<int>(m_circles.size()) && m_corner[id - m_circles.size()];
    }

    // Candidate ids near p. Ids < numCircles() are arc circles, the rest are joins.
    void query(double x, double y, std::vector<int>& out) const
    {
        out.clear();
        for (size_t i = 0; i < m_circles.size(); ++i) {
            const SnapCircle& c = m_circles[i];
            if (x < c.xmin || x > c.xmax || y < c.ymin || y > c.ymax) {
                continue;
            }
            if (std::fabs(std::hypot(x - c.cx, y - c.cy) - c.r) <= m_tol
                && inAnySector(x, y, c.cx, c.cy, c.sectors)) {
                out.push_back(static_cast<int>(i));
            }
        }
        if (m_joins.empty()) {
            return;
        }
        const int64_t ix = cellIndex(x), iy = cellIndex(y);
        for (int64_t gx = ix - 1; gx <= ix + 1; ++gx) {
            for (int64_t gy = iy - 1; gy <= iy + 1; ++gy) {
                auto it = m_grid.find({gx, gy});
                if (it == m_grid.end()) {
                    continue;
                }
                for (int j : it->second) {
                    const PointD& c = m_joins[j];
                    if (std::fabs(std::hypot(x - c.x, y - c.y) - m_joinR) <= m_tol
                        && inAnySector(x, y, c.x, c.y, m_joinSectors[j])) {
                        out.push_back(static_cast<int>(m_circles.size()) + j);
                    }
                }
            }
        }
        std::sort(out.begin(), out.end());
    }

    void get(int id, double& cx, double& cy, double& r) const
    {
        if (id < static_cast<int>(m_circles.size())) {
            const SnapCircle& c = m_circles[id];
            cx = c.cx;
            cy = c.cy;
            r = c.r;
        }
        else {
            const PointD& c = m_joins[id - m_circles.size()];
            cx = c.x;
            cy = c.y;
            r = m_joinR;
        }
    }

private:
    int64_t cellIndex(double v) const
    {
        return static_cast<int64_t>(std::floor(v / m_cell));
    }
    std::pair<int64_t, int64_t> cellKey(double x, double y) const
    {
        return {cellIndex(x), cellIndex(y)};
    }

    double m_tol;
    double m_joinR = 0;
    double m_cell = 1;
    std::vector<SnapCircle> m_circles;
    std::vector<PointD> m_joins;
    std::vector<bool> m_corner;
    std::vector<std::vector<SnapSector>> m_joinSectors;
    std::map<std::pair<int64_t, int64_t>, std::vector<int>> m_grid;
};

// Discretize a closed CCurve into a Clipper path (no z data, no closing duplicate).
// Arcs are split so the chord deviation stays below discTol.
Path64 closedCurveToPath64(const CCurve& curve, double discTol)
{
    Path64 path;
    auto push = [&](double x, double y) {
        const Point64 p = ToPoint64(PointD(x, y, 0));
        if (path.empty() || path.back().x != p.x || path.back().y != p.y) {
            path.push_back(p);
        }
    };

    bool first = true;
    heeks::Point prev(0, 0);
    for (const CVertex& v : curve.m_vertices) {
        if (first) {
            first = false;
            push(v.m_p.x, v.m_p.y);
            prev = v.m_p;
            continue;
        }
        if (v.m_type == 0) {
            push(v.m_p.x, v.m_p.y);
        }
        else {
            const double r = std::hypot(prev.x - v.m_c.x, prev.y - v.m_c.y);
            const double a0 = std::atan2(prev.y - v.m_c.y, prev.x - v.m_c.x);
            double a1 = std::atan2(v.m_p.y - v.m_c.y, v.m_p.x - v.m_c.x);
            if (v.m_type > 0 && a1 <= a0) {
                a1 += 2 * M_PI;
            }
            if (v.m_type < 0 && a1 >= a0) {
                a1 -= 2 * M_PI;
            }
            double step = M_PI / 2;
            if (r > discTol) {
                step = std::min(step, 2 * std::acos(1 - discTol / r));
            }
            const int n = std::max(1, static_cast<int>(std::ceil(std::fabs(a1 - a0) / step)));
            for (int i = 1; i < n; ++i) {
                const double a = a0 + (a1 - a0) * i / n;
                push(v.m_c.x + r * std::cos(a), v.m_c.y + r * std::sin(a));
            }
            push(v.m_p.x, v.m_p.y);
        }
        prev = v.m_p;
    }
    if (path.size() > 1 && path.front().x == path.back().x && path.front().y == path.back().y) {
        path.pop_back();
    }
    return path;
}

struct EdgeMatch
{
    int id;      // candidate circle id
    int dir;     // +1 CCW, -1 CW
    double ang;  // absolute sweep of the edge around the circle
    double err;  // max radial error of the edge's two vertices
};

struct SnapSeg
{
    bool arc;
    int id, dir;
    size_t first, count;  // edges [first, first + count) (cyclic)
};

// Closest point on circle (cx, cy, r) to p
heeks::Point projectToCircle(double cx, double cy, double r, const heeks::Point& p)
{
    const double dx = p.x - cx, dy = p.y - cy;
    const double l = std::hypot(dx, dy);
    if (l < 1e-12) {
        return heeks::Point(cx + r, cy);
    }
    return heeks::Point(cx + dx * r / l, cy + dy * r / l);
}

// Intersection of two circles closest to p (or their tangent point, if they miss by <= gap)
std::optional<heeks::Point> circleIntersection(
    double x1,
    double y1,
    double r1,
    double x2,
    double y2,
    double r2,
    const heeks::Point& p,
    double gap
)
{
    const double dx = x2 - x1, dy = y2 - y1;
    const double d = std::hypot(dx, dy);
    // Tangent circles (an offset arc meeting a round join, or the offsets of two tangent input
    // arcs) rarely intersect exactly in floating point, and real input is often only tangent to
    // within its own precision. Accept a gap up to `gap` and use the tangent point.
    if (d < 1e-12 || d > r1 + r2 + gap || d < std::fabs(r1 - r2) - gap) {
        return std::nullopt;
    }
    const double a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
    const double h = std::sqrt(std::max(0.0, r1 * r1 - a * a));
    const double mx = x1 + a * dx / d, my = y1 + a * dy / d;
    const heeks::Point s1(mx - h * dy / d, my + h * dx / d);
    const heeks::Point s2(mx + h * dy / d, my - h * dx / d);
    const double d1 = std::hypot(s1.x - p.x, s1.y - p.y);
    const double d2 = std::hypot(s2.x - p.x, s2.y - p.y);
    return d1 <= d2 ? s1 : s2;
}

// Move each arc's center (minimally, along the chord's perpendicular bisector) so both endpoints
// are exactly equidistant from it. Merged junctions can leave a radial mismatch of up to ~1e-6;
// this is what Area::toShape() would otherwise do itself, with an "Arc correction" warning.
void equalizeArcs(CCurve& c)
{
    heeks::Point prev(0, 0);
    bool first = true;
    for (CVertex& v : c.m_vertices) {
        if (!first && v.m_type != 0) {
            const double ex = v.m_p.x - prev.x, ey = v.m_p.y - prev.y;
            const double L = std::hypot(ex, ey);
            if (L > 0) {
                const double mx = (prev.x + v.m_p.x) / 2, my = (prev.y + v.m_p.y) / 2;
                const double ux = -ey / L, uy = ex / L;  // unit normal of the chord
                const double t = (v.m_c.x - mx) * ux + (v.m_c.y - my) * uy;
                v.m_c = heeks::Point(mx + t * ux, my + t * uy);
            }
        }
        first = false;
        prev = v.m_p;
    }
}

// Convert one closed Clipper output path into a CCurve, snapping runs to candidate circles
CCurve snapClosedPath(
    const Path64& path,
    const SnapIndex& index,
    double tol,
    double minSagitta,
    bool fitArcs
)
{
    const size_t n = path.size();
    std::vector<heeks::Point> pts(n);
    for (size_t i = 0; i < n; ++i) {
        const PointD p = ToPointD(path[i]);
        pts[i] = heeks::Point(p.x, p.y);
    }

    auto linesOnly = [&]() {
        CCurve c;
        for (const heeks::Point& p : pts) {
            c.m_vertices.emplace_back(p);
        }
        c.m_vertices.emplace_back(pts[0]);
        return c;
    };
    if (!fitArcs || n < 3) {
        return linesOnly();
    }

    // Candidates per vertex, then per edge
    std::vector<std::vector<int>> vc(n);
    for (size_t i = 0; i < n; ++i) {
        index.query(pts[i].x, pts[i].y, vc[i]);
    }
    std::vector<std::vector<EdgeMatch>> em(n);
    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1) % n;
        const heeks::Point& a = pts[i];
        const heeks::Point& b = pts[j];
        const double L = std::hypot(b.x - a.x, b.y - a.y);
        for (int id : vc[i]) {
            if (!std::binary_search(vc[j].begin(), vc[j].end(), id)) {
                continue;
            }
            double cx, cy, r;
            index.get(id, cx, cy, r);
            // The edge must be a short chord of the circle: sagitta within tolerance
            if (L >= 2 * r || r - std::sqrt(r * r - L * L / 4) > tol) {
                continue;
            }
            const double ax = a.x - cx, ay = a.y - cy, bx = b.x - cx, by = b.y - cy;
            const double cross = ax * by - ay * bx;
            if (cross == 0) {
                continue;
            }
            const double ang = std::fabs(std::atan2(cross, ax * bx + ay * by));
            const double err
                = std::max(std::fabs(std::hypot(ax, ay) - r), std::fabs(std::hypot(bx, by) - r));
            em[i].push_back({id, cross > 0 ? 1 : -1, ang, err});
        }
    }

    auto hasMatch = [&](size_t e, int id, int dir) -> const EdgeMatch* {
        for (const EdgeMatch& m : em[e]) {
            if (m.id == id && m.dir == dir) {
                return &m;
            }
        }
        return nullptr;
    };

    // An edge is not given to a circle if the round join of a real corner fits it clearly
    // better. Near a tangent point several circles fit within tolerance; this keeps a small
    // corner join from being swallowed by the neighbouring arcs.
    const double kFitFloor = 5.0 / CArea::m_clipper_scale;
    auto bestFit = [&](size_t e, const EdgeMatch& m) {
        if (m.err <= kFitFloor) {
            return true;
        }
        for (const EdgeMatch& o : em[e]) {
            if (o.id != m.id && index.isCornerJoin(o.id) && o.err * 4 < m.err) {
                return false;
            }
        }
        return true;
    };

    // Full circle: every edge on the same circle in the same direction
    for (const EdgeMatch& m0 : em[0]) {
        bool all = true;
        for (size_t e = 1; e < n && all; ++e) {
            all = hasMatch(e, m0.id, m0.dir) != nullptr;
        }
        if (all) {
            double cx, cy, r;
            index.get(m0.id, cx, cy, r);
            const heeks::Point p0 = projectToCircle(cx, cy, r, pts[0]);
            const heeks::Point pm(2 * cx - p0.x, 2 * cy - p0.y);
            CCurve c;
            c.m_vertices.emplace_back(p0);
            c.m_vertices.emplace_back(m0.dir, pm, heeks::Point(cx, cy));
            c.m_vertices.emplace_back(m0.dir, p0, heeks::Point(cx, cy));
            return c;
        }
    }

    // Start at an edge that cannot continue a run from the previous edge, so runs are not
    // split by the arbitrary start vertex Clipper picked
    size_t start = 0;
    for (size_t e = 0; e < n; ++e) {
        const size_t prevE = (e + n - 1) % n;
        bool continues = false;
        for (const EdgeMatch& m : em[e]) {
            if (hasMatch(prevE, m.id, m.dir)) {
                continues = true;
                break;
            }
        }
        if (!continues) {
            start = e;
            break;
        }
    }

    // Greedy segmentation: at each edge take the candidate giving the longest run
    std::vector<SnapSeg> segs;
    size_t done = 0;
    while (done < n) {
        const size_t e0 = (start + done) % n;
        SnapSeg best {false, -1, 0, e0, 1};
        double bestErr = 0;
        for (const EdgeMatch& m : em[e0]) {
            size_t count = 0;
            double sweep = 0, errSum = 0;
            while (done + count < n) {
                const size_t e = (e0 + count) % n;
                const EdgeMatch* mm = hasMatch(e, m.id, m.dir);
                if (!mm || !bestFit(e, *mm) || sweep + mm->ang >= 2 * M_PI - 1e-9) {
                    break;
                }
                sweep += mm->ang;
                errSum += mm->err;
                ++count;
            }
            if (count == 0) {
                continue;
            }
            // Only emit arcs that are distinguishable from their chord at the discretization
            // tolerance. Smaller "arcs" (tiny round joins, noise on dense input) are within
            // tolerance as lines, and emitting them only produces micro-arcs.
            double cx, cy, r;
            index.get(m.id, cx, cy, r);
            if (!index.isCornerJoin(m.id)
                && r * (1 - std::cos(std::min(sweep, M_PI) / 2)) <= minSagitta) {
                continue;
            }
            // Prefer the longest run; break ties by the smallest total radial error
            if (!best.arc || count > best.count || (count == best.count && errSum < bestErr)) {
                best = {true, m.id, m.dir, e0, count};
                bestErr = errSum;
            }
        }
        segs.push_back(best);
        done += best.count;
    }

    // Distance from q to the Clipper edges covered by segments A and B
    auto distToRuns = [&](const SnapSeg& A, const SnapSeg& B, const heeks::Point& q) {
        double best = 1e300;
        auto edgeDist = [&](size_t e) {
            const heeks::Point& a = pts[e];
            const heeks::Point& b = pts[(e + 1) % n];
            const double dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
            double t = l2 > 0 ? ((q.x - a.x) * dx + (q.y - a.y) * dy) / l2 : 0;
            t = std::clamp(t, 0.0, 1.0);
            best = std::min(best, std::hypot(q.x - (a.x + t * dx), q.y - (a.y + t * dy)));
        };
        for (size_t i = 0; i < A.count; ++i) {
            edgeDist((A.first + i) % n);
        }
        for (size_t i = 0; i < B.count; ++i) {
            edgeDist((B.first + i) % n);
        }
        return best;
    };

    // Junction between segment A (ending at vertex p) and segment B (starting at p).
    // Returns the end point for A and the start point for B; they differ only when two arcs
    // on non-intersecting circles meet, in which case a short connecting line is inserted.
    // Arc ends closer than this are merged into one point instead of being joined by a
    // micro-line; equalizeArcs() below then removes the (tiny) radial mismatch
    constexpr double kSameEnd = 1e-6;
    auto junction = [&](const SnapSeg& A, const SnapSeg& B, const heeks::Point& p) {
        double ax = 0, ay = 0, ar = 0, bx = 0, by = 0, br = 0;
        if (A.arc) {
            index.get(A.id, ax, ay, ar);
        }
        if (B.arc) {
            index.get(B.id, bx, by, br);
        }
        if (!A.arc && !B.arc) {
            return std::make_pair(p, p);
        }
        if (A.arc && !B.arc) {
            const heeks::Point q = projectToCircle(ax, ay, ar, p);
            return std::make_pair(q, q);
        }
        if (!A.arc && B.arc) {
            const heeks::Point q = projectToCircle(bx, by, br, p);
            return std::make_pair(q, q);
        }
        if (A.id == B.id) {
            const heeks::Point q = projectToCircle(ax, ay, ar, p);
            return std::make_pair(q, q);
        }
        if (auto q = circleIntersection(ax, ay, ar, bx, by, br, p, tol)) {
            // Accept the exact junction only if it lies on the Clipper boundary covered by the
            // two runs (near a tangent point the run switch can happen well before the junction)
            if (distToRuns(A, B, *q) <= tol) {
                // Put each arc end exactly on its own circle; for (near-)tangent circles the two
                // projections can differ by up to the tangent gap
                const heeks::Point qa = projectToCircle(ax, ay, ar, *q);
                const heeks::Point qb = projectToCircle(bx, by, br, *q);
                if (std::hypot(qa.x - qb.x, qa.y - qb.y) <= kSameEnd) {
                    return std::make_pair(qa, qa);
                }
                return std::make_pair(qa, qb);
            }
        }
        const heeks::Point qa = projectToCircle(ax, ay, ar, p);
        const heeks::Point qb = projectToCircle(bx, by, br, p);
        if (std::hypot(qa.x - qb.x, qa.y - qb.y) <= kSameEnd) {
            return std::make_pair(qa, qa);
        }
        return std::make_pair(qa, qb);
    };

    // Merge neighbouring arc segments on the same circle and direction (including across the
    // ring's start), e.g. when the start edge happened to fall inside an arc
    auto mergeable = [&](const SnapSeg& a, const SnapSeg& b) {
        return a.arc && b.arc && a.id == b.id && a.dir == b.dir && a.count + b.count < n;
    };
    {
        std::vector<SnapSeg> merged;
        for (const SnapSeg& sg : segs) {
            if (!merged.empty() && mergeable(merged.back(), sg)) {
                merged.back().count += sg.count;
            }
            else {
                merged.push_back(sg);
            }
        }
        if (merged.size() > 1 && mergeable(merged.back(), merged.front())) {
            merged.front().first = merged.back().first;
            merged.front().count += merged.back().count;
            merged.pop_back();
        }
        segs.swap(merged);
    }

    // Sweep actually covered by the edges of an arc segment
    auto runSweep = [&](const SnapSeg& s) {
        double sw = 0;
        for (size_t i = 0; i < s.count; ++i) {
            sw += hasMatch((s.first + i) % n, s.id, s.dir)->ang;
        }
        return sw;
    };
    // Sweep of the arc from a to b around the segment's circle, in the segment's direction
    auto arcSweep = [&](const SnapSeg& s, const heeks::Point& a, const heeks::Point& b) {
        double cx, cy, r;
        index.get(s.id, cx, cy, r);
        double a0 = std::atan2(a.y - cy, a.x - cx), a1 = std::atan2(b.y - cy, b.x - cx);
        double sw = s.dir > 0 ? a1 - a0 : a0 - a1;
        while (sw <= 0) {
            sw += 2 * M_PI;
        }
        while (sw > 2 * M_PI) {
            sw -= 2 * M_PI;
        }
        return sw;
    };

    // Compute joints; demote to lines any arc whose endpoints make it sweep differently from the
    // edges it replaces (e.g. a joint that landed past the end of a very short arc, which would
    // otherwise produce an arc going the long way around). Repeat until stable.
    size_t ns = 0;
    std::vector<std::pair<heeks::Point, heeks::Point>> joints;
    for (int iter = 0; iter < 8; ++iter) {
        ns = segs.size();
        joints.assign(ns, {});
        for (size_t k = 0; k < ns; ++k) {
            const SnapSeg& A = segs[(k + ns - 1) % ns];
            const SnapSeg& B = segs[k];
            joints[k] = junction(A, B, pts[B.first]);
        }
        bool demoted = false;
        std::vector<SnapSeg> next;
        for (size_t k = 0; k < ns; ++k) {
            const SnapSeg& s = segs[k];
            if (s.arc) {
                const double expect = runSweep(s);
                if (expect >= 2 * M_PI - 1e-6) {
                    // Would be a full circle in one CVertex; split in two halves by edge count
                    const size_t half = s.count / 2;
                    next.push_back({true, s.id, s.dir, s.first, half});
                    next.push_back({true, s.id, s.dir, (s.first + half) % n, s.count - half});
                    demoted = true;
                    continue;
                }
                const double got = arcSweep(s, joints[k].second, joints[(k + 1) % ns].first);
                // The exact arc can legitimately differ from Clipper's chords by about one
                // discretization step at each end; what must never happen is an arc running the
                // long way around its circle.
                if (std::fabs(got - expect) > 0.25 * expect + 0.2) {
                    for (size_t i = 0; i < s.count; ++i) {
                        next.push_back({false, -1, 0, (s.first + i) % n, 1});
                    }
                    demoted = true;
                    continue;
                }
            }
            next.push_back(s);
        }
        segs.swap(next);
        if (!demoted) {
            break;
        }
    }
    // Final joints for the final segmentation (also covers hitting the iteration cap)
    ns = segs.size();
    joints.assign(ns, {});
    for (size_t k = 0; k < ns; ++k) {
        joints[k] = junction(segs[(k + ns - 1) % ns], segs[k], pts[segs[k].first]);
    }

    CCurve c;
    c.m_vertices.emplace_back(joints[0].second);
    for (size_t k = 0; k < ns; ++k) {
        const SnapSeg& s = segs[k];
        const auto& endJoint = joints[(k + 1) % ns];
        if (s.arc) {
            double cx, cy, r;
            index.get(s.id, cx, cy, r);
            c.m_vertices.emplace_back(s.dir, endJoint.first, heeks::Point(cx, cy));
        }
        else {
            // Line segments: interior vertices of the run, then the end joint
            for (size_t i = 1; i < s.count; ++i) {
                c.m_vertices.emplace_back(pts[(s.first + i) % n]);
            }
            c.m_vertices.emplace_back(endJoint.first);
        }
        if (endJoint.first.x != endJoint.second.x || endJoint.first.y != endJoint.second.y) {
            c.m_vertices.emplace_back(endJoint.second);
        }
    }
    equalizeArcs(c);
    return c;
}

}  // namespace

void CArea::Offset(double offset)
{
    if (offset == 0) {
        return;
    }

    // Open curves need the tagged (one-sided) machinery; keep the existing implementation
    for (const CCurve& curve : m_curves) {
        if (curve.m_vertices.size() > 1 && !curve.IsClosed()) {
            OffsetNaive(offset);
            return;
        }
    }

    // Same orientation convention as NaiveOffset: CCW = positive area, but accept all-reversed
    if (GetArea() < 0) {
        for (CCurve& curve : m_curves) {
            curve.Reverse();
        }
    }

    // Genuine offset arcs come out of Clipper within discTol of their circle (input chords and
    // round-join steps are both generated at discTol), plus integer rounding. Snapping tolerance
    // is kept just above that so that noise is not mistaken for arcs.
    const double discTol = m_accuracy * 0.25;
    const double tol = discTol * 1.2 + 2.0 / m_clipper_scale;
    const double d = std::fabs(offset);

    // Candidate circles, from the input geometry
    SnapIndex index(tol);
    index.setJoinRadius(d);
    Paths64 paths;
    for (const CCurve& curve : m_curves) {
        if (curve.m_vertices.empty()) {
            continue;
        }
        // Real corners: vertices where the direction actually changes, between two input edges
        // that are long compared to the accuracy (as opposed to vertices of densely sampled
        // polylines, or tangent line/arc transitions, which produce no round join)
        std::vector<CVertex> vs(curve.m_vertices.begin(), curve.m_vertices.end());
        const bool closed = vs.size() > 2 && vs.front().m_p == vs.back().m_p;
        const size_t nv = closed ? vs.size() - 1 : vs.size();
        const double longEdge = 10 * m_accuracy;
        // Edge k (1 <= k < vs.size()) runs from vs[k-1].m_p to vs[k].m_p, shaped by vs[k]
        auto tangentAt = [&](size_t k, bool atEnd) {
            const heeks::Point& a = vs[k - 1].m_p;
            const heeks::Point& b = vs[k].m_p;
            double tx, ty;
            if (vs[k].m_type == 0) {
                tx = b.x - a.x;
                ty = b.y - a.y;
            }
            else {
                const heeks::Point& q = atEnd ? b : a;
                tx = -(q.y - vs[k].m_c.y) * vs[k].m_type;
                ty = (q.x - vs[k].m_c.x) * vs[k].m_type;
            }
            const double l = std::hypot(tx, ty);
            return l > 0 ? heeks::Point(tx / l, ty / l) : heeks::Point(0, 0);
        };
        // Half of the angular discretization step used for an arc edge (closedCurveToPath64): the
        // normals of the chords Clipper sees differ from the true arc normals by up to this much
        auto halfStep = [&](size_t k) {
            if (vs[k].m_type == 0) {
                return 0.0;
            }
            const double r = vs[k - 1].m_p.dist(vs[k].m_c);
            return r > discTol ? std::min(M_PI / 4, std::acos(1 - discTol / r)) : M_PI / 4;
        };
        const double kSectorEps = 1e-3;
        for (size_t i = 0; i < nv; ++i) {
            bool corner = false;
            std::vector<SnapSector> joinSectors;
            const bool hasIn = i > 0 || closed;
            const bool hasOut = i + 1 < vs.size();
            if (hasIn && hasOut) {
                const size_t kin = i > 0 ? i : vs.size() - 1;
                const size_t kout = i + 1;
                const bool longEdges = vs[kin - 1].m_p.dist(vs[kin].m_p) >= longEdge
                    && vs[kout - 1].m_p.dist(vs[kout].m_p) >= longEdge;
                const heeks::Point tin = tangentAt(kin, true);
                const heeks::Point tout = tangentAt(kout, false);
                const double turn = std::fabs(
                    std::atan2(tin.x * tout.y - tin.y * tout.x, tin.x * tout.x + tin.y * tout.y)
                );
                corner = longEdges && turn > 1e-3;

                // The join sweeps from the incoming to the outgoing edge normal, on the outer
                // side (positive offset) or the inner side (negative offset)
                const double signedTurn
                    = std::atan2(tin.x * tout.y - tin.y * tout.x, tin.x * tout.x + tin.y * tout.y);
                const double slack = std::max(halfStep(kin), halfStep(kout)) + kSectorEps;
                const double outer = std::atan2(-tin.x, tin.y);  // right-hand normal of tin
                joinSectors.push_back({outer, signedTurn, slack});
                joinSectors.push_back({outer + M_PI, signedTurn, slack});
            }
            index.addJoinCenter(vs[i].m_p.x, vs[i].m_p.y, corner, std::move(joinSectors));
        }

        heeks::Point prev(0, 0);
        bool first = true;
        for (const CVertex& v : curve.m_vertices) {
            if (!first && v.m_type != 0) {
                const double r = std::hypot(prev.x - v.m_c.x, prev.y - v.m_c.y);
                // Same sweep convention as closedCurveToPath64
                const double a0 = std::atan2(prev.y - v.m_c.y, prev.x - v.m_c.x);
                double a1 = std::atan2(v.m_p.y - v.m_c.y, v.m_p.x - v.m_c.x);
                if (v.m_type > 0 && a1 <= a0) {
                    a1 += 2 * M_PI;
                }
                if (v.m_type < 0 && a1 >= a0) {
                    a1 -= 2 * M_PI;
                }
                const double step = r > discTol
                    ? std::min(M_PI / 2, 2 * std::acos(1 - discTol / r))
                    : M_PI / 2;
                const SnapSector sector {a0, a1 - a0, step + kSectorEps};
                index.addCircle(v.m_c.x, v.m_c.y, r + d, sector);
                index.addCircle(v.m_c.x, v.m_c.y, r - d, sector);
            }
            first = false;
            prev = v.m_p;
        }
        Path64 p = closedCurveToPath64(curve, discTol);
        if (!p.empty()) {
            paths.push_back(std::move(p));
        }
    }

    ClipperOffset co(2.0, discTol * m_clipper_scale);
    co.AddPaths(paths, JoinType::Round, EndType::Polygon);
    Paths64 result;
    co.Execute(offset * m_clipper_scale, result);

    m_curves.clear();
    for (const Path64& path : result) {
        if (path.size() < 2) {
            continue;
        }
        m_curves.push_back(snapClosedPath(path, index, tol, discTol, m_fit_arcs));
    }

    this->Reorder();
}

// Previous implementation: per-edge offset + tagged union. Still used for open curves.
void CArea::OffsetNaive(double offset)
{
    if (offset == 0) {
        return;
    }

    // Perform the naive offset, offsetting each edge and joining
    NaiveOffset(std::abs(offset));

    // If we want to keep the negative edges, flip all the edge labels
    if (offset < 0) {
        for (CCurve& curve : m_curves) {
            for (int& tag : curve.m_edgeTags) {
                tag = -tag;
            }
        }
    }

    // Union (fill rule positive), keeping positive edges and dropping negative edges
    _Clip(ClipType::Union, CArea {}, FillRule::Positive);

    // Note that this code currently has no impact because we call Reorder afterwards, but
    // (to be vetted in a future PR) I think the curves from the previous step have known
    // orientation and this simpler/lighter loop should replace the Reorder call
    //
    // // If negative offset, reverse the curves to put them in the forward direction
    // if (offset < 0) {
    //     for (CCurve& c : m_curves) {
    //         c.Reverse();
    //     }
    // }

    // I'm preserving this Reorder() call to preserve old behavior, but imo this should not be part
    // of Offset's spec
    this->Reorder();
}

CArea CArea::OpenOffset(double offset)
{
    CArea cNeg;
    if (offset == 0) {
        return cNeg;
    }

    // Perform the naive offset, offsetting each edge and joining
    NaiveOffset(std::abs(offset));

    // Union (fill rule positive), separating out the positive and negative edges
    _Clip(ClipType::Union, CArea {}, FillRule::Positive, false, false, std::ref(cNeg));

    // The negative curves are oriented in reverse (to make the union operation work) but we
    // actually want them oriented forwards when we return. Reverse them.
    for (CCurve& c : cNeg.m_curves) {
        c.Reverse();
    }

    // If the offset was supposed to be in the negative direction, swap the negative and positive results
    if (offset < 0) {
        std::swap(m_curves, cNeg.m_curves);
    }
    return cNeg;
}

void CArea::Thicken(double value)
{
    // Perform the naive offset, offsetting each edge and joining
    NaiveOffset(std::abs(value));

    // We want to keep all offset curves, so clear the edge tags
    for (CCurve& curve : m_curves) {
        curve.m_edgeTags.clear();
    }

    // Union (fill rule positive), keeping positive edges and dropping negative edges
    _Clip(ClipType::Union, CArea {}, FillRule::Positive);
}

SegmentData CArea::getParentMetadataFallback(
    const Point64& p1,
    const Point64& p2,
    const ConversionMetadata& metadata
)
{
    // Accumulate a list of edges connecting to p1 or p2
    std::vector<std::pair<int64_t, int64_t>> edges;
    auto p1_edges = metadata.edges.find(p1.z);
    if (p1_edges != metadata.edges.end()) {
        for (int64_t z : p1_edges->second) {
            edges.emplace_back(std::min(p1.z, z), std::max(p1.z, z));
        }
    }

    auto p2_edges = metadata.edges.find(p2.z);
    if (p2_edges != metadata.edges.end()) {
        for (int64_t z : p2_edges->second) {
            edges.emplace_back(std::min(p2.z, z), std::max(p2.z, z));
        }
    }

    // Loop over them, and find the closest one to the provided edge. We require
    // distance less than half the diagnal of a square, since rounding to the
    // nearest integer never produces error larger than that.
    double bestDistSq = 0.5;  // (sqrt(2)/2)^2
    std::optional<SegmentData> best;
    for (const auto& [zMin, zMax] : edges) {
        // Get edge endpoint (x, y) coordinates
        auto itA = metadata.z_to_xy.find(zMin);
        auto itB = metadata.z_to_xy.find(zMax);
        if (itA == metadata.z_to_xy.end() || itB == metadata.z_to_xy.end()) {
            continue;
        }
        const Point64 ptA {itA->second.first, itA->second.second, zMin};
        const Point64 ptB {itB->second.first, itB->second.second, zMax};

        // Bbox check: skip if either p1 or p2 is outside the edge's bounding box.
        // If either is, then that point is too far from the edge.
        if (std::min(p1.x, p2.x) < std::min(ptA.x, ptB.x)
            || std::max(p1.x, p2.x) > std::max(ptA.x, ptB.x)
            || std::min(p1.y, p2.y) < std::min(ptA.y, ptB.y)
            || std::max(p1.y, p2.y) > std::max(ptA.y, ptB.y)) {
            continue;
        }

        // Compute the distance from p1 and p2 to line AB.
        // (P inside AB bounding box implies that the closest point to the line
        // is also inside the segment.)
        const double distSq = std::max(
            PerpendicDistFromLineSqrd(p1, ptA, ptB),
            PerpendicDistFromLineSqrd(p2, ptA, ptB)
        );

        if (distSq < bestDistSq) {
            const auto parentEdge = getParentEdge(ptA, ptB, metadata);
            if (parentEdge) {
                auto it = metadata.edgeData.find(*parentEdge);
                if (it != metadata.edgeData.end()) {
                    bestDistSq = distSq;
                    best = it->second;
                }
            }
        }
    }

    if (best) {
        return *best;
    }

    // Final fallback option: pretend that we know it's a line segment.
    // This fallback requires sentinel values for unknown/missing data:
    //   edgeTag = tagSentinel, to indicate we don't know the tag
    //   curveIndex = vertexIndex = -1, acceptable when used for sorting open paths
    std::cerr << "Warning: getParentMetadataFallback: no parent edge found for z=(" << p1.z << ","
              << p2.z << "), falling back to line\n";
    const PointD pt = ToPointD(p2);
    return {{{pt.x, pt.y}}, tagSentinel, -1, -1};
}

// Return the parent of the provided edge, specified as (zMin, zMax) of its endpoints
std::optional<std::pair<int64_t, int64_t>> CArea::getParentEdge(
    const Point64& p1,
    const Point64& p2,
    const ConversionMetadata& metadata
)
{
    // Check for a direct edge p1.z to p2.z
    std::pair<int64_t, int64_t> testEdge = {std::min(p1.z, p2.z), std::max(p1.z, p2.z)};
    if (metadata.edgeData.count(testEdge)) {
        return {testEdge};
    }

    // Check for an edge from p1.z to the intersection log of p2,
    // or from p2.z to the intersection log of z1
    auto z1its = metadata.intersections.equal_range(p1.z);
    for (auto z1it = z1its.first; z1it != z1its.second; z1it++) {
        const auto& [e1min, e1max, e2min, e2max] = z1it->second;
        if (p2.z == e1min || p2.z == e1max) {
            testEdge = {e1min, e1max};
            if (metadata.edgeData.count(testEdge)) {
                return {testEdge};
            }
        }
        if (p2.z == e2min || p2.z == e2max) {
            testEdge = {e2min, e2max};
            if (metadata.edgeData.count(testEdge)) {
                return {testEdge};
            }
        }
    }

    auto z2its = metadata.intersections.equal_range(p2.z);
    for (auto z2it = z2its.first; z2it != z2its.second; z2it++) {
        const auto& [e1min, e1max, e2min, e2max] = z2it->second;
        if (p1.z == e1min || p1.z == e1max) {
            testEdge = {e1min, e1max};
            if (metadata.edgeData.count(testEdge)) {
                return {testEdge};
            }
        }
        if (p1.z == e2min || p1.z == e2max) {
            testEdge = {e2min, e2max};
            if (metadata.edgeData.count(testEdge)) {
                return {testEdge};
            }
        }
    }

    // Check for any shared edge in the intersection logs of p1 and p2
    for (auto z1it = z1its.first; z1it != z1its.second; z1it++) {
        const auto& [e1min, e1max, e2min, e2max] = z1it->second;
        for (auto z2it = z2its.first; z2it != z2its.second; z2it++) {
            const auto& [e3min, e3max, e4min, e4max] = z2it->second;
            if ((e1min == e3min && e1max == e3max) || (e1min == e4min && e1max == e4max)) {
                testEdge = {e1min, e1max};
                if (metadata.edgeData.count(testEdge)) {
                    return {testEdge};
                }
            }
            if ((e2min == e3min && e2max == e3max) || (e2min == e4min && e2max == e4max)) {
                testEdge = {e2min, e2max};
                if (metadata.edgeData.count(testEdge)) {
                    return {testEdge};
                }
            }
        }
    }

    return {};
}

SegmentData CArea::getParentMetadata(const Point64& p1, const Point64& p2, const ConversionMetadata& metadata)
{
    const auto parentEdge = getParentEdge(p1, p2, metadata);

    if (parentEdge) {
        const auto it = metadata.edgeData.find(*parentEdge);
        if (it != metadata.edgeData.end()) {
            return it->second;
        }
    }

    // Failed to find the parent edge. This should not happen; the parent edge should always exist.
    //
    // Update: Unfortunately, it does seem to happen. I've reported a clipper bug for at least one
    // way it can happen (https://github.com/AngusJohnson/Clipper2/issues/1111). Instead of
    // throwing, for now we will invoke a more intensive fallback to find the parent edge.
    return getParentMetadataFallback(p1, p2, metadata);
    // After the clipper bug is resolved, we can look into removing this fallback code and going
    // back to throwing an error:
    // throw std::logic_error(
    //     "No parent edge found for z=(" + std::to_string(p1.z) + "," + std::to_string(p2.z) + ")"
    //     + " hits=(" + std::to_string(metadata.intersections.count(p1.z)) + ","
    //     + std::to_string(metadata.intersections.count(p2.z)) + ")"
    // );
}

// For open paths, reorder as needed to produce positively oriented and positively ordered paths
void CArea::ReorderOpenPaths(Paths64& paths, const ConversionMetadata& metadata)
{
    std::vector<std::tuple<int, int, double>> pathOrder;  // max (curveIndex, vertexIndex, progress)
                                                          // across edges
    pathOrder.reserve(paths.size());

    for (Path64& path : paths) {
        pathOrder.push_back({-1, 0, 0.0});
        if (path.empty()) {
            continue;
        }

        bool needsReversal = false;

        for (size_t i = 0; i + 1 < path.size(); i++) {
            const Point64& p1 = path[i];
            const Point64& p2 = path[i + 1];

            // Look up parent edge metadata
            const SegmentData& seg = getParentMetadata(p1, p2, metadata);

            // Convert seg endpoint/center to Point64 for consistent units
            const Point64 mp64 = ToPoint64(PointD(seg.orig.m_p.x, seg.orig.m_p.y, 0));
            const Point64 mc64 = ToPoint64(PointD(seg.orig.m_c.x, seg.orig.m_c.y, 0));

            // Check if the current edge points forwards or backwards on the parent edge
            if (seg.orig.m_type == 0) {
                // For lines, compare Euclidean distance to the parent line's end point
                const double d1 = std::hypot(p1.x - mp64.x, p1.y - mp64.y);
                const double d2 = std::hypot(p2.x - mp64.x, p2.y - mp64.y);
                needsReversal = d1 < d2;
                const double progress = std::max(-d1, -d2);
                pathOrder.back()
                    = std::max(pathOrder.back(), {seg.curveIndex, seg.vertexIndex, progress});
            }
            else {
                assert(seg.orig.m_type == 1 || seg.orig.m_type == -1);
                // For arcs, use angular distance. Clipper segments representing lines are
                // always small angles, so center phi1 and phi2 together
                double phi1 = atan2(p1.y - mc64.y, p1.x - mc64.x);
                double phi2 = recenter(atan2(p2.y - mc64.y, p2.x - mc64.x), phi1 - M_PI, 1);
                needsReversal = phi2 * seg.orig.m_type < phi1 * seg.orig.m_type;

                // Then recenter them colectively relative to phi_end
                const double phi_end = atan2(mp64.y - mc64.y, mp64.x - mc64.x);
                while ((phi1 + phi2) / 2 * seg.orig.m_type > phi_end * seg.orig.m_type) {
                    phi1 -= 2 * M_PI * seg.orig.m_type;
                    phi2 -= 2 * M_PI * seg.orig.m_type;
                }
                while ((phi1 + phi2) / 2 * seg.orig.m_type + 2 * M_PI < phi_end * seg.orig.m_type) {
                    phi1 += 2 * M_PI * seg.orig.m_type;
                    phi2 += 2 * M_PI * seg.orig.m_type;
                }

                const double progress = std::max(-std::abs(phi_end - phi1), -std::abs(phi_end - phi2));
                pathOrder.back()
                    = std::max(pathOrder.back(), {seg.curveIndex, seg.vertexIndex, progress});
            }
        }

        if (needsReversal) {
            std::reverse(path.begin(), path.end());
        }
    }

    // Now put the paths in order. Do the sorting in a std::vector copy and then copy them back
    std::vector<std::pair<std::tuple<int, int, double>, Path64>> vpaths;
    vpaths.reserve(paths.size());

    for (size_t i = 0; i < paths.size(); i++) {
        vpaths.emplace_back(pathOrder[i], std::move(paths[i]));
    }

    std::sort(vpaths.begin(), vpaths.end(), [](const auto& a, const auto& b) {
        return a.first < b.first;
    });

    paths.clear();
    for (auto& [key, path] : vpaths) {
        paths.push_back(std::move(path));
    }
}
