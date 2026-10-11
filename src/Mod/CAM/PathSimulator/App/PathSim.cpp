// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
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


#include <cmath>

#include <Base/Exception.h>

#include "PathSim.h"


using namespace Base;
using namespace PathSimulator;

TYPESYSTEM_SOURCE(PathSimulator::PathSim, Base::BaseClass);

PathSim::PathSim()
{}

PathSim::~PathSim()
{}

void PathSim::BeginSimulation(Part::TopoShape* stock, float resolution)
{
    Base::BoundBox3d bbox = stock->getBoundBox();
    m_stock = std::make_unique<cStock>(
        bbox.MinX,
        bbox.MinY,
        bbox.MinZ,
        bbox.LengthX(),
        bbox.LengthY(),
        bbox.LengthZ(),
        resolution
    );

    // Seed the heightmap from the shape's real geometry, not just its
    // bounding box -- otherwise a non-box stock (a casting, bar stock, a
    // "Stock from Shape" object) would silently be treated as a flat box
    // matching its extents. Linear deflection is tied to the simulation's
    // own resolution -- no point tessellating finer than the grid can
    // represent -- though this is a reasonable starting default, not a
    // value validated against real timing/accuracy tradeoffs yet.
    Mesh::MeshObject seedMesh;
    TessellateShapeToMesh(stock->getShape(), resolution, 0.5f, seedMesh);
    m_stock->SeedFromMesh(seedMesh);
}

void PathSim::ReseedFromMesh(
    const Mesh::MeshObject& mesh,
    float px,
    float py,
    float pz,
    float lx,
    float ly,
    float lz,
    float resolution
)
{
    // Replaces the stock with a fresh grid seeded from an
    // already-tessellated mesh -- the cut stock rotated into a new
    // working frame, on a 3+2 workplane change. No BREP tessellation
    // step, unlike BeginSimulation: the caller already has a mesh.
    //
    // Destroys the cStock, and with it the height buffers held by the
    // viewport through GetHeightBuffer(). Callers must re-attach; the
    // Python side guards this with a generation counter.
    m_stock = std::make_unique<cStock>(px, py, pz, lx, ly, lz, resolution);
    m_stock->SeedFromMesh(mesh);
}

cTransferReport PathSim::ReseedFromStock(
    const Base::Matrix4D& motion,
    float px,
    float py,
    float pz,
    float lx,
    float ly,
    float lz,
    float resolution
)
{
    if (!m_stock) {
        throw Base::RuntimeError("PathSim.ReseedFromStock: the simulation has no stock");
    }
    if (!(resolution > 0.0f)) {
        throw Base::ValueError("PathSim.ReseedFromStock: resolution must be positive");
    }

    // Row-major, translation in m[3], m[7], m[11]: the layout
    // SeedFromStock takes. Read element by element rather than through
    // getMatrix(), so the layout is visible here rather than assumed.
    double m[16];
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            m[4 * r + c] = motion[r][c];
        }
    }

    // SeedFromStock carries heights back into the source frame through the
    // TRANSPOSE of the rotation, so anything but a proper rotation -- a
    // scale, a shear, a mirror, a projective row -- would give a plausible
    // grid in the wrong place rather than an error. The tolerance is far
    // looser than any composed solver matrix is off by (1e-15) and far
    // tighter than any real scale.
    const double tol = 1e-6;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            const double dot = m[4 * i] * m[4 * j] + m[4 * i + 1] * m[4 * j + 1]
                + m[4 * i + 2] * m[4 * j + 2];
            if (std::fabs(dot - (i == j ? 1.0 : 0.0)) > tol) {
                throw Base::ValueError(
                    "PathSim.ReseedFromStock: the matrix is not a rotation "
                    "(its rows are not orthonormal)"
                );
            }
        }
    }
    const double det = m[0] * (m[5] * m[10] - m[6] * m[9]) - m[1] * (m[4] * m[10] - m[6] * m[8])
        + m[2] * (m[4] * m[9] - m[5] * m[8]);
    if (det < 0.0 || std::fabs(m[12]) > tol || std::fabs(m[13]) > tol || std::fabs(m[14]) > tol
        || std::fabs(m[15] - 1.0) > tol) {
        throw Base::ValueError(
            "PathSim.ReseedFromStock: the matrix is not a rigid motion (a mirror, or a "
            "projective bottom row)"
        );
    }

    // Built beside the old stock, which is the source, and swapped in
    // only once complete: an exception leaves the simulation as it was.
    // Peak memory is both grids plus the crossings for one pass.
    //
    // Destroys the old cStock on the swap, and with it the buffers held
    // by the viewport through GetHeightBuffer(). Callers must re-attach,
    // as after ReseedFromMesh.
    auto next = std::make_unique<cStock>(px, py, pz, lx, ly, lz, resolution);
    cTransferReport report = next->SeedFromStock(*m_stock, m);
    m_stock = std::move(next);
    return report;
}

double PathSim::MaterialVolume() const
{
    return m_stock ? m_stock->MaterialVolume() : 0.0;
}

void PathSim::SetToolShape(const TopoDS_Shape& toolShape, float resolution)
{
    m_tool = std::make_unique<cSimTool>(toolShape, resolution);
}

Base::Placement* PathSim::ApplyCommand(Base::Placement* pos, Command* cmd)
{
    Point3D fromPos(*pos);
    Point3D toPos(*pos);
    toPos.UpdateCmd(*cmd);
    if (m_tool) {
        if (cmd->Name == "G0" || cmd->Name == "G1") {
            m_stock->ApplyLinearTool(fromPos, toPos, *m_tool);
        }
        else if (cmd->Name == "G2") {
            Vector3d vcent = cmd->getCenter();
            Point3D cent(vcent);
            m_stock->ApplyCircularTool(fromPos, toPos, cent, *m_tool, false);
        }
        else if (cmd->Name == "G3") {
            Vector3d vcent = cmd->getCenter();
            Point3D cent(vcent);
            m_stock->ApplyCircularTool(fromPos, toPos, cent, *m_tool, true);
        }
    }

    Base::Placement* plc = new Base::Placement();
    Vector3d vec(toPos.x, toPos.y, toPos.z);
    plc->setPosition(vec);
    return plc;
}
