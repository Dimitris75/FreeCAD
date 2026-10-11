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

#include <Base/MatrixPy.h>
#include <Base/PlacementPy.h>
#include <Base/PyWrapParseTupleAndKeywords.h>

#include <Mod/Mesh/App/MeshPy.h>
#include <Mod/CAM/App/CommandPy.h>
#include <Mod/Part/App/TopoShapePy.h>

#include "PathSim.h"
// inclusion of the generated files (generated from PathSim.pyi)
#include "PathSimPy.h"
#include "PathSimPy.cpp"


using namespace PathSimulator;

// returns a string which represents the object e.g. when printed in python
std::string PathSimPy::representation() const
{
    return std::string("<PathSim object>");
}

PyObject* PathSimPy::PyMake(struct _typeobject*, PyObject*, PyObject*)  // Python wrapper
{
    // create a new instance of PathSimPy and the Twin object
    return new PathSimPy(new PathSim);
}

// constructor method
int PathSimPy::PyInit(PyObject* /*args*/, PyObject* /*kwd*/)
{
    return 0;
}


PyObject* PathSimPy::BeginSimulation(PyObject* args, PyObject* kwds)
{
    static const std::array<const char*, 3> kwlist {"stock", "resolution", nullptr};
    PyObject* pObjStock;
    float resolution;
    if (!Base::Wrapped_ParseTupleAndKeywords(
            args,
            kwds,
            "O!f",
            kwlist,
            &(Part::TopoShapePy::Type),
            &pObjStock,
            &resolution
        )) {
        return nullptr;
    }
    PathSim* sim = getPathSimPtr();
    Part::TopoShape* stock = static_cast<Part::TopoShapePy*>(pObjStock)->getTopoShapePtr();
    sim->BeginSimulation(stock, resolution);
    Py_IncRef(Py_None);
    return Py_None;
}

PyObject* PathSimPy::ReseedFromMesh(PyObject* args)
{
    PyObject* pObjMesh;
    float px, py, pz, lx, ly, lz, resolution;
    if (!PyArg_ParseTuple(
            args,
            "O!fffffff",
            &(Mesh::MeshPy::Type),
            &pObjMesh,
            &px,
            &py,
            &pz,
            &lx,
            &ly,
            &lz,
            &resolution
        )) {
        return nullptr;
    }
    PathSim* sim = getPathSimPtr();
    Mesh::MeshObject* mesh = static_cast<Mesh::MeshPy*>(pObjMesh)->getMeshObjectPtr();
    sim->ReseedFromMesh(*mesh, px, py, pz, lx, ly, lz, resolution);
    Py_IncRef(Py_None);
    return Py_None;
}

// The 3+2 re-seed without the mesh round trip: the stock's own material,
// carried through `matrix` (old working frame -> new) onto a fresh grid
// at the given box. Returns what cStock::SeedFromStock reports, as a
// dict. The caller must read "imbalanced": non-zero means that many
// columns fell back to the old min/max fill, and nothing else says so.
//
// Invalidates the height buffers, like ReseedFromMesh. On a ValueError
// (not a rotation) or RuntimeError (no stock) the stock is untouched.
PyObject* PathSimPy::ReseedFromStock(PyObject* args)
{
    PyObject* pObjMatrix;
    float px, py, pz, lx, ly, lz, resolution;
    if (!PyArg_ParseTuple(
            args,
            "O!fffffff",
            &(Base::MatrixPy::Type),
            &pObjMatrix,
            &px,
            &py,
            &pz,
            &lx,
            &ly,
            &lz,
            &resolution
        )) {
        return nullptr;
    }
    const Base::Matrix4D motion = static_cast<Base::MatrixPy*>(pObjMatrix)->value();
    PathSim* sim = getPathSimPtr();
    cTransferReport report;
    try {
        report = sim->ReseedFromStock(motion, px, py, pz, lx, ly, lz, resolution);
    }
    catch (const Base::ValueError& e) {
        PyErr_SetString(PyExc_ValueError, e.what());
        return nullptr;
    }
    catch (const Base::Exception& e) {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return nullptr;
    }
    catch (const std::bad_alloc&) {
        PyErr_SetString(PyExc_MemoryError, "PathSim.ReseedFromStock: out of memory");
        return nullptr;
    }

    return Py_BuildValue(
        "{s:n,s:n,s:n,s:n,s:n,s:d,s:d,s:d}",
        "faces",
        static_cast<Py_ssize_t>(report.faces),
        "crossings",
        static_cast<Py_ssize_t>(report.crossings),
        "columns",
        static_cast<Py_ssize_t>(report.columns),
        "imbalanced",
        static_cast<Py_ssize_t>(report.imbalanced),
        "extraSpans",
        static_cast<Py_ssize_t>(sim->m_stock->ExtraSpanCount()),
        "area",
        report.area,
        "surfaceMs",
        report.surfaceMs,
        "assignMs",
        report.assignMs
    );
}

PyObject* PathSimPy::SetToolShape(PyObject* args)
{
    PyObject* pObjToolShape;
    float resolution;
    if (!PyArg_ParseTuple(args, "O!f", &(Part::TopoShapePy::Type), &pObjToolShape, &resolution)) {
        return nullptr;
    }
    PathSim* sim = getPathSimPtr();
    const TopoDS_Shape& toolShape
        = static_cast<Part::TopoShapePy*>(pObjToolShape)->getTopoShapePtr()->getShape();
    sim->SetToolShape(toolShape, resolution);
    Py_IncRef(Py_None);
    return Py_None;
}

PyObject* PathSimPy::GetResultMesh(PyObject* args)
{
    if (!PyArg_ParseTuple(args, "")) {
        return nullptr;
    }
    cStock* stock = getPathSimPtr()->m_stock.get();
    if (!stock) {
        PyErr_SetString(PyExc_RuntimeError, "Simulation has no stock object");
        return nullptr;
    }

    Mesh::MeshObject* meshOuter = new Mesh::MeshObject();
    Mesh::MeshPy* meshOuterpy = new Mesh::MeshPy(meshOuter);
    Mesh::MeshObject* meshInner = new Mesh::MeshObject();
    Mesh::MeshPy* meshInnerpy = new Mesh::MeshPy(meshInner);
    stock->Tessellate(*meshOuter, *meshInner);
    PyObject* tuple = PyTuple_New(2);
    PyTuple_SetItem(tuple, 0, meshOuterpy);
    PyTuple_SetItem(tuple, 1, meshInnerpy);
    return tuple;
}


PyObject* PathSimPy::MaterialVolume()
{
    // No PyObject* args parameter: the .pyi marks this @no_args, and
    // generate_from_py emits a declaration taking nothing at all for
    // those. Copying GetHeightBuffer's signature, which parses an empty
    // tuple because it is NOT decorated, gives a definition that matches
    // no declaration.
    return PyFloat_FromDouble(getPathSimPtr()->MaterialVolume());
}


// The stock's size in X and Y. The grid covers exactly this, so its
// last column and row are narrower than GetHeightBuffer's resolution
// wherever the size is not a whole number of cells. A separate call, not
// part of GetHeightBuffer's tuple: that is unpacked by position.
//
// No PyObject* args: the .pyi marks this @no_args (see MaterialVolume).
PyObject* PathSimPy::GetGridLength()
{
    cStock* stock = getPathSimPtr()->m_stock.get();
    if (!stock) {
        PyErr_SetString(PyExc_RuntimeError, "Simulation has no stock object");
        return nullptr;
    }
    return Py_BuildValue("ff", stock->LengthX(), stock->LengthY());
}


// Bytes of simulation state per grid cell, from the engine itself, so the
// Accuracy label's memory figure cannot drift from the arrays cStock
// actually allocates. Static in C++; an instance method here because it
// needs no stock and a fresh PathSim() costs nothing.
PyObject* PathSimPy::BytesPerCell()
{
    return PyLong_FromLong(cStock::BytesPerCell());
}


// Hands the raw height grid to the viewport as a read-only memoryview,
// with no copy and no Mesh in between. The buffer is allocated in
// cStock's constructor and never reallocated, so the caller can wrap it
// in a numpy array and keep it -- until the stock is recreated.
//
// Returns (heights, originals, nx, ny, res, originX, originY, floorZ, planeZ).
// Both buffers are row-major over X: index = x * ny + y.
PyObject* PathSimPy::GetHeightBuffer(PyObject* args)
{
    if (!PyArg_ParseTuple(args, "")) {
        return nullptr;
    }
    cStock* stock = getPathSimPtr()->m_stock.get();
    if (!stock) {
        PyErr_SetString(PyExc_RuntimeError, "Simulation has no stock object");
        return nullptr;
    }

    const Py_ssize_t count = static_cast<Py_ssize_t>(stock->GridSizeX()) * stock->GridSizeY();
    const Py_ssize_t nbytes = count * static_cast<Py_ssize_t>(sizeof(float));

    PyObject* heights = PyMemoryView_FromMemory(
        const_cast<char*>(reinterpret_cast<const char*>(stock->HeightData())),
        nbytes,
        PyBUF_READ
    );
    if (!heights) {
        return nullptr;
    }
    PyObject* originals = PyMemoryView_FromMemory(
        const_cast<char*>(reinterpret_cast<const char*>(stock->OriginalData())),
        nbytes,
        PyBUF_READ
    );
    if (!originals) {
        Py_DECREF(heights);
        return nullptr;
    }

    return Py_BuildValue(
        "NNiifffff",
        heights,
        originals,
        stock->GridSizeX(),
        stock->GridSizeY(),
        stock->Resolution(),
        stock->OriginX(),
        stock->OriginY(),
        stock->FloorZ(),
        stock->PlaneZ()
    );
}


// The underside, as a separate call rather than a tenth element of
// GetHeightBuffer's tuple: callers unpack that tuple by position, and a
// module and a Python file one build apart would fail with a bare
// "too many values to unpack".
//
// No PyObject* args: the .pyi marks this @no_args (see MaterialVolume).
PyObject* PathSimPy::GetBottomBuffer()
{
    cStock* stock = getPathSimPtr()->m_stock.get();
    if (!stock) {
        PyErr_SetString(PyExc_RuntimeError, "Simulation has no stock object");
        return nullptr;
    }

    const Py_ssize_t count = static_cast<Py_ssize_t>(stock->GridSizeX()) * stock->GridSizeY();
    const Py_ssize_t nbytes = count * static_cast<Py_ssize_t>(sizeof(float));

    return PyMemoryView_FromMemory(
        const_cast<char*>(reinterpret_cast<const char*>(stock->BottomData())),
        nbytes,
        PyBUF_READ
    );
}


// The spans BELOW each column's primary, written straight into caller-
// supplied buffers -- numpy arrays, in practice. A returned bytes object
// per frame would allocate and copy the dirty rectangle twice over.
//
// GetSpanLayers(x0, y0, x1, y1, layers, tops, bottoms). The rectangle is
// inclusive and is clamped to the grid. Both buffers must be writable and
// hold at least layers * width * height floats; layout is layer-major,
// then row-major over X, as cStock::ReadSpanLayers documents.
PyObject* PathSimPy::GetSpanLayers(PyObject* args)
{
    int x0, y0, x1, y1, layers;
    Py_buffer tops {}, bottoms {};
    if (!PyArg_ParseTuple(args, "iiiiiw*w*", &x0, &y0, &x1, &y1, &layers, &tops, &bottoms)) {
        return nullptr;
    }

    // One exit, so neither buffer is ever left checked out: a Py_buffer
    // that is not released keeps the array's memory pinned for good.
    PyObject* result = nullptr;
    cStock* stock = getPathSimPtr()->m_stock.get();
    if (!stock) {
        PyErr_SetString(PyExc_RuntimeError, "Simulation has no stock object");
    }
    else if (layers < 0) {
        PyErr_SetString(PyExc_ValueError, "PathSim.GetSpanLayers: layers must not be negative");
    }
    else if (!PyBuffer_IsContiguous(&tops, 'C') || !PyBuffer_IsContiguous(&bottoms, 'C')) {
        PyErr_SetString(PyExc_ValueError, "PathSim.GetSpanLayers: the buffers must be contiguous");
    }
    else {
        // Clamped here as well as in the engine, because the size the
        // buffers must have depends on the clamped rectangle.
        const int cx0 = std::max(0, x0);
        const int cy0 = std::max(0, y0);
        const int cx1 = std::min(x1, stock->GridSizeX() - 1);
        const int cy1 = std::min(y1, stock->GridSizeY() - 1);
        const Py_ssize_t w = cx1 >= cx0 ? cx1 - cx0 + 1 : 0;
        const Py_ssize_t h = cy1 >= cy0 ? cy1 - cy0 + 1 : 0;
        const Py_ssize_t need = layers * w * h * (Py_ssize_t)sizeof(float);
        if (tops.len < need || bottoms.len < need) {
            PyErr_Format(
                PyExc_ValueError,
                "PathSim.GetSpanLayers: buffers of %zd bytes, %zd needed",
                tops.len < bottoms.len ? tops.len : bottoms.len,
                need
            );
        }
        else {
            if (w > 0 && h > 0) {
                stock->ReadSpanLayers(
                    cx0,
                    cy0,
                    cx1,
                    cy1,
                    layers,
                    static_cast<float*>(tops.buf),
                    static_cast<float*>(bottoms.buf)
                );
            }
            Py_INCREF(Py_None);
            result = Py_None;
        }
    }

    PyBuffer_Release(&tops);
    PyBuffer_Release(&bottoms);
    return result;
}


// A no-op since Tessellate() became a full pass; see cStock.
PyObject* PathSimPy::InvalidateTessellation()
{
    cStock* stock = getPathSimPtr()->m_stock.get();
    if (!stock) {
        PyErr_SetString(PyExc_RuntimeError, "Simulation has no stock object");
        return nullptr;
    }
    stock->InvalidateTessellation();
    Py_IncRef(Py_None);
    return Py_None;
}


// Returns the cell rectangle touched since the last call, as
// (xMin, yMin, xMax, yMax) inclusive, or None if nothing changed. This
// is the only per-tick call the viewport needs -- four integers instead
// of a rebuilt mesh.
PyObject* PathSimPy::ConsumeViewDirty(PyObject* args)
{
    if (!PyArg_ParseTuple(args, "")) {
        return nullptr;
    }
    cStock* stock = getPathSimPtr()->m_stock.get();
    if (!stock) {
        PyErr_SetString(PyExc_RuntimeError, "Simulation has no stock object");
        return nullptr;
    }
    int xMin = 0, yMin = 0, xMax = 0, yMax = 0;
    if (!stock->ConsumeViewDirty(xMin, yMin, xMax, yMax)) {
        Py_RETURN_NONE;
    }
    return Py_BuildValue("iiii", xMin, yMin, xMax, yMax);
}


PyObject* PathSimPy::ApplyCommand(PyObject* args, PyObject* kwds)
{
    static const std::array<const char*, 3> kwlist {"position", "command", nullptr};
    PyObject* pObjPlace;
    PyObject* pObjCmd;
    if (!Base::Wrapped_ParseTupleAndKeywords(
            args,
            kwds,
            "O!O!",
            kwlist,
            &(Base::PlacementPy::Type),
            &pObjPlace,
            &(Path::CommandPy::Type),
            &pObjCmd
        )) {
        return nullptr;
    }
    PathSim* sim = getPathSimPtr();
    Base::Placement* pos = static_cast<Base::PlacementPy*>(pObjPlace)->getPlacementPtr();
    Path::Command* cmd = static_cast<Path::CommandPy*>(pObjCmd)->getCommandPtr();
    Base::Placement* newpos = sim->ApplyCommand(pos, cmd);
    // Base::Console().log("Done...\n");
    // Base::Console().Refresh();
    Base::PlacementPy* newposPy = new Base::PlacementPy(newpos);
    return newposPy;
}

Py::Object PathSimPy::getTool() const
{
    // return Py::Object();
    throw Py::AttributeError("Not yet implemented");
}

PyObject* PathSimPy::getCustomAttributes(const char* /*attr*/) const
{
    return nullptr;
}

int PathSimPy::setCustomAttributes(const char* /*attr*/, PyObject* /*obj*/)
{
    return 0;
}
