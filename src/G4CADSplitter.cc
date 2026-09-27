#include "G4CADSplitter.hh"

#include <cmath>
#include <stdexcept>

#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Splitter.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakeHalfSpace.hxx>
#include <Bnd_Box.hxx>
#include <Precision.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

namespace {

Standard_Real ComputeExtent(const TopoDS_Shape& shape)
{
    Bnd_Box box;
    BRepBndLib::Add(shape, box);
    if (box.IsVoid()) {
        return 1.0;
    }

    Standard_Real xmin = 0.0;
    Standard_Real ymin = 0.0;
    Standard_Real zmin = 0.0;
    Standard_Real xmax = 0.0;
    Standard_Real ymax = 0.0;
    Standard_Real zmax = 0.0;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);

    const Standard_Real dx = xmax - xmin;
    const Standard_Real dy = ymax - ymin;
    const Standard_Real dz = zmax - zmin;
    const Standard_Real diagonal = std::sqrt(dx * dx + dy * dy + dz * dz);
    return diagonal > Precision::Confusion() ? diagonal * 2.0 : 1.0;
}

} // namespace

std::pair<TopoDS_Shape, TopoDS_Shape> G4CADSplitter::Split(const TopoDS_Shape& shape, const gp_Pln& plane) const
{
    if (shape.IsNull()) {
        throw std::invalid_argument("Cannot split a null TopoDS_Shape");
    }

    const Standard_Real extent = ComputeExtent(shape);
    const TopoDS_Face splitFace = BRepBuilderAPI_MakeFace(plane, -extent, extent, -extent, extent).Face();

    const gp_Vec normal(plane.Axis().Direction());
    const gp_Pnt positiveReference = plane.Location().Translated(normal * extent);
    const gp_Pnt negativeReference = plane.Location().Translated(-normal * extent);

    const TopoDS_Solid positiveHalfSpace = BRepPrimAPI_MakeHalfSpace(splitFace, positiveReference).Solid();
    const TopoDS_Solid negativeHalfSpace = BRepPrimAPI_MakeHalfSpace(splitFace, negativeReference).Solid();

    BRepAlgoAPI_Common positiveCommon(shape, positiveHalfSpace);
    positiveCommon.Build();
    if (!positiveCommon.IsDone()) {
        throw std::runtime_error("Failed to build positive half split shape");
    }

    BRepAlgoAPI_Common negativeCommon(shape, negativeHalfSpace);
    negativeCommon.Build();
    if (!negativeCommon.IsDone()) {
        throw std::runtime_error("Failed to build negative half split shape");
    }

    return {positiveCommon.Shape(), negativeCommon.Shape()};
}
