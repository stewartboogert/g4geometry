#include "G4CADSplitter.hh"

#include <stdexcept>

#include <BRepAlgoAPI_Common.hxx>
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

Standard_Real ComputeExtent(const TopoDS_Shape& shape, const gp_Pln& plane)
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

    const gp_Pnt origin = plane.Location();
    const gp_Pnt corners[] = {
        gp_Pnt(xmin, ymin, zmin), gp_Pnt(xmin, ymin, zmax), gp_Pnt(xmin, ymax, zmin), gp_Pnt(xmin, ymax, zmax),
        gp_Pnt(xmax, ymin, zmin), gp_Pnt(xmax, ymin, zmax), gp_Pnt(xmax, ymax, zmin), gp_Pnt(xmax, ymax, zmax)
    };

    Standard_Real maxDistance = 0.0;
    for (const gp_Pnt& corner : corners) {
        const Standard_Real distance = origin.Distance(corner);
        if (distance > maxDistance) {
            maxDistance = distance;
        }
    }

    const Standard_Real margin = 10.0 * Precision::Confusion();
    return maxDistance > Precision::Confusion() ? maxDistance + margin : 1.0;
}

} // namespace

std::pair<TopoDS_Shape, TopoDS_Shape> G4CADSplitter::Split(const TopoDS_Shape& shape, const gp_Pln& plane) const
{
    if (shape.IsNull()) {
        throw std::invalid_argument("Cannot split a null TopoDS_Shape");
    }

    const Standard_Real extent = ComputeExtent(shape, plane);
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
