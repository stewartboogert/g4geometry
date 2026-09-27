#include "G4CADSplitter.hh"

#include <cmath>
#include <stdexcept>
#include <vector>

#include <BRepAlgoAPI_Splitter.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRep_Builder.hxx>
#include <Bnd_Box.hxx>
#include <Precision.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Dir.hxx>
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

Standard_Real SignedDistance(const gp_Pln& plane, const gp_Pnt& point)
{
    const gp_Dir normal = plane.Axis().Direction();
    const gp_Vec offset(plane.Location(), point);
    return normal.Dot(offset);
}

gp_Pnt ShapeCenter(const TopoDS_Shape& shape)
{
    Bnd_Box box;
    BRepBndLib::Add(shape, box);
    if (box.IsVoid()) {
        return gp_Pnt(0.0, 0.0, 0.0);
    }

    Standard_Real xmin = 0.0;
    Standard_Real ymin = 0.0;
    Standard_Real zmin = 0.0;
    Standard_Real xmax = 0.0;
    Standard_Real ymax = 0.0;
    Standard_Real zmax = 0.0;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);

    return gp_Pnt(0.5 * (xmin + xmax), 0.5 * (ymin + ymax), 0.5 * (zmin + zmax));
}

std::vector<TopoDS_Shape> TopLevelParts(const TopoDS_Shape& shape)
{
    std::vector<TopoDS_Shape> parts;
    if (shape.ShapeType() != TopAbs_COMPOUND && shape.ShapeType() != TopAbs_COMPSOLID) {
        parts.push_back(shape);
        return parts;
    }

    for (TopoDS_Iterator iterator(shape); iterator.More(); iterator.Next()) {
        const TopoDS_Shape& value = iterator.Value();
        if (!value.IsNull()) {
            parts.push_back(value);
        }
    }

    if (parts.empty()) {
        parts.push_back(shape);
    }

    return parts;
}

} // namespace

std::pair<TopoDS_Shape, TopoDS_Shape> G4CADSplitter::Split(const TopoDS_Shape& shape, const gp_Pln& plane) const
{
    if (shape.IsNull()) {
        throw std::invalid_argument("Cannot split a null TopoDS_Shape");
    }

    const Standard_Real extent = ComputeExtent(shape);
    const TopoDS_Face splitFace = BRepBuilderAPI_MakeFace(plane, -extent, extent, -extent, extent).Face();

    BRepAlgoAPI_Splitter splitter;
    splitter.AddArgument(shape);
    splitter.AddTool(splitFace);
    splitter.Build();

    if (!splitter.IsDone()) {
        throw std::runtime_error("Failed to split TopoDS_Shape with plane");
    }

    const TopoDS_Shape result = splitter.Shape();
    if (result.IsNull()) {
        throw std::runtime_error("Split operation did not produce a shape");
    }

    TopoDS_Compound positiveSide;
    TopoDS_Compound negativeSide;
    BRep_Builder builder;
    builder.MakeCompound(positiveSide);
    builder.MakeCompound(negativeSide);

    const Standard_Real tolerance = Precision::Confusion();
    for (const TopoDS_Shape& part : TopLevelParts(result)) {
        const Standard_Real distance = SignedDistance(plane, ShapeCenter(part));
        if (distance >= -tolerance) {
            builder.Add(positiveSide, part);
        } else {
            builder.Add(negativeSide, part);
        }
    }

    return {positiveSide, negativeSide};
}
