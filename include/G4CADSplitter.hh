#ifndef GEANT4GEOMETRY_G4CADSPLITTER_HH
#define GEANT4GEOMETRY_G4CADSPLITTER_HH

#include <utility>

#include <TopoDS_Shape.hxx>

class gp_Pln;

class G4CADSplitter {
public:
    G4CADSplitter() = default;

    // Returns {positive-side shape, negative-side shape} relative to the plane normal direction.
    // If one side has no geometry, that entry is an empty compound shape.
    // Geometry exactly on the plane may appear in both outputs.
    std::pair<TopoDS_Shape, TopoDS_Shape> Split(const TopoDS_Shape& shape, const gp_Pln& plane) const;
};

#endif // GEANT4GEOMETRY_G4CADSPLITTER_HH
