#ifndef GEANT4GEOMETRY_G4CADSPLITTER_HH
#define GEANT4GEOMETRY_G4CADSPLITTER_HH

#include <utility>

class TopoDS_Shape;
class gp_Pln;

class G4CADSplitter {
public:
    G4CADSplitter() = default;

    std::pair<TopoDS_Shape, TopoDS_Shape> Split(const TopoDS_Shape& shape, const gp_Pln& plane) const;
};

#endif // GEANT4GEOMETRY_G4CADSPLITTER_HH
