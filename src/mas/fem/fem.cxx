#include "mas/fem/fem.h"

namespace mas {
namespace fem {

FemNode3d::FemNode3d() :
   IndexedPoint3d(), mass(0) {}

FemNode3d::FemNode3d(const FemNode3d& node) :
   IndexedPoint3d(node), mass(0) {}

FemNode3d& FemNode3d::operator=(const FemNode3d& assignMe) {
   x = assignMe.x;
   y = assignMe.y;
   z = assignMe.z;
   idx = assignMe.idx;
   mass = assignMe.mass;
   return *this;
}

FemNode3d::FemNode3d(double x, double y, double z, double m, int idx) :
   IndexedPoint3d(x,y,z,idx), mass(m) {}

void FemNode3d::setMass(double m) {
   mass = m;
}
   
double FemNode3d::getMass() {
   return mass;
}

} // fem
} // mas
