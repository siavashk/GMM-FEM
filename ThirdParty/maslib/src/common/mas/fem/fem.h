#ifndef MAS_FEM_FEM_H
#define MAS_FEM_FEM_H

#include "mas/core/base.h"
#include <vector>
#include <memory>

namespace mas {
namespace fem {

class FemNode3d;
using SharedFemNode3d = std::shared_ptr<FemNode3d>;

class FemElement3d;
using SharedFemElement3d = std::shared_ptr<FemElement3d>;

class HexElement;
class WedgeElement;
class PyramidElement;
class TetElement;

class FemNode3d : IndexedPoint3d {
private:
   double mass;

public:
   FemNode3d();
   FemNode3d(const FemNode3d& node);
   FemNode3d(double x, double y, double z, double m, int idx = -1);
   virtual FemNode3d& operator=(const FemNode3d& assignMe);

   void setMass(double m);
   double getMass();

};


class QuadraturePoint : Point3d {
public:
   double w;  // weight
};

class FemElement3d {
private:
   std::vector<SharedFemNode3d> nodes;
   std::shared_ptr< std::vector<QuadraturePoint> > qpnts;

protected:
   FemElement3d();  // protected constructor

public:
   virtual ~FemElement3d();

   virtual size_t getNumNodes() = 0;
   virtual void getN(double xi, double eta, double mu, double* out);
   virtual void getdNds(int s, double xi, double eta, double mu, double* out);

   virtual const std::shared_ptr<std::vector<QuadraturePoint> >& getQuadraturePoints() const;

   static FemElement3d* create( std::vector<SharedFemNode3d>&& nodes );

   void computeJ(double* dNds);

};

}
}

#endif
