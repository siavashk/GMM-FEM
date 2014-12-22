#ifndef MAS_MESH_SIMPLIFICATION_H
#define MAS_MESH_SIMPLIFICATION_H

#include "mas/mesh/mesh.h"

namespace mas {
namespace mesh {

/**
 * Kp or Qv = [A b; b^T c]
 */
class EdgeCollapseQuadric {
public:
    Matrix3d A;
    Vector3d b;
    double c;
public:
    EdgeCollapseQuadric();
    EdgeCollapseQuadric(const EdgeCollapseQuadric& ecq);
    EdgeCollapseQuadric(const Matrix3d& A, const Vector3d& b, double c);
    EdgeCollapseQuadric(Matrix3d&& A, Vector3d&& b, double c);
    void add(const EdgeCollapseQuadric& ecq);
    void add(const EdgeCollapseQuadric& ecq1, const EdgeCollapseQuadric& ecq2);

    void solve(const Point3d& v1, const Point3d& v2, Point3d& v);
    double cost(const Point3d& v);
};

class EdgeCollapseQuadricCost {
private:
    std::vector<EdgeCollapseQuadric> Qv; // vertex errors

public:
    EdgeCollapseQuadricCost();
    void init(PolygonMesh& mesh);
    double operator ()(PolygonMesh& mesh, HalfEdge& he, Point3d& v);
    void operator ()(HalfEdge& removed);
};

template<typename CostFunc = EdgeCollapseQuadricCost, typename CollapseCallback = EdgeCollapseQuadricCost>
void edge_collapse(PolygonMesh& mesh, double fraction, CostFunc cost = CostFunc(),
        CollapseCallback collapsed = CollapseCallback());

template<typename CostFunc = EdgeCollapseQuadricCost, typename CollapseCallback = EdgeCollapseQuadricCost>
void edge_collapse(PolygonMesh& mesh, size_t targetFaces, CostFunc cost = CostFunc(),
        CollapseCallback collapsed = CollapseCallback());

} // mesh
} // mas

#include "mas/mesh/simplification.hpp"

#endif



