#ifndef MAS_MESH_SIMPLIFICATION_H
#define MAS_MESH_SIMPLIFICATION_H

#include "mas/mesh/mesh.h"
#include "mas/core/queue.h"

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
    void solveConstrained(const Point3d& v1, const Point3d& v2, Point3d& v);
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

template<typename CostFunc, typename CollapseCallback>
void edge_collapse(PolygonMesh& mesh, double fraction, CostFunc& cost,
        CollapseCallback& collapsed);

template<typename CostFunc, typename CollapseCallback>
void edge_collapse(PolygonMesh& mesh, size_t targetFaces, CostFunc& cost,
        CollapseCallback& collapsed);

void VERIFY_HE_INCIDENCE(PolygonMesh& mesh);
void VERIFY_CONNECTIVITY(PolygonMesh& mesh);
void VERIFY_VERTEX_REMOVED(PolygonMesh& mesh, SharedVertex3d& rvtx);

struct EdgeInfo {
    SharedHalfEdge edge;
    size_t pos;
    Point3d v;
    double cost;
public:
    EdgeInfo() :
            edge(nullptr), pos(-1), v(0, 0, 0), cost(0) {
    };
};

template<typename CostFunc, typename CollapseCallback>
class EdgeCollapser {
private:

    CostFunc& cost;
    CollapseCallback& collapsed;
    PolygonMesh& mesh;

    std::vector<std::unique_ptr<EdgeInfo>> edgeinfo;
    //    mas::queue::priority_queue<size_t, std::vector<size_t>,
    //        bool(*)(size_t,size_t), void(*)(size_t, size_t, size_t)> queue;
    mas::queue::priority_queue<size_t, std::vector<size_t>, std::function<bool(size_t,size_t)>, std::function<void(size_t, size_t, size_t)> > queue;


private:
    // disable assignment
    EdgeCollapser(const EdgeCollapser& that) = delete;
    EdgeCollapser& operator=(EdgeCollapser const&) = delete;

    void removeEdge(HalfEdge* he, size_t qpos);
    bool collapseKeepsTopology(const SharedHalfEdge& edge);

public:
    EdgeCollapser(PolygonMesh& mesh, CostFunc& cost,
            CollapseCallback& collapsed);

    void collapse(const SharedHalfEdge& edge);
    void collapseTo(size_t targetFaces);

};


} // mesh
} // mas

#include "mas/mesh/simplification.hpp"

#endif

