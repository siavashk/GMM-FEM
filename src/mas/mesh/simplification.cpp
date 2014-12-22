#include "mas/mesh/simplification.h"
#include <iostream>

namespace mas {
namespace mesh {

EdgeCollapseQuadric::EdgeCollapseQuadric() :
        A(), b(), c() {
    A.setZero();
    b.setZero();
    c = 0;
}

EdgeCollapseQuadric::EdgeCollapseQuadric(const EdgeCollapseQuadric& ecq) :
        A(ecq.A), b(ecq.b), c(ecq.c){}

EdgeCollapseQuadric::EdgeCollapseQuadric(const Matrix3d& A, const Vector3d& b,
        double c) :
        A(A), b(b), c(c) {
}

EdgeCollapseQuadric::EdgeCollapseQuadric(Matrix3d&& A, Vector3d&& b, double c) :
        A(std::move(A)), b(std::move(b)), c(c) {
}

void EdgeCollapseQuadric::add(const EdgeCollapseQuadric& ecq) {
    A.add(ecq.A);
    b.add(ecq.b);
    c += ecq.c;
}
void EdgeCollapseQuadric::add(const EdgeCollapseQuadric& ecq1,
        const EdgeCollapseQuadric& ecq2) {
    A.add(ecq1.A, ecq2.A);
    b.add(ecq1.b, ecq2.b);
    c = ecq1.c + ecq2.c;
}

void EdgeCollapseQuadric::solve(const Point3d& v1, const Point3d& v2,
        Point3d &v) {

    double det = A[IDX3D_00] * A[IDX3D_11] * A[IDX3D_22]
            + 2 * A[IDX3D_01] * A[IDX3D_12] * A[IDX3D_02]
            - A[IDX3D_11] * A[IDX3D_02] * A[IDX3D_02]
            - A[IDX3D_00] * A[IDX3D_12] * A[IDX3D_12]
            - A[IDX3D_01] * A[IDX3D_01] * A[IDX3D_22];

    if (det != 0) {
        // invert symmetric matrix and solve
        double idet = -1.0 / det; // negate for later
        double m00 = idet
                * (A[IDX3D_11] * A[IDX3D_22] - A[IDX3D_12] * A[IDX3D_12]);
        double m01 = idet
                * (A[IDX3D_12] * A[IDX3D_02] - A[IDX3D_01] * A[IDX3D_22]);
        double m02 = idet
                * (A[IDX3D_01] * A[IDX3D_12] - A[IDX3D_11] * A[IDX3D_02]);

        double m11 = idet
                * (A[IDX3D_00] * A[IDX3D_22] - A[IDX3D_02] * A[IDX3D_02]);
        double m12 = idet
                * (A[IDX3D_01] * A[IDX3D_02] - A[IDX3D_00] * A[IDX3D_12]);

        double m22 = idet
                * (A[IDX3D_00] * A[IDX3D_11] - A[IDX3D_01] * A[IDX3D_01]);

        // multiply rhs, v = -Q^(-1)b
        v.x = m00 * b.x + m01 * b.y + m02 * b.z;
        v.y = m01 * b.x + m11 * b.y + m12 * b.z;
        v.z = m02 * b.x + m12 * b.y + m22 * b.z;

    } else {
        // constrain to line connecting vertices
        // (v2-v1)^TA(v2-v1) * t = - [v1^TA(v2-v1)+b^T(v2-v1)], t in [0, 1]
        v.subtract(v2, v1);
        double Adx = A[IDX3D_00] * v.x + A[IDX3D_01] * v.y + A[IDX3D_02] * v.z;
        double Ady = A[IDX3D_01] * v.x + A[IDX3D_11] * v.y + A[IDX3D_12] * v.z;
        double Adz = A[IDX3D_02] * v.x + A[IDX3D_12] * v.y + A[IDX3D_22] * v.z;

        double rhs = -b.dot(v) - v1.dot(Adx, Ady, Adz);
        double lhs = v.dot(Adx, Ady, Adz);

        double t = 0.5;
        if (lhs != 0) {
            double t = rhs / lhs;
            if (t < 0) {
                t = 0;
            } else if (t > 1) {
                t = 0;
            }
        }
        v.interpolate(v1, t, v2);
    }
}

double EdgeCollapseQuadric::cost(const Point3d &v) {

    // v^tQv + 2b.v+c
    double d = v.x * v.x * A[IDX3D_00] + v.y * v.y * A[IDX3D_11] + v.z * v.z * A[IDX3D_22]
            + 2 * v.x * v.y * A[IDX3D_01] + 2 * v.x * v.z * A[IDX3D_02]
            + 2 * v.y * v.z * A[IDX3D_12] + 2 * v.dot(b) + c;
    return d;
}

EdgeCollapseQuadricCost::EdgeCollapseQuadricCost() :
        Qv() {
}

void EdgeCollapseQuadricCost::init(PolygonMesh& mesh) {

    // construct error quadrics
    std::vector<EdgeCollapseQuadric> Kp;  // Kp for each face
    Kp.reserve(mesh.numFaces());
    for (size_t i = 0; i < mesh.numFaces(); i++) {
        SharedPolygon& face = mesh.getFace(i);
        double a = face->plane.normal.x;
        double b = face->plane.normal.y;
        double c = face->plane.normal.z;
        double d = face->plane.d;

        Kp.push_back(
                EdgeCollapseQuadric { Matrix3d(a * a, a * b, a * c, a * b,
                        b * b, b * c, a * c, b * c, c * c), Vector3d(a * d,
                        b * d, c * d), d * d });
    }

    Qv.reserve(mesh.numVertices());
    for (size_t i = 0; i < mesh.numVertices(); i++) {
        SharedVertex3d& vtx = mesh.getVertex(i);

        EdgeCollapseQuadric Qi;

        const std::vector<HalfEdge*>& hedges = vtx->getIncidentEdges();
        for (const HalfEdge* he : hedges) {
            size_t fidx = he->face->getIndex();

            Qi.add(Kp[fidx]);
        }

        //        std::cout << "Qv[" << i << "]: " << Qi.A(0,0) << " " << Qi.A(0,1) << " " << Qi.A(0,2) << " " << Qi.b[0] <<  std::endl;
        //        std::cout << "       " << Qi.A(1,0) << " " << Qi.A(1,1) << " " << Qi.A(1,2) << " " << Qi.b[1] <<  std::endl;
        //        std::cout << "       " << Qi.A(2,0) << " " << Qi.A(2,1) << " " << Qi.A(2,2) << " " << Qi.b[2] <<  std::endl;
        //        std::cout << "       " << Qi.b[0] << " " << Qi.b[1] << " " << Qi.b[2] << " " << Qi.c <<  std::endl;

        Qv.push_back(std::move(Qi));
    }

}

double EdgeCollapseQuadricCost::operator ()(PolygonMesh& mesh, HalfEdge &he,
        Point3d& v) {
    // lazy initialization
    if (Qv.empty()) {
        init(mesh);
    }

    EdgeCollapseQuadric ecq(Qv[he.head->getIndex()]);
    ecq.add(Qv[he.tail->getIndex()]);
    ecq.solve(*(he.head), *(he.tail), v);
    double c = ecq.cost(v);

    //    std::cout << "Edge " << he.idx << ": " << he.head->idx << " --> " << he.tail->idx << std::endl;
    //    std::cout << "    Q: " << ecq.A(0,0) << " " << ecq.A(0,1) << " " << ecq.A(0,2) << " " << ecq.b[0] <<  std::endl;
    //    std::cout << "       " << ecq.A(1,0) << " " << ecq.A(1,1) << " " << ecq.A(1,2) << " " << ecq.b[1] <<  std::endl;
    //    std::cout << "       " << ecq.A(2,0) << " " << ecq.A(2,1) << " " << ecq.A(2,2) << " " << ecq.b[2] <<  std::endl;
    //    std::cout << "       " << ecq.b[0] << " " << ecq.b[1] << " " << ecq.b[2] << " " << ecq.c <<  std::endl;
    //    std::cout << "    v: " << v.x << " " << v.y << " " << v.z << std::endl;
    //    std::cout << "    c: " << c << std::endl << std::endl;

    return c;
}

void EdgeCollapseQuadricCost::operator ()(HalfEdge &he) {

    // lazy initialization
    if (Qv.empty()) {
        return;
    }

    // move Qv over
    size_t hidx = he.head->getIndex();
    size_t tidx = he.tail->getIndex();
    size_t lastidx = Qv.size()-1;
    Qv[hidx].add(Qv[tidx]);
    Qv[tidx] = std::move(Qv[lastidx]);
    Qv.pop_back();  // remove
}

}
}
