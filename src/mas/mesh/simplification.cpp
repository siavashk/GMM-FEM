#include "mas/mesh/simplification.h"

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
    double d = v.x * v.x * A(0, 0) + v.y * v.y * A(1, 1) + v.z * v.z * A(2, 2)
            + 2 * v.x * v.y * A(1, 2) + 2 * v.x * v.z * A(1, 3)
            + 2 * v.y * v.z * A(2, 3) + 2 * v.dot(b) + c;
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
        Qv.push_back(std::move(Qi));
    }
    Kp.clear(); // free some memory
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

    return ecq.cost(v);
}

}
}
