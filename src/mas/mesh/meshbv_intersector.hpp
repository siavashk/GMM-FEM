namespace mas {
namespace mesh {

using namespace bvtree;

/**
 * Intersects the faces of two triangular meshes. The process is accelerated
 * using the default bounding volume hierarchy of each mesh.  The results
 * are returned in the array <code>intersections</code>, which contains
 * information on each detected pair of intersecting triangles.
 * <p>
 * This method detects only face (triangle) intersections; it does
 * not detect if one mesh is completely inside the other.
 *
 * @param intersections returns information for each pair of intersecting
 * triangles.
 * @param mesh1 first mesh to be intersected
 * @param mesh2 second mesh to be intersected
 * @return true if intersecting faces are detected
 */
template<typename BV>
std::vector<TriangleTriangleIntersection> BVIntersector::intersectMeshMesh(
        const PolygonMesh& mesh1, const PolygonMesh& mesh2) const {

    std::unique_ptr<BVTree<SharedBoundablePolygon, BV>> bv1(
            get_bv_tree<BV>(mesh1, epsilon));
    std::unique_ptr<BVTree<SharedBoundablePolygon, BV>> bv2(
            get_bv_tree<BV>(mesh2, epsilon));

    return intersectMeshMesh(*bv1, *bv2);
}

/**
 * Intersects the faces of two triangular meshes, whose faces are contained
 * within supplied bounding volume hierarchies.  The results are returned in
 * the array <code>intersections</code>, which contains information on each
 * detected pair of intersecting triangles.
 *
 * <p> This method detects only face (triangle) intersections; it does not
 * detect if one mesh is completely inside the other.
 *
 * @param intersections returns information for each pair of intersecting
 * triangles.
 * @param bvh1 bounding volume hierarchy for the first mesh to be intersected
 * @param bvh2 bounding volume hierarchy for the second mesh to be intersected
 */
template<typename BV1, typename BV2>
std::vector<TriangleTriangleIntersection> BVIntersector::intersectMeshMesh(
        const BVTree<SharedBoundablePolygon, BV1>& bvh1,
        const BVTree<SharedBoundablePolygon, BV2>& bvh2) const {

    std::vector<TriangleTriangleIntersection> intersections;

    std::vector<BVNode<SharedBoundablePolygon, BV1>*> nodes1;
    std::vector<BVNode<SharedBoundablePolygon, BV2>*> nodes2;

    bvh1.intersectTree(bvh2, nodes1, nodes2);

    for (int i = 0; i < nodes1.size(); i++) {
        intersectBoundingVolumeTriangles(intersections, *nodes1[i], *nodes2[i]);
    }

    return intersections;
}

/**
 * Intersects the faces of a triangular mesh with a plane. The process is
 * accelerated using the default bounding volume hierarchy of the mesh.
 * The results are returned in the array <code>intersections</code>, which
 * contains information on each detected face-plane intersection.
 *
 * @param intersections returns information for each face-plane intersection.
 * @param mesh the mesh to be intersected
 * @param plane the plane to be intersected
 * @return true if the mesh and the plane intersect
 */
template<typename BV>
std::vector<TrianglePlaneIntersection> BVIntersector::intersectMeshPlane(
        const PolygonMesh& mesh, const Plane& plane) const {

    std::unique_ptr<BVTree<SharedBoundablePolygon, BV>> bvh(
            get_bv_tree<BV>(mesh, epsilon));
    return intersectMeshPlane(*bvh, plane);
}

/**
 * Intersects the faces of a triangular mesh with a plane. The faces
 * of the mesh are contained within a supplied bounding volume hierarchy.
 * The results are returned in the array <code>intersections</code>, which
 * contains information on each detected face-plane intersection.
 *
 * @param intersections returns information for each face-plane intersection.
 * @param bvh bounding volume hierarchy containing the mesh faces
 * @param plane the plane to be intersected
 * @return true if the mesh and the plane intersect
 */
template<typename BV>
std::vector<TrianglePlaneIntersection> BVIntersector::intersectMeshPlane(
        const BVTree<SharedBoundablePolygon, BV>& bvh,
        const Plane& plane) const {

    std::vector<BVNode<SharedBoundablePolygon, BV>*> nodes;
    bvh.intersectPlane(plane, nodes);

    double eps = bvh.getRadius() * epsilon;
    // XXX myTriIntersector.setEpsilon(eps);

    std::vector<TrianglePlaneIntersection> intersections;
    for (int i = 0; i < nodes.size(); i++) {
        intersectBoundingVolumeTrianglePlanes(intersections, *nodes[i], plane);
    }

    return intersections;
}

/**
 * Intersects the faces of a triangular mesh with a line. The process is
 * accelerated using the default bounding volume hierarchy of the mesh.
 * The results are returned in the array <code>intersections</code>, which
 * contains information on each detected face-line intersection.
 *
 * @param intersections returns information for each line-plane intersection.
 * @param mesh the mesh to be intersected
 * @param line the line to be intersected
 * @return true if the mesh and the line intersect
 */
template<typename BV>
std::vector<TriangleLineIntersection> BVIntersector::intersectMeshLine(
        const PolygonMesh& mesh, const Line& line) const {

    std::unique_ptr<BVTree<SharedBoundablePolygon, BV>> bvh(
            get_bv_tree<BV>(mesh, epsilon));
    return intersectMeshLine(*bvh, line);
}

/**
 * Intersects the faces of a triangular mesh with a line. The faces
 * of the mesh are contained within a supplied bounding volume hierarchy.
 * The results are returned in the array <code>intersections</code>, which
 * contains information on each detected face-line intersection.
 *
 * @param intersections returns information for each face-line intersection.
 * @param bvh bounding volume hierarchy containing the mesh faces
 * @param line the line to be intersected
 */
template<typename BV>
std::vector<TriangleLineIntersection> BVIntersector::intersectMeshLine(
        const BVTree<SharedBoundablePolygon, BV>& bvh, const Line& line) const {

    std::vector<TriangleLineIntersection> intersections;
    std::vector<BVNode<SharedBoundablePolygon, BV>*> nodes;
    bvh.intersectLine(line.getOrigin(), line.getDirection(), nodes);

    double eps = bvh.getRadius() * epsilon;
    // myTriIntersector.setEpsilon(eps);

    for (int i = 0; i < nodes.size(); i++) {
        intersectBoundingVolumeTriangleLines(intersections, *nodes[i], line);
    }

    return intersections;
}

template<typename BV1, typename BV2>
void BVIntersector::intersectBoundingVolumeTriangles(
        std::vector<TriangleTriangleIntersection>& intersections,
        const BVNode<SharedBoundablePolygon, BV1>& node1,
        const BVNode<SharedBoundablePolygon, BV2>& node2) const {

    const std::vector<SharedBoundablePolygon>& elems1 = node1.getElements();
    const std::vector<SharedBoundablePolygon>& elems2 = node2.getElements();

    for (const auto& bpoly1 : elems1) {

        // for each triangle
        const std::vector<SharedPolygon>& tris = bpoly1->getTriangulation();

        for (const SharedPolygon& tri : tris) {
            const Point3d& p0 = *(tri->verts[0]);
            const Point3d& p1 = *(tri->verts[1]);
            const Point3d& p2 = *(tri->verts[2]);

            for (const auto& bpoly2 : elems2) {

                // for each triangle
                const std::vector<SharedPolygon>& tris2 =
                        bpoly2->getTriangulation();

                for (const SharedPolygon& tri2 : tris2) {

                    const Point3d& q0 = *(tri2->verts[0]);
                    const Point3d& q1 = *(tri2->verts[1]);
                    const Point3d& q2 = *(tri2->verts[2]);

                    std::vector<Point3d> pnts;
                    int num = myTriIntersector.intersectTriangleTriangle(p0, p1,
                            p2, q0, q1, q2, pnts);

                    if (num > 0) {
                        intersections.push_back(
                                TriangleTriangleIntersection(tri, tri2,
                                        std::move(pnts)));
                    }
                } // end looping through node2 triangles
            } // end looping through node2 elements
        }  // end looping through node1 triangles
    } // end looping through node1 elements
}

template<typename BV>
void BVIntersector::intersectBoundingVolumeTriangleLines(
        std::vector<TriangleLineIntersection>& intersections,
        const BVNode<SharedBoundablePolygon, BV>& node, const Line& l) const {

    Vector3d duv;
    const Vector3d& dir = l.getDirection();
    const Point3d& orig = l.getOrigin();
    std::shared_ptr<Line> pline = std::make_shared<Line>(l);

    const std::vector<SharedBoundablePolygon>& elems = node.getElements();

    for (const auto& bpoly : elems) {
        // for each triangle
        const std::vector<SharedPolygon>& tris = bpoly->getTriangulation();

        for (const SharedPolygon& tri : tris) {

            const SharedHalfEdge& he0 = tri->getFirstHalfEdge();

            const Point3d& p0 = *(he0->head);
            const Point3d& p1 = *(he0->next->head);
            const Point3d& p2 = *(he0->next->next->head);

            int isect = myTriIntersector.intersectTriangleLine(p0, p1, p2, orig,
                    dir, duv);

            if (isect > 0) {
                std::vector<Point3d> pnts(1);
                pnts[0].setZero();
                pnts[0].scaledAdd(1 - (duv.y + duv.z), p0);
                pnts[0].scaledAdd(duv.y, p1);
                pnts[0].scaledAdd(duv.z, p2);
                intersections.push_back(
                        TriangleLineIntersection(tri, pline, std::move(pnts)));
            }
        }
    }
}

template<typename BV>
void BVIntersector::intersectBoundingVolumeTrianglePlanes(
        std::vector<TrianglePlaneIntersection>& intersections,
        const BVNode<SharedBoundablePolygon, BV>& node, const Plane& p) const {

    std::shared_ptr<Plane> pplane = std::make_shared<Plane>(p);

    const std::vector<SharedBoundablePolygon>& elems = node.getElements();

    for (const auto& bpoly : elems) {
        // for each triangle
        const std::vector<SharedPolygon>& tris = bpoly->getTriangulation();

        for (const SharedPolygon& tri : tris) {

            const SharedHalfEdge& he0 = tri->getFirstHalfEdge();

            const Point3d& p0 = *(he0->head);
            const Point3d& p1 = *(he0->next->head);
            const Point3d& p2 = *(he0->next->next->head);

            std::vector<Point3d> pnts;
            int num = myTriIntersector.intersectTrianglePlane(p0, p1, p2, p,
                    pnts);

            if (num > 0) {
                intersections.push_back(
                        TrianglePlaneIntersection(tri, pplane,
                                std::move(pnts)));
            }
        }
    }
}

}
}
