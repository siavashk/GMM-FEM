#ifndef MAS_MESH_BV_HPP
#define MAS_MESH_BV_HPP

namespace mas {
namespace mesh {
using namespace mas::bvtree;

template<typename BV>
bool BoundablePolygon::updateBV(BV& bv) const {
    bool updated = false;
    for (SharedVertex3d& vtx : polygon->verts) {
        updated |= bv.updatePoint(*vtx);
    }
    return updated;
}

template<typename BV>
BVTree<SharedBoundablePolygon,BV>*  get_bv_tree(const PolygonMesh &mesh,
		double margin) {

	const std::vector<SharedPolygon>& faces = mesh.faces;
	std::vector<SharedBoundablePolygon> elems;
	elems.reserve(faces.size());

	for (const SharedPolygon& face : faces) {
		elems.push_back(std::make_shared<BoundablePolygon>(face));
	}
	return new BVTree<SharedBoundablePolygon,BV>(std::move(elems), margin);
}

template<typename BV>
SharedPolygon nearest_polygon(const Point3d& pnt, const PolygonMesh& mesh,
        Point3d& nearestPoint) {
    // build tree from polygons, get nearest boundable, cast to PBoundablePolygon
    auto tree = std::unique_ptr<BVTree<SharedBoundablePolygon,BV>>(
            get_bv_tree<BV>(mesh));
    SharedBoundablePolygon nearest = mas::bvtree::nearest_boundable(*tree, pnt, nearestPoint);
    return nearest->polygon;
}

template<typename BV>
SharedPolygon nearest_polygon(const Point3d& pnt, const Vector3d& dir,
        const PolygonMesh& mesh, Point3d& nearestPoint) {
    // build tree from polygons, get nearest boundable, cast to PBoundablePolygon
    auto tree = std::unique_ptr<BVTree<SharedBoundablePolygon,BV>>(
            get_bv_tree<BV>(mesh));
    SharedBoundablePolygon nearest = mas::bvtree::nearest_boundable(*tree, pnt, dir,
            nearestPoint);
    return nearest->polygon;
}

template<typename BV>
SharedBoundablePolygon nearest_polygon(const Point3d& pnt,
        const BVTree<SharedBoundablePolygon,BV>& bvt, Point3d& nearestPoint) {
    // build tree from polygons, get nearest boundable, cast to PBoundablePolygon
    SharedBoundablePolygon nearest = mas::bvtree::nearest_boundable(bvt, pnt,
            nearestPoint);
    return nearest;
}

template<typename BV>
SharedBoundablePolygon nearest_polygon(const Point3d& pnt, const Vector3d& dir,
        const BVTree<SharedBoundablePolygon,BV>& bvt, Point3d& nearestPoint) {
    // build tree from polygons, get nearest boundable, cast to PBoundablePolygon
    SharedBoundablePolygon nearest = mas::bvtree::nearest_boundable(bvt, pnt, dir,
            nearestPoint);

    return nearest;
}

template<typename BV>
bool is_inside(const Point3d& pnt, const PolygonMesh& mesh, double tol,
        int maxRetries) {

    // build tree from polygons, get nearest boundable, cast to PBoundablePolygon
    double margin = tol;
    if (margin < 0) {
        margin = 0;
    }
    auto tree = std::unique_ptr<BVTree<SharedBoundablePolygon,BV>>(
            get_bv_tree<BV>(mesh, margin));
    InsideMeshQueryData data;
    return is_inside(pnt, *tree, data, tol, maxRetries);
}

template<typename BV>
bool is_inside(const Point3d& pnt, const PolygonMesh& mesh, InsideMeshQueryData& data, double tol,
        int maxRetries) {

    // build tree from polygons, get nearest boundable, cast to PBoundablePolygon
    double margin = tol;
    if (margin < 0) {
        margin = 0;
    }
    auto tree = std::unique_ptr<BVTree<SharedBoundablePolygon,BV>>(get_bv_tree<BV>(mesh, margin));
    return is_inside(pnt, *tree, data, tol, maxRetries);
}

template<typename BV>
bool is_inside(const Point3d& pnt, const BVTree<SharedBoundablePolygon,BV>& tree, double tol,
        int maxRetries) {

    InsideMeshQueryData data;
    return is_inside(pnt, tree, data, tol, maxRetries);
}

template<typename BV>
bool is_inside(const Point3d& pnt, const BVTree<SharedBoundablePolygon,BV>& bvt,
        InsideMeshQueryData& data, double tol, int maxRetries) {

    const BVNode<SharedBoundablePolygon,BV>& root = bvt.getRoot();
    Point3d centroid;

    if (tol < 0) {
        // default tolerance
        double r = root.bv->getBoundingSphere(centroid);
        tol = 1e-12 * r;
    }

    // initialize
    data.reset();

    // check if is inside root bounding box
    if (!root.bv->intersectsSphere(pnt, tol)) {
        return false;
    }

    // find the nearest polygon
    data.nearestFace = nearest_polygon(pnt, bvt, data.nearestPoint);

    // cast direction
    Vector3d dir(data.nearestPoint);
    dir.subtract(pnt);

    // first check if within tolerance
    if (dir.normSquared() <= tol * tol) {
        data.in = true;
        data.on = true;
        return true;
    }

    // check if closest point is within the center of the triangle
    // i.e. not on an edge or vertex
    // Because then we just need to check the face normal
    Vector3d bary;
    SharedPolygon tri;
    poly_contains_coordinate(data.nearestPoint,
            *(data.nearestFace), bary, tri);

    // make sure within eps of boundary of face
    double eps = 1e-12;
    if (bary.x > eps && bary.y > eps && bary.z > eps) {
        // check normal
        if (data.nearestFace->polygon->plane.normal.dot(dir) > 0) {
            data.in = true;
            return true;
        } else {
            data.in = false;
            return false;
        }
    }

    // resort to ray-cast, starting with aimed at mid-face
    tri->computeCentroid(centroid);
    dir.set(centroid);
    dir.subtract(pnt);

    // uniform random distribution for direction generation
    std::uniform_real_distribution<double> uniform(-1, 1);
    std::default_random_engine randomengine;

    Polygon* prevPoly = tri.get();

    data.nRetries = 0;
    do {
        data.nHits = 0;
        data.unsure = false;
        BoundablePolygon* unsureFace = nullptr;

        std::vector<BVNode<SharedBoundablePolygon,BV>*> bvnodes;
        bvt.intersectRay(pnt, dir, bvnodes);
        double dmin = mas::math::DOUBLE_INFINITY;
        data.in = false;

        // find closest face among nodes in direction of dir
        for (BVNode<SharedBoundablePolygon,BV>* node : bvnodes) {
            for (SharedBoundablePolygon& boundable : node->elems) {

                // dynamic cast to PBoundablePolygon
                SharedBoundablePolygon poly = std::dynamic_pointer_cast<
                        BoundablePolygon>(boundable);
                if (poly != nullptr) {

                    // intersect ray with face
                    double d = fabs(
                            poly->distanceToPoint(pnt, dir, data.nearestPoint,
                                    bary, tri));

                    // check if ray hits face
                    if (d < dmin) {

                        // check that it hits the face in the correct direction
                        Vector3d ndir(data.nearestPoint);
                        ndir.subtract(pnt);
                        if (ndir.dot(dir) > 0) {

                            dmin = d;
                            // make sure barycentric coordinates are not on edges
                            if (bary.x >= eps && bary.x <= 1 - eps
                                    && bary.y >= eps && bary.y <= 1 - eps
                                    && bary.z >= eps && bary.z <= 1 - eps) {
                                data.nHits++;
                                data.unsure = false;
                                if (poly->polygon->plane.normal.dot(dir) > 0) {
                                    data.in = true;
                                } else {
                                    data.in = false;
                                }
                            } else {
                                data.unsure = true;
                                unsureFace = poly.get();
                            }
                        }
                    }
                }

            }
        }

        if (!data.unsure) {
            return data.in;
        }

        // pick next direction as centroid of closest unsure face
        const SharedPolygon& nextPoly = unsureFace->getTriangulation()[0];
        if (nextPoly.get() == prevPoly) {
            // randomize direction
            do {
                dir.x = uniform(randomengine);
                dir.y = uniform(randomengine);
                dir.z = uniform(randomengine);
            } while (dir.norm() == 0);
            dir.normalize();

            prevPoly = nullptr;
        } else {
            nextPoly->computeCentroid(centroid);
            dir.set(centroid);
            dir.subtract(pnt);
            prevPoly = nextPoly.get();
        }

        data.nRetries++;
    } while (data.nRetries < maxRetries);

    // failed to converge
    data.unsure = true;
    return false;
}

} // end mas
} // end mesh

#endif
