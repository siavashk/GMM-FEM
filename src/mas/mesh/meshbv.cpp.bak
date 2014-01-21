#include "mas/core/math.h"
#include "mas/mesh/meshbv.h"

namespace mas {
namespace mesh {

BoundablePolygon::BoundablePolygon(const PPolygon poly)
: polygon(), tris() {
	setPolygon(poly);
}

void BoundablePolygon::getCentroid(Point3d &c) const {
	c.set(centroid);
}

void BoundablePolygon::computeCentroid() {
	centroid.setZero();
	PVertex3dList &verts = polygon->verts;
	if (verts.size() == 0) {
		return;
	}

	for (PVertex3dList::iterator pit = verts.begin();
			pit < verts.end(); pit++) {
		centroid.add( *(*pit) );
	}
	centroid.scale(1.0/polygon->verts.size());
}

void BoundablePolygon::triangulate() {
	if (polygon->numVertices() == 3) {
		tris.clear();
		tris.push_back(polygon);
	} else {
		tris = MeshFactory::triangulate(polygon);
	}
}

PPolygonList BoundablePolygon::getTriangulation() const {
	PPolygonList out;
	if (polygon->numVertices() == 3) {
		out.push_back(polygon);
	} else if (tris.size() == polygon->numVertices()-2) {
		out = tris;
	} else {
		out = MeshFactory::triangulate(polygon);
	}
	return out;
}

void BoundablePolygon::setPolygon(const PPolygon poly) {
	polygon = poly;
	computeCentroid();
	triangulate();
}

void BoundablePolygon::update() {
	computeCentroid();
}

void BoundablePolygon::getCovariance(const Point3d &center,
		Matrix3d &cov) const {

	cov.setZero();
	PVertex3dList &verts = polygon->verts;
	if (verts.size() == 0) {
		return;
	}

	Point3d diff;
	for (PVertex3dList::iterator pit = verts.begin();
			pit < verts.end(); pit++) {
		diff.set(*(*pit));
		diff.subtract(center);
		cov.addOuterProduct(diff, diff);
	}
	cov.scale(1.0/polygon->verts.size());
}

bool BoundablePolygon::updateBV(BoundingVolume* bv) const {
	PVertex3dList &verts = polygon->verts;
	for (PVertex3dList::iterator pit = verts.begin();
			pit < verts.end(); pit++) {
		bv->updatePoint( *(*pit) );
	}
}

double BoundablePolygon::distanceToPoint(const Point3d &pnt,
		Point3d &nearest, Polygon &tri, Vector3d &bary) const {

	double distance = 0;
	// triangulate if required
	if (polygon->numVertices() == 3) {
		distance = distance_to_triangle(pnt, *(polygon->verts[0]),
				*(polygon->verts[1]), *(polygon->verts[2]), nearest, bary);
		tri.set(*polygon);
	} else {

		// triangulate if not already done
		const PPolygonList &ltris = getTriangulation();

		double distMin = math::DOUBLE_INFINITY;
		double distTmp;
		Point3d nearestTmp;
		Vector3d baryTmp;
		for (PPolygon ltri : ltris) {
			distTmp = distance_to_triangle(pnt, *(ltri->verts[0]),
					*(ltri->verts[1]), *(ltri->verts[2]), nearestTmp, baryTmp);
			if (distTmp < distMin) {
				distMin = distTmp;
				nearest.set(nearestTmp);
				bary.set(baryTmp);
				tri.set(*ltri);
			}
		}
		distance = distMin;
	}

	return distance;
}

double BoundablePolygon::distanceToPoint(const Point3d &pnt,
		Point3d &nearest) const {
	Polygon tri;
	Vector3d bary;
	return distanceToPoint(pnt, nearest, tri, bary);
}

double BoundablePolygon::distanceToPoint(const Point3d &pnt,
		const Vector3d &dir, Point3d &nearest, Polygon &tri,
		Vector3d &bary) const {

	// intersect line with plane
	double distance = polygon->plane.distanceSigned(pnt);
	double denom = dir.dot(polygon->plane.normal);

	// Is the plane parallel to the line?
	if (denom == 0) {
		if (distance == 0) {

			// check if is inside triangles
			if (polygon->numVertices() == 3) {
				// check if is in triangle
				if (point_in_triangle(pnt, *(polygon->verts[0]),
						*(polygon->verts[1]), *(polygon->verts[2]),
						bary)) {
					nearest.set(pnt);
					tri.set(*polygon);
					return 0;
				}
			} else {

				// triangulate if not already done
				const PPolygonList &ltris = getTriangulation();

				for (PPolygon ltri : ltris) {
					Vector3d baryTmp;
					bool inside = point_in_triangle(pnt, *(ltri->verts[0]),
							*(ltri->verts[1]), *(ltri->verts[2]), baryTmp);
					if (inside) {
						nearest.set(pnt);
						bary.set(baryTmp);
						tri.set(*ltri);
						return 0;
					}
				}

			}

			// not inside triangle, do 2D intersection
			Vertex3d &o = *(polygon->verts[0]);
			Vector3d v1 = Vector3d(*(polygon->verts[1]));
			v1.subtract(o);
			Vector3d v2;
			v2.cross(v1, polygon->plane.normal);

			Vector3d vpnt = Vector3d(pnt);
			vpnt.subtract(o);

			// point coordinates
			double px = vpnt.dot(v1);
			double py = vpnt.dot(v2);

			// direction coordinates
			double dx = dir.dot(v1);
			double dy = dir.dot(v2);

			// Objective function: f(x,y) = dx*(y-py)-dy*(x-px)
			// If zero or opposite signs, points cross line
			double p0x, p0y, p1x, p1y, dpx, dpy;
			double f0, f1;
			PVertex3d prevVtx = (polygon->verts.back());
			p1x = prevVtx->dot(v1);
			p1y = prevVtx->dot(v2);
			f1 = dx*(p1y-py)-dy*(p1x-px);
			Point3d pint;

			double dmin = math::DOUBLE_INFINITY;
			bool intersectFound = false;

			for (PVertex3dList::iterator pit = polygon->verts.begin();
					pit < polygon->verts.end(); pit++) {

				PVertex3d pvtx = *pit;
				Vertex3d &vtx = *(*pit);
				f0 = f1;
				p0x = p1x;
				p0y = p1y;
				p1x = vtx.dot(v1);
				p1y = vtx.dot(v1);
				f1 = dx*(p1y-py)-dy*(p1x-px);

				if (f1 == 0) {
					// falls on the line
					intersectFound = true;
					double d = vtx.distance(pnt);
					if (d < dmin) {
						dmin = d;
						nearest.set(vtx);
						// vertex is on the line
						bary.x = 1;
						bary.y = 0;
						bary.z = 0;
						// XXX might want to make true triangle
						tri.set(pvtx, pvtx, pvtx);
					}
				} else if (f0*f1 < 0) {
					// intersection
					intersectFound = true;
					dpx = p1x-p0x;
					dpy = p1y-p0y;

					double s = 1.0/(dx*dpy-dpx*dy);
					double u = (-dpx*px*dy + dpx*dx*py + dx*dpy*p0x
							- dx*dpx*p0y)*s;
					double v = (-dpy*px*dy + dpy*dx*py + dy*dpy*p0x
							- dy*dpx*p0y)*s;
					pint.setZero();
					pint.scaledAdd(u, v1);
					pint.scaledAdd(v, v2);

					double d = pint.distance(pnt);
					if (d < dmin) {
						dmin = d;
						nearest.set(pint);

						// point straddles
						// XXX might want to make true triangle
						tri.set(prevVtx, pvtx, pvtx);
						bary.z = 0;
						Vector3d tmp = Vector3d(nearest);
						tmp.subtract(vtx);

						Vector3d tmp2 = Vector3d(*prevVtx);
						tmp2.subtract(vtx);

						bary.x = tmp.dot(tmp2)/tmp2.dot(tmp2);
						bary.y = 1.0-bary.x;
					}

					prevVtx = *pit;
				}

			}

			if (intersectFound) {
				return dmin;
			}

		} else {
			// parallel, doesn't intersect
			// negative direction
			nearest.x = math::signum(dir.x)*math::DOUBLE_INFINITY;
			nearest.y = math::signum(dir.y)*math::DOUBLE_INFINITY;
			nearest.z = math::signum(dir.z)*math::DOUBLE_INFINITY;
			return math::DOUBLE_INFINITY;
		}
	} else {

		// get intersection point
		double t = -distance/denom;
		nearest.scaledAdd(pnt, t, dir);

		// check if intersects triangle
		if (polygon->numVertices() == 3) {
			// check if is in triangle
			if (point_in_triangle(nearest, *(polygon->verts[0]),
					*(polygon->verts[1]), *(polygon->verts[2]),
					bary)) {
				tri.set(*polygon);
				return t*dir.norm();
			}
		} else {

			// triangulate if not already done
			const PPolygonList &ltris = getTriangulation();

			for (PPolygon ltri : ltris) {
				bool inside = point_in_triangle(nearest, *(ltri->verts[0]),
						*(ltri->verts[1]), *(ltri->verts[2]), bary);
				if (inside) {
					return t*dir.norm();
				}
			}

		}
	}

	// not in any triangle
	return math::DOUBLE_INFINITY;
}

double BoundablePolygon::distanceToPoint(const Point3d &pnt,
		const Vector3d &dir, Point3d &nearest) const {
	Polygon tri;
	Vector3d bary;
	return distanceToPoint(pnt, dir, nearest, tri, bary);
}

PBoundablePolygon BoundableFactory::createBoundablePolygon(
		const PPolygon poly) {
	return std::make_shared<BoundablePolygon>(poly);
}

// methods for meshes
POBBTree get_obb_tree(const PolygonMesh &mesh, double margin) {
	return get_bv_tree_T<OBB>(mesh, margin);
}

PAABBTree get_aabb_tree(const PolygonMesh &mesh, double margin) {
	return get_bv_tree_T<AABB>(mesh, margin);
}

PBSTree get_bs_tree(const PolygonMesh &mesh, double margin) {
	return get_bv_tree_T<BoundingSphere>(mesh, margin);
}

PPolygon nearest_polygon(const Point3d &pnt,
		const PolygonMesh &mesh, Point3d &nearestPoint) {
	// build tree from polygons, get nearest boundable, cast to PBoundablePolygon
	POBBTree tree = get_obb_tree(mesh);
	PBoundable nearest = mas::bvtree::nearest_boundable(tree, pnt, nearestPoint);
	PBoundablePolygon poly = std::static_pointer_cast<BoundablePolygon>(nearest);
	return poly->polygon;
}

PPolygon nearest_polygon(const Point3d &pnt,
		const Vector3d &dir, const PolygonMesh &mesh,
		Point3d &nearestPoint) {
	// build tree from polygons, get nearest boundable, cast to PBoundablePolygon
	POBBTree tree = get_obb_tree(mesh);
	PBoundable nearest = mas::bvtree::nearest_boundable(tree, pnt, dir, nearestPoint);
	PBoundablePolygon poly = std::static_pointer_cast<BoundablePolygon>(nearest);
	return poly->polygon;
}

PBoundablePolygon nearest_polygon(const Point3d &pnt, const PBVTree bvt, Point3d &nearestPoint) {
	// build tree from polygons, get nearest boundable, cast to PBoundablePolygon
	PBoundable nearest = mas::bvtree::nearest_boundable(bvt, pnt, nearestPoint);
	PBoundablePolygon poly = std::dynamic_pointer_cast<BoundablePolygon>(nearest);
	return poly;
}

PBoundablePolygon nearest_polygon(const Point3d &pnt, const Vector3d &dir,
		const PBVTree bvt, Point3d &nearestPoint) {
	// build tree from polygons, get nearest boundable, cast to PBoundablePolygon
	PBoundable nearest = mas::bvtree::nearest_boundable(bvt, pnt, dir, nearestPoint);
	PBoundablePolygon poly = std::dynamic_pointer_cast<BoundablePolygon>(nearest);
	return poly;
}

int is_inside(const Point3d &pnt, const PolygonMesh &mesh, double tol,
		int numRetries, double baryEpsilon) {
	// build tree from polygons, get nearest boundable, cast to PBoundablePolygon
	double margin = tol;
	if (margin < 0) {
		margin = 0;
	}
	POBBTree tree = get_obb_tree(mesh, margin);
	//PBSTree tree = get_bs_tree(mesh);
	return is_inside(pnt, mesh, tree, tol, numRetries, baryEpsilon);
}

int is_inside(const Point3d &pnt, const PolygonMesh &mesh,
		const PBVTree bvt, double tol, int numRetries, double baryEpsilon) {

	int inside = is_inside_or_on(pnt, mesh, bvt, tol, numRetries, baryEpsilon);

	if (inside == MESH_INSIDE_ON) {
		inside = MESH_INSIDE_TRUE;
	}
	return inside;
}

int is_inside_or_on(const Point3d &pnt, const PolygonMesh &mesh,
		const PBVTree bvt, double tol, int numRetries,
		double baryEpsilon) {
	InsideMeshQueryData data;
	return is_inside_or_on(pnt, mesh, bvt, data, tol, numRetries, baryEpsilon);
}

int is_inside_or_on(const Point3d &pnt, const PolygonMesh &mesh,
		const PBVTree bvt, InsideMeshQueryData &data, double tol, int numRetries,
		double baryEpsilon) {

	data.in = false;
	data.nHits = 0;
	data.nRetries = 0;
	data.unsure = false;
	data.nearestFace = NULL;
	data.nearestPoint = pnt;

	if (tol < 0) {
		// default tolerance
		double r = bvt->getRoot()->bv->getBoundingSphere().getRadius();
		tol = 1e-12*r;
	}

	// check if is inside root bounding box
	PBVNode root = bvt->getRoot();
	if (!root->bv->intersectsSphere(pnt, tol)) {
		return MESH_INSIDE_FALSE;
	}

	// find the nearest polygon
	data.nearestFace = nearest_polygon(pnt, bvt, data.nearestPoint);

	// first check if within tolerance
	if (pnt.distance(data.nearestPoint) <= tol) {
		data.in = true;
		data.on = true;
		return MESH_INSIDE_ON;
	}

	/// first direction is away from nearest face (fastest when
	// outside convex surface)
	Vector3d dir = Vector3d(pnt);
	dir.subtract(data.nearestPoint);
	dir.normalize();

	const PPolygonList &meshFaces = mesh.faces;
	Point3d centroid;

	data.nRetries = 0;
	do {
		data.nHits = 0;
		data.unsure = false;

		PBVNodeList bvnodes;
		bvt->intersectRay(pnt, dir, bvnodes);

		for (PBVNode node : bvnodes) {

			for (PBoundable boundable : node->elems) {

				// dynamic cast to PBoundablePolygon
				PBoundablePolygon poly =
						std::dynamic_pointer_cast<BoundablePolygon>(boundable);
				if (poly != NULL) {
					// intersect ray with face
					Vector3d bary;
					Polygon tri;
					double d = poly->distanceToPoint(pnt, dir, data.nearestPoint, tri, bary);

					// check if close to face
					if (d < tol) {
						data.in = true;
						data.on = true;
						return MESH_INSIDE_TRUE;
					}

					// check if ray hits face
					if (d != math::DOUBLE_INFINITY && d != -math::DOUBLE_INFINITY) {
						// check that it hits the face in the correct direction
						Vector3d ndir(data.nearestPoint);
						ndir.subtract(pnt);
						if (ndir.dot(dir) > 0) {
							// make sure barycentric coordinates are not on edges
							if (bary.x >= baryEpsilon && bary.x <= 1-baryEpsilon
									&& bary.y >= baryEpsilon && bary.y <= 1-baryEpsilon
									&& bary.z >= baryEpsilon && bary.z <= 1-baryEpsilon) {
								data.nHits++;
							} else {
								data.unsure = true;
								break;
							}
						}
					}
				}

			}

			if (data.unsure) {
				break;
			}
		}

		if (!data.unsure) {
			// we are sure about our intersection count
			if (data.nHits % 2 == 1) {
				data.in = true;
				return MESH_INSIDE_TRUE;
			} else {
				return MESH_INSIDE_FALSE;
			}
		}

		// randomize direction and try again
		// do {
		//	dir.x = ((double)rand())/((double)RAND_MAX)-0.5;
		//	dir.y = ((double)rand())/((double)RAND_MAX)-0.5;
		//	dir.z = ((double)rand())/((double)RAND_MAX)-0.5;
		// } while (dir.norm() == 0);
		// dir.normalize();

		// pick a random face and shoot towards it
		int iface = rand() % meshFaces.size();
		meshFaces[iface]->computeCentroid(centroid);
		dir.subtract(pnt, centroid);
		if (dir.norm() <= tol) {
			data.in = true;
			data.on = true;
			return MESH_INSIDE_TRUE;
		}
		dir.normalize();

		data.nRetries++;
	} while (data.nRetries < numRetries);

	data.unsure = true;
	return MESH_INSIDE_UNSURE;	// unsure

}

bool poly_contains_coordinate(Point3d pnt, PBoundablePolygon bpoly, Polygon &tri, Vector3d &bary) {
	// check if is inside triangles
	PPolygon &polygon = bpoly->polygon;
	if (polygon->numVertices() == 3) {
		// check if is in triangle
		if (point_in_triangle(pnt, *(polygon->verts[0]),
				*(polygon->verts[1]), *(polygon->verts[2]),
				bary)) {
			tri.set(*polygon);
			return true;
		}
		return false;
	} else {

		// triangulate if not already done
		const PPolygonList &ltris = bpoly->getTriangulation();

		for (PPolygon ltri : ltris) {
			Vector3d baryTmp;
			bool inside = point_in_triangle(pnt, *(ltri->verts[0]),
					*(ltri->verts[1]), *(ltri->verts[2]), baryTmp);
			if (inside) {
				bary.set(baryTmp);
				tri.set(*ltri);
				return true;
			}
		}
	}
	return false;
}

// Faster?
bool is_inside(const Point3d &pnt, const PBVTree bvt, InsideMeshQueryData &data, double tol, int maxRetries) {

	if (tol < 0) {
		// default tolerance
		double r = bvt->getRoot()->bv->getBoundingSphere().getRadius();
		tol = 1e-12*r;
	}

	data.in = false;
	data.on = false;

	// check if is inside root bounding box
	PBVNode root = bvt->getRoot();
	if (!root->bv->intersectsSphere(pnt, tol)) {
		return false;
	}

	PBoundable nearestBoundable = nearest_boundable(bvt, pnt, data.nearestPoint);
	data.nearestFace = std::static_pointer_cast<BoundablePolygon>(nearestBoundable);
	Vector3d dir(data.nearestPoint);
	dir.subtract(pnt);

	// check if on
	if (dir.norm() < tol) {
		data.in = true;
		data.on = true;
		return true;
	}
	data.on = false;

	// check if on edge or vertex


	Vector3d bary;
	Polygon tri;
	bool success = poly_contains_coordinate(data.nearestPoint, data.nearestFace, tri, bary);
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

	// resort to raycast
	Point3d centroid;
	tri.computeCentroid(centroid);
	dir.set(centroid);
	dir.subtract(pnt);

	data.nRetries = 0;
	do {
		data.nHits = 0;
		data.unsure = false;

		PBVNodeList bvnodes;
		bvt->intersectRay(pnt, dir, bvnodes);
		PBoundablePolygon unsureFace;

		for (PBVNode node : bvnodes) {
			for (PBoundable boundable : node->elems) {

				// dynamic cast to PBoundablePolygon
				PBoundablePolygon poly =
						std::dynamic_pointer_cast<BoundablePolygon>(boundable);
				if (poly != NULL) {
					// intersect ray with face
					double d = poly->distanceToPoint(pnt, dir, data.nearestPoint, tri, bary);

					// check if ray hits face
					if (d != math::DOUBLE_INFINITY && d != -math::DOUBLE_INFINITY) {
						// check that it hits the face in the correct direction
						Vector3d ndir(data.nearestPoint);
						ndir.subtract(pnt);
						if (ndir.dot(dir) > 0) {
							// make sure barycentric coordinates are not on edges
							if (bary.x >= eps && bary.x <= 1-eps
									&& bary.y >= eps && bary.y <= 1-eps
									&& bary.z >= eps && bary.z <= 1-eps) {
								data.nHits++;
							} else {
								data.unsure = true;
								unsureFace = poly;
								break;
							}
						}
					}
				}

			}

			if (data.unsure) {
				break;
			}
		}

		if (!data.unsure) {
			// we are sure about our intersection count
			if (data.nHits % 2 == 1) {
				data.in = true;
				return true;
			} else {
				return false;
			}
		}

		// pick next direction as centroid of unsure face
		unsureFace->getTriangulation()[0]->computeCentroid(centroid);
		dir.set(centroid);
		dir.subtract(pnt);

		data.nRetries++;
	} while (data.nRetries < maxRetries);

	return false;
}


}
}
