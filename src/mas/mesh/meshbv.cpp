#include "mas/core/math.h"
#include <math.h>
#include <random>
#include "mas/mesh/meshbv.h"

#include <stdlib.h>

namespace mas {
namespace mesh {

BoundablePolygon::BoundablePolygon(const SharedPolygon& poly) :
		Boundable(-1), polygon(), tris() {
	setPolygon(poly);
}

void BoundablePolygon::getCentroid(Point3d& c) const {
	c.set(centroid);
}

void BoundablePolygon::computeCentroid() {
	centroid.setZero();
	std::vector<SharedVertex3d>& verts = polygon->verts;
	if (verts.size() == 0) {
		return;
	}

	for (SharedVertex3d& vtx : verts) {
		centroid.add(*vtx);
	}
	centroid.scale(1.0 / verts.size());
}

void BoundablePolygon::triangulate() {
	if (polygon->numVertices() == 3) {
		tris.clear();
		tris.push_back(polygon);
	} else {
		auto newtris = MeshFactory::triangulate(*polygon);
		tris.clear();
		tris.reserve(newtris.size());
		for (auto& ntri : newtris) {
			tris.push_back(std::move(ntri));
		}
	}
}

const std::vector<SharedPolygon>& BoundablePolygon::getTriangulation() const {
	return tris;
}

void BoundablePolygon::setPolygon(const SharedPolygon& poly) {
	polygon = poly;
	computeCentroid();
	triangulate();
}

void BoundablePolygon::update() {
	computeCentroid();
	// XXX verify triangulation is still valid?
	// Then again, polygon is supposed to be planar, so
	// are vertices allowed to move anyways?
	triangulate();  // always retriangulate
}

void BoundablePolygon::getCovariance(const Point3d& center,
		Matrix3d& cov) const {

	cov.setZero();
	std::vector<SharedVertex3d>& verts = polygon->verts;
	if (verts.size() == 0) {
		return;
	}

	Point3d diff;
	for (SharedVertex3d& vtx : verts) {
		diff.set(*vtx);
		diff.subtract(center);
		cov.addOuterProduct(diff, diff);
	}
	cov.scale(1.0 / verts.size());
}

bool BoundablePolygon::updateBV(BoundingVolume& bv) const {
	for (SharedVertex3d& vtx : polygon->verts) {
		bv.updatePoint(*vtx);
	}
}

double BoundablePolygon::distanceToPoint(const Point3d& pnt, Point3d& nearest,
		Vector3d& bary, SharedPolygon& tri) const {

	double distance = 0;

	// triangulate if required
	if (polygon->numVertices() == 3) {
		distance = distance_to_triangle(pnt, *(polygon->verts[0]),
				*(polygon->verts[1]), *(polygon->verts[2]), nearest, bary);
		tri = polygon;
	} else {

		// triangulate if not already done
		const std::vector<SharedPolygon>& ltris = getTriangulation();

		double distMin = math::DOUBLE_INFINITY;
		double distTmp;
		Point3d nearestTmp;
		Vector3d baryTmp;
		for (const SharedPolygon& ltri : ltris) {
			distTmp = distance_to_triangle(pnt, *(ltri->verts[0]),
					*(ltri->verts[1]), *(ltri->verts[2]), nearestTmp, baryTmp);
			if (distTmp < distMin) {
				distMin = distTmp;
				nearest.set(nearestTmp);
				bary.set(baryTmp);

				tri = ltri;
			}
		}
		distance = distMin;
	}

	return distance;
}

double BoundablePolygon::distanceToPoint(const Point3d& pnt,
		Point3d& nearest) const {
	Vector3d bary;
	SharedPolygon tri = nullptr;
	return distanceToPoint(pnt, nearest, bary, tri);
}

double BoundablePolygon::distanceToPoint(const Point3d& pnt,
		const Vector3d& dir, Point3d& nearest, Vector3d& bary,
		SharedPolygon& tri) const {

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
						*(polygon->verts[1]), *(polygon->verts[2]), bary)) {
					nearest.set(pnt);
					tri = polygon;
					return 0;
				}
			} else {

				// triangulate if not already done
				const std::vector<SharedPolygon>& ltris = getTriangulation();

				for (const SharedPolygon& ltri : ltris) {
					Vector3d baryTmp;
					bool inside = point_in_triangle(pnt, *(ltri->verts[0]),
							*(ltri->verts[1]), *(ltri->verts[2]), baryTmp);
					if (inside) {
						nearest.set(pnt);
						bary.set(baryTmp);
						tri = ltri;
						return 0;
					}
				}

			}

			// not inside triangle, do 2D intersection
			Vertex3d& o = *(polygon->verts[0]);
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

			typedef std::vector<SharedVertex3d>::iterator VIt;

			VIt prevVtx = std::prev(polygon->verts.end());
			p1x = (*prevVtx)->dot(v1);
			p1y = (*prevVtx)->dot(v2);
			f1 = dx * (p1y - py) - dy * (p1x - px);
			Point3d pint;

			double dmin = math::DOUBLE_INFINITY;
			bool intersectFound = false;

			for (VIt vit = polygon->verts.begin();
					vit < polygon->verts.end(); vit++) {

				SharedVertex3d& vtx = *vit;
				f0 = f1;
				p0x = p1x;
				p0y = p1y;
				p1x = vtx->dot(v1);
				p1y = vtx->dot(v1);
				f1 = dx * (p1y - py) - dy * (p1x - px);

				if (f1 == 0) {
					// falls on the line
					intersectFound = true;
					double d = vtx->distance(pnt);
					if (d < dmin) {
						dmin = d;
						nearest.set(*vtx);
						// vertex is on the line
						bary.x = 1;
						bary.y = 0;
						bary.z = 0;
						// XXX might want to make true triangle
						tri = std::make_shared<Polygon>(vtx, vtx, vtx);
					}
				} else if (f0 * f1 < 0) {
					// intersection
					intersectFound = true;
					dpx = p1x - p0x;
					dpy = p1y - p0y;

					double s = 1.0 / (dx * dpy - dpx * dy);
					double u = (-dpx * px * dy + dpx * dx * py + dx * dpy * p0x
							- dx * dpx * p0y) * s;
					double v = (-dpy * px * dy + dpy * dx * py + dy * dpy * p0x
							- dy * dpx * p0y) * s;
					pint.setZero();
					pint.scaledAdd(u, v1);
					pint.scaledAdd(v, v2);

					double d = pint.distance(pnt);
					if (d < dmin) {
						dmin = d;
						nearest.set(pint);

						// point straddles
						// XXX might want to make true triangle
						tri =  std::make_shared<Polygon>(*prevVtx, vtx, vtx);
						bary.z = 0;
						Vector3d tmp = Vector3d(nearest);
						tmp.subtract(*vtx);

						Vector3d tmp2(**prevVtx);
						tmp2.subtract(*vtx);

						bary.x = tmp.dot(tmp2) / tmp2.dot(tmp2);
						bary.y = 1.0 - bary.x;
					}

					prevVtx = vit;
				}

			}

			if (intersectFound) {
				return fabs(dmin);
			}

		} else {
			// parallel, doesn't intersect
			// negative direction
			nearest.x = math::signum(dir.x) * math::DOUBLE_INFINITY;
			nearest.y = math::signum(dir.y) * math::DOUBLE_INFINITY;
			nearest.z = math::signum(dir.z) * math::DOUBLE_INFINITY;
			return math::DOUBLE_INFINITY;
		}
	} else {

		// get intersection point
		double t = -distance / denom;
		nearest.scaledAdd(pnt, t, dir);

		// check if intersects triangle
		if (polygon->numVertices() == 3) {
			// check if is in triangle
			if (point_in_triangle(nearest, *(polygon->verts[0]),
					*(polygon->verts[1]), *(polygon->verts[2]), bary)) {
				tri = polygon;
				return fabs(t * dir.norm());
			}
		} else {

			// triangulate if not already done
			const std::vector<SharedPolygon>& ltris = getTriangulation();

			for (const SharedPolygon& ltri : ltris) {
				bool inside = point_in_triangle(nearest, *(ltri->verts[0]),
						*(ltri->verts[1]), *(ltri->verts[2]), bary);
				if (inside) {
					return fabs(t * dir.norm());
				}
			}

		}
	}

	// not in any triangle
	return math::DOUBLE_INFINITY;
}

double BoundablePolygon::distanceToPoint(const Point3d& pnt,
		const Vector3d& dir, Point3d& nearest) const {
	SharedPolygon tri;
	Vector3d bary;
	return distanceToPoint(pnt, dir, nearest, bary, tri);
}

// methods for meshes

BSTree* get_bs_tree(const PolygonMesh& mesh, double margin) {
	return get_bv_tree_T<BoundingSphere>(mesh, margin);
}

AABBTree* get_aabb_tree(const PolygonMesh& mesh, double margin) {
	return get_bv_tree_T<AABB>(mesh, margin);
}

OBBTree* get_obb_tree(const PolygonMesh& mesh, double margin) {
	return get_bv_tree_T<OBB>(mesh, margin);
}

SharedPolygon nearest_polygon(const Point3d& pnt, const PolygonMesh& mesh,
		Point3d& nearestPoint) {
	// build tree from polygons, get nearest boundable, cast to PBoundablePolygon
	UniqueOBBTree tree = std::unique_ptr<OBBTree>(get_obb_tree(mesh));
	SharedBoundable nearest = mas::bvtree::nearest_boundable(*tree, pnt, nearestPoint);
	SharedBoundablePolygon poly = std::static_pointer_cast<BoundablePolygon>(nearest);
	return poly->polygon;
}

SharedPolygon nearest_polygon(const Point3d& pnt, const Vector3d& dir,
		const PolygonMesh& mesh, Point3d& nearestPoint) {
	// build tree from polygons, get nearest boundable, cast to PBoundablePolygon
	UniqueOBBTree tree = std::unique_ptr<OBBTree>(get_obb_tree(mesh));
	SharedBoundable nearest = mas::bvtree::nearest_boundable(*tree, pnt, dir,
			nearestPoint);
	SharedBoundablePolygon poly = std::static_pointer_cast<BoundablePolygon>(
			nearest);
	return poly->polygon;
}

SharedBoundablePolygon nearest_polygon(const Point3d& pnt, const BVTree& bvt,
		Point3d& nearestPoint) {
	// build tree from polygons, get nearest boundable, cast to PBoundablePolygon
	SharedBoundable nearest = mas::bvtree::nearest_boundable(bvt, pnt, nearestPoint);
	SharedBoundablePolygon poly = std::dynamic_pointer_cast<BoundablePolygon>(
			nearest);
	return poly;
}

SharedBoundablePolygon nearest_polygon(const Point3d& pnt, const Vector3d& dir,
		const BVTree& bvt, Point3d& nearestPoint) {
	// build tree from polygons, get nearest boundable, cast to PBoundablePolygon
	SharedBoundable nearest = mas::bvtree::nearest_boundable(bvt, pnt, dir,
			nearestPoint);
	SharedBoundablePolygon poly = std::dynamic_pointer_cast<BoundablePolygon>(
			nearest);
	return poly;
}

MeshQueryResult is_inside(const Point3d& pnt, const PolygonMesh& mesh, double tol,
		int numRetries, double baryEpsilon) {
	// build tree from polygons, get nearest boundable, cast to PBoundablePolygon
	double margin = tol;
	if (margin < 0) {
		margin = 0;
	}
	UniqueOBBTree tree = std::unique_ptr<OBBTree>(get_obb_tree(mesh, margin));
	//PBSTree tree = get_bs_tree(mesh);
	return is_inside(pnt, mesh, *tree, tol, numRetries, baryEpsilon);
}

MeshQueryResult is_inside(const Point3d& pnt, const PolygonMesh& mesh, const BVTree& bvt,
		double tol, int numRetries, double baryEpsilon) {

	MeshQueryResult inside = is_inside_or_on(pnt, mesh, bvt, tol, numRetries, baryEpsilon);

	if (inside == MeshQueryResult::ON) {
		inside = MeshQueryResult::INSIDE;
	}
	return inside;
}

MeshQueryResult is_inside_or_on(const Point3d& pnt, const PolygonMesh& mesh,
		const BVTree& bvt, double tol, int numRetries, double baryEpsilon) {
	InsideMeshQueryData data;
	return is_inside_or_on(pnt, mesh, bvt, data, tol, numRetries, baryEpsilon);
}

MeshQueryResult is_inside_or_on(const Point3d& pnt, const PolygonMesh& mesh,
		const BVTree& bvt, InsideMeshQueryData& data, double tol,
		int numRetries, double baryEpsilon) {

	const SharedBVNode& root = bvt.getRoot();
	if (tol < 0) {
		// default tolerance
		double r = root->bv->getBoundingSphere().getRadius();
		tol = 1e-12 * r;
	}

	data.reset();

	// check if is inside root bounding box
	if (!root->bv->intersectsSphere(pnt, tol)) {
		return MeshQueryResult::OUTSIDE;
	}

	// find the nearest polygon
	data.nearestFace = nearest_polygon(pnt, bvt, data.nearestPoint);

	// first check if within tolerance
	if (pnt.distance(data.nearestPoint) <= tol) {
		data.in = true;
		data.on = true;
		return MeshQueryResult::ON;
	}

	/// first direction is away from nearest face (fastest when
	// outside convex surface)
	Vector3d dir = Vector3d(pnt);
	dir.subtract(data.nearestPoint);
	dir.normalize();

	const std::vector<SharedPolygon>& meshFaces = mesh.faces;
	Point3d centroid;

	data.nRetries = 0;
	do {
		data.nHits = 0;
		data.unsure = false;

		std::vector<BVNode*> bvnodes;
		bvt.intersectRay(pnt, dir, bvnodes);

		for (BVNode* node : bvnodes) {

			for (SharedBoundable& boundable : node->elems) {

				// dynamic cast to PBoundablePolygon
				SharedBoundablePolygon poly = std::dynamic_pointer_cast<
						BoundablePolygon>(boundable);
				if (poly != nullptr) {

					// intersect ray with face
					Vector3d bary;
					SharedPolygon tri;
					double d = fabs(
							poly->distanceToPoint(pnt, dir, data.nearestPoint,
									bary, tri));

					// check if close to face
					if (d < tol) {
						data.in = true;
						data.on = true;
						return MeshQueryResult::ON;
					}

					// check if ray hits face
					if (d != math::DOUBLE_INFINITY
							&& d != -math::DOUBLE_INFINITY) {
						// check that it hits the face in the correct direction
						Vector3d ndir(data.nearestPoint);
						ndir.subtract(pnt);
						if (ndir.dot(dir) > 0) {
							// make sure barycentric coordinates are not on edges
							if (bary.x >= baryEpsilon
									&& bary.x <= 1 - baryEpsilon
									&& bary.y >= baryEpsilon
									&& bary.y <= 1 - baryEpsilon
									&& bary.z >= baryEpsilon
									&& bary.z <= 1 - baryEpsilon) {
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
				return MeshQueryResult::INSIDE;
			} else {
				return MeshQueryResult::OUTSIDE;
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
			return MeshQueryResult::INSIDE;
		}
		dir.normalize();

		data.nRetries++;
	} while (data.nRetries < numRetries);

	data.unsure = true;
	return MeshQueryResult::UNSURE;	// unsure

}

bool poly_contains_coordinate(Point3d& pnt, const BoundablePolygon& bpoly,
		Vector3d& bary, SharedPolygon& tri) {

	// check if is inside triangles
	const SharedPolygon& polygon = bpoly.polygon;
	if (polygon->numVertices() == 3) {
		tri = polygon;
		// check if is in triangle
		if (point_in_triangle(pnt, *(polygon->verts[0]), *(polygon->verts[1]),
				*(polygon->verts[2]), bary)) {
			return true;
		}
		return false;
	} else {

		// triangulate if not already done
		const std::vector<SharedPolygon>& ltris = bpoly.getTriangulation();

		for (const SharedPolygon& ltri : ltris) {
			tri = ltri;
			bool inside = point_in_triangle(pnt, *(ltri->verts[0]),
					*(ltri->verts[1]), *(ltri->verts[2]), bary);
			if (inside) {
				return true;
			}
		}
	}
	return false;
}

// Faster?
bool is_inside(const Point3d& pnt, const BVTree& bvt, InsideMeshQueryData& data,
		double tol, int maxRetries) {

	const SharedBVNode& root = bvt.getRoot();

	if (tol < 0) {
		// default tolerance
		double r = root->bv->getBoundingSphere().getRadius();
		tol = 1e-12 * r;
	}

	// initialize
	data.reset();

	// check if is inside root bounding box
	if (!root->bv->intersectsSphere(pnt, tol)) {
		return false;
	}

	// find the nearest polygon
	data.nearestFace = nearest_polygon(pnt, bvt, data.nearestPoint);

	// cast direction
	Vector3d dir(data.nearestPoint);
	dir.subtract(pnt);

	// first check if within tolerance
	if (dir.normSquared() <= tol*tol) {
		data.in = true;
		data.on = true;
		return true;
	}

	// check if closest point is within the center of the triangle
	// i.e. not on an edge or vertex
	// Because then we just need to check the face normal
	Vector3d bary;
	SharedPolygon tri;
	bool success = poly_contains_coordinate(data.nearestPoint, *(data.nearestFace),
			bary, tri);

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
	Point3d centroid;
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

		std::vector<BVNode*> bvnodes;
		bvt.intersectRay(pnt, dir, bvnodes);
		double dmin = mas::math::DOUBLE_INFINITY;
		data.in = false;

		// find closest face among nodes in direction of dir
		for (BVNode* node : bvnodes) {
			for (SharedBoundable& boundable : node->elems) {

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

}
}
