#include "mas/core/math.h"
#include <math.h>
#include <random>
#include "mas/mesh/meshbv.h"

#include <stdlib.h>

namespace mas {
namespace mesh {

BoundablePolygon::BoundablePolygon(const SharedPolygon& poly) :
		polygon(), tris(), centroid(0,0,0) {
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

			for (VIt vit = polygon->verts.begin(); vit < polygon->verts.end();
					vit++) {

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
						tri = std::make_shared<Polygon>(*prevVtx, vtx, vtx);
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

InsideMeshQueryData::InsideMeshQueryData() :
		on(false), in(false), unsure(false), nearestFace(nullptr), nearestPoint(
				0, 0, 0), nHits(0), nRetries(0) {
}

void InsideMeshQueryData::reset() {
	in = false;
	on = false;
	unsure = false;
	nHits = 0;
	nRetries = 0;
	nearestFace = nullptr;
	nearestPoint.setZero();
}

// methods for meshes

BVTree<SharedBoundablePolygon,BoundingSphere>* get_bs_tree(const PolygonMesh& mesh, double margin) {
	return get_bv_tree<BoundingSphere>(mesh, margin);
}

BVTree<SharedBoundablePolygon,AABB>* get_aabb_tree(const PolygonMesh& mesh, double margin) {
	return get_bv_tree<AABB>(mesh, margin);
}

BVTree<SharedBoundablePolygon,OBB>* get_obb_tree(const PolygonMesh& mesh, double margin) {
	return get_bv_tree<OBB>(mesh, margin);
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

}
}
