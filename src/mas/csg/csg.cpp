#include "mas/csg/csg.h"
#include "mas/mesh/meshbv.h"
#include "mas/core/math.h"
#include <math.h>
#include <limits>

#define SIGNUM(a) (a > 0) ? 1 : ((a < 0) ? -1 : 0)

namespace mas {
namespace csg {

void meshFromPolygons(PPolygonList &polygons, PolygonMesh &mesh) {
	mesh.set(polygons);
}

double dice(const PolygonMesh &mesh1, const PolygonMesh &mesh2,
		double tol, int numRetries, double baryEpsilon) {

	double margin = tol;
	if (margin < 0) {
		margin = 0;
	}
	PBVTree tree1 = get_obb_tree(mesh1, margin);
	PBVTree tree2 = get_obb_tree(mesh2, margin);

	return dice(mesh1, tree1, mesh2, tree2, tol, numRetries, baryEpsilon);
}

bool clip_polygons(const PPolygonList &polys, const Plane &plane,
		double tol, PPolygonList &front, PPolygonList &back,
		PPolygonList &coplanarFront, PPolygonList &coplanarBack) {

	const int COPLANAR = 0;
	const int FRONT = 1;
	const int BACK = 2;
	const int SPANNING = 3;
	bool clipped = false;

	for (PPolygon poly : polys) {

		// classify each polygon vertex type
		int pType = 0;
		const int nVerts = poly->numVertices();
		int vTypes[nVerts];

		for (size_t i=0; i<nVerts; i++) {
			Vertex3d &v = *poly->verts[i];
			double t = plane.distanceSigned(v);
			int type;
			if (t < -tol) {
				type = BACK;
			} else if (t > tol) {
				type = FRONT;
			} else {
				type = COPLANAR;
			}
			vTypes[i] = type;
			pType |= type;
		}

		// divide up polygon, either placing it in the correct
		// bin, or splitting it
		switch(pType) {
		case COPLANAR: {
			if (plane.normal.dot(poly->plane.normal) > 0) {
				coplanarFront.push_back(poly);
			} else {
				coplanarBack.push_back(poly);
			}
			break;
		}
		case FRONT: {
			front.push_back(poly);
			break;
		}
		case BACK: {
			back.push_back(poly);
			break;
		}
		case SPANNING: {
			PVertex3dList f;
			PVertex3dList b;

			for (int i=0; i<poly->verts.size(); i++) {
				int j = (i+1)%poly->verts.size();
				int ti = vTypes[i];
				int tj = vTypes[j];

				PVertex3d vi = poly->verts[i];
				PVertex3d vj = poly->verts[j];

				// front or back
				if (ti != BACK) {
					f.push_back(vi);
				}
				if (ti != FRONT) {
					if (ti != BACK) {
						b.push_back(vi);  // was clone
					} else {
						b.push_back(vi);
					}
				}

				if ((ti | tj)==SPANNING) {
					Vector3d diff = Vector3d(*vj);
					diff.subtract(*vi);

					double t = -plane.distanceSigned(*vi) / plane.normal.dot(diff);
					PVertex3d v = MeshFactory::createVertex();
					v->interpolate(*vi, t, *vj);
					f.push_back(v);
					b.push_back(v); //v.clone()); // was clone
				}

			}
			if (f.size() >= 3) {
				front.push_back(MeshFactory::createPolygon(f));
				clipped = true;
			}
			if (b.size() >= 3) {
				back.push_back(MeshFactory::createPolygon(b));
				clipped = true;
			}
			break;
		}
		}

	}
	return clipped;

}

void print_poly(PPolygon poly, std::string header) {

	printf("%s ", header.c_str());
	for (PVertex3d vtx : poly->verts) {
		printf(" ( %.2lf, %.2lf, %.2lf ) ", vtx->x, vtx->y, vtx->z);
	}
	printf("\n");
	fflush(stdout);

}

bool clip_polygons(const PPolygonList &polys, const PPolygonList &cutters,
		double tol, PPolygonList &out) {

	// copy values
	out = polys;

	bool clipped = false;
	for (PPolygon knife : cutters) {

		PPolygonList sliced;
		PPolygonList coplanar;
		bool kclipped = clip_polygons(out, knife->plane, tol, sliced, sliced, coplanar, coplanar);

		if (coplanar.size() > 0) {

			// clip coplanar polygons around edges
			Vector3d dir;
			int nVerts = knife->numVertices();
			PVertex3d vtx0 = knife->verts[nVerts-1];
			PVertex3d vtx1;

			for (int i=0; i<nVerts; i++) {
				vtx1 = knife->verts[i];
				dir.subtract(*vtx0, *vtx1);
				dir.cross(knife->plane.normal);
				bool cclipped = false;

				PPolygonList csliced;
				double dn = dir.norm();
				if (dn > 0) {
					dir.scale(1.0/dn);
					Plane cplane(dir, *vtx0);
					cclipped |= clip_polygons(coplanar, cplane, tol, csliced,
						csliced, csliced, csliced);
				}

				vtx0 = vtx1;

				// XXX not sure if efficient
				if (cclipped) {
					coplanar = csliced;
					kclipped = true;
				}
			}

			sliced.insert(sliced.end(), coplanar.begin(), coplanar.end());
		}

		if (kclipped) {
			out = sliced;
			clipped = true;
		}
	}

	return clipped;
}

typedef std::unordered_map<PBVNode,PPolygonList> LeafPolyMap;

void do_mesh_slicing(PBVTree bvtree1, LeafPolyMap &map1, PBVTree bvtree2, LeafPolyMap &map2, double tol) {

	// get leaves
	PBVNodeList nodes1;
	PBVNodeList nodes2;
	bvtree1->getLeaves(nodes1);
	bvtree2->getLeaves(nodes2);

	// collect polygons
	for (PBVNode node : nodes1) {
		PPolygonList polys;
		PBoundableList &elems = node->elems;

		for (PBoundable elem : elems) {
			PBoundablePolygon bpoly =
				std::static_pointer_cast<BoundablePolygon>(elem);
			polys.push_back(bpoly->polygon);
		}
		map1.insert(LeafPolyMap::value_type(node, polys));
	}

	for (PBVNode node : nodes2) {
		PPolygonList polys;
		PBoundableList &elems = node->elems;

		for (PBoundable elem : elems) {
			PBoundablePolygon bpoly =
				std::static_pointer_cast<BoundablePolygon>(elem);
			polys.push_back(bpoly->polygon);
		}
		map2.insert(LeafPolyMap::value_type(node, polys));
	}

	// step #1 intersect bvtrees
	nodes1.clear();
	nodes2.clear();
	bvtree1->intersectTree(bvtree2, nodes1, nodes2);

	// step #2 split faces in intersecting boxes
	int nNodes = nodes1.size();
	for (int i=0; i<nNodes; i++) {
		PBVNode node1 = nodes1[i];
		PBVNode node2 = nodes2[i];

		PPolygonList &faces1 = map1[node1];
		PPolygonList &faces2 = map2[node2];

		PPolygonList clipped1;
		PPolygonList clipped2;

		// cut faces in 1 by planes in 2
		bool f1clipped = clip_polygons(faces1, faces2, tol, clipped1);
		// cut faces in 2 by planes in 1
		bool f2clipped = clip_polygons(faces2, faces1, tol, clipped2);

		// XXX note sure if this is efficient, maybe only do if changed
		if (f1clipped) {
			map1[node1] = clipped1;
		}
		if (f2clipped) {
			map2[node2] = clipped2;
		}
	}

}

double intersection_volume(const PolygonMesh &mesh1, const PBVTree bvtree1,
		const PolygonMesh &mesh2, const PBVTree bvtree2,
		double tol, int numRetries, double baryEpsilon) {

	PPolygonList intersectionPolys;
	cheap_intersect(mesh1, bvtree1, mesh2, bvtree2, intersectionPolys,
			tol, numRetries, baryEpsilon);

	double vol = volume_integral(intersectionPolys);

	return vol;

}

double dice(const PolygonMesh &mesh1, const PBVTree bvtree1,
		const PolygonMesh &mesh2, const PBVTree bvtree2,
		double tol, int numRetries, double baryEpsilon) {

	double vol = intersection_volume(mesh1, bvtree1, mesh2, bvtree2,
			tol, numRetries, baryEpsilon);

	// Step #5, compute mesh volumes and dice coeff
	double vola = volume_integral(mesh1.faces);
	double volb = volume_integral(mesh2.faces);

	double d = 2*vol/(vola + volb);
	// account for round-off error
	if (d > 1) {
		d = 1;
	}

	return 2*vol/(vola + volb);
}

// tight-fit box
OBB tight_fit_box(const PolygonMesh &mesh1, const PolygonMesh &mesh2) {

//
//			Vector3d m1, m2, p;
//			Vector3d m1t, m2t, pt;
//
//			double vol = volume_integrals(mesh1.faces, m1, m2, p);
//			vol = vol + volume_integrals(mesh1.faces, m1t, m2t, pt);
//			m1.add(m1t);
//			m2.add(m2t);
//			p.add(pt);
//
//			// centre of volume
//			Point3d c;
//			c.scale(1.0/vol, m1);
//
//			Matrix3d cov;
//			cov.set(0, -c.z, c.y, c.z, 0, -c.x, -c.y, c.x, 0);
//
//			Matrix3d J;
//			J.set(m2.y+m2.z, -p.z, -p.y, -p.z, m2.x+m2.z, -p.x,
//				-p.y, -p.x, m2.x+m2.y);
//
//			Matrix3d Jc;
//			Jc.multiply(cov, cov);
//			Jc.scale(vol);
//			Jc.add(J);
//
//			Matrix3d U;
//			Matrix3d V;
//			Vector3d s;
//			svd(Jc, U, s, V);


	// Use point distribution to determine c and R
	Point3d c;
	Matrix3d cov;

	for (PVertex3d vtx : mesh1.verts) {
		c.add(*vtx);
	}
	for (PVertex3d vtx : mesh2.verts) {
		c.add(*vtx);
	}
	c.scale(1.0/(mesh1.numVertices() + mesh2.numVertices()));

	Vector3d diff;
	for (PVertex3d vtx : mesh1.verts) {
		diff.subtract(*vtx, c);
		cov.addOuterProduct(diff, diff);
	}
	for (PVertex3d vtx : mesh2.verts) {
		diff.subtract(*vtx, c);
		cov.addOuterProduct(diff, diff);
	}

	Vector3d s;
	Matrix3d U;
	Matrix3d V;
	mas::math::svd3(cov, U, s, V);

	if (U.determinant() < 0) {
		U.scaleColumn(2, -1);
	}

	RotationMatrix3d R = RotationMatrix3d(U.m);
	Vector3d hw;
	OBB obb = OBB(c, R, hw);

	for (PVertex3d vtx : mesh1.verts) {
		obb.updatePoint(*vtx);
	}
	for (PVertex3d vtx : mesh2.verts) {
		obb.updatePoint(*vtx);
	}

	return obb;
}

double dice_estimate(const PolygonMesh &mesh1, const PolygonMesh &mesh2,
		int resolution[], double tol, int numRetries, double baryEpsilon) {

	double margin = tol;
	if (margin < 0) {
		margin = 0;
	}
	PBVTree bvtree1 = get_obb_tree(mesh1, margin);
	PBVTree bvtree2 = get_obb_tree(mesh2, margin);

	return dice_estimate(mesh1, bvtree1, mesh2, bvtree2, resolution, tol,
			numRetries, baryEpsilon);
}

double dice_estimate(const PolygonMesh &mesh1, const PBVTree bvtree1,
		const PolygonMesh &mesh2, const PBVTree bvtree2,
		int resolution[], double tol, int numRetries,
		double baryEpsilon) {

	OBB obb = tight_fit_box(mesh1, mesh2);

	double dx = obb.halfWidths.x/resolution[0];
	double dy = obb.halfWidths.y/resolution[1];
	double dz = obb.halfWidths.z/resolution[2];

	double vol1 = 0;
	double vol2 = 0;
	double volint = 0;

	Point3d p;	// world
	Point3d pl;	// local
	for (int i=0; i<resolution[0]+1; i++) {
		pl.x = -obb.halfWidths.x + dx*i;
		for (int j=0; j<resolution[1]+1; j++) {
			pl.y = -obb.halfWidths.y + dy*j;
			for (int k=0; k<resolution[2]+1; k++) {
				pl.z = -obb.halfWidths.z + dz*k;

				obb.getWorldCoords(pl, p);

				// check if inside
				InsideMeshQueryData data;
				bool in1 = is_inside(p, bvtree1, data, tol);
				bool in2 = is_inside(p, bvtree2, data, tol);

				if (in1) {
					vol1++;
				}
				if (in2) {
					vol2++;
				}
				if ( in1 && in2 ) {
					volint++;
				}

				/*
				int in1 = is_inside(p, mesh1, bvtree1, tol, numRetries,
						baryEpsilon);
				int in2 = is_inside(p, mesh2, bvtree2, tol, numRetries,
						baryEpsilon);

				if (in1 == MESH_INSIDE_TRUE) {
					vol1++;

					if (in2 == MESH_INSIDE_TRUE) {
						vol2++;
						volint++;
					}
				} else if (in2 == MESH_INSIDE_TRUE) {
					vol2++;
				}
				*/

			}
		}
	}

	return 2*volint/(vol1+vol2);

}

Point3d get_interior_point(PPolygon poly) {

	Point3d out(0,0,0);

	int nV = poly->numVertices();

	if (nV > 3) {

		// find first convex vertex
		PVertex3d vtx0 = poly->verts[nV-2];
		PVertex3d vtx1 = poly->verts[nV-1];

		Vector3d v1;
		Vector3d v2;

		for (PVertex3d vtx2 : poly->verts) {
			v1.subtract(*vtx0, *vtx1);
			v2.subtract(*vtx2, *vtx1);
			v2.cross(v1);

			if (v2.dot(poly->plane.normal) > 0) {
				// get centroid
				out.set(*vtx0);
				out.add(*vtx1);
				out.add(*vtx2);
				out.scale(1.0/3);
				return out;
			}

			vtx0 = vtx1;
			vtx1 = vtx2;
		}
	}

	// default use first 3 vertices
	out.set(*(poly->verts[0]));
	out.add(*(poly->verts[1]));
	out.add(*(poly->verts[2]));
	out.scale(1.0/3);
	return out;
}

bool cheap_intersect(const PolygonMesh &mesh1, const PBVTree bvtree1,
		const PolygonMesh &mesh2, const PBVTree bvtree2,
		PPolygonList &out,
		double tol, int numRetries,
		double baryEpsilon) {

	LeafPolyMap map1;
	LeafPolyMap map2;
	do_mesh_slicing(bvtree1, map1, bvtree2, map2, tol);

	// do cheap intersection
	out.clear();

	// for every face in mesh1, check if inside mesh2
	//    inside -> keep, consistent coplanar -> keep, other -> discard
	for(auto keypair : map1) {
		for (PPolygon poly : keypair.second) {

			// get a point in middle of face
			Point3d pnt = get_interior_point(poly);

			InsideMeshQueryData data;
			bool inside = is_inside(pnt, bvtree2, data, tol);
			if (data.on) {
				// check normal
				PPolygon face = data.nearestFace->polygon;
				if (poly->plane.normal.dot(face->plane.normal) > 0) {
					out.push_back(poly);
				}
			} else if (data.in) {
				out.push_back(poly);
			}

			/*
			int inside = is_inside_or_on(pnt, mesh2, bvtree2, data,
					tol, numRetries, baryEpsilon);

			if (data.unsure) {
				return false;
			} else if (data.on ) {
				// check normal
				PPolygon face = data.nearestFace->polygon;
				if (poly->plane.normal.dot(face->plane.normal) > 0) {
					out.push_back(poly);
				}
			} else if (data.in) {
				out.push_back(poly);
			}
			*/
		}
	}

	// for every face in mesh2, check if inside mesh1
	//    inside -> keep, other -> discard
	for(auto keypair : map2) {
		for (PPolygon poly : keypair.second) {

			// get a point in middle of face
			Point3d pnt = get_interior_point(poly);

			InsideMeshQueryData data;
			bool inside = is_inside(pnt, bvtree2, data, tol);
			if (data.on) {
				// discard
			} else if (data.in) {
				out.push_back(poly);
			}

			/*
			int inside = is_inside_or_on(pnt, mesh1, bvtree1, data,
					tol, numRetries, baryEpsilon);

			if (data.unsure) {
				return false;
			} else if (data.on ) {
				// discard
			} else if (data.in) {
				out.push_back(poly);
			}
			*/
		}
	}

	return true;
}

bool cheap_intersect(const PolygonMesh &mesh1, const PolygonMesh &mesh2,
		PolygonMesh &out,	double tol, int numRetries,	double baryEpsilon) {

	double margin = tol;
	if (margin < 0) {
		margin = 0;
	}
	PBVTree tree1 = get_obb_tree(mesh1, margin);
	PBVTree tree2 = get_obb_tree(mesh2, margin);

	return cheap_intersect(mesh1, tree1, mesh2, tree2, out, tol, numRetries, baryEpsilon);
}

bool cheap_intersect(const PolygonMesh &mesh1, const PBVTree bvtree1,
		const PolygonMesh &mesh2, const PBVTree bvtree2,
		PolygonMesh &out,
		double tol, int numRetries,
		double baryEpsilon) {

	PPolygonList faces;
	bool success = cheap_intersect(mesh1, bvtree1, mesh2, bvtree2, faces, tol, numRetries, baryEpsilon);
	out.set(faces);
	return success;
}

}
}
