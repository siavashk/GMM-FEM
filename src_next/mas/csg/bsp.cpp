#include "mas/csg/bsp.h"

namespace mas {
namespace csg {
namespace bsp {

size_t CSGNode::nextId = 0;

// CSGNode implementation
CSGNode::CSGNode()
: plane(NULL), frontNode(NULL), backNode(NULL), polygons(),
  tol(BSP_DEFAULT_EPSILON) {
}

CSGNode::CSGNode(const CSGNode &copyMe)
: plane(NULL), frontNode(NULL), backNode(NULL), tol(copyMe.tol),
  polygons(copyMe.polygons) {

	if (copyMe.plane != NULL) {
		plane = new Plane(*copyMe.plane);
	}
	if (copyMe.frontNode != NULL) {
		frontNode = new CSGNode(*copyMe.frontNode);
	}
	if (copyMe.backNode != NULL) {
		backNode = new CSGNode(*copyMe.backNode);
	}
}

CSGNode::CSGNode(const PPolygonList &polygons)
: plane(), frontNode(NULL), backNode(NULL), polygons(),
  tol(BSP_DEFAULT_EPSILON) {

	build(polygons);
}

CSGNode::CSGNode(const PolygonMesh &mesh)
: plane(), frontNode(NULL), backNode(NULL), polygons(),
  tol(BSP_DEFAULT_EPSILON) {

	build(mesh.faces);
}

CSGNode::~CSGNode() {
	cleanup();
}

CSGNode& CSGNode::operator=(const CSGNode& assignMe) {
	cleanup();
	tol = assignMe.tol;
	if (assignMe.frontNode != NULL) {
		frontNode = new CSGNode(*(assignMe.frontNode));
	}
	if (assignMe.backNode != NULL) {
		backNode = new CSGNode(*(assignMe.backNode));
	}
	if (assignMe.plane != NULL) {
		plane = new Plane(*(assignMe.plane));
	}

	// should be fine, since we are using same vtxManager
			polygons = assignMe.polygons;
			return *this;
}

void CSGNode::cleanup() {
	if (frontNode != NULL) {
		delete frontNode;
		frontNode = NULL;
	}

	if (backNode != NULL) {
		delete backNode;
		backNode = NULL;
	}

	if (plane != NULL) {
		delete plane;
		plane = NULL;
	}
}

void CSGNode::build(const PPolygonList &polygons) {
	if (polygons.size() == 0) {
		return;
	}

	if (plane == NULL) {
		plane = new Plane(polygons[0]->plane);
	}

	PPolygonList frontP;
	PPolygonList backP;

	for (PPolygonList::const_iterator pit = polygons.begin();
			pit < polygons.end(); pit++ ) {
		split_polygon(*pit, *plane, frontP, backP,
				this->polygons, this->polygons, tol);
	}

	if (frontP.size() > 0) {
		if (this->frontNode != NULL) {
			delete this->frontNode;
			this->frontNode = NULL;
		}
		if (this->frontNode == NULL) {
			this->frontNode = new CSGNode();
		}
		this->frontNode->build(frontP);
	}

	if (backP.size() > 0) {
		if (this->backNode != NULL) {
			delete this->backNode;
			this->backNode = NULL;
		}
		if (this->backNode == NULL) {
			this->backNode = new CSGNode();
		}
		this->backNode->build(backP);
	}
}

void CSGNode::invert() {
	for (PPolygonList::iterator pit = polygons.begin();
			pit < polygons.end(); pit++) {
		(*pit)->flip();
	}
	if (plane != NULL) {
		plane->flip();
	}
	if (frontNode != NULL) {
		frontNode->invert();
	}
	if (backNode != NULL) {
		backNode->invert();
	}

	// switch front/back
	CSGNode *tmp = frontNode;
	frontNode = backNode;
	backNode = tmp;
}

PPolygonList CSGNode::clipPolygons(const PPolygonList &polygons) const {
	PPolygonList frontP;
	PPolygonList backP;

	if (plane == NULL) {
		return polygons;
	}

	for (PPolygonList::const_iterator pit = polygons.begin();
			pit < polygons.end(); pit++) {
		split_polygon(*pit, *plane, frontP, backP, frontP, backP,
				tol);
	}
	if (frontNode != NULL) {
		frontP = frontNode->clipPolygons(frontP);
	}
	if (backNode != NULL) {
		backP = backNode->clipPolygons(backP);
		// append back to front list for returning
		frontP.insert(frontP.end(), backP.begin(), backP.end());
	} else {
		backP.clear();
	}

	return frontP;
}

void CSGNode::clipTo(const CSGNode &node) {
	polygons = node.clipPolygons(polygons);
	if (frontNode != NULL) {
		frontNode->clipTo(node);
	}
	if (backNode != NULL) {
		backNode->clipTo(node);
	}
}

PPolygonList CSGNode::getAllPolygons() const {
	PPolygonList out;
	// append my polygons
	out.insert(out.end(), polygons.begin(), polygons.end());

	if (frontNode != NULL) {
		PPolygonList addThese = frontNode->getAllPolygons();
		out.insert(out.end(), addThese.begin(), addThese.end());
	}
	if (backNode != NULL) {
		PPolygonList addThese = backNode->getAllPolygons();
		out.insert(out.end(), addThese.begin(), addThese.end());
	}
	return out;
}

size_t CSGNode::numPolygons() const {
	return polygons.size();
}

void CSGNode::setTolerance(double tol) {
	this->tol = tol;
}

double CSGNode::getTolerance() const {
	return tol;
}

// Splits a polygon across a plane
// front: list of polygons on the +normal side
// back:  list of polygons on the -normal side
// coplanarFront/Back: if coplanar, placed in one of these depending
//        whether the poly's normal is consistent with the plane (front)
//        or opposite (back)
// Returns the number of split polygons
int split_polygon(PPolygon poly, Plane &plane,
		PPolygonList &front, PPolygonList &back,
		PPolygonList &coplanarFront, PPolygonList &coplanarBack,
		double tol) {

	const int COPLANAR = 0;
	const int FRONT = 1;
	const int BACK = 2;
	const int SPANNING = 3;

	// classify each point
	int pType = 0;
	int vTypes[poly->numVertices()];

	for (size_t i=0; i<poly->verts.size(); i++) {
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
	case COPLANAR:
		if (plane.normal.dot(poly->plane.normal) > 0) {
			coplanarFront.push_back(poly);
		} else {
			coplanarBack.push_back(poly);
		}
		break;
	case FRONT:
		front.push_back(poly);
		break;
	case BACK:
		back.push_back(poly);
		break;
	case SPANNING:
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
		}
		if (b.size() >= 3) {
			back.push_back(MeshFactory::createPolygon(b));
		}
		break;

	}
}

PolygonMesh vol_union(const PolygonMesh &a, const PolygonMesh &b,
		double tol) {
	PolygonMesh out = PolygonMesh(vol_union(a.faces,
			b.faces, tol));
	return out;
}

PPolygonList vol_union(const PPolygonList &a, const PPolygonList &b,
		double tol) {
	CSGNode nodeA = CSGNode(a);
	nodeA.setTolerance(tol);
	CSGNode nodeB = CSGNode(b);
	nodeB.setTolerance(tol);
	return vol_union(nodeA, nodeB).getAllPolygons();
}

CSGNode vol_union(CSGNode &a, CSGNode &b) {
	a.clipTo(b);
	b.clipTo(a);
	b.invert();
	b.clipTo(a);
	b.invert();
	a.build(b.getAllPolygons());
	return a;
}

PolygonMesh vol_intersection(const PolygonMesh &a, const PolygonMesh &b,
		double tol) {
	PolygonMesh out = PolygonMesh(vol_intersection(a.faces,
			b.faces, tol));
	return out;
}

PPolygonList vol_intersection(const PPolygonList &a, const PPolygonList &b,
		double tol) {
	CSGNode nodeA = CSGNode(a);
	nodeA.setTolerance(tol);
	CSGNode nodeB = CSGNode(b);
	nodeB.setTolerance(tol);
	return vol_intersection(nodeA, nodeB).getAllPolygons();
}

CSGNode vol_intersection(CSGNode &a, CSGNode &b) {

	// degenerate
	if (a.numPolygons() ==0 || b.numPolygons() == 0) {
		return CSGNode();
	}

	a.invert();
	b.clipTo(a);
	b.invert();
	a.clipTo(b);
	b.clipTo(a);
	a.build(b.getAllPolygons());
	a.invert();
	return a;
}

PolygonMesh vol_subtraction(const PolygonMesh &a, const PolygonMesh &b,
		double tol) {
	PolygonMesh out = PolygonMesh(vol_subtraction(a.faces, b.faces, tol));
	return out;
}

PPolygonList vol_subtraction(const PPolygonList &a, const PPolygonList &b,
		double tol) {
	CSGNode nodeA = CSGNode(a);
	nodeA.setTolerance(tol);
	CSGNode nodeB = CSGNode(b);
	nodeB.setTolerance(tol);
	return vol_subtraction(nodeA, nodeB).getAllPolygons();
}

CSGNode vol_subtraction(CSGNode &a, CSGNode &b) {

	if (a.numPolygons() == 0 || b.numPolygons() == 0) {
		return CSGNode(a);
	}

	a.invert();
	a.clipTo(b);
	b.clipTo(a);
	b.invert();
	b.clipTo(a);;
	b.invert();

	a.build(b.getAllPolygons());
	a.invert();
	return a;
}

}
}
}
