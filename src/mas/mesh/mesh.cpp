#include "mas/mesh/mesh.h"
#include <unordered_map>
#include <list>
#include <algorithm>

namespace mas {
namespace mesh {

// Vertex3d implementation
Vertex3d::Vertex3d()
: Point3d(), _incident(), idx(-1) {}

Vertex3d::Vertex3d(const Point3d& vertexMe)
: Point3d(vertexMe), idx(-1), _incident() {}

Vertex3d::Vertex3d(double x, double y, double z)
: Point3d(x,y,z), idx(-1), _incident() {}

Vertex3d::Vertex3d(double x, double y, double z, size_t idx)
: Point3d(x,y,z), idx(idx), _incident() {}

Vertex3d::~Vertex3d() {}

void Vertex3d::addIncidentEdge(const PHalfEdge he) {
	_incident.push_back(he);
}

PHalfEdgeList Vertex3d::getIncidentEdges() const {
	PHalfEdgeList out;
	for (WHalfEdgeList::const_iterator hit = _incident.begin();
			hit < _incident.end(); hit++) {
		out.push_back(hit->lock());
	}
	return out;
}

void Vertex3d::removeIncidentEdge(const HalfEdge &he) {

	WHalfEdgeList::iterator found = _incident.end();
	for (WHalfEdgeList::iterator hit = _incident.begin();
			hit < _incident.end(); hit++) {
		if (hit->lock().get() == &he) {
			found = hit;
			break;
		}
	}

	if (found != _incident.end()) {
		_incident.erase(found);
	}
}

void Vertex3d::removeIncidentEdge(const PHalfEdge he) {

	WHalfEdgeList::iterator found = _incident.end();
	for (WHalfEdgeList::iterator hit = _incident.begin();
			hit < _incident.end(); hit++) {
		if (hit->lock() == he) {
			found = hit;
			break;
		}
	}

	if (found != _incident.end()) {
		_incident.erase(found);
	}
}

// Polygon
Polygon::Polygon()
: plane(), verts(), _data(NULL) {}

Polygon::Polygon(const Plane &plane, const PVertex3dList &verts)
: plane(plane), verts(verts), _data(NULL) {}

Polygon::Polygon(const PVertex3dList &verts)
: plane(), verts(verts), _data(NULL) {
	plane = getPlane(verts);
}

Polygon::Polygon(const PVertex3d v0, const PVertex3d v1,
		const PVertex3d v2)
: plane(*v0, *v1, *v2), verts(), _data(NULL) {
	verts.push_back(v0);
	verts.push_back(v1);
	verts.push_back(v2);
}

Polygon::~Polygon() {}

Plane Polygon::getPlane(const PVertex3dList &verts) {
	size_t n = verts.size();

	PVertex3d v0 = verts[0];
	PVertex3d v1 = verts[1];
	PVertex3d v2 = verts[2];
	Plane plane;

	bool success = plane.set(*v0, *v1, *v2);
	int idx = 1;
	while (!success && idx < n) {
		v0 = v1;
		v1 = v2;
		v2 = verts.at( (idx+2)%n );
		success = plane.set(*v0, *v1, *v2);
		idx++;
	}
	return plane;
}

void Polygon::set(const Polygon &poly) {
	_data = NULL;
	this->verts = poly.verts;
	this->plane = poly.plane;
}

void Polygon::set(const PVertex3dList &verts) {
	_data = NULL;	// for triggering rebuild
	this->verts = verts;
	plane = getPlane(verts);
}

void Polygon::set(const PVertex3d v0, const PVertex3d v1,
		const PVertex3d v2) {
	_data = NULL;	// for triggering rebuild
	verts.clear();
	verts.push_back(v0);
	verts.push_back(v1);
	verts.push_back(v2);
	plane.set(*v0, *v1, *v2);
}

size_t Polygon::numVertices() const {
	return verts.size();
}

void Polygon::computeCentroid(Point3d &centroid) const {
	centroid.setZero();
	for (PVertex3dList::const_iterator pit = verts.begin();
			pit < verts.end(); pit++) {
		centroid.add(*(*pit));
	}
	centroid.scale(1.0/numVertices());
}

void Polygon::flip() {
	_data = NULL;	// for triggering rebuild
	std::reverse(verts.begin(), verts.end());
	plane.flip();
}

bool Polygon::isTriangular() const {
	return (numVertices() == 3);
}

bool Polygon::isConvex(PVertex3dList &reflex) const {

	int nV = numVertices();

	if (nV <=3 ) {
		return true;
	}
	bool ic = true;
	Vector3d v1;
	Vector3d v2;

	PVertex3d vtx0 = verts[nV-2];
	PVertex3d vtx1 = verts[nV-1];
	for (PVertex3d vtx2 : verts) {
		v1.subtract(*vtx0, *vtx1);
		v2.subtract(*vtx2, *vtx1);
		v2.cross(v1);

		if (v2.dot(plane.normal) < 0) {
			reflex.push_back(vtx1);
			ic = false;
		}

		vtx0 = vtx1;
		vtx1 = vtx2;
	}

	return ic;
}

bool Polygon::isConvex() const {

	int nV = numVertices();
	if (nV <=3 ) {
		return true;
	}

	Vector3d v1;
	Vector3d v2;
	PVertex3d vtx0 = verts[nV-2];
	PVertex3d vtx1 = verts[nV-1];
	for (PVertex3d vtx2 : verts) {
		v1.subtract(*vtx0, *vtx1);
		v2.subtract(*vtx2, *vtx1);
		v2.cross(v1);

		if (v2.dot(plane.normal) < 0) {
			return false;
		}

		vtx0 = vtx1;
		vtx1 = vtx2;
	}

	return true;
}

PPolyData Polygon::getData() {
	return _data;
}

void Polygon::attach(const PPolyData data) {
	_data = data;
	_data->attach(_data);	// attach data to structure
}

PPolyData Polygon::getOrCreateData(const PPolygon pthis) {
	if (_data == NULL) {
		if (pthis.get() == this) {
			MeshFactory::createPolyData(pthis);
		}
	}
	return _data;
}

HalfEdge::HalfEdge(const PVertex3d tail, const PVertex3d head,
		const PPolygon face, const PHalfEdge opposite)
: head(head), tail(tail), face(face), opposite(opposite), next(NULL) {}

HalfEdge::~HalfEdge() {
	disconnect();
}
void HalfEdge::disconnect() {
	// clear opposite's reference to me
	if (opposite != NULL) {
		opposite->opposite = NULL;
	}

	next = NULL;
	opposite = NULL;
}

double HalfEdge::getLength() const {
	return head->distance(*tail);
}

PolyData::PolyData(const PPolygon poly)
: firstHE(NULL) {

	// build half-edges from vertex list
	PVertex3dList &verts = poly->verts;
	PVertex3d v0 = verts.back();
	PVertex3d v1 = verts.front();
	firstHE = MeshFactory::createHalfEdge(v0, v1, poly);
	PHalfEdge last = firstHE;

	for (PVertex3dList::iterator pit = verts.begin()+1;
			pit < verts.end(); pit++) {
		v0 = v1;
		v1 = *pit;
		last->next = MeshFactory::createHalfEdge(v0, v1, poly);
		last = last->next;
	}
	last->next = firstHE;
}

PolyData::~PolyData() {
	// break cyclic chain to allow deletion
	if (firstHE != NULL) {
		firstHE->next = NULL;
	}
}

void PolyData::attach(const PPolyData pthis) {
	// add incident he to head, find opposite he
	PHalfEdge he = firstHE;
	do {
		PVertex3d &head = he->head;
		head->addIncidentEdge(he);

		// loop through tail incident edges to find opposite
		PHalfEdgeList tails = he->tail->getIncidentEdges();
		for (PHalfEdgeList::iterator pit = tails.begin();
				pit < tails.end(); pit++) {

			PHalfEdge popp = (*pit);
			if (   (popp->tail == head)
					&& (popp->opposite == NULL) ) {
				// && (popp->face.lock()->mesh == he->mesh)) {

				he->opposite = popp;
				break;
			}
		}

		he = he->next;
	} while (he != firstHE);
}

/*
		Face::Face(const Face &copyMe) {
			set(copyMe);
		}

		Face::Face(const PVertex3dList &verts) {
			set(verts);
		}

		Face::Face(const Plane &plane, const PVertex3dList &verts) {
			set(plane, verts);
		}

		Face::Face(const Polygon &poly) {
			set(poly.plane, poly.verts);
		}

		Face::Face(const PVertex3d v0, const PVertex3d v1, 
			const PVertex3d v2) {
			set(v0, v1, v2);
		}

		Face::~Face() {
			cleanup();
		}

		void Face::cleanup() {
			if (_firstHE != NULL) {
				// destroy ring of half-edges
				PHalfEdge here = _firstHE->next;
				_firstHE->next = NULL;	// break loop
				PHalfEdge next;
				do {
					next = here->next;
					delete here;
					here = next;
				} while (here != NULL);
			}
			_nVerts = 0;
		}

		Face &Face::operator=(const Face &assignMe) {
			set(assignMe);
			return *this;
		}

		void Face::set(const Face &face) {
			cleanup();

			plane = face.plane;
			_nVerts = face._nVerts;

			// need to make copies of all half-edges
			// then reconnect them
			PHalfEdge here = face._firstHE;
			_firstHE = new HalfEdge(here->head, here->tail, this);
			here = here->next;
			PHalfEdge last = _firstHE;
			do {
				last->next = new HalfEdge(here->head, here->tail, this);
				last = last->next;
				here = here->next;
			} while (here != face._firstHE);
			last->next = _firstHE;
			// Face should be constructed, but disconnected
		}

		void Face::set(const PVertex3dList &verts) {
			if (verts.size() > 2) {
				_nVerts = verts.size();
				plane = Plane(*verts[0], *verts[1], *verts[2]);
			}
		}

 */


// PolygonMesh implementation
PolygonMesh::PolygonMesh()
: verts(), faces() {}

PolygonMesh::PolygonMesh(const PolygonMesh &copyMe)
: verts(copyMe.verts), faces(copyMe.faces) {}

PolygonMesh::PolygonMesh(const PVertex3dList &verts,
		const PPolygonList &faces)
: verts(verts), faces(faces) {}

PolygonMesh::PolygonMesh(const PPolygonList &faces, PVertex3dList &verts)
: verts(verts), faces(faces) {
}

PolygonMesh::PolygonMesh(const PVertex3dList &verts,
		const IndexListList &facesIdxs)
: verts(), faces() {
	set(verts, facesIdxs);
}

PolygonMesh::PolygonMesh(const PPolygonList &faces)
: verts(), faces() {
	set(faces);
}

PolygonMesh::~PolygonMesh() {
	cleanup();
}

PolygonMesh& PolygonMesh::operator=(const PolygonMesh& assignMe) {
	cleanup();
	set(assignMe.verts, assignMe.faces);
	return *this;
}

void PolygonMesh::cleanup() {
	verts.clear();
	faces.clear();
}

PolygonMesh::PVertex3dMap PolygonMesh::copyVertices(const PVertex3dList &verts) {
	this->verts.clear();
	PVertex3dMap vtxMap;

	size_t idx = 0;
	PVertex3dList::const_iterator vit;
	for (vit = verts.begin(); vit < verts.end(); vit++) {
		this->verts.push_back(MeshFactory::createVertex(*verts[idx]));
		vtxMap.insert(PVertex3dMap::value_type(
				*vit, this->verts[idx]));
		idx++;
	}

	return vtxMap;
}

void PolygonMesh::set(const PVertex3dList &verts,
		const PPolygonList &faces) {
	cleanup();
	PVertex3dMap vtxMap = copyVertices(verts);

	for (PPolygonList::const_iterator fit = faces.begin();
			fit < faces.end(); fit ++) {

		PVertex3dList vtxs;
		for (PVertex3dList::const_iterator it = (*fit)->verts.begin();
				it < (*fit)->verts.end(); it++) {
			vtxs.push_back( vtxMap[*(it)] );
		}
		this->faces.push_back(MeshFactory::createPolygon(vtxs));
	}

}

// modifies scratch index of vertices
void PolygonMesh::set(const PPolygonList &faces, PVertex3dList &verts) {
	cleanup();

	// create a local copy of all vertices
	copyVertices(verts);

	// we've got to change all the vertices to our new
	// internal ones.  For this, we'll  use the scratch-index
	for (size_t i=0; i<verts.size(); i++) {
		verts[i]->scratchIdx = i;
	}

	for (PPolygonList::const_iterator pit = faces.begin();
			pit < faces.end(); pit++) {

		const PPolygon &oldFace = *(pit);
		const PVertex3dList &oldVtxs = oldFace->verts;

		PVertex3dList newVtxs;
		for (size_t j=0; j<oldVtxs.size(); j++) {
			newVtxs.push_back(
					this->verts[oldVtxs[j]->scratchIdx]);
		}

		PPolygon newFace = MeshFactory::createPolygon(
				oldFace->plane, newVtxs);
		this->faces.push_back(newFace);
	}
}

void PolygonMesh::set(const PVertex3dList &verts,
		const IndexListList &facesIdxs) {
	cleanup();
	copyVertices(verts);
	typedef std::vector<std::vector<size_t> > PolygonIdxList;

	for (PolygonIdxList::const_iterator fit = facesIdxs.begin();
			fit < facesIdxs.end(); fit++) {

		PVertex3dList vtxs;
		for (std::vector<size_t>::const_iterator it = fit->begin();
				it < fit->end(); it++) {
			vtxs.push_back( this->verts[*(it)] );
		}
		this->faces.push_back(MeshFactory::createPolygon(vtxs));
	}

}

void PolygonMesh::set(const PPolygonList &faces) {
	cleanup();

	PVertex3dMap vtxMap;

	// build a face index list
	for (PPolygonList::const_iterator pit = faces.begin();
			pit < faces.end(); pit++) {

		PVertex3dList fvtxs;
		for (PVertex3dList::const_iterator vit = (*pit)->verts.begin();
				vit < (*pit)->verts.end(); vit++) {

			const PVertex3d oldVtx = *vit;
			PVertex3d newVtx = NULL;

			// check if vertex is in map
			PVertex3dMap::iterator lb = vtxMap.find(oldVtx);

			if(lb != vtxMap.end()) {
				// key already exists
				newVtx = lb->second;
			} else {
				// the key does not exist in the map
				newVtx = MeshFactory::createVertex(*oldVtx);
				this->verts.push_back(newVtx);
				vtxMap.insert(PVertex3dMap::value_type(oldVtx, newVtx));
			}
			fvtxs.push_back(newVtx);
		}

		this->faces.push_back(MeshFactory::createPolygon(fvtxs));
	}
}

size_t PolygonMesh::numVertices() const {
	return this->verts.size();
}

void PolygonMesh::triangulate() {

	PPolygonList newFaces;

	// convert all faces to triangles
	for (PPolygon face : faces) {
		if (face->numVertices() == 3) {
			newFaces.push_back(face);
		} else {
			PPolygonList tris = MeshFactory::triangulate(face);
			for (PPolygon tri : tris) {
				newFaces.push_back(tri);
			}
		}
	}

	faces = newFaces;
}

void PolygonMesh::addVertex(PVertex3d vtx) {
	verts.push_back(vtx);
}

PVertex3d PolygonMesh::addVertex(Point3d &pnt) {
	PVertex3d vtx = MeshFactory::createVertex(pnt);
	addVertex(vtx);
	return vtx;
}

void PolygonMesh::addFace(PPolygon poly) {
	faces.push_back(poly);
}

PPolygon PolygonMesh::addFace(PVertex3dList &vtxs) {
	PPolygon poly = MeshFactory::createPolygon(vtxs);
	addFace(poly);
	return poly;
}

PPolygon PolygonMesh::addFace(PVertex3d v0, PVertex3d v1, PVertex3d v2) {
	PPolygon poly = MeshFactory::createPolygon(v0, v1, v2);
	addFace(poly);
	return poly;
}

double PolygonMesh::volumeIntegrals(Vector3d* m1, Vector3d* m2,
		Vector3d* p) const {
	return volume_integrals(faces, m1, m2, p);
}

double PolygonMesh::volumeIntegral() const {
	return volume_integral(faces);
}

double PolygonMesh::volume() const {
	return volumeIntegral();
}


PVertex3d MeshFactory::createVertex() {
	return std::make_shared<Vertex3d>();
}

PVertex3d MeshFactory::createVertex(double x, double y, double z,
		int idx) {
	return std::make_shared<Vertex3d>(x,y,z,idx);
}

PVertex3d MeshFactory::createVertex(const Vector3d &v) {
	return std::make_shared<Vertex3d>(
			v.x, v.y, v.z, -1);
}

PVertex3d MeshFactory::createVertex(const Vertex3d &vert) {
	return std::make_shared<Vertex3d>(
			vert.x, vert.y, vert.z, vert.idx);
}

PPolygon MeshFactory::createPolygon() {
	return std::make_shared<Polygon>();
}

PPolygon MeshFactory::createPolygon(const Plane &plane,
		const PVertex3dList &verts) {
	return std::make_shared<Polygon>(plane, verts);
}

PPolygon MeshFactory::createPolygon(const PVertex3dList &verts) {
	return std::make_shared<Polygon>(verts);
}

PPolygon MeshFactory::createPolygon(const PVertex3d &v0,
		const PVertex3d &v1, const PVertex3d &v2) {
	return std::make_shared<Polygon>(v0, v1, v2);
}

PPolygon MeshFactory::createPolygon(const Polygon &poly) {
	return std::make_shared<Polygon>(poly.plane, poly.verts);
}

// maximum cosine angle in the triangle
double MeshFactory::maximumCosine(const PVertex3d v0,
		const PVertex3d v1, const PVertex3d v2) {

	Vector3d u01 = Vector3d(*v1);
	u01.subtract(*v0);
	u01.normalize();

	Vector3d u12 = Vector3d(*v2);
	u12.subtract(*v1);
	u12.normalize();

	Vector3d u20 = Vector3d(*v0);
	u20.subtract(*v2);
	u20.normalize();

	double maxC = u01.dot(u12);
	double c = u12.dot(u20);
	if (c > maxC) {
		maxC = c;
	}
	c = u20.dot (u01);
	if (c > maxC) {
		maxC = c;
	}

	return maxC;
}

// vertex index to remove in triangulation corresponding to the
// 2nd vertex is the most well-formed triangle
int MeshFactory::bestTriangle(PVertex3dList &verts) {

	if (verts.size() == 3) {
		return 1;
	}

	int n = verts.size();

	PVertex3d v0 = verts.at(n-1);
	PVertex3d v1 = verts.at(0);
	PVertex3d v2 = verts.at(1);

	double minC = maximumCosine(v0, v1, v2);
	int idx = 0;

	for (int i=1; i<verts.size(); i++) {
		v0 = v1;
		v1 = v2;
		v2 = verts.at( (i+1)%n );

		double c = maximumCosine(v0, v1, v2);
		if (c < minC) {
			minC = c;
			idx = i;
		}
	}

	return idx;
}

PPolygonList MeshFactory::triangulate(const PPolygon poly) {

	PPolygonList tris;

	if (poly->verts.size()==3) {
		tris.push_back(poly);
		return tris;
	}

	// PVertex3dList oldVtxs = poly->verts;
	size_t nverts = poly->numVertices();
	PVertex3dList reflex;

	if (poly->isConvex(reflex)) {
		// vertices
		PVertex3dList oldVtxs = poly->verts;

		// whittle down to four vertices, cutting off best triangles
		if (nverts > 4) {
			int n = nverts;

			for (int i=0; i<nverts-4; i++) {
				int idx = bestTriangle(oldVtxs);
				PVertex3d v0 = oldVtxs[ (idx+n-1)%n ];
				PVertex3d v1 = oldVtxs[ idx ];
				PVertex3d v2 = oldVtxs[ (idx+1)%n ];
				tris.push_back(	createPolygon(v0, v1, v2));

				// erase the idx'th element
				oldVtxs.erase (oldVtxs.begin()+idx);
				n--;
			}
		}

		// now we should have 4 vertices, try both diagonals for best split
		double cos301 = maximumCosine(
				oldVtxs[3], oldVtxs[0], oldVtxs[1]);
		double cos012 = maximumCosine(
				oldVtxs[0], oldVtxs[1], oldVtxs[2]);

		if (cos301 < cos012) {
			tris.push_back(
					createPolygon(oldVtxs[3], oldVtxs[0], oldVtxs[1]));
			tris.push_back(
					createPolygon(oldVtxs[1], oldVtxs[2], oldVtxs[3]));
		} else {
			tris.push_back(
					createPolygon(oldVtxs[0], oldVtxs[1], oldVtxs[2]));
			tris.push_back(
					createPolygon(oldVtxs[2], oldVtxs[3], oldVtxs[0]));
		}
	} else {

		// create linked list of vertices
		typedef std::list<PVertex3d> PVertex3dLList;
		PVertex3dLList oldVtxs;
		oldVtxs.insert(oldVtxs.begin(), poly->verts.begin(), poly->verts.end());

		// ear-clipping algorithm
		PVertex3dLList::iterator vit = oldVtxs.begin();
		PVertex3d vtxPrev;
		PVertex3d vtx;
		PVertex3d vtxNext;

		Vector3d v1;
		Vector3d v2;
		Vector3d vp;
		Vector3d bary;

		while(oldVtxs.size() > 3) {

			// look for an ear and remove it
			vit = oldVtxs.begin();
			vtxPrev = *vit;  vit++;
			vtx = *vit;
			PVertex3dLList::iterator earit = vit;
			vit++;
			vtxNext = *vit;

			bool foundEar = true;
			do {

				// check angle
				v1.subtract(*vtxPrev, *vtx);
				v2.subtract(*vtxNext, *vtx);
				v2.cross(v1);

				// search for ear
				if (v2.dot(poly->plane.normal) >= 0) {

					// check that all other vertices are outside triangle
					double dot11 = v1.dot(v1);
					double dot12 = v1.dot(v2);
					double dot22 = v2.dot(v2);

					double dot1p;
					double dot2p;

					foundEar = true;
					for (PVertex3d vtxr : reflex) {
						if (vtxr != vtxPrev && vtxr != vtx && vtxr != vtxNext) {

							// determine barycentric coordinates
							vp.subtract(*vtxr, *vtx);
							dot1p = v1.dot(vp);
							dot2p = v2.dot(vp);

							// Compute barycentric coordinates
							double invDenom = 1 / (dot11 * dot22 - dot12 * dot12);
							double u = (dot22 * dot1p - dot12 * dot2p) * invDenom;
							double v = (dot11 * dot2p - dot12 * dot1p) * invDenom;

							// Check if point is in triangle
							if ( (u >= 0) && (v >= 0) && (u + v < 1) ) {
								foundEar = false;
								break;
							}
						}
					} // broken out if found ear

				}

				if (!foundEar) {
					vtxPrev = vtx;
					vtx = vtxNext;

					earit = vit;
					vit++;
					if (vit == oldVtxs.end()) {
						vit = oldVtxs.begin();
					}
					vtxNext = *vit;
				}

			} while (!foundEar && vtxPrev != oldVtxs.front());

			// split off ear and remove vertex
			tris.push_back(
					createPolygon(vtxPrev, vtx, vtxNext)) ;
			oldVtxs.erase(earit);

		}

		// final triangle
		vit = oldVtxs.begin();
		vtxPrev = *vit;
		vit++;
		vtx = *vit;
		vit++;
		vtxNext = *vit;

		tris.push_back(
				createPolygon(vtxPrev, vtx, vtxNext)) ;

	}

	return tris;
}

PHalfEdge MeshFactory::createHalfEdge(const PVertex3d v0,
		const PVertex3d v1, const PPolygon face) {
	PHalfEdge he = std::make_shared<HalfEdge>(v0, v1, face);
	return he;
}

void MeshFactory::createPolyData(const PPolygon poly) {
	PPolyData data = std::make_shared<PolyData>(poly);
	poly->attach(data);
}

/**
 * Computes the volume integrals, assuming the mesh is closed/manifold.
 * Based on:
 * "Fast and Accurate Computation of Polyhedral Mass Properties,"
 * Brian Mirtich, journal of graphics tools, volume 1, number 2, 1996.
 */
double volume_integrals(const PPolygonList &polygons, Vector3d* m1, Vector3d* m2,
		Vector3d* p) {

	if (m1 != NULL) {
		m1->set(0,0,0);
	}
	if (m2 != NULL) {
		m2->set(0,0,0);
	}
	if (p != NULL) {
		p->set(0,0,0);
	}

	double vol = 0;

	for (PPolygon poly : polygons) {

		PVertex3d vtxBase = poly->verts.front();

		// compute projection direction
		Vector3d* nrm = &(poly->plane.normal);
		double x = fabs(nrm->x);
		double y = fabs(nrm->y);
		double z = fabs(nrm->z);

		int c = (x >= y) ? ((x >= z) ? 0 : 2) : ((y >= z) ? 1 : 2);
		int a = (c + 1) % 3;
		int b = (c + 2) % 3;

		// inertial moments
		double I = 0, Ia = 0, Iaa = 0, Iaaa = 0;
		double Ib = 0, Ibb = 0, Ibbb = 0;
		double Iab = 0, Iaab = 0, Iabb = 0;

		// walk around the face
		PVertex3d v0;
		PVertex3d v1 = poly->verts.back();
		for (PVertex3d vtx : poly->verts) {

			v0 = v1;
			v1 = vtx;

			double a0 = v0->get(a);
			double a1 = v1->get(a);
			double b0 = v0->get(b);
			double b1 = v1->get(b);

			double diffa = a1-a0;
			double diffb = b1-b0;

			double a0_2 = a0*a0;
			double a0_3 = a0_2*a0;
			double a0_4 = a0_3*a0;

			double a1_2 = a1*a1;
			double a1_3 = a1_2*a1;

			double b0_2 = b0*b0;
			double b0_3 = b0_2*b0;
			double b0_4 = b0_3*b0;

			double b1_2 = b1*b1;
			double b1_3 = b1_2*b1;

			double C0 = a0 + a1;
			double Ca = a1*C0 + a0_2;
			double Caa = a1*Ca + a0_3;
			double Caaa = a1*Caa + a0_4;

			double Cb = b1*(b0 + b1) + b0_2;
			double Cbb = b1*Cb + b0_3;
			double Cbbb = b1*Cbb + b0_4;

			double Cab = 3 * a1_2 + 2 * a1 * a0 + a0_2;
			double Kab = a1_2 + 2 * a1 * a0 + 3 * a0_2;
			double Caab = a0 * Cab + 4 * a1_3;
			double Kaab = a1 * Kab + 4 * a0_3;
			double Cabb = 4 * b1_3 + 3 * b1_2 * b0 + 2 * b1 * b0_2 + b0_3;
			double Kabb = b1_3 + 2 * b1_2 * b0 + 3 * b1 * b0_2 + 4 * b0_3;

			I += diffb * C0;
			Ia += diffb * Ca;
			Iaa += diffb * Caa;
			Iaaa += diffb * Caaa;
			Ib += diffa * Cb;
			Ibb += diffa * Cbb;
			Ibbb += diffa * Cbbb;
			Iab += diffb * (b1 * Cab + b0 * Kab);
			Iaab += diffb * (b1 * Caab + b0 * Kaab);
			Iabb += diffa * (a1 * Cabb + a0 * Kabb);
		}

		I /= 2.0;
		Ia /= 6.0;
		Iaa /= 12.0;
		Iaaa /= 20.0;
		Ib /= -6.0;
		Ibb /= -12.0;
		Ibbb /= -20.0;
		Iab /= 24.0;
		Iaab /= 60.0;
		Iabb /= -60.0;

		double d = -nrm->dot (vtxBase->x, vtxBase->y, vtxBase->z);

		double na = nrm->get(a);
		double nb = nrm->get(b);
		double nc = nrm->get(c);
		double ncinv = 1.0 / nc;	// for fast division

		if (a == 0) {
			vol += ncinv * na * Ia;
		} else if (b == 0) {
			vol += ncinv * nb * Ib;
		} else {
			vol -= ((d * I + na * Ia + nb * Ib) * ncinv);
		}

		double na_2 = na*na;
		double nb_2 = nb*nb;
		double ncinv_2 = ncinv*ncinv;

		if (m1 != NULL) {
			double na_3 = na_2*na;
			double nb_3 = nb_2*nb;
			double Icc = ( na_2*Iaa + 2*na*nb*Iab + nb_2*Ibb
					+ d	* ( 2*(na*Ia + nb*Ib) + d*I)) * (ncinv*ncinv);

			m1->set(a, m1->get(a) + ncinv*na*Iaa);
			m1->set(b, m1->get(b) + ncinv*nb*Ibb);
			m1->set(c, m1->get(c) + Icc);
		}

		if (m2 != NULL) {
			double na_3 = na_2*na;
			double nb_3 = nb_2*nb;
			double ncinv_3 = ncinv_2*ncinv;
			double Iccc = -(na_3*Iaaa + 3*na_2*nb*Iaab + 3*na*nb_2*Iabb
					+ nb_3*Ibbb + 3*(na_2*Iaa + 2*na*nb*Iab + nb_2*Ibb)*d
					+ d*d * (3*(na*Ia + nb*Ib) + d*I))*ncinv_3;

			m2->set (a, m2->get(a) + ncinv * na * Iaaa);
			m2->set (b, m2->get(b) + ncinv * nb * Ibbb);
			m2->set (c, m2->get(c) + Iccc);
		}

		if (p != NULL) {
			double Ibbc = -(d * Ibb + na * Iabb + nb * Ibbb) * ncinv;
			double Icca = (na_2 * Iaaa + 2 * na * nb * Iaab + nb_2 * Iabb
					+ d * (2 * (na * Iaa + nb * Iab) + d * Ia)) * ncinv_2;

			p->set(a, p->get(a) + ncinv * nb * Ibbc);
			p->set(b, p->get(b) + Icca);
			p->set(c, p->get(c) + ncinv * na * Iaab);
		}
	}

	if (m1 != NULL) {
		m1->scale(0.5);
	}
	if (m2 != NULL) {
		m2->scale (1.0 / 3.0);
	}
	if (p != NULL) {
		p->scale (0.5);
	}

	return vol;
}

/**
 * Computes the volume integral, assuming the mesh is closed/manifold.
 * Based on:
 * "Fast and Accurate Computation of Polyhedral Mass Properties,"
 * Brian Mirtich, journal of graphics tools, volume 1, number 2, 1996.
 */
double volume_integral(const PPolygonList &polygons) {

	double vol = 0;

	for (PPolygon poly : polygons) {

		PVertex3d vtxBase = poly->verts.front();

		// compute projection direction
		Vector3d* nrm = &(poly->plane.normal);
		double x = fabs(nrm->x);
		double y = fabs(nrm->y);
		double z = fabs(nrm->z);

		int c = (x >= y) ? ((x >= z) ? 0 : 2) : ((y >= z) ? 1 : 2);
		int a = (c + 1) % 3;
		int b = (c + 2) % 3;

		// inertial moments
		double I = 0, Ia = 0, Ib = 0;

		// walk around the face
		PVertex3d v0;
		PVertex3d v1 = poly->verts.back();
		for (PVertex3d vtx : poly->verts) {

			v0 = v1;
			v1 = vtx;

			double a0 = v0->get(a);
			double a1 = v1->get(a);
			double b0 = v0->get(b);
			double b1 = v1->get(b);

			double C0 = a0 + a1;
			double Ca = a1*C0 + a0*a0;
			double Cb = b1*(b0 + b1) + b0*b0;

			I += (b1-b0) * C0;
			Ia += (b1-b0) * Ca;
			Ib += (a1-a0) * Cb;
		}

		I /= 2.0;
		Ia /= 6.0;
		Ib /= -6.0;

		double d = -nrm->dot (vtxBase->x, vtxBase->y, vtxBase->z);

		double na = nrm->get(a);
		double nb = nrm->get(b);
		double nc = nrm->get(c);
		double ncinv = 1.0 / nc;	// for fast division

		if (a == 0) {
			vol += ncinv * na * Ia;
		} else if (b == 0) {
			vol += ncinv * nb * Ib;
		} else {
			vol -= ((d * I + na * Ia + nb * Ib) * ncinv);
		}
	}

	return vol;
}

double polygon_area(PPolygon poly) {
	double a = 0;

	PVertex3d v0 = poly->verts[0];
	PVertex3d v1 = poly->verts[1];

	double d2x = v1->x - v0->x;
	double d2y = v1->y - v0->y;
	double d2z = v1->z - v0->z;

	for (int i=2; i<poly->numVertices(); i++) {

		PVertex3d v2 = poly->verts[i];

		double d1x = d2x;
		double d1y = d2y;
		double d1z = d2z;

		d2x = v2->x - v0->x;
		d2y = v2->y - v0->y;
		d2z = v2->z - v0->z;

		double nx = d1y * d2z - d1z * d2y;
		double ny = d1z * d2x - d1x * d2z;
		double nz = d1x * d2y - d1y * d2x;

		a += sqrt (nx*nx + ny*ny + nz*nz);
	}

	return a/2;
}

double area_integral(PPolygonList &polygons) {
	double area = 0;

	for (PPolygon poly : polygons) {
		area += polygon_area(poly);
	}

	return area;
}

bool point_in_triangle(const Point3d &pnt,
		const Point3d &p0, const Point3d &p1, const Point3d &p2,
		Vector3d &bary) {

	// barycentric coordinate test
	Vector3d v0 = Vector3d(p2);
	v0.subtract(p0);
	Vector3d v1 = Vector3d(p1);
	v1.subtract(p0);
	Vector3d v2 = Vector3d(pnt);
	v2.subtract(p0);

	double dot00 = v0.dot(v0);
	double dot01 = v0.dot(v1);
	double dot02 = v0.dot(v2);
	double dot11 = v1.dot(v1);
	double dot12 = v1.dot(v2);

	double denom = 1.0/(dot00*dot11-dot01*dot01);
	double u = (dot11*dot02-dot01*dot12)*denom;
	double v = (dot00*dot12-dot01*dot02)*denom;

	bary.x = 1-u-v;
	bary.y = v;
	bary.z = u;

	return ( (u >= 0) && (v >= 0) && (u+v <= 1));

}

// point-triangle intersection
double distance_to_triangle(
		const Point3d &pnt, const Point3d &p0, const Point3d &p1,
		const Point3d &p2, Point3d &nearest, Vector3d &bary) {

	Vector3d kDiff = Vector3d(p0);
	kDiff.subtract(pnt);

	Vector3d kEdge0 = Vector3d(p1);
	kEdge0.subtract(p0);
	Vector3d kEdge1 = Vector3d(p2);
	kEdge1.subtract(p0);

	double fA00 = kEdge0.normSquared();
	double fA01 = kEdge0.dot (kEdge1);
	double fA11 = kEdge1.normSquared();
	double fB0 = kDiff.dot (kEdge0);
	double fB1 = kDiff.dot (kEdge1);
	double fC = kDiff.normSquared();
	double fDet = fabs(fA00 * fA11 - fA01 * fA01);
	double fS = fA01 * fB1 - fA11 * fB0;
	double fT = fA01 * fB0 - fA00 * fB1;
	double fSqrDistance;

	if (fS + fT <= fDet) {
		if (fS < 0) {
			// region 4
			if (fT < 0) {
				if (fB0 < 0) {
					fT = 0;
					if (-fB0 >= fA00) {
						fS = 1.0;
						fSqrDistance = fA00 + 2 * fB0 + fC;
					} else {
						fS = -fB0 / fA00;
						fSqrDistance = fB0 * fS + fC;
					}
				} else {
					fS = 0;
					if (fB1 >= 0) {
						fT = (double)0.0;
						fSqrDistance = fC;
					} else if (-fB1 >= fA11) {
						fT = 1.0;
						fSqrDistance = fA11 + 2 * fB1 + fC;
					} else {
						fT = -fB1 / fA11;
						fSqrDistance = fB1 * fT + fC;
					}
				}
			} else  {
				// region 3
				fS = 0.0;
				if (fB1 >= 0) {
					fT = 0;
					fSqrDistance = fC;
				} else if (-fB1 >= fA11) {
					fT = 1.0;
					fSqrDistance = fA11 + 2 * fB1 + fC;
				} else {
					fT = -fB1 / fA11;
					fSqrDistance = fB1 * fT + fC;
				}
			}
		} else if (fT < 0) {
			// region 5
			fT = 0;
			if (fB0 >= 0) {
				fS = 0;
				fSqrDistance = fC;
			} else if (-fB0 >= fA00) {
				fS = 1.0;
				fSqrDistance = fA00 + 2 * fB0 + fC;
			} else {
				fS = -fB0 / fA00;
				fSqrDistance = fB0 * fS + fC;
			}
		} else {
			// region 0
			// minimum at interior point
			double fInvDet = 1.0/fDet;
			fS *= fInvDet;
			fT *= fInvDet;
			fSqrDistance = fS * (fA00 * fS + fA01 * fT + 2 * fB0)
            					+ fT * (fA01 * fS + fA11 * fT + 2 * fB1) + fC;
		}
	} else {
		double fTmp0, fTmp1, fNumer, fDenom;
		// region 2
		if (fS < 0) {
			fTmp0 = fA01 + fB0;
			fTmp1 = fA11 + fB1;
			if (fTmp1 > fTmp0) {
				fNumer = fTmp1 - fTmp0;
				fDenom = fA00 - 2 * fA01 + fA11;
				if (fNumer >= fDenom) {
					fS = 1.0;
					fT = 0;
					fSqrDistance = fA00 + 2 * fB0 + fC;
				} else {
					fS = fNumer / fDenom;
					fT = 1.0 - fS;
					fSqrDistance  = fS * (fA00*fS + fA01*fT + 2*fB0)
                  						+ fT*(fA01*fS + fA11*fT + 2*fB1) + fC;
				}
			} else {
				fS = 0;
				if (fTmp1 <= 0) {
					fT = 1.0;
					fSqrDistance = fA11 + 2 * fB1 + fC;
				} else if (fB1 >= 0) {
					fT = 0;
					fSqrDistance = fC;
				} else {
					fT = -fB1 / fA11;
					fSqrDistance = fB1 * fT + fC;
				}
			}
		} else if (fT < 0) {
			// region 6
			fTmp0 = fA01 + fB1;
			fTmp1 = fA00 + fB0;
			if (fTmp1 > fTmp0) {
				fNumer = fTmp1 - fTmp0;
				fDenom = fA00 - 2 * fA01 + fA11;
				if (fNumer >= fDenom) {
					fT = 1.0;
					fS = 0.0;
					fSqrDistance = fA11 + 2 * fB1 + fC;
				} else {
					fT = fNumer / fDenom;
					fS = 1.0 - fT;
					fSqrDistance = fS*(fA00*fS + fA01*fT + 2*fB0)
                  						+ fT*(fA01*fS + fA11*fT + 2*fB1) + fC;
				}
			} else {
				fT = 0;
				if (fTmp1 <= 0) {
					fS = 1.0;
					fSqrDistance = fA00 + 2*fB0 + fC;
				} else if (fB0 >= 0) {
					fS = 0;
					fSqrDistance = fC;
				} else {
					fS = -fB0 / fA00;
					fSqrDistance = fB0 * fS + fC;
				}
			}
		} else {
			// region 1
			fNumer = fA11 + fB1 - fA01 - fB0;
			if (fNumer <= 0) {
				fS = 0;
				fT = 1.0;
				fSqrDistance = fA11 + 2 * fB1 + fC;
			} else {
				fDenom = fA00 - 2 * fA01 + fA11;
				if (fNumer >= fDenom) {
					fS = 1.0;
					fT = 0;
					fSqrDistance = fA00 + 2 * fB0 + fC;
				} else {
					fS = fNumer / fDenom;
					fT = 1.0 - fS;
					fSqrDistance = fS*(fA00*fS + fA01*fT + 2*fB0)
                  						+ fT*(fA01*fS + fA11*fT + 2*fB1) + fC;
				}
			}
		}
	}

	// barycentric coordinates
	bary.x = 1-fS-fT;
	bary.y = fS;
	bary.z = fT;

	// closest point
	nearest.scaledAdd (p0, fS, kEdge0);
	nearest.scaledAdd (fT, kEdge1);

	if (fSqrDistance < 0) {
		return 0;
	}
	return sqrt (fSqrDistance);
}
}
}
