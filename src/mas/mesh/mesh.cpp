#include "mas/mesh/mesh.h"
#include <unordered_map>
#include <list>
#include <algorithm>

#define MAX_SIZE_T (size_t)(-1)

namespace mas {
namespace mesh {

// Vertex3d implementation
Vertex3d::Vertex3d() :
        IndexedPoint3d(), incident() {
}

Vertex3d::Vertex3d(const Point3d& vertexMe) :
        IndexedPoint3d(vertexMe, -1), incident() {
}

Vertex3d::Vertex3d(const Point3d& vertexMe, size_t idx) :
        IndexedPoint3d(vertexMe, idx), incident() {
}

Vertex3d::Vertex3d(const IndexedPoint3d& vertexMe) :
        IndexedPoint3d(vertexMe), incident() {
}

Vertex3d::Vertex3d(double x, double y, double z) :
        IndexedPoint3d(x, y, z, -1), incident() {
}

Vertex3d::Vertex3d(double x, double y, double z, size_t idx) :
        IndexedPoint3d(x, y, z, idx), incident() {
}

void Vertex3d::addIncidentEdge(HalfEdge* he) {
    incident.push_back(he);
}

bool Vertex3d::removeIncidentEdge(const HalfEdge* he) {

    for (std::vector<HalfEdge*>::iterator hit = incident.begin();
            hit < incident.end(); hit++) {
        if (*hit == he) {
            incident.erase(hit);
            return true;
        }
    }

    return false;
}

std::vector<HalfEdge*>& Vertex3d::getIncidentEdges() {
    return incident;
}

// Polygon
Polygon::Polygon() :
        plane(), verts(), he0(nullptr), idx(MAX_SIZE_T) {
}

Polygon::Polygon(Plane&& plane, std::vector<SharedVertex3d>&& verts) :
        plane(std::move(plane)), verts(std::move(verts)), he0(nullptr), idx(
                MAX_SIZE_T) {
}

Polygon::Polygon(const Plane& plane, const std::vector<SharedVertex3d>& verts) :
        plane(plane), verts(verts), he0(nullptr), idx(MAX_SIZE_T) {
}

Polygon::Polygon(std::vector<SharedVertex3d>&& verts) :
        plane(), verts(std::move(verts)), he0(nullptr) {
    plane = getPlane(this->verts);
}

Polygon::Polygon(const std::vector<SharedVertex3d>& verts) :
        plane(), verts(verts), he0(nullptr), idx(MAX_SIZE_T) {
    plane = getPlane(this->verts);
}

Polygon::Polygon(const SharedVertex3d& v0, const SharedVertex3d& v1,
        const SharedVertex3d& v2) :
        plane(*v0, *v1, *v2), verts(), he0(nullptr), idx(MAX_SIZE_T) {
    verts.reserve(3);
    verts.push_back(v0);
    verts.push_back(v1);
    verts.push_back(v2);
}

Polygon::~Polygon() {
    // take down connectivity
    if (he0 != nullptr) {
        disconnect();
    }
}

Plane Polygon::getPlane(const std::vector<SharedVertex3d>& verts) {
    size_t n = verts.size();

    int v0 = 0;
    int v1 = 1;
    int v2 = 2;

    Plane plane;
    bool success = plane.set(*verts[v0], *verts[v1], *verts[v2]);
    int idx = 1;
    while (!success && idx < n) {
        v0 = v1;
        v1 = v2;
        v2 = (idx + 2) % n;
        success = plane.set(*verts[v0], *verts[v1], *verts[v2]);
        idx++;
    }
    return plane;
}

const Plane& Polygon::getPlane() const {
    return plane;
}

void Polygon::updatePlane() {
    plane = getPlane(verts);
}

void Polygon::set(const Polygon& poly) {
    he0 = nullptr; // for triggering rebuild
    this->verts = poly.verts;
    this->plane = poly.plane;
}

void Polygon::set(std::vector<SharedVertex3d>&& verts) {
    he0 = nullptr;	// for triggering rebuild
    this->verts = std::move(verts);
    plane = getPlane(this->verts);
}

void Polygon::set(const std::vector<SharedVertex3d>& verts) {
    he0 = nullptr;	// for triggering rebuild
    this->verts = verts;
    plane = getPlane(this->verts);
}

void Polygon::set(const SharedVertex3d& v0, const SharedVertex3d& v1,
        const SharedVertex3d& v2) {
    he0 = nullptr;	// for triggering rebuild
    verts.clear();
    verts.push_back(v0);
    verts.push_back(v1);
    verts.push_back(v2);
    plane.set(*v0, *v1, *v2);
}

size_t Polygon::numVertices() const {
    return verts.size();
}

void Polygon::computeCentroid(Point3d& centroid) const {
    centroid.setZero();
    for (const SharedVertex3d& v : verts) {
        centroid.add(*v);
    }
    centroid.scale(1.0 / numVertices());
}

void Polygon::flip() {
    he0 = nullptr;	// for triggering rebuild
    std::reverse(verts.begin(), verts.end());
    plane.flip();
}

bool Polygon::isTriangular() const {
    return (numVertices() == 3);
}

bool Polygon::isConvex(std::vector<SharedVertex3d>& reflex) const {

    int nV = numVertices();

    if (nV <= 3) {
        return true;
    }
    bool ic = true;

    // temp vectors for computing cross products
    Vector3d v1;
    Vector3d v2;

    int vtx0 = nV - 2;
    int vtx1 = nV - 1;
    for (size_t i = 0; i < verts.size(); i++) {
        v1.subtract(*verts[vtx0], *verts[vtx1]);
        v2.subtract(*verts[i], *verts[vtx1]);
        v2.cross(v1);

        if (v2.dot(plane.normal) < 0) {
            reflex.push_back(verts[vtx1]);
            ic = false;
        }

        vtx0 = vtx1;
        vtx1 = i;
    }

    return ic;
}

bool Polygon::isConvex() const {

    int nV = numVertices();
    if (nV <= 3) {
        return true;
    }

    // temp vectors for computing cross products
    Vector3d v1;
    Vector3d v2;

    int vtx0 = nV - 2;
    int vtx1 = nV - 1;
    for (int i = 0; i < verts.size(); i++) {
        v1.subtract(*verts[vtx0], *verts[vtx1]);
        v2.subtract(*verts[i], *verts[vtx1]);
        v2.cross(v1);

        if (v2.dot(plane.normal) < 0) {
            return false;
        }

        vtx0 = vtx1;
        vtx1 = i;
    }

    return true;
}

bool Polygon::isConnected() {
    return (he0 != nullptr);
}

void Polygon::connect() {

    // build half-edges from vertex list
    int v0 = verts.size() - 1;
    int v1 = 0;

    he0 = std::make_shared<HalfEdge>(verts[v0], verts[v1], this);
    HalfEdge* he = he0.get();

    // attach half-edges to themselves
    for (int i = 1; i < verts.size(); i++) {
        v0 = v1;
        v1 = i;
        he->next = std::make_shared<HalfEdge>(verts[v0], verts[v1], this);
        he->connect();  // add connections to vertices
        he = he->next.get();
    }
    he->connect();
    he->next = he0;

}

void Polygon::disconnect() {

    if (he0 == nullptr) {
        return;
    }

    // loop around all half edges, disconnecting
    HalfEdge* he = he0.get();
    HalfEdge* next = he0->next.get();
    do {
        he->disconnect();
        he->next = nullptr;  // clear next
        he = next;
        next = he->next.get();
    } while (he != he0.get());
    he0 = nullptr;  // destroy

}

SharedHalfEdge& Polygon::getFirstHalfEdge() {
    return he0;
}

size_t Polygon::getIndex() const {
    return idx;
}

void Polygon::setIndex(size_t index) {
    idx = index;
}

HalfEdge::HalfEdge(const SharedVertex3d& tail, const SharedVertex3d& head,
        Polygon* face) :
        head(head), tail(tail), face(face), opposite(nullptr), next(nullptr), idx(
                MAX_SIZE_T), primary(false) {
}

HalfEdge::~HalfEdge() {
    disconnect();
}

bool HalfEdge::isConnected() const {
    return next != nullptr;
}

void HalfEdge::connect() {

    // add incident
    head->addIncidentEdge(this);
    opposite = nullptr;

    // find opposite
    for (HalfEdge* he : tail->incident) {
        if (he->tail == head) {
            opposite = he;
            he->opposite = this;
        }
    }

}

void HalfEdge::disconnect() {
    // clear opposite's reference to me
    if (opposite != nullptr) {
        opposite->opposite = nullptr;
    }
    opposite = nullptr; // remove my reference to opposite

    // remove all vertex references to me
    head->removeIncidentEdge(this);

    // prev he needs to be disconnected outside
}

double HalfEdge::getLength() const {
    return head->distance(*tail);
}

size_t HalfEdge::getIndex() const {
    return idx;
}

bool HalfEdge::isPrimary() const {
    return primary;
}

void HalfEdge::setIndex(size_t index, bool primary) {
    idx = index;
    this->primary = primary;
}

// PolygonMesh implementation
PolygonMesh::PolygonMesh() :
        verts(), faces() {
}

PolygonMesh::PolygonMesh(const PolygonMesh& copyMe) :
    verts(), faces() {
    // create a local copy of all vertices
    SharedVertex3dMap copyMap = copyVertices(copyMe.verts);

    for (const SharedPolygon& face : copyMe.faces) {

        // duplicate face
        std::vector<SharedVertex3d> vtxs;
        vtxs.reserve(face->numVertices());
        for (const SharedVertex3d& vtx : face->verts) {
            // find corresponding vertex in map
            vtxs.push_back(copyMap[vtx]);
        }
        addFace(std::move(vtxs));
    }

    // copy vertices over
    for (auto& x : copyMap) {
        addVertex(std::move(x.second));
    }
}

PolygonMesh::PolygonMesh(std::vector<SharedVertex3d>&& verts,
        std::vector<SharedPolygon>&& faces) :
        verts(std::move(verts)), faces(std::move(faces)) {
}

PolygonMesh::PolygonMesh(std::vector<SharedVertex3d>&& verts,
        const std::vector<std::vector<size_t>>& facesIdxs) :
        verts(std::move(verts)), faces() {
    buildFaces(facesIdxs);
}

PolygonMesh::~PolygonMesh() {
    cleanup();
}

PolygonMesh& PolygonMesh::operator=(const PolygonMesh& assignMe) {
    clone(assignMe.verts, assignMe.faces);
    return *this;
}

void PolygonMesh::cleanup() {
    verts.clear();
    faces.clear();
}

void PolygonMesh::buildFaces(
        const std::vector<std::vector<size_t>>& facesIdxs) {

    for (const std::vector<size_t>& face : facesIdxs) {
        std::vector<SharedVertex3d> vtxs;
        vtxs.reserve(face.size());
        for (size_t i : face) {
            vtxs.push_back(verts[i]);
        }
        addFace(std::move(vtxs));
    }

}

PolygonMesh::SharedVertex3dMap PolygonMesh::copyVertices(
        const std::vector<SharedVertex3d>& verts) {
    SharedVertex3dMap vtxMap;
    vtxMap.reserve(verts.size());

    for (const SharedVertex3d& vtx : verts) {
        SharedVertex3d nvtx = std::make_shared<Vertex3d>(vtx->x, vtx->y, vtx->z,
                vtx->getIndex());
        vtxMap.insert(SharedVertex3dMap::value_type(vtx, std::move(nvtx)));
    }

    return vtxMap;
}

void PolygonMesh::set(std::vector<SharedVertex3d>&& verts,
        std::vector<SharedPolygon>&& faces) {
    cleanup();
    this->verts = std::move(verts);
    this->faces = std::move(faces);
}

void PolygonMesh::set(std::vector<SharedVertex3d>&& verts,
        const std::vector<std::vector<size_t>>& facesIdxs) {
    cleanup();
    this->verts = std::move(verts);
    buildFaces(facesIdxs);
}

void PolygonMesh::clone(const PolygonMesh& mesh) {
    clone(mesh.verts, mesh.faces);
}

// modifies scratch index of vertices
void PolygonMesh::clone(const std::vector<SharedVertex3d> verts,
        const std::vector<SharedPolygon> faces) {
    cleanup();

    // create a local copy of all vertices
    SharedVertex3dMap copyMap = copyVertices(verts);

    for (const SharedPolygon& face : faces) {

        // duplicate face
        std::vector<SharedVertex3d> vtxs;
        vtxs.reserve(face->numVertices());
        for (const SharedVertex3d& vtx : face->verts) {
            // find corresponding vertex in map
            vtxs.push_back(copyMap[vtx]);
        }
        addFace(std::move(vtxs));
    }

    // copy vertices over
    for (auto& x : copyMap) {
        addVertex(std::move(x.second));
    }

}

// modifies scratch index of vertices
void PolygonMesh::clone(const std::vector<SharedPolygon> faces) {
    cleanup();

    // create a local copy of all vertices
    SharedVertex3dMap copyMap;

    for (const SharedPolygon& face : faces) {

        // duplicate face
        std::vector<SharedVertex3d> vtxs;
        vtxs.reserve(face->numVertices());
        for (const SharedVertex3d& vtx : face->verts) {

            // detect if in map
            auto entry = copyMap.find(vtx);
            if (entry == copyMap.end()) {
                // add vertex
                SharedVertex3d nvtx = std::make_shared<Vertex3d>(vtx->x, vtx->y,
                        vtx->z, vtx->getIndex());
                copyMap[vtx] = nvtx;
                vtxs.push_back(std::move(nvtx));
            } else {
                vtxs.push_back(entry->second);
            }
        }
        addFace(std::move(vtxs));
    }

    // copy vertices over
    for (auto& x : copyMap) {
        addVertex(std::move(x.second));
    }

}

void PolygonMesh::updateIndices() {

    size_t idx = 0;
    for (SharedVertex3d& vtx : verts) {
        vtx->setIndex(idx++);
    }

    idx = 0;
    bool hasEdges = false;
    for (SharedPolygon& face : faces) {
        face->setIndex(idx++);

        // reset indices on edges
        if (face->he0 != nullptr) {
            hasEdges = true;
            HalfEdge* he = face->he0.get();
            do {
                he->setIndex(MAX_SIZE_T, false);
                he = he->next.get();
            } while (he != face->he0.get());
        }
    }

    if (hasEdges) {
        idx = 0;
        for (SharedPolygon& face : faces) {
            // reset indices on edges
            if (face->he0 != nullptr) {
                hasEdges = true;
                HalfEdge* he = face->he0.get();
                do {
                    if (he->getIndex() == MAX_SIZE_T) {
                        he->setIndex(idx, true);
                        // opposite to same index
                        if (he->opposite != nullptr) {
                            he->opposite->setIndex(idx, false);
                        }
                        idx++;
                    }
                    he = he->next.get();
                } while (he != face->he0.get());
            }
        }
    }

}

void PolygonMesh::addVertex(SharedVertex3d&& vtx) {
    verts.push_back(std::move(vtx));
    verts.back()->setIndex(verts.size() - 1);
}

SharedVertex3d& PolygonMesh::addVertex(const Point3d& pnt) {
    SharedVertex3d vtx = std::make_shared<Vertex3d>(pnt, -1);
    addVertex(std::move(vtx));
    return verts.back();
}

SharedVertex3d PolygonMesh::removeVertex(size_t idx, bool swapLast) {

    SharedVertex3d out = std::move(verts[idx]);
    size_t last = verts.size() - 1;
    out->setIndex(last);

    if (idx != last) {
        if (swapLast) {
            verts[idx] = std::move(verts[last]);
            verts[idx]->setIndex(idx);

        } else {
            // shift
            for (int i = idx; i < last; i++) {
                verts[i] = std::move(verts[i + 1]);
                verts[i]->setIndex(i);
            }
        }
    }
    verts.pop_back();

    return out;
}

void PolygonMesh::addFace(SharedPolygon&& poly) {
    faces.push_back(std::move(poly));
    faces.back()->setIndex(faces.size() - 1);  // update index
}

SharedPolygon& PolygonMesh::addFace(std::vector<SharedVertex3d>&& vtxs) {
    SharedPolygon poly = std::make_shared<Polygon>(std::move(vtxs));
    addFace(std::move(poly));
    return faces.back();
}

SharedPolygon& PolygonMesh::addFace(const SharedVertex3d& v0,
        const SharedVertex3d& v1, const SharedVertex3d& v2) {
    SharedPolygon poly = std::make_shared<Polygon>(v0, v1, v2);
    addFace(std::move(poly));
    return faces.back();
}

SharedPolygon PolygonMesh::removeFace(size_t idx, bool swapLast) {

    SharedPolygon out = std::move(faces[idx]);
    size_t last = faces.size() - 1;
    out->setIndex(last);

    if (idx != last) {
        if (swapLast) {

            faces[idx] = std::move(faces[last]);
            faces[idx]->setIndex(idx);

        } else {
            // shift
            for (int i = idx; i < last; i++) {
                faces[i] = std::move(faces[i + 1]);
                faces[i]->setIndex(i);
            }
        }
    }
    faces.pop_back();

    return out;
}

SharedVertex3d& PolygonMesh::getVertex(size_t idx) {
    return verts[idx];
}

SharedPolygon& PolygonMesh::getFace(size_t idx) {
    return faces[idx];
}

size_t PolygonMesh::numVertices() const {
    return this->verts.size();
}

size_t PolygonMesh::numFaces() const {
    return this->faces.size();
}

void PolygonMesh::triangulate() {

    std::vector<SharedPolygon> oldFaces = std::move(faces);
    faces.clear();
    faces.reserve(oldFaces.size());

    // convert all old faces to triangles
    for (SharedPolygon& face : oldFaces) {
        if (face->numVertices() == 3) {
            addFace(std::move(face));
        } else {
            std::vector<std::unique_ptr<Polygon> > tris =
                    MeshFactory::triangulate(*face);
            for (auto& tri : tris) {
                addFace(std::move(tri));
            }
        }
    }

}

void PolygonMesh::connect() {

    size_t idx = 0;
    for (SharedPolygon& poly : faces) {
        poly->connect();
        // update indices
        HalfEdge* he0 = poly->he0.get();
        HalfEdge* he = he0;
        do {
            if (he->opposite != nullptr) {
                he->setIndex(he->opposite->getIndex(), false);
            } else {
                he->setIndex(idx, true);
                idx++;
            }
            he = he->next.get();
        } while (he != he0);
    }
}

void PolygonMesh::disconnect() {
    for (SharedPolygon& poly : faces) {
        poly->disconnect();
    }
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

// maximum cosine angle in the triangle
double MeshFactory::maximumCosine(const Point3d& v0, const Point3d& v1,
        const Point3d& v2) {

    Vector3d u01 = v1;
    u01.subtract(v0);
    u01.normalize();

    Vector3d u12 = v2;
    u12.subtract(v1);
    u12.normalize();

    Vector3d u20 = v0;
    u20.subtract(v2);
    u20.normalize();

    double maxC = u01.dot(u12);
    double c = u12.dot(u20);
    if (c > maxC) {
        maxC = c;
    }
    c = u20.dot(u01);
    if (c > maxC) {
        maxC = c;
    }

    return maxC;
}

// vertex index to remove in triangulation corresponding to the
// 2nd vertex is the most well-formed triangle
int MeshFactory::bestTriangle(const std::vector<SharedVertex3d>& verts) {

    if (verts.size() == 3) {
        return 1;
    }

    int n = verts.size();

    int v0 = n - 1;
    int v1 = 0;
    int v2 = 1;

    double minC = maximumCosine(*verts[v0], *verts[v1], *verts[v2]);
    int idx = 0;

    for (int i = 1; i < verts.size(); i++) {
        v0 = v1;
        v1 = v2;
        v2 = (i + 1) % n;

        double c = maximumCosine(*verts[v0], *verts[v1], *verts[v2]);
        ;
        if (c < minC) {
            minC = c;
            idx = i;
        }
    }

    return idx;
}

std::vector<std::unique_ptr<Polygon>> MeshFactory::triangulate(
        const Polygon& poly) {

    std::vector<std::unique_ptr<Polygon> > tris;

    if (poly.verts.size() == 3) {
        // need to duplicate existing polygon to make unique ptr
        tris.push_back(std::unique_ptr<Polygon>(new Polygon(poly.verts)));
        return tris;
    }

    size_t nverts = poly.numVertices();
    std::vector<SharedVertex3d> reflex;

    if (poly.isConvex(reflex)) {

        // vertices
        std::vector<SharedVertex3d> oldVtxs = poly.verts;

        // whittle down to four vertices, cutting off best triangles
        if (nverts > 4) {
            int n = nverts;

            for (int i = 0; i < nverts - 4; i++) {
                int idx = bestTriangle(oldVtxs);
                int v0 = (idx + n - 1) % n;
                int v1 = idx;
                int v2 = (idx + 1) % n;
                tris.push_back(
                        std::unique_ptr<Polygon>(
                                new Polygon(oldVtxs[v0], oldVtxs[v1],
                                        oldVtxs[v2])));

                // erase the idx'th element
                oldVtxs.erase(oldVtxs.begin() + idx);
                n--;
            }
        }

        // now we should have 4 vertices, try both diagonals for best split
        double cos301 = maximumCosine(*oldVtxs[3], *oldVtxs[0], *oldVtxs[1]);
        double cos012 = maximumCosine(*oldVtxs[0], *oldVtxs[1], *oldVtxs[2]);

        if (cos301 < cos012) {
            tris.push_back(
                    std::unique_ptr<Polygon>(
                            new Polygon(oldVtxs[3], oldVtxs[0], oldVtxs[1])));
            tris.push_back(
                    std::unique_ptr<Polygon>(
                            new Polygon(oldVtxs[1], oldVtxs[2], oldVtxs[3])));
        } else {
            tris.push_back(
                    std::unique_ptr<Polygon>(
                            new Polygon(oldVtxs[0], oldVtxs[1], oldVtxs[2])));
            tris.push_back(
                    std::unique_ptr<Polygon>(
                            new Polygon(oldVtxs[2], oldVtxs[3], oldVtxs[0])));
        }
    } else {

        // create linked list of vertices
        typedef std::list<SharedVertex3d> SharedVertex3dList;
        SharedVertex3dList oldVtxs(poly.verts.begin(), poly.verts.end());

        // ear-clipping algorithm
        SharedVertex3dList::iterator vtxPrev;
        SharedVertex3dList::iterator vtx;
        SharedVertex3dList::iterator vtxNext;
        SharedVertex3dList::iterator vit;

        Vector3d v1;
        Vector3d v2;
        Vector3d vp;
        Vector3d bary;

        while (oldVtxs.size() > 3) {

            // look for an ear and remove it
            vit = oldVtxs.begin();
            vtxPrev = vit;
            vit++;
            vtx = vit;
            SharedVertex3dList::iterator earit = vit;
            vit++;
            vtxNext = vit;

            bool foundEar = true;
            do {

                // check angle
                v1.subtract(**vtxPrev, **vtx);
                v2.subtract(**vtxNext, **vtx);
                v2.cross(v1);

                // search for ear
                if (v2.dot(poly.plane.normal) >= 0) {

                    // check that all other vertices are outside triangle
                    double dot11 = v1.dot(v1);
                    double dot12 = v1.dot(v2);
                    double dot22 = v2.dot(v2);

                    double dot1p;
                    double dot2p;

                    foundEar = true;
                    for (SharedVertex3d& vtxr : reflex) {
                        if (vtxr != *vtxPrev && vtxr != *vtx
                                && vtxr != *vtxNext) {

                            // determine barycentric coordinates
                            vp.subtract(*vtxr, **vtx);
                            dot1p = v1.dot(vp);
                            dot2p = v2.dot(vp);

                            // Compute barycentric coordinates
                            double invDenom = 1
                                    / (dot11 * dot22 - dot12 * dot12);
                            double u = (dot22 * dot1p - dot12 * dot2p)
                                    * invDenom;
                            double v = (dot11 * dot2p - dot12 * dot1p)
                                    * invDenom;

                            // Check if point is in triangle
                            if ((u >= 0) && (v >= 0) && (u + v < 1)) {
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
                    vtxNext = vit;
                }

            } while (!foundEar && vtxPrev != oldVtxs.begin());

            // split off ear and remove vertex
            tris.push_back(
                    std::unique_ptr<Polygon>(
                            new Polygon(*vtxPrev, *vtx, *vtxNext)));
            oldVtxs.erase(earit);

        }

        // final triangle
        vit = oldVtxs.begin();
        vtxPrev = vit;
        vit++;
        vtx = vit;
        vit++;
        vtxNext = vit;

        tris.push_back(
                std::unique_ptr<Polygon>(
                        new Polygon(*vtxPrev, *vtx, *vtxNext)));

    }

    return tris;
}

/**
 * Computes the volume integrals, assuming the mesh is closed/manifold.
 * Based on:
 * "Fast and Accurate Computation of Polyhedral Mass Properties,"
 * Brian Mirtich, journal of graphics tools, volume 1, number 2, 1996.
 */
double volume_integrals(const std::vector<SharedPolygon>& polygons,
        Vector3d* m1, Vector3d* m2, Vector3d* p) {

    if (m1 != nullptr) {
        m1->set(0, 0, 0);
    }
    if (m2 != nullptr) {
        m2->set(0, 0, 0);
    }
    if (p != nullptr) {
        p->set(0, 0, 0);
    }

    double vol = 0;

    typedef std::vector<SharedVertex3d>::iterator SharedVertex3dIterator;

    for (const SharedPolygon& poly : polygons) {

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
        SharedVertex3dIterator v0;
        SharedVertex3dIterator v1 = poly->verts.begin() + poly->verts.size(); // back
        for (SharedVertex3dIterator vit = poly->verts.begin();
                vit < poly->verts.end(); vit++) {

            v0 = v1;
            v1 = vit;

            double a0 = (**v0)[a];
            double a1 = (**v1)[a];
            double b0 = (**v0)[b];
            double b1 = (**v1)[b];

            double diffa = a1 - a0;
            double diffb = b1 - b0;

            double a0_2 = a0 * a0;
            double a0_3 = a0_2 * a0;
            double a0_4 = a0_3 * a0;

            double a1_2 = a1 * a1;
            double a1_3 = a1_2 * a1;

            double b0_2 = b0 * b0;
            double b0_3 = b0_2 * b0;
            double b0_4 = b0_3 * b0;

            double b1_2 = b1 * b1;
            double b1_3 = b1_2 * b1;

            double C0 = a0 + a1;
            double Ca = a1 * C0 + a0_2;
            double Caa = a1 * Ca + a0_3;
            double Caaa = a1 * Caa + a0_4;

            double Cb = b1 * (b0 + b1) + b0_2;
            double Cbb = b1 * Cb + b0_3;
            double Cbbb = b1 * Cbb + b0_4;

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

        const Vertex3d& vtxBase = *(poly->verts[0]);
        double d = -nrm->dot(vtxBase.x, vtxBase.y, vtxBase.z);

        double na = (*nrm)[a];
        double nb = (*nrm)[b];
        double nc = (*nrm)[c];
        double ncinv = 1.0 / nc;	// for fast division

        if (a == 0) {
            vol += ncinv * na * Ia;
        } else if (b == 0) {
            vol += ncinv * nb * Ib;
        } else {
            vol -= ((d * I + na * Ia + nb * Ib) * ncinv);
        }

        double na_2 = na * na;
        double nb_2 = nb * nb;
        double ncinv_2 = ncinv * ncinv;

        if (m1 != nullptr) {
            double na_3 = na_2 * na;
            double nb_3 = nb_2 * nb;
            double Icc = (na_2 * Iaa + 2 * na * nb * Iab + nb_2 * Ibb
                    + d * (2 * (na * Ia + nb * Ib) + d * I)) * (ncinv * ncinv);

            (*m1)[a] += ncinv * na * Iaa;
            (*m1)[b] += ncinv * nb * Ibb;
            (*m1)[c] += Icc;
        }

        if (m2 != nullptr) {
            double na_3 = na_2 * na;
            double nb_3 = nb_2 * nb;
            double ncinv_3 = ncinv_2 * ncinv;
            double Iccc = -(na_3 * Iaaa + 3 * na_2 * nb * Iaab
                    + 3 * na * nb_2 * Iabb + nb_3 * Ibbb
                    + 3 * (na_2 * Iaa + 2 * na * nb * Iab + nb_2 * Ibb) * d
                    + d * d * (3 * (na * Ia + nb * Ib) + d * I)) * ncinv_3;

            (*m2)[a] += ncinv * na * Iaaa;
            (*m2)[b] += ncinv * nb * Ibbb;
            (*m2)[c] += Iccc;
        }

        if (p != nullptr) {
            double Ibbc = -(d * Ibb + na * Iabb + nb * Ibbb) * ncinv;
            double Icca = (na_2 * Iaaa + 2 * na * nb * Iaab + nb_2 * Iabb
                    + d * (2 * (na * Iaa + nb * Iab) + d * Ia)) * ncinv_2;

            (*p)[a] += ncinv * nb * Ibbc;
            (*p)[b] += Icca;
            (*p)[c] += ncinv * na * Iaab;
        }
    }

    if (m1 != nullptr) {
        m1->scale(0.5);
    }
    if (m2 != nullptr) {
        m2->scale(1.0 / 3.0);
    }
    if (p != nullptr) {
        p->scale(0.5);
    }

    return vol;
}

/**
 * Computes the volume integral, assuming the mesh is closed/manifold.
 * Based on:
 * "Fast and Accurate Computation of Polyhedral Mass Properties,"
 * Brian Mirtich, journal of graphics tools, volume 1, number 2, 1996.
 */
double volume_integral(const std::vector<SharedPolygon>& polygons) {

    double vol = 0;

    typedef std::vector<SharedVertex3d>::iterator SharedVertex3dIterator;

    for (const SharedPolygon& poly : polygons) {

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
        // walk around the face
        SharedVertex3dIterator v0;
        SharedVertex3dIterator v1 = poly->verts.begin() + poly->verts.size(); // back
        for (SharedVertex3dIterator vit = poly->verts.begin();
                vit < poly->verts.end(); vit++) {

            v0 = v1;
            v1 = vit;

            double a0 = (**v0)[a];
            double a1 = (**v1)[a];
            double b0 = (**v0)[b];
            double b1 = (**v1)[b];

            double C0 = a0 + a1;
            double Ca = a1 * C0 + a0 * a0;
            double Cb = b1 * (b0 + b1) + b0 * b0;

            I += (b1 - b0) * C0;
            Ia += (b1 - b0) * Ca;
            Ib += (a1 - a0) * Cb;
        }

        I /= 2.0;
        Ia /= 6.0;
        Ib /= -6.0;

        const Vertex3d& vtxBase = *(poly->verts[0]);
        double d = -nrm->dot(vtxBase.x, vtxBase.y, vtxBase.z);

        double na = (*nrm)[a];
        double nb = (*nrm)[b];
        double nc = (*nrm)[c];
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

double polygon_area(const Polygon& poly) {
    double a = 0;

    const SharedVertex3d& v0 = poly.verts[0];
    const SharedVertex3d& v1 = poly.verts[1];

    double d2x = v1->x - v0->x;
    double d2y = v1->y - v0->y;
    double d2z = v1->z - v0->z;

    for (int i = 2; i < poly.numVertices(); i++) {

        const SharedVertex3d& v2 = poly.verts[i];

        double d1x = d2x;
        double d1y = d2y;
        double d1z = d2z;

        d2x = v2->x - v0->x;
        d2y = v2->y - v0->y;
        d2z = v2->z - v0->z;

        double nx = d1y * d2z - d1z * d2y;
        double ny = d1z * d2x - d1x * d2z;
        double nz = d1x * d2y - d1y * d2x;

        a += sqrt(nx * nx + ny * ny + nz * nz);
    }

    return a / 2;
}

double area_integral(const std::vector<SharedPolygon>& polygons) {
    double area = 0;

    for (const SharedPolygon& poly : polygons) {
        area += polygon_area(*poly);
    }

    return area;
}

bool point_in_triangle(const Point3d& pnt, const Point3d& p0, const Point3d& p1,
        const Point3d& p2, Vector3d& bary) {

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

    double denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    double u = (dot11 * dot02 - dot01 * dot12) * denom;
    double v = (dot00 * dot12 - dot01 * dot02) * denom;

    bary.x = 1 - u - v;
    bary.y = v;
    bary.z = u;

    return ((u >= 0) && (v >= 0) && (u + v <= 1));

}

// point-triangle intersection
double distance_to_triangle(const Point3d& pnt, const Point3d& p0,
        const Point3d& p1, const Point3d& p2, Point3d& nearest,
        Vector3d& bary) {

    Vector3d kDiff = Vector3d(p0);
    kDiff.subtract(pnt);

    Vector3d kEdge0 = Vector3d(p1);
    kEdge0.subtract(p0);
    Vector3d kEdge1 = Vector3d(p2);
    kEdge1.subtract(p0);

    double fA00 = kEdge0.normSquared();
    double fA01 = kEdge0.dot(kEdge1);
    double fA11 = kEdge1.normSquared();
    double fB0 = kDiff.dot(kEdge0);
    double fB1 = kDiff.dot(kEdge1);
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
                        fT = (double) 0.0;
                        fSqrDistance = fC;
                    } else if (-fB1 >= fA11) {
                        fT = 1.0;
                        fSqrDistance = fA11 + 2 * fB1 + fC;
                    } else {
                        fT = -fB1 / fA11;
                        fSqrDistance = fB1 * fT + fC;
                    }
                }
            } else {
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
            double fInvDet = 1.0 / fDet;
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
                    fSqrDistance = fS * (fA00 * fS + fA01 * fT + 2 * fB0)
                            + fT * (fA01 * fS + fA11 * fT + 2 * fB1) + fC;
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
                    fSqrDistance = fS * (fA00 * fS + fA01 * fT + 2 * fB0)
                            + fT * (fA01 * fS + fA11 * fT + 2 * fB1) + fC;
                }
            } else {
                fT = 0;
                if (fTmp1 <= 0) {
                    fS = 1.0;
                    fSqrDistance = fA00 + 2 * fB0 + fC;
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
                    fSqrDistance = fS * (fA00 * fS + fA01 * fT + 2 * fB0)
                            + fT * (fA01 * fS + fA11 * fT + 2 * fB1) + fC;
                }
            }
        }
    }

    // barycentric coordinates
    bary.x = 1 - fS - fT;
    bary.y = fS;
    bary.z = fT;

    // closest point
    nearest.scaledAdd(p0, fS, kEdge0);
    nearest.scaledAdd(fT, kEdge1);

    if (fSqrDistance < 0) {
        return 0;
    }
    return sqrt(fSqrDistance);

}

} // end mesh
} // end mas

