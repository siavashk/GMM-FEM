#ifndef MAS_MESH_H
#define MAS_MESH_H

#include "mas/core/base.h"
#include <vector>
#include <unordered_map>
#include <memory>

namespace mas {
namespace mesh {

// Basic point building block
// Expected to be shared by multiple edges/faces,
// so use smart pointers
class Vertex3d;
typedef std::shared_ptr<Vertex3d> SharedVertex3d;

// Ordered set of points, not necessarily part of a mesh
// useful for temporary structures while building a mesh
// Expected to be shared across multiple objects, so
// use smart pointers
class Polygon;
typedef std::shared_ptr<Polygon> SharedPolygon;

// basic half-edge building block
class HalfEdge;
typedef std::shared_ptr<HalfEdge> SharedHalfEdge;

// made of faces and vertices
class PolygonMesh;

// spits out smart vertices and polygons
class MeshFactory;

class Vertex3d: public IndexedPoint3d {
public:
	// list in case non-manifold
	std::vector<HalfEdge*> incident;
private:
	// disable copying/assigning
	Vertex3d(const Vertex3d& copyMe);
	Vertex3d& operator=(const Vertex3d& assignMe);
public:
	Vertex3d();
	Vertex3d(const Point3d& vertexMe);
	Vertex3d(const Point3d& vertexMe, size_t index);
	Vertex3d(const IndexedPoint3d& vertexMe);
	Vertex3d(double x, double y, double z);
	Vertex3d(double x, double y, double z, size_t idx);

	void addIncidentEdge(HalfEdge* he);
	bool removeIncidentEdge(const HalfEdge* he);

	std::vector<HalfEdge*>& getIncidentEdges();
};

// Polygon, makes up faces
class Polygon {

public:
	std::vector<SharedVertex3d> verts;
	Plane plane;
	SharedHalfEdge he0;
	size_t idx;

private:
	static Plane getPlane(const std::vector<SharedVertex3d>& verts);

	Polygon(const Polygon& copyMe);
	Polygon& operator=(const Polygon& assignMe);
public:
	Polygon();
	Polygon(Plane&& Plane, std::vector<SharedVertex3d>&& verts);
	Polygon(const Plane& Plane, const std::vector<SharedVertex3d>& verts);
	Polygon(std::vector<SharedVertex3d>&& verts);
	Polygon(const std::vector<SharedVertex3d>& verts);
	Polygon(const SharedVertex3d& v0, const SharedVertex3d& v1,
			const SharedVertex3d& v2);

	~Polygon();

	void set(const Polygon& poly);
	void set(std::vector<SharedVertex3d>&& verts);
	void set(const std::vector<SharedVertex3d>& verts);
	void set(const SharedVertex3d& v0, const SharedVertex3d& v1,
			const SharedVertex3d& v2);
	void set(Plane&& p, std::vector<SharedVertex3d>&& verts);
	void set(const Plane& p, const std::vector<SharedVertex3d>& verts);
	void set(const SharedPolygon& poly);

    void updatePlane();
	const Plane& getPlane() const;

	size_t numVertices() const;
	void computeCentroid(Point3d& centroid) const;

	void flip();

	bool isTriangular() const;
	bool isConvex() const;
	bool isConvex(std::vector<SharedVertex3d>& reflex) const;

	// connectivity data
	bool isConnected();
	void connect();
	void disconnect();
	SharedHalfEdge& getFirstHalfEdge();

	// index
	size_t getIndex() const;
	void setIndex(size_t index);
};

class HalfEdge {
public:
    size_t idx;
	SharedVertex3d head;
	SharedVertex3d tail;
	Polygon* face;

	// set when connected
	SharedHalfEdge next;
	HalfEdge* opposite;

private:
	HalfEdge(const HalfEdge& copyMe);
	HalfEdge& operator=(const HalfEdge& assignMe);

public:
	HalfEdge(const SharedVertex3d& tail, const SharedVertex3d& head,
			Polygon* face);
	virtual ~HalfEdge();   // needed to disconnect shared pointers
	double getLength() const;

	bool isConnected() const;
	void connect();
	void disconnect();  // clear opposite's opposite if assigned

	size_t getIndex() const;
	void setIndex(size_t index);
};

class PolygonMesh {
private:
	typedef std::unordered_map<SharedVertex3d, SharedVertex3d> SharedVertex3dMap;

public:
	// need permanent storage of vertices because of
	// shared links in faces
	std::vector<SharedVertex3d> verts;
	std::vector<SharedPolygon> faces;

private:
	static SharedVertex3dMap copyVertices(const std::vector<SharedVertex3d>& verts);

protected:
	void cleanup();
	void buildFaces(const std::vector<std::vector<size_t>>& facesIdxs);

public:
	PolygonMesh();
	PolygonMesh(const PolygonMesh& copyMe);
	PolygonMesh(std::vector<SharedVertex3d>&& verts,
			std::vector<SharedPolygon>&& faces);
	PolygonMesh(std::vector<SharedVertex3d>&& verts,
			const std::vector<std::vector<size_t>>& facesIdxs);
	virtual ~PolygonMesh();

	virtual PolygonMesh& operator=(const PolygonMesh& assignMe);

	void set(std::vector<SharedVertex3d>&& verts,
			std::vector<SharedPolygon>&& faces);
	void set(std::vector<SharedVertex3d>&& verts,
			const std::vector<std::vector<size_t>>& facesIdxs);

	void clone(const PolygonMesh& mesh);
	void clone(const std::vector<SharedVertex3d> verts,
			const std::vector<SharedPolygon> faces);
	void clone(const std::vector<SharedPolygon> faces);

	void updateIndices();

	void addVertex(SharedVertex3d&& vtx);
	SharedVertex3d& addVertex(const Point3d& pnt);
	SharedVertex3d removeVertex(size_t idx, bool swapLast = true);

	void addFace(SharedPolygon&& poly);
	SharedPolygon& addFace(std::vector<SharedVertex3d>&& vtxs);
	SharedPolygon& addFace(const SharedVertex3d& v0, const SharedVertex3d& v1,
			const SharedVertex3d& v2);
	SharedPolygon removeFace(size_t idx, bool swapLast = true);

	SharedVertex3d& getVertex(size_t idx);
    SharedPolygon& getFace(size_t idx);

	size_t numVertices() const;
	size_t numFaces() const;
	void triangulate();


    // connectivity data
    void connect();
    void disconnect();

	double volume() const;
	double volumeIntegral() const;
	double volumeIntegrals(Vector3d* m1, Vector3d* m2 = nullptr, Vector3d* p =
			nullptr) const;
};

// contains factory methods for creating and managing
// various structures
class MeshFactory {
private:
	static double maximumCosine(const Point3d& v0,	const Point3d& v1, const Point3d& v2);
	static int bestTriangle(const std::vector<SharedVertex3d>& verts);
public:

	static std::vector<std::unique_ptr<Polygon>> triangulate(
			const Polygon& poly);

};

double volume_integrals(const std::vector<SharedPolygon>& polygons,
		Vector3d* m1 = nullptr, Vector3d* m2 = nullptr, Vector3d* p = nullptr);

double volume_integral(const std::vector<SharedPolygon>& polygons);

double area_integral(const std::vector<SharedPolygon>& polygons);

bool point_in_triangle(const Point3d& pnt, const Point3d& p0, const Point3d& p1,
		const Point3d& p2, Vector3d& bary);

double distance_to_triangle(const Point3d& pnt, const Point3d& p1,
		const Point3d& p2, const Point3d& p3, Point3d& nearest, Vector3d& bary);

}
}

#endif
