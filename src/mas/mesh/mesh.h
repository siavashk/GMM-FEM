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
typedef std::shared_ptr<Vertex3d> PVertex3d;
typedef std::vector<Vertex3d> Vertex3dList;
typedef std::vector<PVertex3d> PVertex3dList;

// Ordered set of points, not necessarily part of a mesh
// useful for temporary structures while building a mesh
// Expected to be shared across multiple objects, so
// use smart pointers
class Polygon;
typedef std::shared_ptr<Polygon> PPolygon;
typedef std::weak_ptr<Polygon> WPolygon;
typedef std::vector<Polygon> PolygonList;
typedef std::vector<PPolygon> PPolygonList;
typedef std::vector<WPolygon> WPolygonList;

// basic half-edge building block
class HalfEdge;
typedef std::shared_ptr<HalfEdge> PHalfEdge;
typedef std::weak_ptr<HalfEdge> WHalfEdge;
typedef std::vector<PHalfEdge> PHalfEdgeList;
typedef std::vector<WHalfEdge> WHalfEdgeList;

// polygon connectivity data, adding
// connectivity structure to mesh
class PolyData;
typedef std::shared_ptr<PolyData> PPolyData;
typedef std::weak_ptr<PolyData> WPolyData;

typedef std::vector<size_t> IndexList;
typedef std::vector<IndexList> IndexListList;

// made of faces and vertices
class PolygonMesh;

// spits out smart vertices and polygons
class MeshFactory;


class Vertex3d : public Point3d {
private:
	// list in case non-manifold
	WHalfEdgeList _incident;
public:
	size_t idx;
	size_t scratchIdx;	// for temporary use
private:
	// disable copying/assigning
	Vertex3d(const Vertex3d& copyMe);
	Vertex3d& operator=(const Vertex3d& assignMe);
public:
	Vertex3d();
	Vertex3d(const Point3d& vertexMe);
	Vertex3d(double x, double y, double z);
	Vertex3d(double x, double y, double z, size_t idx);
	virtual ~Vertex3d();

	void removeIncidentEdge(const HalfEdge &he);
	void removeIncidentEdge(const PHalfEdge he);
	void addIncidentEdge(const PHalfEdge he);
	PHalfEdgeList getIncidentEdges() const;
};

// Polygon, makes up faces
class Polygon {
private:
	PPolyData _data;
public:
	PVertex3dList verts;
	Plane plane;

private:
	static Plane getPlane(const PVertex3dList &verts);

	Polygon(const Polygon& copyMe);
	Polygon& operator=(const Polygon& assignMe);
public:
	Polygon();
	Polygon(const Plane &Plane, const PVertex3dList &verts);
	Polygon(const PVertex3dList &verts);
	Polygon(const PVertex3d v0, const PVertex3d v1,
			const PVertex3d v2);
	virtual ~Polygon();

	void set(const Polygon &poly);
	void set(const PVertex3dList &verts);
	void set(const PVertex3d v0, const PVertex3d v1,
			const PVertex3d v2);
	void set(Plane &p, PVertex3dList &verts);
	void set(const PPolygon &poly);

	size_t numVertices() const;
	void computeCentroid(Point3d &centroid) const;

	void flip();

	bool isTriangular() const;
	bool isConvex() const;
	bool isConvex(PVertex3dList &reflex) const;

	// connectivity data
	PPolyData getData();
	void attach(const PPolyData data);
	PPolyData getOrCreateData(const PPolygon pthis);
};

class HalfEdge {
public:
	PVertex3d head;
	PVertex3d tail;
	WPolygon face;

	// set when connected
	PHalfEdge next;
	PHalfEdge opposite;

private:
	HalfEdge(const HalfEdge &copyMe);
	HalfEdge& operator=(const HalfEdge &assignMe);
	void disconnect();  // clear opposite's opposite if assigned
public:
	HalfEdge(const PVertex3d tail, const PVertex3d head,
			const PPolygon face = NULL, const PHalfEdge opposite = NULL);
	virtual ~HalfEdge();
	double getLength() const ;
};

// contains half-edge info
class PolyData {
public:
	PHalfEdge firstHE;
private:
	PolyData(const PolyData &copyMe);
	PolyData& operator=(const PolyData &assignMe);
public:
	PolyData(const PPolygon poly);
	virtual ~PolyData();
	void attach(const PPolyData pthis);
};

class PolygonMesh {
private:
	typedef std::unordered_map<PVertex3d, PVertex3d> PVertex3dMap;

public:
	// need permanent storage of vertices because of
	// shared links in faces
	PVertex3dList verts;
	PPolygonList faces;

private:
	PVertex3dMap copyVertices(const PVertex3dList &verts);
protected:
	void cleanup();
public:
	PolygonMesh();
	PolygonMesh(const PolygonMesh &copyMe);
	PolygonMesh(const PVertex3dList &verts, const PPolygonList &faces);
	PolygonMesh(const PPolygonList &faces, PVertex3dList &verts);
	PolygonMesh(const PVertex3dList &verts,
			const IndexListList &facesIdxs);
	PolygonMesh(const PPolygonList &faces);
	virtual ~PolygonMesh();
	virtual PolygonMesh& operator=(const PolygonMesh& assignMe);

	void set(const PVertex3dList &verts, const PPolygonList &faces);
	// faster, uses scratch index for optimization
	void set(const PPolygonList &faces, PVertex3dList &verts);
	void set(const PPolygonList &faces);
	void set(const PVertex3dList &verts, const IndexListList &facesIdxs);

	void addVertex(PVertex3d vtx);
	PVertex3d addVertex(Point3d &pnt);
	void addFace(PPolygon poly);
	PPolygon addFace(PVertex3dList &vtxs);
	PPolygon addFace(PVertex3d v0, PVertex3d v1, PVertex3d v2);

	size_t numVertices() const;
	void triangulate();
	double volume() const;
	double volumeIntegral() const;
	double volumeIntegrals(Vector3d* m1, Vector3d* m2,
			Vector3d* p) const;
};


// contains factory methods for creating and managing
// various structures
class MeshFactory {
private:
	static double maximumCosine(const PVertex3d v0,
			const PVertex3d v1, const PVertex3d v2);
	static int bestTriangle(PVertex3dList &verts);
public:
	static PVertex3d createVertex();
	static PVertex3d createVertex(double x, double y, double z,
			int idx = -1);
	static PVertex3d createVertex(const Vector3d &v);
	static PVertex3d createVertex(const Vertex3d &vert);

	/*
			static PHalfEdge createHalfEdge(PVertex3d &tail, PVertex3d &head,
				PHalfEdge opposite = NULL);
	 */
	static PPolygon createPolygon();
	static PPolygon createPolygon(const Plane &plane,
			const PVertex3dList &verts);
	static PPolygon createPolygon(const PVertex3dList &verts);
	static PPolygon createPolygon(const PVertex3d &v0, const PVertex3d &v1,
			const PVertex3d &v2);
	static PPolygon createPolygon(const Polygon &poly);
	static PPolygonList triangulate(const PPolygon poly);

	static PHalfEdge createHalfEdge(const PVertex3d p0, const PVertex3d p1,
			const PPolygon face = NULL);

	static void createPolyData(const PPolygon poly);

};

double volume_integrals(const PPolygonList &polygons, Vector3d* m1, Vector3d* m2,
		Vector3d* p);

double volume_integral(const PPolygonList &polygons);

double area_integral(const PPolygonList &polygons);

bool point_in_triangle(const Point3d &pnt,
		const Point3d &p0, const Point3d &p1, const Point3d &p2,
		Vector3d &bary);

double distance_to_triangle(const Point3d &pnt,
		const Point3d &p1, const Point3d &p2, const Point3d &p3,
		Point3d &nearest, Vector3d &bary);

}
}

#endif
