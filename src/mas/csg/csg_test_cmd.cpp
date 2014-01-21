#include "mas/core/base.h"
#include "mas/core/math.h"

#include "mas/mesh/mesh.h"
#include "mas/mesh/io.h"
#include "mas/mesh/meshbv.h"

#include "mas/csg/bsp.h"
#include "mas/bvtree/bvtree.h"
#include "mas/csg/csg.h"

#include <iostream>
#include <stdlib.h>
#include <typeinfo>

// timings
#include <chrono>

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

using namespace mas;

std::vector<std::string> filterClassName(std::string name) {
	// remove namespace info, terminating E
	std::vector<std::string> out;
	if (name[0] == 'N') {
		int idx = 1;
		while (idx < name.length() && name[idx] >= '1' && name[idx] <= '9') {

			// gather number
			int len = 0;
			while (idx < name.length() && name[idx] >= '1' && name[idx] <= '9') {
				len = len*10+ (name[idx]-'0');
				idx++;
			}
			out.push_back(name.substr(idx, len));
			idx += len;
		}
	} else {
		out.push_back(name);
	}
	return out;
}

void printMat(const mas::Matrix3d &mat) {
	printf("% 2.3lf % 2.3lf % 2.3lf\n", mat.m[0], mat.m[1], mat.m[2]);
	printf("% 2.3lf % 2.3lf % 2.3lf\n", mat.m[3], mat.m[4], mat.m[5]);
	printf("% 2.3lf % 2.3lf % 2.3lf\n", mat.m[6], mat.m[7], mat.m[8]);
	printf("\n");
}

bool doOBBTreeTest(const char filename1[], const char filename2[]) {

	using mas::mesh::PolygonMesh;
	using mas::Vector3d;
	using namespace mas::bvtree;

	mas::mesh::io::SimpleObjReader reader;

	printf("Reading mesh 1\n");
	fflush(stdout);
	reader.read(filename1);
	PolygonMesh mesh1 = reader.getPolygonMesh();

	reader.clear();
	printf("Reading mesh 2\n");
	fflush(stdout);
	reader.read(filename2);
	PolygonMesh mesh2 = reader.getPolygonMesh();

	std::chrono::time_point<std::chrono::system_clock> start, end;

	printf("Constructing OBBTrees... \n");

	start = std::chrono::high_resolution_clock::now();
	POBBTree obbt1 = get_obb_tree(mesh1, 0);
	end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed_seconds = end-start;
	double contime = elapsed_seconds.count();
	printf(" tree 1: %g s, ", contime);

	PBVNodeList nodes1;
	obbt1->getLeaves(nodes1);
	printf("has %ld leaves\n", nodes1.size());

	start = std::chrono::high_resolution_clock::now();
	POBBTree obbt2 = get_obb_tree(mesh2, 0);
	end = std::chrono::high_resolution_clock::now();
	elapsed_seconds = end-start;
	contime = elapsed_seconds.count();
	printf(" tree 2: %g s, ", contime);

	PBVNodeList nodes2;
	obbt2->getLeaves(nodes2);
	printf("has %ld leaves\n", nodes2.size());

	printf("Intersecting OBBTrees... ");

	// print out BVTreeT:
	/*
	using namespace mas::mesh;
	for (int i=0; i<nodes1.size(); i++) {
		mas::Point3d c1;
		nodes1[i]->getBoundingVolume().getCentre(c1);
		printf("C: % 2.2lf % 2.2lf % 2.2lf\n", c1.x, c1.y, c1.z);


		PBoundableList elems = nodes1[i]->getElements();

		for (int j=0; j < elems.size(); j++) {
			PBoundablePolygon bp = std::static_pointer_cast<BoundablePolygon>(elems[j]);
			PPolygon p = bp->polygon;
			printf("   ");
			for (PVertex3dList::iterator pit = p->verts.begin();
				pit < p->verts.end(); pit++) {
				printf(" %d", (*pit)->idx);
			}
			printf("\n");
		}
	}
	fflush(stdout);
	 */
	nodes1.clear();
	nodes2.clear();
	start = std::chrono::high_resolution_clock::now();
	obbt1->intersectTree(obbt2, nodes1, nodes2);
	end = std::chrono::high_resolution_clock::now();
	elapsed_seconds = end-start;
	contime = elapsed_seconds.count();
	printf(" %g s\n", contime);

	printf("Intersection sizes: %ld, %ld\n\n", nodes1.size(), nodes2.size());


	return true;

}

bool doAABBTreeTest(const char filename1[], const char filename2[]) {

	using mas::mesh::PolygonMesh;
	using mas::Vector3d;
	using namespace mas::bvtree;

	mas::mesh::io::SimpleObjReader reader;

	printf("Reading mesh 1\n");
	fflush(stdout);
	reader.read(filename1);
	PolygonMesh mesh1 = reader.getPolygonMesh();

	reader.clear();
	printf("Reading mesh 2\n");
	fflush(stdout);
	reader.read(filename2);
	PolygonMesh mesh2 = reader.getPolygonMesh();

	printf("Constructing AABBTrees... \n");

	std::chrono::time_point<std::chrono::system_clock> start, end;
	start = std::chrono::high_resolution_clock::now();
	PAABBTree aabbt1 = get_aabb_tree(mesh1, 0);
	end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double>  elapsed_seconds = end-start;
	double contime = elapsed_seconds.count();
	printf(" tree 1: %g s, ", contime);

	PBVNodeList nodes1;
	aabbt1->getLeaves(nodes1);
	printf("has %ld leaves\n", nodes1.size());

	/*
	// print out leaves
	using namespace mas::bvtree;
	printf("Leaves: \n");
	for (int i=0; i<nodes1.size(); i++) {
		AABB aabb = nodes1[i]->getBoundingVolume();
		printf("   C: [ % 2.2lf % 2.2lf % 2.2lf ] ", aabb.c.x, aabb.c.y, aabb.c.z);
		printf("   HW: [ % 2.2lf % 2.2lf % 2.2lf ] \n", aabb.halfWidths.x, aabb.halfWidths.y, aabb.halfWidths.z);
	}
	 */

	start = std::chrono::high_resolution_clock::now();
	PAABBTree aabbt2 = get_aabb_tree(mesh2, 0);
	end = std::chrono::high_resolution_clock::now();
	elapsed_seconds = end-start;
	contime = elapsed_seconds.count();
	printf(" tree 2: %g s, ", contime);

	PBVNodeList nodes2;
	aabbt2->getLeaves(nodes2);
	printf("has %ld leaves\n", nodes2.size());

	printf("Intersecting AABBTrees... ");
	using namespace mas::mesh;
	// print out BVTreeT:
	/*
	for (int i=0; i<nodes1.size(); i++) {
		mas::Point3d c1;
		nodes1[i]->getBoundingVolume().getCentre(c1);
		printf("C: % 2.2lf % 2.2lf % 2.2lf\n", c1.x, c1.y, c1.z);


		PBoundableList elems = nodes1[i]->getElements();

		for (int j=0; j < elems.size(); j++) {
			PBoundablePolygon bp = std::static_pointer_cast<BoundablePolygon>(elems[j]);
			PPolygon p = bp->polygon;
			printf("   ");
			for (PVertex3dList::iterator pit = p->verts.begin();
				pit < p->verts.end(); pit++) {
				printf(" %d", (*pit)->idx);
			}
			printf("\n");
		}
	}
	fflush(stdout);
	 */

	nodes1.clear();
	nodes2.clear();
	start = std::chrono::high_resolution_clock::now();
	aabbt1->intersectTree(aabbt2, nodes1, nodes2);
	end = std::chrono::high_resolution_clock::now();
	elapsed_seconds = end-start;
	contime = elapsed_seconds.count();
	printf(" %g s\n", contime);

	printf("Intersection sizes: %ld, %ld\n\n", nodes1.size(), nodes2.size());
	return true;

}

bool doBSTreeTest(const char filename1[], const char filename2[]) {

	using mas::mesh::PolygonMesh;
	using mas::Vector3d;
	mas::mesh::io::SimpleObjReader reader;
	using namespace mas::bvtree;

	printf("Reading mesh 1\n");
	fflush(stdout);
	reader.read(filename1);
	PolygonMesh mesh1 = reader.getPolygonMesh();

	reader.clear();
	printf("Reading mesh 2\n");
	fflush(stdout);
	reader.read(filename2);
	PolygonMesh mesh2 = reader.getPolygonMesh();

	printf("Constructing BSTrees... \n");

	std::chrono::time_point<std::chrono::system_clock> start, end;
	start = std::chrono::high_resolution_clock::now();
	PBSTree bst1 = get_bs_tree(mesh1, 0);
	end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed_seconds = end-start;
	double contime = elapsed_seconds.count();
	printf(" tree 1: %g s, ", contime);

	PBVNodeList nodes1;
	bst1->getLeaves(nodes1);
	printf("has %ld leaves\n", nodes1.size());

	start = std::chrono::high_resolution_clock::now();
	mas::bvtree::PBSTree bst2 = get_bs_tree(mesh2, 0);
	end = std::chrono::high_resolution_clock::now();
	elapsed_seconds = end-start;
	contime = elapsed_seconds.count();
	printf(" tree 2: %g s, ", contime);

	PBVNodeList nodes2;
	bst2->getLeaves(nodes2);
	printf("has %ld leaves\n", nodes2.size());

	printf("Intersecting BSTrees... ");
	using namespace mas::mesh;
	// print out BVTreeT:
	/*
	for (int i=0; i<nodes1.size(); i++) {
		mas::Point3d c1;
		nodes1[i]->getBoundingVolume().getCentre(c1);
		printf("C: % 2.2lf % 2.2lf % 2.2lf\n", c1.x, c1.y, c1.z);


		PBoundableList elems = nodes1[i]->getElements();

		for (int j=0; j < elems.size(); j++) {
			PBoundablePolygon bp = std::static_pointer_cast<BoundablePolygon>(elems[j]);
			PPolygon p = bp->polygon;
			printf("   ");
			for (PVertex3dList::iterator pit = p->verts.begin();
				pit < p->verts.end(); pit++) {
				printf(" %d", (*pit)->idx);
			}
			printf("\n");
		}
	}
	fflush(stdout);
	 */

	nodes1.clear();
	nodes2.clear();
	start = std::chrono::high_resolution_clock::now();
	bst1->intersectTree(bst2, nodes1, nodes2);
	end = std::chrono::high_resolution_clock::now();
	elapsed_seconds = end-start;
	contime = elapsed_seconds.count();
	printf(" %g s\n", contime);

	printf("Intersection sizes: %ld, %ld\n\n", nodes1.size(), nodes2.size());
	return true;

}

template<typename BV1, typename BV2>
bool doCrossTreeTest(const char filename1[], const char filename2[]) {

	using namespace csg;
	using namespace mas::mesh;
	using namespace mas::bvtree;
	mas::mesh::io::SimpleObjReader reader;

	BV1 bv1;
	BV2 bv2;

	std::string class1 = filterClassName(typeid(bv1).name()).back();
	std::string class2 = filterClassName(typeid(bv2).name()).back();

	printf("\nDoing cross test: %s, %s\n", class1.c_str(), class2.c_str());
	printf("Reading mesh 1\n");
	fflush(stdout);
	reader.read(filename1);
	PolygonMesh mesh1 = reader.getPolygonMesh();

	reader.clear();
	printf("Reading mesh 2\n");
	fflush(stdout);
	reader.read(filename2);
	PolygonMesh mesh2 = reader.getPolygonMesh();

	printf("Constructing BVTrees... \n");

	std::chrono::time_point<std::chrono::system_clock> start, end;
	start = std::chrono::high_resolution_clock::now();
	PBVTree bvt1 = get_bv_tree<BV1>(mesh1, 0);
	end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed_seconds = end-start;
	double contime = elapsed_seconds.count();
	printf(" tree 1: %g s, ", contime);

	PBVNodeList nodes1;
	bvt1->getLeaves(nodes1);
	printf("has %ld leaves\n", nodes1.size());

	start = std::chrono::high_resolution_clock::now();
	PBVTree bvt2 = get_bv_tree<BV2>(mesh2, 0);
	end = std::chrono::high_resolution_clock::now();
	elapsed_seconds = end-start;
	contime = elapsed_seconds.count();
	printf(" tree 2: %g s, ", contime);

	PBVNodeList nodes2;
	bvt2->getLeaves(nodes2);
	printf("has %ld leaves\n", nodes2.size());

	printf("Intersecting BVTrees... ");
	// print out BVTreeT:
	/*
	for (int i=0; i<nodes1.size(); i++) {
		mas::Point3d c1;
		nodes1[i]->getBoundingVolume().getCentre(c1);
		printf("C: % 2.2lf % 2.2lf % 2.2lf\n", c1.x, c1.y, c1.z);


		PBoundableList elems = nodes1[i]->getElements();

		for (int j=0; j < elems.size(); j++) {
			PBoundablePolygon bp = std::static_pointer_cast<BoundablePolygon>(elems[j]);
			PPolygon p = bp->polygon;
			printf("   ");
			for (PVertex3dList::iterator pit = p->verts.begin();
				pit < p->verts.end(); pit++) {
				printf(" %d", (*pit)->idx);
			}
			printf("\n");
		}
	}
	fflush(stdout);
	 */

	nodes1.clear();
	nodes2.clear();
	start = std::chrono::high_resolution_clock::now();
	bvt1->intersectTree(bvt2, nodes1, nodes2);
	end = std::chrono::high_resolution_clock::now();
	elapsed_seconds = end-start;
	contime = elapsed_seconds.count();
	printf(" %g s\n", contime);

	printf("Intersection sizes: %ld, %ld\n\n", nodes1.size(), nodes2.size());
	return true;

}

bool doBVTreeTest(const char filename1[], const char filename2[]) {
	doOBBTreeTest(filename1, filename2);
	doAABBTreeTest(filename1, filename2);
	doBSTreeTest(filename1, filename2);

	// cross tree test
	doCrossTreeTest<mas::bvtree::AABB, mas::bvtree::OBB>(filename1, filename2);
	doCrossTreeTest<mas::bvtree::OBB, mas::bvtree::AABB>(filename1, filename2);

	doCrossTreeTest<mas::bvtree::AABB, mas::bvtree::BoundingSphere>(filename1, filename2);
	doCrossTreeTest<mas::bvtree::BoundingSphere, mas::bvtree::AABB>(filename1, filename2);

	doCrossTreeTest<mas::bvtree::OBB, mas::bvtree::BoundingSphere>(filename1, filename2);
	doCrossTreeTest<mas::bvtree::BoundingSphere, mas::bvtree::OBB>(filename1, filename2);
}

bool doBVLeafTest(const char filename[]) {
	mas::mesh::io::SimpleObjReader reader;
	reader.read(filename);
	using mas::mesh::PolygonMesh;
	using mas::Point3d;
	using mas::mesh::PPolygon;
	using mas::bvtree::PBVTree;
	using mas::bvtree::PBVNodeList;
	using mas::bvtree::OBB;
	using mas::bvtree::POBB;

	PolygonMesh mesh = reader.getPolygonMesh();

	PBVTree tree = mas::mesh::get_obb_tree(mesh);
	PBVNodeList leaves;
	tree->getLeaves(leaves);

	for (PBVNodeList::iterator pit = leaves.begin();
			pit < leaves.end(); pit++) {

		POBB obb = std::static_pointer_cast<OBB>((*pit)->bv);
		printf("OBB: c: %lf %lf %lf, hw: %lf %lf %lf\n", 
				obb->c.x, obb->c.y, obb->c.z,
				obb->halfWidths.x, obb->halfWidths.y, obb->halfWidths.z);
		printf("R: ");
		printMat(obb->R);
	}

	return true;
}

bool doIntersectionTest(const char filename[], double pnt[], double dir[]) {

	mas::mesh::io::SimpleObjReader reader;
	reader.read(filename);
	using mas::mesh::PolygonMesh;
	using mas::Point3d;
	using mas::Vector3d;
	using mas::mesh::PPolygon;
	using namespace mas::bvtree;

	PolygonMesh mesh = reader.getPolygonMesh();

	Point3d p;
	p.x = pnt[0];
	p.y = pnt[1];
	p.z = pnt[2];

	Vector3d v;
	v.x = dir[0];
	v.y = dir[1];
	v.z = dir[2];

	PBVNodeList bvnodes;
	POBBTree obbt = mas::mesh::get_obb_tree(mesh);
	obbt->intersectRay(p, v, bvnodes);

	for (PBVNodeList::iterator pit = bvnodes.begin();
			pit < bvnodes.end(); pit++) {

		for (PBoundableList::iterator bit = (*pit)->elems.begin();
				bit < (*pit)->elems.end(); bit++) {

			Point3d nearest;
			double d = (*bit)->distanceToPoint(p, v, nearest);
			if (d < 1e10 && d > -1e10) {
				printf("Intersects at: (%lf %lf %lf)\n", nearest.x, nearest.y, nearest.z);
			}
		}
	}

}

bool doNearestTest(const char filename[], double pnt[]) {

	mas::mesh::io::SimpleObjReader reader;
	reader.read(filename);
	using mas::mesh::PolygonMesh;
	using mas::Point3d;
	using mas::mesh::PPolygon;

	PolygonMesh mesh = reader.getPolygonMesh();

	Point3d p;
	p.x = pnt[0];
	p.y = pnt[1];
	p.z = pnt[2];

	Point3d nearestPoint;
	PPolygon nearest = mas::mesh::nearest_polygon(p, mesh, nearestPoint);
	int inside = mas::mesh::is_inside(p, mesh);

	printf("Polygon: %d %d %d, Point: (%lf, %lf, %lf), ", 
			nearest->verts[0]->idx, nearest->verts[1]->idx, nearest->verts[2]->idx,
			nearestPoint.x, nearestPoint.y, nearestPoint.z);

	if (inside == MESH_INSIDE_TRUE) {
		printf("Inside: true\n");
	} else if (inside == MESH_INSIDE_FALSE) {
		printf("Inside: false\n");
	} else {
		printf("Inside: unsure\n");
	}


}

using mas::mesh::PolygonMesh;

void fillMesh(PolygonMesh &mesh, double *verts, int nVerts, double *faceIdxs,
		int nVertsPerFace, int nFaces) {

	using namespace mas::mesh;
	PVertex3dList vtxList;
	IndexListList faceIdxList;

	for (int i=0; i<nVerts; i++) {
		// Failing here?
		PVertex3d vtx = MeshFactory::createVertex(verts[i*3], verts[i*3+1],
				verts[i*3+2], i);
		vtxList.push_back(vtx);
	}

	for (int i=0; i<nFaces; i++) {
		IndexList face;
		for (int j=0; j<nVertsPerFace; j++) {
			int idx = ((int)faceIdxs[i*nVertsPerFace+j])-1;
			face.push_back(idx);
		}
		faceIdxList.push_back(face);
	}

	mesh.set(vtxList, faceIdxList);

}

bool doDiceTestMatlab() {

	double verts1[] = {0, 0, -1, -1, 0, 0, 0, -1, 0,
			1, 0,  0,  0, 1, 0, 0,  0, 1 };
	double verts2[18];
	for (int i=0; i<18; i++) {
		verts2[i] = verts1[i] + 0.25;
	}

	double faces1[] = {1, 2, 5, 1, 5, 4, 1, 4, 3, 1, 3, 2,
			6, 5, 2, 6, 4, 5, 6, 3, 4, 6, 2, 3};
	double *faces2 = faces1;

	int nFaceVerts1 = 3;
	int nFaceVerts2 = 3;
	int nVerts1 = 6;
	int nVerts2 = 6;
	int nFaces1 = 8;
	int nFaces2 = 8;

	double eps = 1e-12;

	// create meshes and do dice test
	PolygonMesh mesh1;
	PolygonMesh mesh2;

	fillMesh(mesh1, verts1, nVerts1, faces1, nFaceVerts1, nFaces1);
	fillMesh(mesh2, verts2, nVerts2, faces2, nFaceVerts2, nFaces2);

	using mas::bvtree::POBBTree;
	POBBTree bvtree1 = mas::mesh::get_obb_tree(mesh1, eps);
	POBBTree bvtree2 = mas::mesh::get_obb_tree(mesh2, eps);

	double voli = mas::csg::intersection_volume(mesh1, bvtree1,
			mesh2, bvtree2, eps);

	double vol1 = mesh1.volumeIntegral();
	double vol2 = mesh2.volumeIntegral();
	double dice = 2*voli/(vol1+vol2);
	if (dice > 1) {
		dice = 1;
	}

}

using namespace mas::bvtree;

void printNode(PBVNode node, int depth) {

	std::shared_ptr<OBB> obb = std::static_pointer_cast<OBB>(node->bv);
	Point3d &c = obb->c;
	Vector3d &hw = obb->halfWidths;

	for (int i=0; i<depth; i++) {
		printf("----");
	}
	printf("c: %.3lf %.3lf %.3lf\n", c.x, c.y, c.z);
	for (int i=0; i<depth; i++) {
		printf("    ");
	}
	printf("hw: %.3lf %.3lf %.3lf\n", hw.x, hw.y, hw.z);

	for (PBVNode child : node->children) {
		printNode(child, depth+1);
	}


}

void printTree(POBBTree obbt) {

	PBVNode root = obbt->getRoot();
	printNode(root, 0);

}

bool doDiceTest(const char filename1[], const char filename2[], const char *outfilename){

	using mas::mesh::PolygonMesh;
	using mas::Vector3d;
	mas::mesh::io::SimpleObjReader reader;
	using namespace mas::bvtree;

	reader.read(filename1);
	PolygonMesh mesh1 = reader.getPolygonMesh();

	reader.clear();
	reader.read(filename2);
	PolygonMesh mesh2 = reader.getPolygonMesh();

	POBBTree obbt1 = mas::mesh::get_obb_tree(mesh1, 1e-10);
	POBBTree obbt2 = mas::mesh::get_obb_tree(mesh2, 1e-10);

	// printTree(obbt1);
	// printTree(obbt2);

	PBVNodeList nodes1;
	PBVNodeList nodes2;
	obbt1->intersectTree(obbt2, nodes1, nodes2);

	printf("# Intersecting nodes: %i\n", nodes1.size());
	double d = mas::csg::dice(mesh1, obbt1, mesh2, obbt2, 1e-15);

	printf("# Dice: %lg\n", d);
	fflush(stdout);

	int res[] = {30, 30, 30};
	d = mas::csg::dice_estimate(mesh1, mesh2, res);
	printf("# Approx Dice (%i, %i, %i): %lg\n", res[0], res[1], res[2], d);
	fflush(stdout);

	/*
	PolygonMesh meshi;
	mas::csg::cheap_intersect(mesh1, mesh2, meshi, 1e-10);

	mas::mesh::io::SimpleObjWriter writer;
	if (outfilename != NULL) {
		writer.write(outfilename);
	} else {
		writer.setPolygonMesh(&meshi);
		writer.write(std::cout);
	}

	fflush(stdout);
	*/
}

bool doInsideTest(const char filename[], double pnt[], double dx[], int nx[]) {

	mas::mesh::io::SimpleObjReader reader;
	reader.read(filename);
	using mas::mesh::PolygonMesh;
	using mas::Point3d;
	using mas::mesh::PPolygon;
	using namespace mas::bvtree;

	PolygonMesh mesh = reader.getPolygonMesh();

	Point3d pStart;
	pStart.x = pnt[0];
	pStart.y = pnt[1];
	pStart.z = pnt[2];

	POBBTree obbt = get_obb_tree(mesh);

	Point3d p;
	for (int i=0; i<nx[0]; i++) {
		p.x = pStart.x + i*dx[0];
		for (int j=0; j<nx[1]; j++) {
			p.y = pStart.y + j*dx[1];
			for (int k=0; k<nx[2]; k++) {
				p.z = pStart.z + k*dx[2];
				int inside = mas::mesh::is_inside(p, mesh, obbt);

				printf("Point: (%.5lf, %.5lf, %.5lf), Inside: ", p.x, p.y, p.z);
				if (inside == MESH_INSIDE_TRUE) {
					printf("true\n");
				} else if (inside == MESH_INSIDE_FALSE) {
					printf("false\n");
				} else {
					printf("unsure\n");
				}

			}
		}
	}

	return true;
}

bool doBoundTest() {

	using namespace csg;
	using namespace mas::mesh;
	using namespace mas::bvtree;

	PVertex3d vtx0 = MeshFactory::createVertex(0,0,1);
	PVertex3d vtx1 = MeshFactory::createVertex(1,0,0);
	PVertex3d vtx2 = MeshFactory::createVertex(0,1,0);

	PPolygon p0 = MeshFactory::createPolygon(vtx0, vtx1, vtx2);
	PBoundablePolygon bp0 = BoundableFactory::createBoundablePolygon(p0);

	PBoundableList elems;
	elems.push_back(bp0);

	OBB obb;
	obb.bound(elems);

	Point3d c;
	Vector3d hw;
	RotationMatrix3d R;
	obb.getCentre(c);
	obb.getHalfWidths(hw);
	obb.getRotation(R);

	printf("C: \t % 2.2lf % 2.2lf % 2.2lf\n", c.x, c.y, c.z);
	printf("hw: \t % 2.2lf % 2.2lf % 2.2lf\n", hw.x, hw.y, hw.z);
	printf("R: \n");
	printMat(R);

	return true;
}

bool doCSGTest(const char filename1[], const char filename2[]) {

	using mas::mesh::PolygonMesh;
	using mas::Vector3d;
	mas::mesh::io::SimpleObjReader reader;

	printf("Reading mesh 1\n");
	fflush(stdout);
	reader.read(filename1);
	PolygonMesh mesh = reader.getPolygonMesh();

	reader.clear();
	printf("Reading mesh 2\n");
	fflush(stdout);
	reader.read(filename2);
	PolygonMesh mesh2 = reader.getPolygonMesh();

	printf("Computing union\n");
	PolygonMesh meshunion = mas::csg::bsp::vol_union(mesh, mesh2);

	// write out volume results
	Vector3d m1, m2, p;
	double vol2 = meshunion.volumeIntegrals(&m1, &m2, &p);

	printf("Volume integral: %g\n", vol2);
	printf("   m1: %g %g %g\n", m1.x, m1.y, m1.z);
	printf("   m2: %g %g %g\n", m2.x, m2.y, m2.z);
	printf("    p: %g %g %g\n", p.x, p.y, p.z);

	return true;

}

bool doObjReadTest(const char filename[]) {

	mas::mesh::io::SimpleObjReader reader;
	reader.read(filename);

	using mas::mesh::PolygonMesh;
	using mas::Vector3d;
	using mas::mesh::Vertex3d;
	using mas::mesh::Polygon;

	printf("Retrieving read results\n");
	PolygonMesh mesh = reader.getPolygonMesh();

	double vol1 = mesh.volume();

	Vector3d m1, m2, p;
	double vol2 = mesh.volumeIntegrals(&m1, &m2, &p);

	printf("Volume solo: %g\n", vol1);
	printf("Volume integral: %g\n", vol2);
	printf("   m1: %g %g %g\n", m1.x, m1.y, m1.z);
	printf("   m2: %g %g %g\n", m2.x, m2.y, m2.z);
	printf("    p: %g %g %g\n", p.x, p.y, p.z);

	return true;
}

bool doObjectTests() {
	using mas::Point3d;

	Point3d pointA = Point3d(1,2,3);
	printf("Point A: (%f, %f, %f)\n", pointA.x, pointA.y, pointA.z);

	Point3d pointB = Point3d(pointA);
	pointB.x = 3;
	printf("Point B: (%f, %f, %f)\n", pointB.x, pointB.y, pointB.z);

	Point3d pointC = Point3d();
	printf("Point C: (%f, %f, %f)\n", pointC.x, pointC.y, pointC.z);	

	using mas::Vector3d;

	Vector3d vectorA = Vector3d(4,5,6);
	printf("Vector A: (%f, %f, %f)\n", vectorA.x, vectorA.y, vectorA.z);

	Vector3d vectorB = Vector3d(vectorA);
	vectorB.x = 3;
	printf("Vector B: (%f, %f, %f)\n", vectorB.x, vectorB.y, vectorB.z);

	Vector3d vectorC = Vector3d();
	printf("Vector C: (%f, %f, %f)\n", vectorC.x, vectorC.y, vectorC.z);

	Vector3d vectorD = Vector3d(pointA);
	printf("Vector D: (%f, %f, %f)\n", vectorD.x, vectorD.y, vectorD.z);

	vectorD.add(vectorB);
	printf("Vector D + B: (%f, %f, %f)\n", vectorD.x, vectorD.y, vectorD.z);

	vectorA.cross(vectorA, vectorB);
	vectorA.normalize();
	printf("Vector A x B: (%f, %f, %f), norm: %g\n", vectorA.x, vectorA.y, vectorA.z, vectorA.norm());	

	using mas::mesh::PVertex3d;
	using mas::mesh::MeshFactory;

	PVertex3d vertexA = MeshFactory::createVertex(vectorA);
	PVertex3d vertexB = MeshFactory::createVertex(1,2,3,2);
	printf("Vertex A: (%f, %f, %f, %d)\n", vertexA->x, vertexA->y, vertexA->z, vertexA->idx);	

	vertexA->set(1, 2, 3);
	vertexB->set(1, 3, 5);
	PVertex3d vertexC = MeshFactory::createVertex(3, 7, 1);

	using mas::Plane;
	Plane plane = Plane(*vertexA, *vertexB, *vertexC);
	printf("Plane: %gx + %gy + %gz = %g\n", plane.normal.x, plane.normal.y, plane.normal.z, plane.d);
	plane.flip();
	printf("Negative Plane: %gx + %gy + %gz = %g\n", plane.normal.x, plane.normal.y, plane.normal.z, plane.d);

	using mas::mesh::Polygon;
	using mas::mesh::PolygonList;
	using mas::mesh::PolygonMesh;

	using mas::mesh::Vertex3d;
	using mas::mesh::Vertex3dList;
	using mas::mesh::PVertex3dList;


	PVertex3dList verts;
	verts.push_back(MeshFactory::createVertex(0,0,0,0));
	verts.push_back(MeshFactory::createVertex(0,1,0,1));
	verts.push_back(MeshFactory::createVertex(1,2,0,2));
	verts.push_back(MeshFactory::createVertex(2,1,0,3));
	verts.push_back(MeshFactory::createVertex(2,0,0,4));

	using mas::mesh::MeshFactory;
	using mas::mesh::PPolygon;
	using mas::mesh::PPolygonList;

	PVertex3dList pverts;
	PVertex3dList preverts;
	for (int i=0; i<verts.size(); i++) {
		pverts.push_back(verts[i]);
	}
	for (int i=verts.size()-1; i>0; --i) {
		preverts.push_back(verts[i]);
	}

	PPolygon poly = MeshFactory::createPolygon(pverts);
	PPolygonList tris = MeshFactory::triangulate(poly);

	for (int i=0; i<tris.size(); i++) {
		Polygon &p = *tris.at(i);
		printf("Face %d: (%d, %d, %d)\n", i, 
				p.verts.at(0)->idx, p.verts.at(1)->idx, p.verts.at(2)->idx);
	}

	mas::mesh::PPolygonList polys;
	polys.push_back(MeshFactory::createPolygon(pverts));
	polys.push_back(MeshFactory::createPolygon(preverts));

	printf( "PolygonMesh: \n");
	PolygonMesh mesh = PolygonMesh(pverts,polys);
	mesh.triangulate();
	for (int i=0; i<mesh.faces.size(); i++) {
		Polygon &p = *(mesh.faces.at(i));
		printf("Face %d: (%d, %d, %d)\n", i, 
				p.verts.at(0)->idx, p.verts.at(1)->idx, p.verts.at(2)->idx);
	}	

	return true;
}

using mas::Matrix3d;
using mas::Vector3d;

void printSVD(const Matrix3d &mat, const Matrix3d &U, const Vector3d &S, 
		const Matrix3d &V) {
	printf("% 2.3lf % 2.3lf % 2.3lf     % 2.3lf % 2.3lf % 2.3lf     "
			"% 2.3lf % 2.3lf % 2.3lf     % 2.3lf % 2.3lf % 2.3lf\n",
			mat.m[0], mat.m[1], mat.m[2],   U.m[0], U.m[1], U.m[2],
			S.x, 0.0, 0.0, V.m[0], V.m[1], V.m[2]);
	printf("% 2.3lf % 2.3lf % 2.3lf     % 2.3lf % 2.3lf % 2.3lf     "
			"% 2.3lf % 2.3lf % 2.3lf     % 2.3lf % 2.3lf % 2.3lf\n",
			mat.m[3], mat.m[4], mat.m[5],   U.m[3], U.m[4], U.m[5],
			0.0, S.y, 0, V.m[3], V.m[4], V.m[5]);
	printf("% 2.3lf % 2.3lf % 2.3lf     % 2.3lf % 2.3lf % 2.3lf     "
			"% 2.3lf % 2.3lf % 2.3lf     % 2.3lf % 2.3lf % 2.3lf\n",
			mat.m[6], mat.m[7], mat.m[8],   U.m[6], U.m[7], U.m[8],
			0.0, 0.0, S.z, V.m[6], V.m[7], V.m[8]);
	printf("\n");
}

bool doSVDTest(double *mat) {

	//identity
	Matrix3d U, V;
	Vector3d S;

	Matrix3d Q;

	if (mat != NULL) {
		Q.set(mat);
	} else {
		Q.set(10, 3, 7, 4, 8, 9, 1, 3, 1);
	}
	mas::math::svd3(Q, U, S, V);
	printSVD(Q, U, S, V);

	Matrix3d Smat = Matrix3d(S.x, 0, 0, 0, S.y, 0, 0, 0, S.z);
	U.multiply(Smat);
	V.transpose();
	U.multiply(V);

	printMat(U);

}


int main( int argc, const char* argv[] ) {


	if (argc == 2) {
		// doBVLeafTest(argv[1]);
		doObjReadTest(argv[1]);
	} else if (argc == 3) {
		//doBVTreeTest(argv[1], argv[2]);
		//doOBBTreeTest(argv[1], argv[2]);
		doDiceTest(argv[1], argv[2], NULL);
	} else if (argc == 4) {
		doDiceTest(argv[1], argv[2], argv[3]);
	} else if (argc ==5) {
		double pnt[3];
		for (int i=0; i<3; i++) {
			sscanf(argv[i+2], "%lf", &pnt[i]);
		}
		doNearestTest(argv[1], pnt);
	} else if (argc == 8) {
		double pnt[3];
		double dir[3];
		for (int i=0; i<3; i++) {
			sscanf(argv[i+2], "%lf", &pnt[i]);
		}
		for (int i=0; i<3; i++) {
			sscanf(argv[i+5], "%lf", &dir[i]);
		}
		doIntersectionTest(argv[1], pnt, dir);
	} else if (argc == 9) {
		double *v = new double[argc-1];
		for (int i=0; i<argc-1; i++) {
			sscanf(argv[i+1], "%lf", &v[i]);
		}
		doSVDTest(v);
		delete [] v;
	} else if (argc == 11) {
		double pnt[3];
		double dx[3];
		int nx[3];

		for (int i=0; i<3; i++) {
			sscanf(argv[i+2], "%lf", &pnt[i]);
		}
		for (int i=0; i<3; i++) {
			sscanf(argv[i+5], "%lf", &dx[i]);
		}
		for (int i=0; i<3; i++) {
			sscanf(argv[i+8], "%i", &nx[i]);
		}

		doInsideTest(argv[1], pnt, dx, nx);
	} else {
		//doSVDTest(NULL);
		//doBoundTest();
		doDiceTestMatlab();
	}
}

