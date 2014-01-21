#include "mas/mesh/io.h"
#include <stdlib.h>
#include <iostream>
#include <sstream>

namespace mas {
namespace mesh {
namespace io {

namespace str {
// trim from start
std::string ltrim(const std::string &s,
		const std::string &chars) {

	for (int i=0; i<s.length(); i++) {
		size_t idx = chars.find(s[i]);
		if (idx == std::string::npos) {
			return s.substr(i);
		}
	}
	return "";

}

// trim from end
std::string rtrim(const std::string &s,
		const std::string &chars) {

	for (int i=s.length()-1; i>1; --i) {
		size_t idx = chars.find(s[i]);
		if (idx == std::string::npos) {
			return s.substr(0,i+1);
		}
	}
	return "";
}

// trim from both ends
std::string trim(const std::string &s,
		const std::string &chars) {
	return ltrim(rtrim(s, chars), chars);
}
}


SimpleObjReader::SimpleObjReader()
: mesh(NULL), needsCleanup(false), zeroIndexed(MESH_IO_ZERO_INDEXED) {}

SimpleObjReader::SimpleObjReader(PolygonMesh* mesh)
: mesh(mesh), needsCleanup(false), zeroIndexed(MESH_IO_ZERO_INDEXED) {};

SimpleObjReader::SimpleObjReader(const SimpleObjReader& copyMe) {
	if (copyMe.needsCleanup) {
		// duplicate mesh
		mesh = new PolygonMesh(*(copyMe.mesh));
		needsCleanup = true;
	} else {
		// copy by reference
		mesh = copyMe.mesh;
		needsCleanup = false;
	}
	zeroIndexed = copyMe.zeroIndexed;
}

SimpleObjReader::~SimpleObjReader() {
	cleanup();
}

void SimpleObjReader::cleanup() {
	if (needsCleanup) {
		if (mesh != NULL) {
			delete mesh;
			mesh = NULL;
		}
		needsCleanup = false;
	}
}

void SimpleObjReader::setMesh(PolygonMesh* mesh) {
	cleanup();
	this->mesh = mesh;
	needsCleanup = false;
}

void SimpleObjReader::setZeroIndexed() {
	zeroIndexed = true;
}

void SimpleObjReader::setOneIndexed() {
	zeroIndexed = false;
}

PVertex3dList* SimpleObjReader::getVertices() const {
	if (mesh != NULL) {
		return &(mesh->verts);
	} else {
		return NULL; // empty
	}
}

PPolygonList SimpleObjReader::getPolygons() const {
	if (mesh != NULL) {
		mesh->faces;
	} else {
		return PPolygonList(); // empty
	}
}

PolygonMesh SimpleObjReader::getPolygonMesh() const {
	if (mesh != NULL) {
		return *mesh;
	} else {
		return PolygonMesh();	// empty
	}
}

void SimpleObjReader::read(const char* filename) {
	std::ifstream fin;
	fin.open(filename, std::ifstream::in);
	if (fin.is_open()) {
		read(fin);
	}
	fin.close();
}

void SimpleObjReader::read(std::string filename) {
	read(filename.c_str());
}

PVertex3d SimpleObjReader::parseVertex(const std::string &line) {
	double x,y,z;
	//int res = sscanf(line.c_str(), "v %f %f %f", &(vtx.x), &(vtx.y), &(vtx.z));
	int res = sscanf(line.c_str(), "v %lf %lf %lf", &x, &y, &z);
	if (res == 3) {
		return MeshFactory::createVertex(x,y,z);
	}
	return NULL;
}

IndexList SimpleObjReader::parseFace(const std::string &line) {
	IndexList faceIdxs;

	// need to be careful with normal information
	int n = line.length();
	int idx = 2;
	while (idx < n) {

		char c = line[idx];

		// advance to first number
		while ( (c < '0' || c > '9') && idx < n) {
			idx++;
			c = line[idx];
		}
		if (idx >= n) {
			continue;
		}

		// collect digits
		int len = 0;
		while ( c >= '0' && c <= '9' && idx+len <= n) {
			len++;
			c = line[idx+len];
		}

		// if we found a number, add it
		if (len > 0) {
			size_t next = atoi(line.substr(idx,len).c_str());
			if (!zeroIndexed) {
				next--;
			}
			faceIdxs.push_back(next);
		}
		idx += len;
		if (idx >= n) {
			continue;
		}

		// advance to space or end
		while ( c != ' ' && idx < n) {
			idx++;
			c = line[idx];
		}

		idx++;
	}

	return faceIdxs;
}

void SimpleObjReader::read(std::istream &fin) {

	size_t nVerts = 0;
	PVertex3dList vtxs;
	IndexListList facesIdxs;
	size_t maxFIdx = 0;

	// loop through every line
	std::string line;
	while (getline(fin, line)) {
		line = str::trim(line);

		switch(line[0]) {
		case 'v':
			if (line[1] == ' ') {
				PVertex3d vtx = parseVertex(line);
				vtx->idx = nVerts;
				nVerts++;
				vtxs.push_back(vtx);
			}
			break;
		case 'f':
			if (line[1] == ' ') {
				IndexList fIdxs = parseFace(line);
				facesIdxs.push_back(fIdxs);
				for (size_t i=0; i<fIdxs.size(); i++) {
					if (fIdxs[i] > maxFIdx) {
						maxFIdx = fIdxs[i];
					}
				}
			}
			break;
			// ignore
		}
	}

	// correct for possible error in indexing
	if (maxFIdx == nVerts) {
		// it was actually 1-based, so subtract 1 from all indices
		for (size_t i=0; i<facesIdxs.size(); i++) {
			IndexList &fIdxs = facesIdxs[i];
			for (size_t j=0; j<fIdxs.size(); j++) {
				fIdxs[j]--;
			}
		}
	}

	// build polygonal mesh
	if (mesh == NULL) {
		mesh = new PolygonMesh();
		needsCleanup = true;
	}
	mesh->set(vtxs, facesIdxs);

}

void SimpleObjReader::clear() {
	cleanup();
}

// OBJ Writer
SimpleObjWriter::SimpleObjWriter()
: mesh(NULL), header(""), zeroIndexed(MESH_IO_ZERO_INDEXED),
  doubleFormat(MESH_IO_DEFAULT_DOUBLE_FORMAT), needsCleanup(false) {}

SimpleObjWriter::SimpleObjWriter(PolygonMesh* mesh)
: mesh(mesh), header(""), zeroIndexed(MESH_IO_ZERO_INDEXED),
  doubleFormat(MESH_IO_DEFAULT_DOUBLE_FORMAT), needsCleanup(false) {}

SimpleObjWriter::~SimpleObjWriter() {
	cleanup();
}

void SimpleObjWriter::cleanup() {
	if (needsCleanup) {
		if (mesh != NULL) {
			delete mesh;
			mesh = NULL;
		}
		needsCleanup = false;
	}
}

void SimpleObjWriter::clear() {
	cleanup();
}

void SimpleObjWriter::write(const char* filename) {
	std::ofstream fout;
	fout.open(filename, std::ifstream::out);
	if (fout.is_open()) {
		write(fout);
	}
	fout.close();
}

void SimpleObjWriter::write(std::string filename) {
	write(filename.c_str());
}

void SimpleObjWriter::write(std::ostream &fout) {

	fout << header;
	char line[MESH_IO_MAX_LINE_WIDTH];

	std::stringstream vformatss;
	vformatss << "v " << doubleFormat << " " << doubleFormat << " "
			<< doubleFormat << "\n";
	std::string vformat = vformatss.str();


	if (mesh != NULL) {

		// vertices
		PVertex3dList verts = mesh->verts;
		for (size_t i=0; i<mesh->numVertices(); i++) {
			verts[i]->scratchIdx = i;	// cue index for writing
			if (!zeroIndexed) {
				verts[i]->scratchIdx++;
			}
			snprintf(line, MESH_IO_MAX_LINE_WIDTH, vformat.c_str(),
					verts[i]->x, verts[i]->y, verts[i]->z);
			fout << line;
		}

		// faces
		PPolygonList &faces = mesh->faces;
		for (PPolygonList::iterator pit=faces.begin();
				pit < faces.end(); pit++) {

			fout << "f";
			PPolygon &face = *pit;
			for (PVertex3dList::iterator pvit=face->verts.begin();
					pvit < face->verts.end(); pvit++) {
				fout << " " << (*pvit)->scratchIdx;
			}
			fout << std::endl;
		}

	}
}

void SimpleObjWriter::setPolygons(PPolygonList &polys) {
	cleanup();
	mesh = new PolygonMesh(polys);
}

void SimpleObjWriter::setPolygonMesh(PolygonMesh *mesh) {
	cleanup();
	this->mesh = mesh;
}

void SimpleObjWriter::setZeroIndexed() {
	zeroIndexed = true;
}

void SimpleObjWriter::setOneIndexed() {
	zeroIndexed = false;
}

void SimpleObjWriter::setHeaderString(const char* header) {
	this->header = std::string(header);
}

void SimpleObjWriter::setHeaderString(const std::string header) {
	this->header = header;
}

void SimpleObjWriter::setDoubleFormat(const char* fmt) {
	doubleFormat = std::string(fmt);
}

void SimpleObjWriter::setDoubleFormat(const std::string fmt) {
	doubleFormat = fmt;
}

}
}
}
