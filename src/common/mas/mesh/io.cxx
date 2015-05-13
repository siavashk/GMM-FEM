#define  _CRT_SECURE_NO_WARNINGS // prevent visual studio warning
#include "mas/mesh/io.h"
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <unordered_map>

namespace mas {
namespace mesh {
namespace io {

namespace str {

// some useful string operations

// trim from start
std::string ltrim(const std::string& s, const std::string& chars = "\n\r ") {

	for (size_t i = 0; i < s.length(); i++) {
		size_t idx = chars.find(s[i]);
		if (idx == std::string::npos) {
			return s.substr(i);
		}
	}
	return "";

}

// trim from end
std::string rtrim(const std::string& s, const std::string& chars = "\n\r ") {

	for (int i = (int)s.length() - 1; i > 1; --i) {
		size_t idx = chars.find(s[i]);
		if (idx == std::string::npos) {
			return s.substr(0, i + 1);
		}
	}
	return "";
}

// trim from both ends
std::string trim(const std::string& s, const std::string& chars = "\n\r ") {
	return ltrim(rtrim(s, chars), chars);
}
} // end str

SimpleObjReader::SimpleObjReader() :
		zeroIndexed(MESH_IO_ZERO_INDEXED) {
}

void SimpleObjReader::setZeroIndexed() {
	zeroIndexed = true;
}

void SimpleObjReader::setOneIndexed() {
	zeroIndexed = false;
}

PolygonMesh* SimpleObjReader::read(const char* filename) {
	std::ifstream fin;
	fin.open(filename, std::ifstream::in);
	PolygonMesh* mesh = nullptr;
	if (fin.is_open()) {
		mesh = read(fin);
	}
	fin.close();
	return mesh;
}

PolygonMesh* SimpleObjReader::read(std::string filename) {
	return read(filename.c_str());
}

Vertex3d* SimpleObjReader::parseVertex(const std::string& line, size_t idx) {
	double x, y, z;
	//int res = sscanf(line.c_str(), "v %f %f %f",& (vtx.x),& (vtx.y),& (vtx.z));
	int res = sscanf(line.c_str(), "v %lf %lf %lf", &x, &y, &z);
	if (res == 3) {
		return new Vertex3d(x, y, z, idx);
	}
	return nullptr;
}

std::vector<size_t> SimpleObjReader::parseFace(const std::string& line) {
	std::vector<size_t> faceIdxs;

	// need to be careful with normal information
	int n = (int)(line.length());
	int idx = 2;
	while (idx < n) {

		char c = line[idx];

		// advance to first number
		while ((c < '0' || c > '9') && idx < n) {
			idx++;
			c = line[idx];
		}
		if (idx >= n) {
			continue;
		}

		// collect digits
		int len = 0;
		while (c >= '0' && c <= '9' && idx + len <= n) {
			len++;
			c = line[idx + len];
		}

		// if we found a number, add it
		if (len > 0) {
			size_t next = atoi(line.substr(idx, len).c_str());
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
		while (c != ' ' && idx < n) {
			idx++;
			c = line[idx];
		}

		idx++;
	}

	return faceIdxs;
}

PolygonMesh* SimpleObjReader::read(std::istream& fin) {

	size_t nVerts = 0;
	std::vector<SharedVertex3d> vtxs;
	std::vector<std::vector<size_t> > facesIdxs;
	size_t maxFIdx = 0;

	// loop through every line
	std::string line;
	while (getline(fin, line)) {
		line = str::trim(line);

		switch (line[0]) {
		case 'v':
			if (line[1] == ' ') {
				SharedVertex3d vtx(parseVertex(line, nVerts++));
				vtxs.push_back(std::move(vtx));
			}
			break;
		case 'f':
			if (line[1] == ' ') {
				std::vector<size_t> fIdxs = parseFace(line);
				facesIdxs.push_back(std::move(fIdxs));
				for (size_t i = 0; i < fIdxs.size(); i++) {
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
		for (size_t i = 0; i < facesIdxs.size(); i++) {
			std::vector<size_t>& fIdxs = facesIdxs[i];
			for (size_t j = 0; j < fIdxs.size(); j++) {
				fIdxs[j]--;
			}
		}
	}

	// build polygonal mesh
	return new PolygonMesh(std::move(vtxs), facesIdxs);
}

// OBJ Writer
SimpleObjWriter::SimpleObjWriter() :
		header(""), zeroIndexed(MESH_IO_ZERO_INDEXED), doubleFormat(
				MESH_IO_DEFAULT_DOUBLE_FORMAT) {
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

void SimpleObjWriter::write(const PolygonMesh& mesh, const char* filename) {
	std::ofstream fout;
	fout.open(filename, std::ifstream::out);
	if (fout.is_open()) {
		write(mesh, fout);
	}
	fout.close();
}

void SimpleObjWriter::write(const PolygonMesh& mesh, const std::string& filename) {
	write(mesh, filename.c_str());
}

void SimpleObjWriter::write(const PolygonMesh& mesh, std::ostream& fout) {

	fout << header;
	size_t linelength = MESH_IO_MAX_LINE_WIDTH;
	char *line = new char[linelength];

	std::stringstream vformatss;
	vformatss << "v " << doubleFormat << " " << doubleFormat << " "
			<< doubleFormat;
	std::string vformat = vformatss.str();

	// index map
	std::unordered_map<SharedVertex3d, size_t> idxmap;
	idxmap.reserve(mesh.numVertices());

	int idx = 0;
	if (!zeroIndexed) {
		idx = 1;
	}
	for (const SharedVertex3d& vtx : mesh.verts) {

		idxmap[vtx] = idx++;
		size_t len = snprintf(line, linelength, vformat.c_str(), vtx->x, vtx->y, vtx->z);
		if (len >= linelength) {
			linelength = len+1;
			delete[] line;
			line = new char[linelength];
			snprintf(line, linelength, vformat.c_str(), vtx->x, vtx->y, vtx->z);
		}
		fout << line << std::endl;
	}

	// faces
	for (const SharedPolygon& face : mesh.faces) {
		fout << "f";
		for (const SharedVertex3d& vtx : face->verts) {
			fout << " " << idxmap[vtx];
		}
		fout << std::endl;
	}
	fout.flush();

}

} // end io
} // end mesh
} // end mas
