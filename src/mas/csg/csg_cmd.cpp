#include "mas/core/base.h"
#include "mas/core/math.h"
#include "mas/mesh/io.h"
//#include "mas/csg/bsp.h"
#include <iostream>
#include <stdlib.h>

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#define USAGE \
		"Usage:  csg <command> [options] <mesh1> [mesh2] ...\n\n" \
		"Commands: \n" \
		"\t(i)ntersect      compute mesh intersection, mesh1 ∩ mesh2 ∩ mesh3 ∩ ...\n" \
		"\t(u)nion          compute mesh union, mesh1 U mesh2 U mesh3 U ...\n" \
		"\t(s)ubtraction    compute subtraction, mesh1 - mesh2 - mesh3 -...\n" \
		"\t(m)erge          merges multiple meshes into one, mesh1 + mesh2 + ...\n" \
		"\t(t)riangulate    triangulate the mesh(es).  For multiple inputs, performs" \
		"\t                 a merge." \
		"\t(v)olume         compute volume\n" \
		"\t(a)nalyze        analyze mesh, displaying info\n\n" \
		"Options:\n" \
		"\t-o <output_file> specify output file (default stdout)\n" \
		"\t-t <tol>         specify tolerance (default " TOSTRING(CSG_DEFAULT_EPSILON) ")\n" \
		"\t-d <format>      specify double format (default '" MESH_IO_DEFAULT_DOUBLE_FORMAT "'')\n"

#define CMD_INTERSECTION 0
#define CMD_UNION        1
#define CMD_SUBTRACTION  2
#define CMD_VOLUME       3
#define CMD_ANALYZE      4
#define CMD_MERGE		 5
#define CMD_TRIANGULATE  6

namespace mas {
namespace csg {
namespace cmd {

using namespace mas::mesh;

void printUsage() {
	std::cout << USAGE;
}

void doCSG(int cmd, std::vector<std::string> files, std::ostream &out,
		double tol = 1e-15,
		std::string doubleFormat = MESH_IO_DEFAULT_DOUBLE_FORMAT){

	/*
	io::SimpleObjReader reader;
	using csg::bsp::CSGNode;

	reader.read(files[0]);
	PolygonMesh mesh1 = reader.getPolygonMesh();
	PPolygonList polys = mesh1.faces;
	PolygonMesh mesh1b;
	mesh1b.set(polys);

	CSGNode csg1 = CSGNode(mesh1);
	csg1.setTolerance(tol);
	PolygonMesh mesh1a;
	mesh1a.set(csg1.getAllPolygons());

	for (int i=1; i<files.size(); i++) {

		reader.clear();
		reader.read(files[i]);
		PolygonMesh mesh2 = reader.getPolygonMesh();
		CSGNode csg2 = CSGNode(mesh2);
		csg2.setTolerance(tol);

		switch (cmd) {
		case CMD_INTERSECTION:
			csg1 = bsp::vol_intersection(csg1, csg2);
			break;
		case CMD_UNION:
			csg1 = bsp::vol_union(csg1, csg2);
			break;
		case CMD_SUBTRACTION:
			csg1 = bsp::vol_subtraction(csg1, csg2);
			break;
		}
	}

	PolygonMesh outMesh = PolygonMesh(csg1.getAllPolygons());

	csg::io::SimpleObjWriter writer;
	writer.setPolygonMesh(&outMesh);
	writer.setDoubleFormat(doubleFormat);
	writer.write(out);
	*/

}

void doAnalyze(int cmd, std::vector<std::string> files, std::ostream &out,
		std::string doubleFormat = MESH_IO_DEFAULT_DOUBLE_FORMAT) {

	mas::mesh::io::SimpleObjReader reader;
	const int MAX_NUMBER_WIDTH = 64;
	char line[MAX_NUMBER_WIDTH];	// for formatted number

	for (int i=0; i<files.size(); i++) {
		reader.clear();
		reader.read(files[i]);
		PolygonMesh mesh = reader.getPolygonMesh();

		if (files.size() > 1) {
			out << files[i] << ": ";
		}

		switch (cmd) {
		case CMD_VOLUME: {
			double vol = mesh.volume();
			snprintf(line, MAX_NUMBER_WIDTH,
					doubleFormat.c_str(), vol);
			out << line << std::endl;
			break;
		}
		case CMD_ANALYZE: {
			mas::Vector3d m1;
			mas::Vector3d m2;
			mas::Vector3d p;
			double vol = mesh.volumeIntegrals(&m1, &m2, &p);

			out << std::endl;
			out << "    # Vertices: " << mesh.numVertices()
									<< std::endl;
			out << "    # Faces:    " << mesh.faces.size()
									<< std::endl;
			snprintf(line, MAX_NUMBER_WIDTH,
					doubleFormat.c_str(), vol);
			out << "    Volume:     " << line << std::endl;
			out << "    Moment m1:  ";
			snprintf(line, MAX_NUMBER_WIDTH,
					doubleFormat.c_str(), m1.x);
			out << line << " ";
			snprintf(line, MAX_NUMBER_WIDTH,
					doubleFormat.c_str(), m1.y);
			out << line << " ";
			snprintf(line, MAX_NUMBER_WIDTH,
					doubleFormat.c_str(), m1.z);
			out << line << std::endl;
			out << "    Moment m2:  ";
			snprintf(line, MAX_NUMBER_WIDTH,
					doubleFormat.c_str(), m2.x);
			out << line << " ";
			snprintf(line, MAX_NUMBER_WIDTH,
					doubleFormat.c_str(), m2.y);
			out << line << " ";
			snprintf(line, MAX_NUMBER_WIDTH,
					doubleFormat.c_str(), m2.z);
			out << line << std::endl;
			out << "    Moment p:   ";
			snprintf(line, MAX_NUMBER_WIDTH,
					doubleFormat.c_str(), p.x);
			out << line << " ";
			snprintf(line, MAX_NUMBER_WIDTH,
					doubleFormat.c_str(), p.y);
			out << line << " ";
			snprintf(line, MAX_NUMBER_WIDTH,
					doubleFormat.c_str(), p.z);
			out << line << std::endl;
		}
		out << std::endl;
		}
	}
}

void doMerge(int cmd, std::vector<std::string> files, std::ostream &out,
		std::string doubleFormat = MESH_IO_DEFAULT_DOUBLE_FORMAT) {

	mas::mesh::io::SimpleObjReader reader;
	PolygonMesh outMesh;

	for (int i=0; i<files.size(); i++) {
		reader.clear();
		reader.read(files[i]);
		PolygonMesh mesh2 = reader.getPolygonMesh();
		PPolygonList polys = mesh2.faces;
		polys.insert(polys.end(), outMesh.faces.begin(), outMesh.faces.end());
		outMesh.set(polys);
	}

	switch(cmd) {
	case CMD_TRIANGULATE:
		outMesh.triangulate();
	}
	mas::mesh::io::SimpleObjWriter writer;
	writer.setPolygonMesh(&outMesh);
	writer.setDoubleFormat(doubleFormat);
	writer.write(out);

}

}
}
}

int main( int argc, const char* argv[] ) {

	bool success = true;

	if (argc < 2) {
		mas::csg::cmd::printUsage();
		return 1;
	}

	double tol = 1e-15;
	std::string dfmt = MESH_IO_DEFAULT_DOUBLE_FORMAT;
	std::vector<std::string> files;
	std::ostream *out = &(std::cout);

	std::ofstream fout;
	std::string outFilename;
	bool fileOutput = false;

	int cmd = CMD_ANALYZE;

	// determine command
	switch(argv[1][0]) {
	case 'i':
		cmd = CMD_INTERSECTION;
		break;
	case 'u':
		cmd = CMD_UNION;
		break;
	case 's':
		cmd = CMD_SUBTRACTION;
		break;
	case 'v':
		cmd = CMD_VOLUME;
		break;
	case 'a':
		cmd = CMD_ANALYZE;
		break;
	case 'm':
		cmd = CMD_MERGE;
		break;
	case 't':
		cmd = CMD_TRIANGULATE;
		break;
	default:
		std::cout << "Invalid command specifier: " << argv[1];
		std::cout << std::endl << std::endl;
		mas::csg::cmd::printUsage();
		return 1;
	}

	// parse rest of arguments
	for (int i=2; i<argc; i++) {
		if (argv[i][0] == '-') {
			switch (argv[i][1]) {
			case 'o':
				outFilename = std::string(argv[++i]);
				fileOutput = true;
				break;
			case 't':
				tol = atof(argv[++i]);
				break;
			case 'd':
				dfmt = std::string(argv[++i]);
				break;
			default:
				std::cout << "Invalid option: " << argv[i];
				std::cout << std::endl << std::endl;
				mas::csg::cmd::printUsage();
				return 2;
			}
		} else {
			files.push_back(std::string(argv[i]));
		}
	}

	if (files.size() == 0) {
		std::cout << "You must specify at least one mesh file";
		std::cout << std::endl << std::endl;
		mas::csg::cmd::printUsage();
		return 3;
	}

	// we should now be safe to perform function

	// open file if necessary
	if (fileOutput) {
		fout.open(outFilename.c_str(), std::ofstream::out);
		out = &fout;
	}

	switch(cmd) {
	case CMD_INTERSECTION:
	case CMD_UNION:
	case CMD_SUBTRACTION:
		mas::csg::cmd::doCSG(cmd, files, *out, tol, dfmt);
		break;
	case CMD_VOLUME:
	case CMD_ANALYZE:
		mas::csg::cmd::doAnalyze(cmd, files, *out, dfmt);
		break;
	case CMD_MERGE:
	case CMD_TRIANGULATE:
		mas::csg::cmd::doMerge(cmd, files, *out, dfmt);
	}

	if (fileOutput) {
		fout.close();
	}

	if (success) {
		return 0;
	}
}
