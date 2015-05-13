#ifndef MAS_MESH_IO_H
#define MAS_MESH_IO_H

#include <string>
#include <fstream>
#include "mas/core/base.h"
#include "mas/mesh/mesh.h"

#ifndef MESH_IO_ZERO_INDEXED
#define MESH_IO_ZERO_INDEXED false
#endif

#ifndef MESH_IO_DEFAULT_DOUBLE_FORMAT
#define MESH_IO_DEFAULT_DOUBLE_FORMAT "%g"
#endif

#ifndef MESH_IO_MAX_LINE_WIDTH
#define MESH_IO_MAX_LINE_WIDTH 80
#endif

namespace mas {
namespace mesh {
namespace io {

class PolygonMeshReader {
public:
    virtual PolygonMesh* read(const char* filename) = 0 ;
    virtual PolygonMesh* read(std::string filename) = 0 ;
    virtual PolygonMesh* read(std::istream& fin) = 0;
};

class PolygonMeshWriter {
public:
    virtual void write(const PolygonMesh& mesh, const char* filename) = 0;
    virtual void write(const PolygonMesh& mesh, const std::string& filename) = 0;
    virtual void write(const PolygonMesh& mesh, std::ostream& fout)  = 0;
};

// Only reads header, vertices, faces
class SimpleObjReader: public PolygonMeshReader {
private:
	SimpleObjReader(const SimpleObjReader& copyMe);
    bool zeroIndexed;

protected:
    virtual Vertex3d* parseVertex(const std::string& line, size_t idx = (size_t)(-1));
    virtual std::vector<size_t> parseFace(const std::string& line);

public:
    SimpleObjReader();
    void setZeroIndexed();
    void setOneIndexed();

    virtual PolygonMesh* read(const char* filename);
    virtual PolygonMesh* read(std::string filename);
    virtual PolygonMesh* read(std::istream& fin);
};

// Only writes header, vertices, faces
class SimpleObjWriter: public PolygonMeshWriter {
private:
    std::string header;
    bool zeroIndexed;
    std::string doubleFormat;
private:
    SimpleObjWriter(const SimpleObjWriter& copyMe);
public:
    SimpleObjWriter();

    void setZeroIndexed();
    void setOneIndexed();

    void setHeaderString(const char* header);
    void setHeaderString(const std::string header);

    void setDoubleFormat(const char* fmt);
    void setDoubleFormat(const std::string fmt);

    virtual void write(const PolygonMesh& mesh, const char* filename);
    virtual void write(const PolygonMesh& mesh, const std::string& filename);
    virtual void write(const PolygonMesh& mesh, std::ostream& fout);

};

}
}
}

#endif
