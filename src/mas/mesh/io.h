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

// some useful string operations
namespace str {

// trim characters from start
static std::string ltrim(const std::string &s, const std::string &chars =
        "\n\r ");

// trim characters from end
static std::string rtrim(const std::string &s, const std::string &chars =
        "\n\r ");

// trim characters from both ends
static std::string trim(const std::string &s,
        const std::string &chars = "\n\r ");
}

class PolygonReader {
public:
    virtual void read(const char* filename) = 0 ;
    virtual void read(std::string filename) = 0 ;
    virtual void read(std::istream &fin) = 0;
    virtual PPolygonList getPolygons() const {
    }
    ;
    virtual PolygonMesh getPolygonMesh() const {
    }
    ;
};

class PolygonWriter {
public:
    virtual ~PolygonWriter() {
    }
    ;
    virtual void write(const char* filename) {
    }
    ;
    virtual void write(std::string filename) {
    }
    ;
    virtual void write(std::ostream &fout) {
    }
    ;
    virtual void setPolygons(PPolygonList &polys) {
    }
    ;
    virtual void setPolygonMesh(PolygonMesh* mesh) {
    }
    ;
};

// Only reads header, vertices, faces
class SimpleObjReader: public PolygonReader {
private:
    PolygonMesh* mesh;
    bool needsCleanup;
    bool zeroIndexed;

protected:
    virtual void cleanup();
    virtual PVertex3d parseVertex(const std::string &line);
    virtual IndexList parseFace(const std::string &line);

public:
    SimpleObjReader();
    SimpleObjReader(PolygonMesh* mesh);
    SimpleObjReader(const SimpleObjReader& copyMe);
    virtual ~SimpleObjReader();

    void setMesh(PolygonMesh* mesh);
    void setZeroIndexed();
    void setOneIndexed();

    virtual void read(const char* filename);
    virtual void read(std::string filename);
    virtual void read(std::istream &fin);
    virtual PVertex3dList* getVertices() const;
    virtual PPolygonList getPolygons() const;
    virtual PolygonMesh getPolygonMesh() const;
    virtual void clear();

};

// Only writes header, vertices, faces
class SimpleObjWriter: public PolygonWriter {
private:
    PolygonMesh* mesh;
    std::string header;
    bool zeroIndexed;
    bool needsCleanup;
    std::string doubleFormat;
private:
    virtual void cleanup();

public:
    SimpleObjWriter();
    SimpleObjWriter(PolygonMesh* mesh);
    virtual ~SimpleObjWriter();

    virtual void write(const char* filename);
    virtual void write(std::string filename);
    virtual void write(std::ostream &fout);
    virtual void setPolygons(PPolygonList &polys);
    virtual void setPolygonMesh(PolygonMesh* mesh);

    void setZeroIndexed();
    void setOneIndexed();

    void setHeaderString(const char* header);
    void setHeaderString(const std::string header);

    void setDoubleFormat(const char* fmt);
    void setDoubleFormat(const std::string fmt);

    void clear();

};

}
}
}

#endif
