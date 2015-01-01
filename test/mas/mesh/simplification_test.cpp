#include "mas/mesh/io.h"
#include "mas/mesh/meshbv.h"

#include <iostream>
#include <stdlib.h>

// timings
#include "mas/core/time.h"
#include "mas/mesh/simplification.h"
#include "mas/core/exception.h"
#include <algorithm>

void printstuff(mas::mesh::PolygonMesh& mesh, int offset) {

    // vertices
    for (mas::mesh::SharedVertex3d& vtx : mesh.verts) {
        std::cout << "v(" << (vtx->idx + offset) << ",:) = ["
                << vtx->toString("%g, ", "%g") << "];" << std::endl;
    }

    // faces
    for (mas::mesh::SharedPolygon& face : mesh.faces) {
        std::cout << "f(" << (face->idx + offset) << ",:) = [";
        for (int i = 0; i < face->numVertices() - 1; i++) {
            std::cout << (face->verts[i]->idx + offset) << ", ";
        }
        std::cout << (face->verts.back()->idx + offset) << "];" << std::endl;
    }

    // edges
    for (mas::mesh::SharedPolygon& face : mesh.faces) {

        mas::mesh::HalfEdge* he0 = face->he0.get();
        mas::mesh::HalfEdge* he = he0;
        do {
            if (he->isPrimary()) {
                std::cout << "e(" << (he->idx + offset) << ",:) = ["
                        << (he->tail->idx + offset) << ", "
                        << (he->head->idx + offset) << "];" << std::endl;
            }
            he = he->next.get();
        } while (he != he0);
    }

}

bool doMallocDebug(const char filename[]) {
    mas::mesh::io::SimpleObjReader reader;
    mas::mesh::io::SimpleObjWriter writer;

    using mas::mesh::PolygonMesh;
    using mas::Point3d;
    using mas::mesh::SharedPolygon;
    using mas::mesh::SharedHalfEdge;
    using mas::mesh::HalfEdge;

    std::vector<SharedPolygon> facesCopy;
    // local scope
    {
        std::unique_ptr<PolygonMesh> mesh = std::unique_ptr<PolygonMesh>(
                reader.read(filename));

        if (mesh == nullptr) {
            std::cout << "Cannot find mesh " << filename << std::endl;
            return false;
        }

        mesh->connect(); // build connectivity

        facesCopy = mesh->faces;
    }

    // dealloc faces one at a time
    size_t len = facesCopy.size();
    std::reverse(facesCopy.begin(), facesCopy.end());
    for (int i = 0; i < len; i++) {
        size_t idx = facesCopy.back()->idx;
        facesCopy.pop_back();
        facesCopy.shrink_to_fit();
        std::cout << "deallocated face " << idx << std::endl;
    }

    return true;
}

bool doEdgeCollapseTest(const char filename[], const char out[], int reduce) {

    mas::mesh::io::SimpleObjReader reader;
    mas::mesh::io::SimpleObjWriter writer;

    using mas::mesh::PolygonMesh;
    using mas::Point3d;
    using mas::mesh::SharedPolygon;
    using mas::mesh::SharedHalfEdge;
    using mas::mesh::HalfEdge;

    std::unique_ptr<PolygonMesh> mesh = std::unique_ptr<PolygonMesh>(
            reader.read(filename));

    if (mesh == nullptr) {
        std::cout << "Cannot find mesh " << filename << std::endl;
        return false;
    }

    mesh->connect(); // build connectivity

    if (mesh->numFaces() < 20) {
        printstuff(*mesh, 0);
    }

    std::cout << "# faces: " << mesh->numFaces() << std::endl;
    std::cout << "# vertices: " << mesh->numVertices() << std::endl;

    // check connectivity
    for (SharedPolygon& face : mesh->faces) {
        HalfEdge* he0 = face->he0.get();
        HalfEdge* he = he0;
        do {
            if (he->opposite == nullptr) {
                std::cout << "OPPOSITE BROKEN!!!" << std::endl;
            }
            if (he->face != face.get()) {
                std::cout << "FACE BROKEN!!!" << std::endl;
            }
            he = he->next.get();
        } while (he != he0);
    }

    mas::mesh::EdgeCollapseQuadricCost cost;
    int len = mesh->numFaces()-reduce;
    if (len < 0) {
        len = 2;
    }
    mas::mesh::edge_collapse(*mesh, (size_t)len, cost, cost);

    writer.write(*mesh, out);

    std::cout << "# faces: " << mesh->numFaces() << std::endl;
    std::cout << "# vertices: " << mesh->numVertices() << std::endl;
    if (mesh->numFaces() < 20) {
        printstuff(*mesh, 0);
    }

    // remove faces?
    mesh->disconnect();

    return true;
}

int main(int argc, const char* argv[]) {

    int reduce = 1;
    if (argc > 3) {
        sscanf(argv[3], "%d", &reduce);
    }

    mas::time::Timer timer;
    timer.start();
    doEdgeCollapseTest(argv[1], argv[2], reduce);
    // doMallocDebug(argv[1]);
    timer.stop();
    double micros = timer.getMicroseconds();
    printf("Took %f us\n", micros);
    fflush(stdout);

    return 0;
}

