#include "mas/core/exception.h"
#include "mas/core/queue.h"

namespace mas {
namespace mesh {

#define MAX_SIZE_T (size_t)(-1)

template<typename CostFunc>
void edge_collapse(PolygonMesh& mesh, double fraction, const CostFunc& cost) {
    size_t nf = fraction * mesh.numFaces();
    edge_collapse(mesh, nf, cost);
}

template<typename CostFunc>
void edge_collapse(PolygonMesh& mesh, size_t targetFaces,
        const CostFunc& cost) {

    // assumes mesh is connected

    // count edges/number them/track costs
    size_t idx = 0;
    for (SharedPolygon& face : mesh.faces) {
        // reset indices on edges
        if (face->he0 == nullptr) {
            // throw std::runtime_error
            throw mas::exception(
                    "edge_collapse: mesh must be connected [see PolygonMesh.connect()]");
        }
        HalfEdge* he = face->he0.get();
        do {
            he->setIndex(MAX_SIZE_T);
            he = he->next.get();
        } while (he != face->he0.get());
    }

    // number edges and count
    for (SharedPolygon& face : mesh.faces) {
        // reset indices on edges
        HalfEdge* he = face->he0.get();
        do {
            if (he->getIndex() == MAX_SIZE_T) {
                he->setIndex(idx);
                // opposite to same index
                if (he->opposite != nullptr) {
                    he->opposite->setIndex(idx);
                }
                idx++;
            }
            he = he->next.get();
        } while (he != face->he0.get());
    }
    size_t nedges = idx;

    // gather edges
    std::vector<HalfEdge*> edges(nedges, nullptr);
    for (SharedPolygon& face : mesh.faces) {
        // reset indices on edges
        HalfEdge* he = face->he0.get();
        do {
            size_t hidx = he->getIndex();
            if (edges[hidx] != nullptr) {
                edges[hidx] = he;
            }
            he = he->next.get();
        } while (he != face->he0.get());
    }

    struct edge_info {
        size_t pos;
        Point3d v;
        double cost;
    };

    std::vector<edge_info> edgeinfo(nedges);
    std::vector<size_t> edgepos(nedges);

    for (int i = 0; i < nedges; i++) {
        edgepos[i] = i;
        edgeinfo[i].pos = i;
        edgeinfo[i].cost = cost(mesh, *edges[i], edgeinfo[i].v);
    }

    auto costcmp =
            [&edgeinfo](size_t i, size_t j) {return edgeinfo[i].cost < edgeinfo[j].cost;};
    auto movecb =
            [&edgeinfo](size_t v, size_t opos, size_t npos) {edgeinfo[v].pos = npos;};
    mas::queue::priority_queue<size_t, std::vector<size_t>, decltype(costcmp),
            decltype(movecb)> queue(std::move(edgepos), costcmp, movecb);

}

} // mesh
} // mas
