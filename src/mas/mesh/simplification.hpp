#include "mas/core/exception.h"
#include "mas/core/queue.h"
#include "mas/core/math.h"

namespace mas {
namespace mesh {

#define MAX_SIZE_T (size_t)(-1)

template<typename CostFunc, typename CollapseCallback>
void edge_collapse(PolygonMesh& mesh, double fraction, CostFunc cost,
        CollapseCallback collapsed) {
    size_t nf = fraction * mesh.numFaces();
    edge_collapse(mesh, nf, cost, collapsed);
}

void VERIFY_CONNECTIVITY(PolygonMesh& mesh) {

    for (SharedPolygon& face : mesh.faces) {
        HalfEdge* he0 = face->he0.get();
        HalfEdge* he = he0;

        do {
            if (he->opposite == nullptr) {
                throw mas::exception("mesh is open");
            } else if (he->getIndex() != he->opposite->getIndex()) {
                throw mas::exception("half-edges incorrectly numbered");
            }
            he = he->next.get();
        } while (he != he0);
    }
}

void VERIFY_VERTEX_REMOVED(PolygonMesh& mesh, SharedVertex3d& rvtx) {

    // check indexing
    size_t idx = 0;
    for (SharedVertex3d& vtx : mesh.verts) {
        if (vtx == rvtx) {
            throw mas::exception("Vertex still belongs to mesh");
        }
        if (vtx->getIndex() != idx) {
            throw mas::exception("Vertex incorrectly indexed");
        }
        idx++;
    }

    // check that no face still contains the vertex
    for (SharedPolygon& face : mesh.faces) {
        for (SharedVertex3d& vtx : face->verts) {
            if (vtx == rvtx) {
                throw mas::exception("Vertex still belongs to a face");
            }
        }
    }

}

template<typename CostFunc, typename CollapseCallback>
void edge_collapse(PolygonMesh& mesh, size_t targetFaces, CostFunc cost,
        CollapseCallback collapsed) {

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
            he->setIndex(MAX_SIZE_T, false);
            he = he->next.get();
        } while (he != face->he0.get());
    }

    // number edges and count
    for (SharedPolygon& face : mesh.faces) {
        // reset indices on edges
        HalfEdge* he = face->he0.get();
        do {
            if (he->getIndex() == MAX_SIZE_T) {
                he->setIndex(idx, true);
                // opposite to same index
                if (he->opposite != nullptr) {
                    he->opposite->setIndex(idx, false);
                }
                idx++;
            }
            he = he->next.get();
        } while (he != face->he0.get());
    }
    size_t nedges = idx;

    struct edge_info {
        HalfEdge *edge;
        size_t pos;
        Point3d v;
        double cost;
    public:
        edge_info() : edge(nullptr), pos(-1), v(0,0,0), cost(0) {};
    };

    // gather edges
    std::vector<edge_info> edgeinfo(nedges);
    std::vector<size_t> edgepos(nedges);  // temp vector for initializing queue

    for (SharedPolygon& face : mesh.faces) {
        // reset indices on edges
        HalfEdge* he = face->he0.get();
        do {
            size_t hidx = he->getIndex();
            if (edgeinfo[hidx].edge == nullptr) {
                edgeinfo[hidx].edge = he;
                edgeinfo[hidx].pos = hidx;
                edgeinfo[hidx].cost = cost(mesh, *he, edgeinfo[hidx].v);
                edgepos[hidx] = hidx;
            }
            he = he->next.get();
        } while (he != face->he0.get());
    }

    auto costcmp =
            [&edgeinfo](size_t i, size_t j) {return edgeinfo[i].cost > edgeinfo[j].cost;};
    auto movecb =
            [&edgeinfo](size_t v, size_t opos, size_t npos) {edgeinfo[v].pos = npos;};
    mas::queue::priority_queue<size_t, std::vector<size_t>, decltype(costcmp), decltype(movecb)> queue(costcmp, std::move(edgepos), movecb);

    auto remove_edge = [&edgeinfo, &queue](HalfEdge* he) {

        size_t idx = he->getIndex();
        size_t lastidx = edgeinfo.size() - 1;

        he->head->removeIncidentEdge(he);

        // remove edge from queue
        queue.pop(edgeinfo[idx].pos);

        if (idx != lastidx) {
            // swap in last edge and renumber
            edgeinfo[idx] = std::move(edgeinfo[lastidx]);
            edgeinfo[idx].edge->setIndex(idx, true);
            if (edgeinfo[idx].edge->opposite != nullptr) {
                edgeinfo[idx].edge->opposite->setIndex(idx, false);
            }
            queue.get(edgeinfo[idx].pos) = idx; //update edge number in queue
        }

        edgeinfo.pop_back();

    };


    // go through and collapse edges
    int nfaces = mesh.numFaces();
    while (nfaces > targetFaces && !queue.empty()) {

        size_t eidx = queue.top();
        queue.pop();

        // check if no more valid collapses
        if (edgeinfo[eidx].cost == mas::math::DOUBLE_INFINITY) {
            break;
        }

        HalfEdge* edge = edgeinfo[eidx].edge;
        HalfEdge* oedge = edge->opposite;
        Polygon *face1 = edge->face;
        Polygon *face2 = nullptr;
        if (oedge != nullptr) {
            face2 = oedge->face;
        }

        // check if valid
        bool valid = true;

        // check no faces flipped
        // XXX

        // topology: one face is triangular but the other is not
        if (valid && face2 != nullptr
                && (face1->isTriangular() != face2->isTriangular())) {
            valid = false;
        }

        // topology: any attached face to one vertex
        // does not contain the other, apart from the two faces that contain the edge
        if (valid) {
            for (HalfEdge* he : edge->head->getIncidentEdges()) {
                Polygon* poly = he->face;
                // if the face isn't one of the two
                if (poly != face1 && poly != face2) {

                    // make sure no other face contains the tail
                    for (SharedVertex3d& vtx : poly->verts) {
                        if (vtx == edge->tail) {
                            valid = false;
                            break;
                        }
                    }
                }

                if (!valid) {
                    break;
                }
            }
        }

        // check topology: opposite nodes identical
        if (valid && face1->isTriangular()) {
            // attached faces are both triangular
            if (face2 != nullptr) {
                if (edge->next->head == oedge->next->head) {
                    valid = false;
                } else if (oedge->next->opposite == nullptr
                        && oedge->next->next->opposite == nullptr) {
                    // destroying opposite face leads to a disconnect
                    valid = false;
                }
            }

            // make sure doesn't lead to another collapsed edge
            if (valid && edge->next->opposite == nullptr
                    && edge->next->next->opposite == nullptr) {
                valid = false;
            }
        }

        if (valid) {
            // go ahead and remove edge
            // keep track of info
            edge_info remove_this = std::move(edgeinfo[eidx]);
            // remove edge from list/vertices
            remove_edge(edge);
            // remove opposite incidence if need-be
            if (oedge != nullptr) {
                edge->tail->removeIncidentEdge(oedge);
            }

            // move vertex to new position
            edge->head->set(remove_this.v);

            // keep edge's head, replace tail
            for (HalfEdge* he : edge->tail->getIncidentEdges()) {

                // only modify face if it's not one of the two faces to remove
                if (he->face != face1 && he->face != face2) {
                    // replace half-edge vertices
                    // and incident edges
                    he->head->removeIncidentEdge(he);
                    he->head = edge->head;
                    he->head->addIncidentEdge(he);

                    he->next->tail = edge->head;
                    // replace face vertex
                    for (int i = 0; i < he->face->verts.size(); i++) {
                        if (he->face->verts[i] == edge->tail) {
                            he->face->verts[i] = edge->head;
                            break;
                        }
                    }
                }
            }

            // remove destroyed triangular faces
            if (face1->isTriangular()) {
                // connect up
                SharedHalfEdge& prev = edge->next->next;
                SharedHalfEdge& next = edge->next;

                if (prev->opposite != nullptr) {
                    if (next->opposite != nullptr) {
                        prev->opposite->opposite = next->opposite;
                        next->opposite->opposite = prev->opposite;

                        // re-index merged edge, prev's edge gets merged into next
                        size_t oldidx = prev->getIndex();
                        prev->opposite->setIndex(next->getIndex(), true);
                        prev->opposite->opposite->setIndex(next->getIndex(), false); // ensure only one primary
                        // edge at the new index might be referring to the half-edge belonging to the removed face
                        edgeinfo[next->getIndex()].edge = prev->opposite;


                        // remove edge prev
                        remove_edge(prev.get());

                    } else {
                        prev->opposite->opposite = nullptr;

                        // next is completely removed
                        remove_edge(next.get());

                    }
                } else {

                    if (next->opposite != nullptr) {
                        next->opposite->opposite = nullptr;
                    } else {
                        // next is completely removed
                        remove_edge(next.get());
                    }

                    // prev is completely removed
                    remove_edge(prev.get());
                }

                // detach
                prev->opposite = nullptr;
                next->opposite = nullptr;

                // remove face
                // face should be completely disconnected now
                mesh.removeFace(face1->getIndex(), true);
                nfaces--;

            } else {

                // shift edges along
                const SharedHalfEdge& next = edge->next;
                HalfEdge* prev = next.get();
                while (prev->next.get() != edge) {
                    prev = prev->next.get();
                }
                prev->next = next; // skip edge
                prev->head->addIncidentEdge(prev);
                prev->head->removeIncidentEdge(edge);

                edge->next = nullptr;

            }

            if (face2 != nullptr && face2->isTriangular()) {

                // connect up
                SharedHalfEdge& prev = oedge->next->next;
                SharedHalfEdge& next = oedge->next;

                if (prev->opposite != nullptr) {
                    if (next->opposite != nullptr) {
                        prev->opposite->opposite = next->opposite;
                        next->opposite->opposite = prev->opposite;

                        // re-index merged edge, prev's edge gets merged into next
                        size_t oldidx = prev->getIndex();
                        prev->opposite->setIndex(next->getIndex(), true);
                        prev->opposite->opposite->setIndex(next->getIndex(), false); // ensure only one primary
                        // edge at the new index might be referring to the half-edge belonging to the removed face
                        edgeinfo[next->getIndex()].edge = prev->opposite;

                        // remove edge prev
                        remove_edge(prev.get());

                    } else {
                        prev->opposite->opposite = nullptr;

                        // next is completely removed
                        remove_edge(next.get());
                    }
                } else {

                    if (next->opposite != nullptr) {
                        next->opposite->opposite = nullptr;
                    } else {
                        // next is completely removed
                        remove_edge(next.get());
                    }

                    // prev is completely removed
                    remove_edge(prev.get());
                }
                prev->opposite = nullptr;
                next->opposite = nullptr;

                // remove face
                // face 2 should be disconnected now
                mesh.removeFace(face2->getIndex(), true);
                nfaces--;

            } else {
                // shift edges along
                const SharedHalfEdge& next = oedge->next;
                HalfEdge* prev = next.get();
                while (prev->next.get() != edge) {
                    prev = prev->next.get();
                }
                prev->next = next; // skip edge
                prev->head->addIncidentEdge(prev);
                prev->head->removeIncidentEdge(edge);
                edge->next = nullptr;

            }

            VERIFY_CONNECTIVITY(mesh);

            // remove tail vertex
            size_t oldvidx = edge->tail->getIndex();
            mesh.removeVertex(oldvidx, true);
            VERIFY_VERTEX_REMOVED(mesh, edge->tail);

            // notify of collapsed edge
            edge->tail->setIndex(oldvidx);  // restore original vertex index in case others were keeping track
            edge->setIndex(eidx, true);
            if (oedge != nullptr) {
                oedge->setIndex(eidx, false);
            }
            collapsed(*edge); // XXX really should be enough to reconstruct (progressive)

            // update normals
            for (HalfEdge* he : edge->head->getIncidentEdges()) {
                Polygon& poly = *(he->face);
                poly.updatePlane();
            }

            // update costs
            for (HalfEdge* he : edge->head->getIncidentEdges()) {
                size_t idx = he->getIndex();
                edgeinfo[idx].cost = cost(mesh, *he, edgeinfo[idx].v);
                queue.update(edgeinfo[idx].pos);  // update position in queue
            }

        } else {
            edgeinfo[eidx].cost = mas::math::DOUBLE_INFINITY;
            queue.push(eidx); // re-insert into queue
        }
    }

}

} // mesh
} // mas
