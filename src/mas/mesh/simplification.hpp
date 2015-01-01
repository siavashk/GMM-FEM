#include "mas/core/exception.h"
#include "mas/core/math.h"
#include <iostream>

namespace mas {
namespace mesh {

#define MAX_SIZE_T (size_t)(-1)

template<typename CostFunc, typename CollapseCallback>
void edge_collapse(PolygonMesh& mesh, double fraction, CostFunc& cost,
        CollapseCallback& collapsed) {
    size_t nf = fraction * mesh.numFaces();
    edge_collapse(mesh, nf, cost, collapsed);
}

template<typename CostFunc, typename CollapseCallback>
void edge_collapse(PolygonMesh& mesh, size_t targetFaces, CostFunc& cost,
        CollapseCallback& collapsed) {

    EdgeCollapser<CostFunc, CollapseCallback> collapser(mesh, cost, collapsed);
    collapser.collapseTo(targetFaces);

}

template<typename CostFunc, typename CollapseCallback>
void EdgeCollapser<CostFunc, CollapseCallback>::removeEdge(HalfEdge* he,
        size_t qpos) {

    size_t idx = he->getIndex();
    size_t lastidx = edgeinfo.size() - 1;

    // disconnect
    he->head->removeIncidentEdge(he);

    // remove edge from queue
    queue.pop(qpos);

    if (idx != lastidx) {
        // swap in last edge and renumber
        edgeinfo[idx] = std::move(edgeinfo[lastidx]);
        edgeinfo[idx]->edge->setIndex(idx, true);
        if (edgeinfo[idx]->edge->opposite != nullptr) {
            edgeinfo[idx]->edge->opposite->setIndex(idx, false);
        }

        queue.get(edgeinfo[idx]->pos) = idx; //update edge number in queue
    }

    edgeinfo.pop_back();

}

template<typename CostFunc, typename CollapseCallback>
EdgeCollapser<CostFunc, CollapseCallback>::EdgeCollapser(PolygonMesh& mesh,
        CostFunc& cost, CollapseCallback& collapsed) :
        mesh(mesh), cost(cost), collapsed(collapsed) {

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

    // gather edges
    edgeinfo = std::vector<std::unique_ptr<EdgeInfo> >();
    edgeinfo.reserve(nedges);
    for (int i = 0; i < nedges; i++) {
        edgeinfo.push_back(std::unique_ptr<EdgeInfo>(new EdgeInfo()));
    }

    // temp vector for initializing queue
    std::vector<size_t> edgepos(nedges);

    for (SharedPolygon& face : mesh.faces) {
        // reset indices on edges
        SharedHalfEdge he0 = face->he0;
        SharedHalfEdge he = he0;
        do {
            size_t hidx = he->getIndex();

            if (edgeinfo[hidx]->edge == nullptr) {
                edgeinfo[hidx]->edge = he;
                edgeinfo[hidx]->pos = hidx;
                edgeinfo[hidx]->cost = this->cost(mesh, *he, edgeinfo[hidx]->v);
                edgepos[hidx] = hidx;
            }
            he = he->next;
        } while (he != he0);
    }

    auto costcmp = [&](size_t i, size_t j) {
        return edgeinfo[i]->cost > edgeinfo[j]->cost;
    };
    auto movecb = [&](size_t v, size_t opos, size_t npos) {
        if (edgeinfo[v] != nullptr) {
            edgeinfo[v]->pos = npos;
        }
    };

    //    queue = mas::queue::priority_queue<size_t, std::vector<size_t>,
    //            bool (*)(size_t, size_t), void (*)(size_t, size_t, size_t)>(costcmp,
    //            std::move(edgepos), movecb);

    queue = mas::queue::priority_queue<size_t, std::vector<size_t>,
                std::function<bool(size_t,size_t)>,
                        std::function<void(size_t,size_t,size_t)> >(costcmp, std::move(edgepos), movecb);

}

template<typename CostFunc, typename CollapseCallback>
bool EdgeCollapser<CostFunc, CollapseCallback>::collapseKeepsTopology(
        const SharedHalfEdge& edge) {

    Polygon *face1 = edge->face;
    Polygon *face2 = nullptr;
    HalfEdge* oedge = edge->opposite;

    if (oedge != nullptr) {
        face2 = oedge->face;
    }

    // topology: one face is triangular but the other is not
    if (face2 != nullptr && (face1->isTriangular() != face2->isTriangular())) {
        return false;
    }

    // topology: any attached face to one vertex
    // does not contain the other, apart from the two faces that contain the edge
    for (HalfEdge* heh : edge->head->getIncidentEdges()) {
        if (heh != edge.get()) {
            for (HalfEdge* het : edge->tail->getIncidentEdges()) {
                if (het != oedge) {

                    if (face1->isTriangular()) {
                        // no tails in common that aren't supposed to merge
                        if (heh->tail == het->tail) {
                            if (!(heh->face == face2 && het->opposite != nullptr
                                    && het->opposite->face == face2)
                                    && !(het->face == face1
                                            && heh->opposite != nullptr
                                            && heh->opposite->face == face1)) {
                                return false;
                            }
                        }
                    } else {
                        // no tails in common at all
                        if (heh->tail == het->tail) {
                            return false;
                        }
                    }

                }
            }
        }

    }

    // check topology: opposite nodes identical
    if (face1->isTriangular()) {
        // attached faces are both triangular
        if (face2 != nullptr) {
            if (edge->next->head == oedge->next->head) {
                return false;
            } else if (oedge->next->opposite == nullptr
                    && oedge->next->next->opposite == nullptr) {
                // destroying opposite face leads to a disconnect
                return false;
            }
        }

        // make sure doesn't lead to another collapsed edge
        if (edge->next->opposite == nullptr
                && edge->next->next->opposite == nullptr) {
            return false;
        }
    }

    //        // check no faces flipped... maybe should be moved to cost function
    //        bool flipped = false;
    //        Point3d oldHeadPos = *(edge->head);
    //        Point3d oldTailPos = *(edge->tail);
    //
    //        if (valid) {
    //            // move vertex to new position
    //            Point3d& v = edgeinfo[eidx]->v;
    //            edge->head->set(v);
    //            edge->tail->set(v);
    //
    //            // XXX
    //            // update faces
    //            for (HalfEdge* he : edge->head->getIncidentEdges()) {
    //                Polygon& poly = *(he->face);
    //                if (he->face != face1 && he->face != face2) {
    //                    Vector3d n = poly.plane.normal;
    //                    poly.updatePlane();
    //                    if (poly.plane.normal.dot(n) < 0.5) {
    //                        // flipped
    //                        // flipped = true;
    //                        // valid = false;
    //                        // break;
    //                    }
    //                }
    //            }
    //            for (HalfEdge* he : edge->tail->getIncidentEdges()) {
    //                Polygon& poly = *(he->face);
    //                if (he->face != face1 && he->face != face2) {
    //                    Vector3d n = poly.plane.normal;
    //                    poly.updatePlane();
    //                    if (poly.plane.normal.dot(n) < 0.5) {
    //                        // flipped
    //                        // flipped = true;
    //                        // valid = false;
    //                        // break;
    //                    }
    //                }
    //            }
    //        }
    return true;
}

template<typename CostFunc, typename CollapseCallback>
void EdgeCollapser<CostFunc, CollapseCallback>::collapse(
        const SharedHalfEdge& edge) {

    // keep track of info
    size_t eidx = edge->idx;
    std::unique_ptr<EdgeInfo> remove_this = std::move(edgeinfo[eidx]);
    const SharedHalfEdge& oedge = edge->opposite->findSharedPointer();

    Polygon *face1 = edge->face;
    Polygon *face2 = nullptr;

    if (oedge != nullptr) {
        face2 = oedge->face;
    }

    // remove edge from list/vertices
    // and disconnect opposite
    removeEdge(edge.get(), remove_this->pos);
    if (oedge != nullptr) {
        oedge->head->removeIncidentEdge(oedge.get());
    }

    // keep edge's head, replace tail
    std::vector<HalfEdge*> hedges = edge->tail->getIncidentEdges();
    for (HalfEdge* he : hedges) {
        size_t idx = he->idx;
        //                std::cout << "idx: " << idx << std::endl;

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
                    // XXX more than one? break;
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
                edgeinfo[next->getIndex()]->edge =
                        prev->opposite->findSharedPointer();

                // remove edge prev
                removeEdge(prev.get(), edgeinfo[prev->idx]->pos);

            } else {
                prev->opposite->opposite = nullptr;

                // next is completely removed
                removeEdge(next.get(), edgeinfo[next->idx]->pos);

            }
        } else {

            if (next->opposite != nullptr) {
                next->opposite->opposite = nullptr;
            } else {
                // next is completely removed
                removeEdge(next.get(), edgeinfo[next->idx]->pos);
            }

            // prev is completely removed
            removeEdge(prev.get(), edgeinfo[prev->idx]->pos);
        }

        // detach
        prev->opposite = nullptr;
        next->opposite = nullptr;

        // remove face
        // face should be completely disconnected now
        mesh.removeFace(face1->getIndex(), true);

    } else {

        // shift edges along
        const SharedHalfEdge& next = edge->next;
        HalfEdge* prev = next.get();
        while (prev->next != edge) {
            prev = prev->next.get();
        }
        prev->next = next; // skip edge
        prev->head->addIncidentEdge(prev);
        prev->head->removeIncidentEdge(edge.get());

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
                edgeinfo[next->getIndex()]->edge =
                        prev->opposite->findSharedPointer();

                // remove edge prev
                removeEdge(prev.get(), edgeinfo[prev->idx]->pos);

            } else {
                prev->opposite->opposite = nullptr;

                // next is completely removed
                removeEdge(next.get(), edgeinfo[next->idx]->pos);
            }
        } else {

            if (next->opposite != nullptr) {
                next->opposite->opposite = nullptr;
            } else {
                // next is completely removed
                removeEdge(next.get(), edgeinfo[next->idx]->pos);
            }

            // prev is completely removed
            removeEdge(prev.get(), edgeinfo[prev->idx]->pos);
        }
        prev->opposite = nullptr;
        next->opposite = nullptr;

        // remove face
        // face 2 should be disconnected now
        mesh.removeFace(face2->getIndex(), true);

    } else {
        // shift edges along
        const SharedHalfEdge& next = oedge->next;
        HalfEdge* prev = next.get();
        while (prev->next != edge) {
            prev = prev->next.get();
        }
        prev->next = next; // skip edge
        prev->head->addIncidentEdge(prev);
        prev->head->removeIncidentEdge(edge.get());
        edge->next = nullptr;

    }

    // VERIFY_CONNECTIVITY(mesh);

    // remove tail vertex
    size_t tvidx = edge->tail->getIndex();
    size_t hvidx = edge->head->getIndex();
    mesh.removeVertex(tvidx, true);
    VERIFY_VERTEX_REMOVED(mesh, edge->tail);
    size_t ntvidx = edge->tail->getIndex();
    size_t nhvidx = edge->head->getIndex();

    // notify of collapsed edge
    edge->tail->setIndex(tvidx); // restore original vertex indices in case others were keeping track
    edge->head->setIndex(hvidx);
    edge->setIndex(eidx, true);
    if (oedge != nullptr) {
        oedge->setIndex(eidx, false);
    }
    edge->opposite = oedge.get();

    collapsed(*edge); // XXX really should be enough to reconstruct (i.e. progressive)

    // re-set vertex indices
    edge->tail->setIndex(ntvidx);
    edge->head->setIndex(nhvidx);

    edge->head->set(remove_this->v);
    // update faces
    for (HalfEdge* he : edge->head->getIncidentEdges()) {
        Polygon& poly = *(he->face);
        if (he->face != face1 && he->face != face2) {
            poly.updatePlane();
        }
    }

    // update costs
    for (HalfEdge* he : edge->head->getIncidentEdges()) {
        size_t idx = he->getIndex();
        edgeinfo[idx]->cost = cost(mesh, *he, edgeinfo[idx]->v);
        queue.update(edgeinfo[idx]->pos);  // update position in queue
    }

}

template<typename CostFunc, typename CollapseCallback>
void EdgeCollapser<CostFunc, CollapseCallback>::collapseTo(size_t targetFaces) {

    // go through and collapse edges
    int nfaces = mesh.numFaces();
    while (mesh.numFaces() > targetFaces && !queue.empty()) {

        size_t eidx = queue.top();

        // check if no more valid collapses
        if (edgeinfo[eidx]->cost == mas::math::DOUBLE_INFINITY) {
            break;
        }

        SharedHalfEdge edge = edgeinfo[eidx]->edge;
        bool valid = collapseKeepsTopology(edge);
        if (valid) {
            collapse(edge);
        } else {
            edgeinfo[eidx]->cost = mas::math::DOUBLE_INFINITY;
            queue.push(eidx); // re-insert into queue
        }
    }

}

} // mesh
} // mas
