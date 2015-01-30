#ifndef MAS_MESH_BV_HPP
#define MAS_MESH_BV_HPP

namespace mas {
namespace mesh {
using namespace mas::bvtree;

template<typename BV>
BVTreeT<BV>*  get_bv_tree_T(const PolygonMesh &mesh,
		double margin) {

	std::vector<SharedBoundable> elems;
	const std::vector<SharedPolygon>& faces = mesh.faces;

	for (const SharedPolygon& face : faces) {
		elems.push_back(
				std::make_shared<BoundablePolygon>(face));
	}
	return new BVTreeT<BV>(std::move(elems), margin);
}

template<typename BV>
BVTree* get_bv_tree(const PolygonMesh &mesh, double margin) {
	return get_bv_tree_T<BV>(mesh, margin);
}

} // end mas
} // end mesh

#endif
