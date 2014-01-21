#ifndef MAS_MESH_BV_HPP
#define MAS_MESH_BV_HPP

namespace mas {
namespace mesh {
using namespace mas::bvtree;

template<typename BV>
PBVTree get_bv_tree(const PolygonMesh &mesh, double margin) {

	PBoundableList elems;
	const PPolygonList &faces = mesh.faces;

	for (PPolygonList::const_iterator pit = faces.begin();
			pit < faces.end(); pit++) {
		elems.push_back(
				BoundableFactory::createBoundablePolygon(*pit));
	}
	PBVTree tree =
			BVTreeFactory::createTree<BV>(elems, margin);
	return tree;
}

template<typename BV>
std::shared_ptr<BVTreeT<BV> >  get_bv_tree_T(const PolygonMesh &mesh,
		double margin) {

	PBoundableList elems;
	const PPolygonList &faces = mesh.faces;

	for (PPolygonList::const_iterator pit = faces.begin();
			pit < faces.end(); pit++) {
		elems.push_back(
				BoundableFactory::createBoundablePolygon(*pit));
	}
	std::shared_ptr<BVTreeT<BV> >  tree =
			BVTreeFactory::createTreeT<BV>(elems, margin);
	return tree;
}

}
}

#endif
