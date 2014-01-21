#ifndef MAS_BVTREE_HPP
#define MAS_BVTREE_HPP

// template implementation
namespace mas {
namespace bvtree {

template<typename BV>
BVNodeT<BV>::BVNodeT(double margin) {
	bv = BVFactory::createBV<BV>();
	bv->setMargin(margin);
}

template<typename BV>
BVNodeT<BV>::BVNodeT(const PBoundableList &elems, double margin) {
	bv = BVFactory::createBV<BV>();
	bv->setMargin(margin);
	bv->bound(elems);
}

template<typename BV>
void BVNodeT<BV>::setChildren(const PBVTNodeList &children) {
	clearChildren();
	for (typename PBVTNodeList::iterator pit = children.begin();
			pit < children.end(); pit++) {
		this->children.push_back(*pit);
	}
}

template<typename BV>
PBVNode BVNodeT<BV>::spawnChild(const PBoundableList &elems) {
	PBVNode node = BVTreeFactory::createNode<BV>(elems, getMargin());
	node->attach(node);
	node->_parent = _this;
	return node;
}

// template tree
template <typename BV>
BVTreeT<BV>::BVTreeT(double margin)
: BVTree() {
	_root = BVTreeFactory::createNode<BV>(margin);
}

template <typename BV>
BVTreeT<BV>::BVTreeT(const PBoundableList &elems, double margin)
: BVTree() {
	build(elems, margin);
}

template <typename BV>
BVTreeT<BV>::BVTreeT(BVTNode root)
: BVTree(root) {}

template <typename BV>
std::shared_ptr<BVNodeT<BV> > BVTreeT<BV>::getRootT() {
	return std::static_pointer_cast<BVTreeT<BV> >(_root);
}

template <typename BV>
void BVTreeT<BV>::build(const PBoundableList &elems, double margin) {
	if (_root == NULL) {
		_root = BVTreeFactory::createNode<BV>(elems, margin);
	} else {
		_root->clear();
		_root->setMargin(margin);
		_root->setElements(elems);
	}
	_root->growRecursively();
}


template <typename BV>
std::shared_ptr<BV> BVFactory::createBV() {
	return std::make_shared<BV>();
}


template <typename BV>
PBVNode BVTreeFactory::createNode(double margin) {
	PBoundingVolume bv = BVFactory::createBV<BV>();
	PBVNode node = std::make_shared<BVNode>(bv, margin);
	node->attach(node);
	return node;
}

template <typename BV>
PBVNode BVTreeFactory::createNode(const PBoundableList &elems,
		double margin) {
	PBoundingVolume bv = BVFactory::createBV<BV>();
	PBVNode node = std::make_shared<BVNode>(bv, elems, margin);
	node->attach(node);
	return node;
}

template <typename BV>
PBVTree BVTreeFactory::createTree(double margin) {
	PBoundingVolume bv = BVFactory::createBV<BV>();
	PBVTree tree = std::make_shared<BVTree>(bv, margin);
	return tree;
}

template <typename BV>
PBVTree BVTreeFactory::createTree(const PBoundableList &elems,
		double margin) {
	PBoundingVolume bv = BVFactory::createBV<BV>();
	PBVTree tree = std::make_shared<BVTree>(bv, elems, margin);
	return tree;
}

// Fixed template versions
template <typename BV>
std::shared_ptr<BVNodeT<BV> > BVTreeFactory::createNodeT(
		double margin) {
	std::shared_ptr<BVNodeT<BV> > node =
			std::make_shared<BVNodeT<BV> >();
	node->setMargin(margin);
	node->attach(node);
	return node;
}

template <typename BV>
std::shared_ptr<BVNodeT<BV> > BVTreeFactory::createNodeT(
		const PBoundableList &elems, double margin) {
	std::shared_ptr<BVNodeT<BV> > node =
			std::make_shared<BVNodeT<BV> >(elems, margin);

	node->attach(node);
	return node;
}

template <typename BV>
std::shared_ptr<BVTreeT<BV> > BVTreeFactory::createTreeT(
		double margin) {
	std::shared_ptr<BVTreeT<BV> > tree =
			std::make_shared<BVTreeT<BV> >(margin);
	return tree;
}

template <typename BV>
std::shared_ptr<BVTreeT<BV> > BVTreeFactory::createTreeT(
		const PBoundableList &elems, double margin) {

	std::shared_ptr<BVTreeT<BV> > tree =
			std::make_shared<BVTreeT<BV> >(elems, margin);
	return tree;
}

}
}

#endif
