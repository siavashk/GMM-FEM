#ifndef MAS_BVTREE_HPP
#define MAS_BVTREE_HPP

// template implementation
namespace mas {
namespace bvtree {

template<typename BV>
BVNodeT<BV>::BVNodeT(double margin)
: BVNode(std::move(BVFactory::createBV<BV>())) {
	bv->setMargin(margin);
}

template<typename BV>
BVNodeT<BV>::BVNodeT(const std::vector<SharedBoundable>& elems, double margin)
: BVNode(std::move(BVFactory::createBV<BV>())) {
	bv->setMargin(margin);
	bv->bound(elems);
}

template<typename BV>
BVNodeT<BV>::BVNodeT(std::vector<SharedBoundable>&& elems, double margin)
: BVNode(std::move(BVFactory::createBV<BV>())) {
	bv->setMargin(margin);
	bv->bound(elems);
}

template<typename BV>
void BVNodeT<BV>::setChildren(const std::vector<SharedBVNode>& children) {
	clearChildren();
	this->children = children;
}

template<typename BV>
void BVNodeT<BV>::setChildren(std::vector<SharedBVNode>&& children) {
	clearChildren();
	this->children = std::move(children);
}

template<typename BV>
BVNodeT<BV>* BVNodeT<BV>::spawnChild(const std::vector<SharedBoundable>& elems) {
	std::unique_ptr<BVNodeT<BV> > node = BVTreeFactory::createNodeT<BV>(elems, getMargin());
	node->setParent(this);
	return node.release();
}

template<typename BV>
BVNodeT<BV>* BVNodeT<BV>::spawnChild(std::vector<SharedBoundable>&& elems) {
	std::unique_ptr<BVNodeT<BV> > node = BVTreeFactory::createNodeT<BV>(elems, getMargin());
	node->setParent(this);
	return node.release();
}

// template tree
template <typename BV>
BVTreeT<BV>::BVTreeT(double margin)
: BVTree(BVTreeFactory::createNode<BV>(margin)) {
}

template <typename BV>
BVTreeT<BV>::BVTreeT(const std::vector<SharedBoundable>& elems, double margin)
: BVTree() {
	build(elems, margin);
}

template <typename BV>
BVTreeT<BV>::BVTreeT(std::vector<SharedBoundable>&& elems, double margin)
: BVTree() {
	build(elems, margin);
}

template <typename BV>
BVTreeT<BV>::BVTreeT(SharedBVNode&& root)
: BVTree(root) {}

template <typename BV>
void BVTreeT<BV>::build(const std::vector<SharedBoundable>& elems, double margin) {
	if (!root) {
		root = std::shared_ptr<BVNode>(std::move(BVTreeFactory::createNode<BV>(elems, margin)));
	} else {
		root->clear();
		root->setMargin(margin);
		root->setElements(elems);
	}
	root->growRecursively();
}

template <typename BV>
void BVTreeT<BV>::build(std::vector<SharedBoundable>&& elems, double margin) {
	if (!root) {
		root = std::shared_ptr<BVNode>(std::move(BVTreeFactory::createNode<BV>(elems, margin)));
	} else {
		root->clear();
		root->setMargin(margin);
		root->setElements(elems);
	}
	root->growRecursively();
}


template <typename BV>
std::unique_ptr<BV> BVFactory::createBV() {
	return std::unique_ptr<BV>(new BV());
}


template <typename BV>
UniqueBVNode BVTreeFactory::createNode(double margin) {
	std::unique_ptr<BV> bv = BVFactory::createBV<BV>();
	return std::unique_ptr<BVNode>(new BVNode(std::move(bv), margin) );
}

template <typename BV>
UniqueBVNode BVTreeFactory::createNode(const std::vector<SharedBoundable>& elems, double margin) {
	std::unique_ptr<BV> bv = BVFactory::createBV<BV>();
	return std::unique_ptr<BVNode>( new BVNode(std::move(bv), elems, margin) );
}

template <typename BV>
UniqueBVNode BVTreeFactory::createNode(std::vector<SharedBoundable>&& elems, double margin) {
	std::unique_ptr<BV> bv = BVFactory::createBV<BV>();
	return std::unique_ptr<BVNode>(new BVNode(std::move(bv), elems, margin) );
}

// Fixed template versions
template <typename BV>
std::unique_ptr<BVNodeT<BV> > BVTreeFactory::createNodeT(double margin) {
	return std::unique_ptr<BVNodeT<BV> >( new BVNodeT<BV>(margin) );
}

template <typename BV>
std::unique_ptr<BVNodeT<BV> > BVTreeFactory::createNodeT(
		const std::vector<SharedBoundable>& elems, double margin) {
	return std::unique_ptr<BVNodeT<BV> > ( new BVNodeT<BV> >(elems, margin) );
}

template <typename BV>
std::unique_ptr<BVNodeT<BV> > BVTreeFactory::createNodeT(
		std::vector<SharedBoundable>&& elems, double margin) {
	return std::unique_ptr<BVNodeT<BV> > ( new BVNodeT<BV> >(elems, margin) );
}


template <typename BV>
UniqueBVTree BVTreeFactory::createTree(double margin) {
	std::unique_ptr<BV> bv (std::move(BVFactory::createBV<BV>()));
	return std::unique_ptr<BVTree>( new BVTree(std::move(bv), margin) );
}

template <typename BV>
UniqueBVTree BVTreeFactory::createTree(const std::vector<SharedBoundable>& elems,
		double margin) {
	std::unique_ptr<BV> bv( std::move(BVFactory::createBV<BV>()) );
	return std::unique_ptr<BVTree>( new BVTree(std::move(bv), elems, margin) );
}

template <typename BV>
UniqueBVTree BVTreeFactory::createTree(std::vector<SharedBoundable>&& elems,
		double margin) {
	std::unique_ptr<BV> bv ( std::move(BVFactory::createBV<BV>()) );
	return std::make_shared<BVTree>( new BVTree(std::move(bv), elems, margin) );
}



template <typename BV>
std::unique_ptr<BVTreeT<BV> > BVTreeFactory::createTreeT( double margin) {
	return std::unique_ptr<BVTreeT<BV> > ( new BVTreeT<BV>(margin) );
}

template <typename BV>
std::unique_ptr<BVTreeT<BV> > BVTreeFactory::createTreeT(
		const std::vector<SharedBoundable>& elems, double margin) {
	return std::unique_ptr<BVTreeT<BV> > ( new BVTreeT<BV>(elems, margin) );
}

template <typename BV>
std::unique_ptr<BVTreeT<BV> > BVTreeFactory::createTreeT(
		std::vector<SharedBoundable>&& elems, double margin) {
	return std::unique_ptr<BVTreeT<BV> > ( new BVTreeT<BV>(elems, margin) );
}

}
}

#endif
