#ifndef BVTREE_MEX_H
#define BVTREE_MEX_H

#define MESH_TREE_SIGNATURE 0x85856A6A

#include "mas/mesh/meshbv.h"
using BVTreeType = mas::bvtree::BVTree<mas::mesh::SharedBoundablePolygon,mas::bvtree::OBB>;

#endif
