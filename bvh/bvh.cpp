#include "bvh.h"
#include "primitive.h"
#include "../watch.h"
#include <iostream>
#include <algorithm>
#include <bitset>

using namespace std;
extern inline void copyFromCPUtoGPU(void** dst, void* src, int size);
extern inline void copyFromGPUtoCPU(void** dst, void* src, int size);

//__global__ void extract_bboxes_mortonCode(int num, BBox* boxes,unsigned int* morton_code)
//{
//	unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
//	if (index >= num)
//		return;
//
//}

// Expands a 10-bit integer into 30 bits
// by inserting 2 zeros after each bit.
unsigned int BVHAccel::expandBits(unsigned int v)
{
	v = (v * 0x00010001u) & 0xFF0000FFu;
	v = (v * 0x00000101u) & 0x0F00F00Fu;
	v = (v * 0x00000011u) & 0xC30C30C3u;
	v = (v * 0x00000005u) & 0x49249249u;
	return v;
}

// Calculates a 30-bit Morton code for the
// given 3D point located within the unit cube [0,1].
unsigned int BVHAccel::morton3D(float x, float y, float z)
{
	x = min(max(x * 1024.0f, 0.0f), 1023.0f);
	y = min(max(y * 1024.0f, 0.0f), 1023.0f);
	z = min(max(z * 1024.0f, 0.0f), 1023.0f);
	unsigned int xx = expandBits((unsigned int)x);
	unsigned int yy = expandBits((unsigned int)y);
	unsigned int zz = expandBits((unsigned int)z);
	return xx * 4 + yy * 2 + zz;
}

/**
* a wrapper to calculate morton code from
* the position of an object inside the
* unit cube.
*/
unsigned int BVHAccel::morton3D(glm::vec3 pos)
{
	return morton3D(pos.x, pos.y, pos.z);
}

/**
* comparer used to sort primitives acoording
* to their morton code.
*/
bool BVHAccel::mortonCompare(const Primitive& p1, const Primitive& p2)
{
	return p1.morton_code < p2.morton_code;
}

void BVHAccel::ParallelBVHFromBRTree(BRTreeNode* _d_leaf_nodes, BRTreeNode* _d_internal_nodes)
{
	d_leaf_nodes = _d_leaf_nodes;
	d_internal_nodes = _d_internal_nodes;
}

BVHAccel::BVHAccel(const std::vector<Primitive> &_primitives,
	size_t max_leaf_size)
{
	stop_watch watch;

	this->primitives = _primitives;

	// edge case
	if (primitives.empty()) {
		return;
	}

	// calculate root AABB size
	watch.start();
	BBox bb;
	for (size_t i = 0; i < primitives.size(); ++i) {
		bb.expand(primitives[i].get_bbox());
	}
	watch.stop();
	cout << "expand time elapsed: " << watch.elapsed() << "us" << endl;

	watch.restart();
	// calculate morton code for each primitives
	for (size_t i = 0; i < primitives.size(); ++i) {
		unsigned int morton_code = morton3D(bb.getUnitcubePosOf(primitives[i].get_bbox().centroid()));
		primitives[i].morton_code = morton_code;
	}
	watch.stop();
	cout << "morton code time elapsed: " << watch.elapsed() << "us" << endl;

	watch.restart();
	// sort primitives using morton code -> use thrust::sort(parrallel sort)?
	std::sort(primitives.begin(), primitives.end(), mortonCompare);
	watch.stop();
	cout << "sort time elapsed: " << watch.elapsed() << "us" << endl;

	watch.restart();
	//remove duplicates
	vector<Primitive> new_pri;
	for (int i = 1; i < primitives.size();i++)
	{
		new_pri.push_back(primitives[i-1]);
		while (primitives[i].morton_code == primitives[i - 1].morton_code)
		{
			i++;
		}

	}
	primitives = new_pri;
	watch.stop();
	cout << "remove duplicate time elapsed: " << watch.elapsed() << "us" << endl;
	cout << "triangle size: " << primitives.size() << endl;


	watch.restart();
	//whether to set h_vertices = NULL before send to gpu?
	copyFromCPUtoGPU((void**)&d_primitives, &primitives[0], sizeof(Primitive)*primitives.size());

	//unsigned int numThreads0, numBlocks0;
	//int n = primitives.size();
	//numThreads0 = min(512, n);
	//numBlocks0 = (n % numThreads0 != 0) ? (n / numThreads0 + 1) : (n / numThreads0);
	//extract_bboxes_mortonCode << < numBlocks, numThreads >> > (n,);
	// extract bboxes array
	std::vector<BBox> bboxes(primitives.size());
	for (int i = 0; i < primitives.size(); i++)
	{
		bboxes[i] = primitives[i].get_bbox();
	}


	// extract sorted morton code for parallel binary radix tree construction
	vector<unsigned int> sorted_morton_codes(primitives.size());
	for (size_t i = 0; i < primitives.size(); ++i) {
		sorted_morton_codes[i] = primitives[i].morton_code;
	}

	watch.stop();
	cout << "others time elapsed: " << watch.elapsed() << "us" << endl;

	// delegate the binary radix tree construction process to GPU
	cout << "start building parallel brtree" << endl;
	watch.restart();
	ParallelBRTreeBuilder builder(&sorted_morton_codes[0], &bboxes[0], primitives.size());
	builder.build();
	watch.stop();
	cout << "done with time elapsed: " << watch.elapsed() << "us" << endl;

	watch.restart();
	numInternalNode = builder.numInternalNode;
	numLeafNode = builder.numLeafNode;
	// construct BVH based on Binary Radix Tree --> need to be built in gpu?
	ParallelBVHFromBRTree(builder.get_d_leaf_nodes(), builder.get_d_internal_nodes());
	builder.set_d_leaf_nodes(NULL);
	builder.set_d_internal_nodes(NULL);

	// free the host memory because I am a good programmer
	builder.freeDeviceMemory();
	builder.freeHostMemory();
	watch.stop();
	cout << "free time elapsed: " << watch.elapsed() << "us" << endl;
	//exit(0);

}

BVHAccel::~BVHAccel() {  }

void BVHAccel::access(BRTreeNode* root, vector<BRTreeNode*>& bad_bode)
{
	if (root->bbox.min.x > root->bbox.max.x)
	{
		if (is_leaf(root))
		{
			bad_bode.push_back(root);
			return;
		}
		else
		{
			access(get_left_child(root), bad_bode);
			access(get_right_child(root), bad_bode);
		}
	}


}

void BVHAccel::pre_drawoutline()
{
	copyFromGPUtoCPU((void**)&h_internal_nodes, d_internal_nodes, sizeof(BRTreeNode)*numInternalNode);
	copyFromGPUtoCPU((void**)&h_leaf_nodes, d_leaf_nodes, sizeof(BRTreeNode)*numLeafNode);

}


void BVHAccel::draw(BRTreeNode* root)
{

	root->bbox.draw();
	if (is_leaf(root))
	{
		return;
	}
	else
	{
		draw(get_left_child(root));
		draw(get_right_child(root));
	}
}

bool BVHAccel::intersect(const glm::vec3 point, int& idx) const
{
	// Allocate traversal stack from thread-local memory,
	// and push NULL to indicate that there are no postponed nodes.
	BRTreeNode* stack[64];
	BRTreeNode** stackPtr = stack;
	*stackPtr++ = NULL; // push

						// Traverse nodes starting from the root.
	BRTreeNode* node = get_root();
	do
	{
		// Check each child node for overlap.
		BRTreeNode* childA = get_left_child(node);
		BRTreeNode* childB = get_right_child(node);
		bool overlapL = check_overlap(point, childA);
		bool overlapR = check_overlap(point, childB);

		// Query overlaps a leaf node => report collision with the first collision.
		if (overlapL && is_leaf(childA))
		{
			idx = childA->getIdx();
			//idx = -(idx + 1);   //is a leaf, and we can get it through primitive[idx]
			return true;
		}

		if (overlapR && is_leaf(childB))
		{
			idx = childB->getIdx();
			//idx = -(idx + 1);   //is a leaf
			return true;
		}

		// Query overlaps an internal node => traverse.
		bool traverseL = (overlapL && !is_leaf(childA));
		bool traverseR = (overlapR && !is_leaf(childB));

		if (!traverseL && !traverseR)
			node = *--stackPtr; // pop
		else
		{
			node = (traverseL) ? childA : childB;
			if (traverseL && traverseR)
				*stackPtr++ = childB; // push
		}
	} while (node != NULL);
	return false;
}


BRTreeNode* BVHAccel::get_root() const
{
	return &h_internal_nodes[0];
}


BRTreeNode* BVHAccel::get_left_child(BRTreeNode* node)const
{
	bool is_leaf = false;
	bool is_null = false;
	int  child_idx = false;
	child_idx = node->getChildA(is_leaf, is_null);
	if (!is_null)
	{
		if (is_leaf)
		{
			return &h_leaf_nodes[child_idx];
		}
		else
		{
			return &h_internal_nodes[child_idx];
		}
	}
	else
		return nullptr;
}

BRTreeNode* BVHAccel::get_right_child(BRTreeNode* node)const
{
	bool is_leaf = false;
	bool is_null = false;
	int  child_idx = false;
	child_idx = node->getChildB(is_leaf, is_null);
	if (!is_null)
	{
		if (is_leaf)
		{
			return &h_leaf_nodes[child_idx];
		}
		else
		{
			return &h_internal_nodes[child_idx];
		}
	}
	else
		return nullptr;
}

bool BVHAccel::is_leaf(BRTreeNode* node)const
{
	bool is_leaf = false;
	bool is_null_a = false;
	bool is_null_b = false;
	int  child_idx_a = false;
	int  child_idx_b = false;
	child_idx_a = node->getChildA(is_leaf, is_null_a);
	child_idx_b = node->getChildB(is_leaf, is_null_b);

	if (is_null_a && is_null_b)
		return true;
	return false;

}

bool BVHAccel::check_overlap(const glm::vec3 point, BRTreeNode* node)const
{
	return node->bbox.intersect(point);
}

