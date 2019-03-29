#include "bvh.h"
#include "primitive.h"
#include "../watch.h"
#include <iostream>
#include <algorithm>
#include <bitset>
#include <thrust/sort.h>
#include <thrust/execution_policy.h>

using namespace std;
extern inline void copyFromCPUtoGPU(void** dst, void* src, int size);
extern inline void copyFromGPUtoCPU(void** dst, void* src, int size);

// Expands a 10-bit integer into 30 bits
// by inserting 2 zeros after each bit.
__device__ unsigned int d_expandBits(unsigned int v)
{
	v = (v * 0x00010001u) & 0xFF0000FFu;
	v = (v * 0x00000101u) & 0x0F00F00Fu;
	v = (v * 0x00000011u) & 0xC30C30C3u;
	v = (v * 0x00000005u) & 0x49249249u;
	return v;
}

// Calculates a 30-bit Morton code for the
// given 3D point located within the unit cube [0,1].
__device__ unsigned int d_morton3D(glm::vec3 p)
{
	float x = p.x, float y = p.y, float z = p.z;
	x = min(max(x * 1024.0f, 0.0f), 1023.0f);
	y = min(max(y * 1024.0f, 0.0f), 1023.0f);
	z = min(max(z * 1024.0f, 0.0f), 1023.0f);
	unsigned int xx = d_expandBits((unsigned int)x);
	unsigned int yy = d_expandBits((unsigned int)y);
	unsigned int zz = d_expandBits((unsigned int)z);
	return xx * 4 + yy * 2 + zz;
}


bool d_mortonCompare(const Primitive& p1, const Primitive& p2)
{
	return p1.morton_code < p2.morton_code;
}

__global__ void get_bb(int num, int m, Primitive* d_primitives,BBox* d_bb)
{
	unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= num +1)
		return;
	int div = m / num;
	int res = m%num;
	if (index == num + 1)
	{
		for (int i = m - res; i < m; i++)
		{
			d_bb[index].expand(d_primitives[i].d_get_bbox());
		}
	}
	else
	{
		BBox tem;
		for (int i = 0; i < div; i++)  //use shared to replace
		{
			tem.expand(d_primitives[i*num + index].d_get_bbox());
		}
		d_bb[index].expand(tem);
	}
	
	__syncthreads();

	//if (index == 0)
	//{
	//	for (int i = 0; i < num; i++)
	//	{
	//		d_bb[0].expand(d_bb[i]);
	//	}
	//	for (int i = m - res; i < m; i++)
	//	{
	//		d_bb[0].expand(d_primitives[i].d_get_bbox());
	//	}
	//}
}

__global__ void get_morton(int num, Primitive* d_primitives, BBox bb)
{
	unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= num)
		return;

	unsigned int morton_code = d_morton3D(bb.getUnitcubePosOf(d_primitives[index].d_get_bbox().centroid()));
	d_primitives[index].morton_code = morton_code;
}

__global__ void get_bb_morton(int num, BBox* d_bbox, unsigned int* d_morton, Primitive* d_primitives)
{
	unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= num)
		return;

	d_bbox[index] = d_primitives[index].d_get_bbox();
	d_morton[index] = d_primitives[index].morton_code;
}

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
	const unsigned int num_threads = 128;
	vector<BBox> c_bb(num_threads+1);
	BBox* d_bb;
	Primitive* d_tem_primitives;
	copyFromCPUtoGPU((void**)&d_tem_primitives, &primitives[0], sizeof(Primitive)*primitives.size());
	copyFromCPUtoGPU((void**)&d_bb, &c_bb[0], sizeof(BBox)* c_bb.size());
	get_bb <<<1, c_bb.size() >>> (num_threads, primitives.size(),d_tem_primitives, d_bb);
	
	BBox* cc_bb, bb;;
	copyFromGPUtoCPU((void**)&cc_bb, d_bb, sizeof(BBox)*c_bb.size());
	for (int i = 0; i < c_bb.size(); i++)
	{
		bb.expand(cc_bb[i]);
	}
	
	watch.stop();

	watch.restart();
	// calculate morton code for each primitives
	/*for (size_t i = 0; i < primitives.size(); ++i) {
		unsigned int morton_code = morton3D(bb.getUnitcubePosOf(primitives[i].get_bbox().centroid()));
		primitives[i].morton_code = morton_code;
	}*/
	unsigned int numThreads, numBlocks;
	unsigned int blockSize = 512;
	unsigned int n = primitives.size();
	numThreads = min(blockSize, n);
	numBlocks = (n % numThreads != 0) ? (n / numThreads + 1) : (n / numThreads);
	get_morton << <numBlocks, numThreads >> > (n,d_tem_primitives,bb);
	
	watch.stop();
	//cout << "morton code time elapsed: " << watch.elapsed() << "us" << endl;

	watch.restart();
	// sort primitives using morton code -> use thrust::sort(parrallel sort)?
	//std::sort(primitives.begin(), primitives.end(), mortonCompare);
	cudaMemcpy(&primitives[0], d_tem_primitives, sizeof(Primitive)*primitives.size(), cudaMemcpyDeviceToHost);
	//thrust::sort(thrust::host, primitives.begin(),primitives.end(), mortonCompare);
	std::sort(primitives.begin(), primitives.end(), mortonCompare);    //cpu is faster than gpu, are u kidding me?
	watch.stop();

	cudaFree(d_tem_primitives);
	cudaFree(d_bb);
	


	watch.restart();
	//remove duplicates
	vector<Primitive> new_pri;
	for (int i = 1; i < primitives.size(); i++)
	{
		new_pri.push_back(primitives[i - 1]);
		while (primitives[i].morton_code == primitives[i - 1].morton_code)
		{
			i++;
		}

	}
	primitives = new_pri;
	watch.stop();

	watch.restart();
	
	//whether to set h_vertices = NULL before send to gpu?
	copyFromCPUtoGPU((void**)&d_primitives, &primitives[0], sizeof(Primitive)*primitives.size());

	//unsigned int numThreads0, numBlocks0;
	//int n = primitives.size();
	//numThreads0 = min(512, n);
	//numBlocks0 = (n % numThreads0 != 0) ? (n / numThreads0 + 1) : (n / numThreads0);
	//extract_bboxes_mortonCode << < numBlocks, numThreads >> > (n,);

	// extract bboxes array // extract sorted morton code for parallel binary radix tree construction
	std::vector<BBox> bboxes(primitives.size());
	vector<unsigned int> sorted_morton_codes(primitives.size());
	/*for (int i = 0; i < primitives.size(); i++)
	{
		bboxes[i] = primitives[i].get_bbox();
		sorted_morton_codes[i] = primitives[i].morton_code;
	}*/
	
	blockSize = 512;
	n = primitives.size();
	numThreads = min(blockSize, n);
	numBlocks = (n % numThreads != 0) ? (n / numThreads + 1) : (n / numThreads);
	BBox* d_tem_bbox;
	unsigned int* d_tem_morton;
	copyFromCPUtoGPU((void**)&d_tem_bbox, &bboxes[0], sizeof(BBox)*primitives.size());  //just cudaMalloc
	copyFromCPUtoGPU((void**)&d_tem_morton, &sorted_morton_codes[0], sizeof(unsigned int)*primitives.size());

	get_bb_morton << <numBlocks, numThreads >> > (n, d_tem_bbox, d_tem_morton, d_primitives);

	cudaMemcpy(&bboxes[0], d_tem_bbox, sizeof(BBox)*primitives.size(), cudaMemcpyDeviceToHost);
	cudaMemcpy(&sorted_morton_codes[0], d_tem_morton, sizeof(unsigned int)*primitives.size(), cudaMemcpyDeviceToHost);
	watch.stop();
	//cout << "others time elapsed: " << watch.elapsed() << "us" << endl;


	// delegate the binary radix tree construction process to GPU
	//cout << "start building parallel brtree" << endl;
	watch.restart();
	ParallelBRTreeBuilder builder(&sorted_morton_codes[0], &bboxes[0], primitives.size());
	builder.build();
	watch.stop();
	//cout << "done with time elapsed: " << watch.elapsed() << "us" << endl;

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
	//cout << "free time elapsed: " << watch.elapsed() << "us" << endl;
}

BVHAccel::~BVHAccel() {  }


#ifdef _DEBUG
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
bool BVHAccel::check_overlap(const glm::vec3 point, BRTreeNode* node)const
{
	return node->bbox.intersect(point);
}
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
#endif

// GPU version for bvh tree 
__device__ bool  intersect(BRTreeNode*  leaf_nodes, BRTreeNode*  internal_nodes, const glm::vec3 point, int& idx)
{
	// Allocate traversal stack from thread-local memory,
	// and push NULL to indicate that there are no postponed nodes.
	BRTreeNode* stack[64];
	BRTreeNode** stackPtr = stack;
	*stackPtr++ = NULL; // push

						// Traverse nodes starting from the root.
	BRTreeNode* node = get_root(leaf_nodes, internal_nodes);
	do
	{
		// Check each child node for overlap.
		BRTreeNode* childA = get_left_child(leaf_nodes, internal_nodes, node);
		BRTreeNode* childB = get_right_child(leaf_nodes, internal_nodes, node);
		bool overlapL = check_overlap(point, childA);
		bool overlapR = check_overlap(point, childB);

		// Query overlaps a leaf node => report collision with the first collision.
		if (overlapL && is_leaf(leaf_nodes, internal_nodes, childA))
		{
			idx = childA->getIdx();       //is a leaf, and we can get it through primitive[idx]
			return true;
		}

		if (overlapR && is_leaf(leaf_nodes, internal_nodes, childB))
		{
			idx = childB->getIdx();
			return true;
		}

		// Query overlaps an internal node => traverse.
		bool traverseL = (overlapL && !is_leaf(leaf_nodes, internal_nodes, childA));
		bool traverseR = (overlapR && !is_leaf(leaf_nodes, internal_nodes, childB));

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
__device__ BRTreeNode*  get_root(BRTreeNode* leaf_nodes, BRTreeNode* internal_nodes)
{
	return &internal_nodes[0];
}
__device__ BRTreeNode*  get_left_child(BRTreeNode*  leaf_nodes, BRTreeNode*  internal_nodes, BRTreeNode* node)
{
	bool is_leaf = false;
	bool is_null = false;
	int  child_idx = false;
	child_idx = node->getChildA(is_leaf, is_null);
	if (!is_null)
	{
		if (is_leaf)
		{
			return &leaf_nodes[child_idx];
		}
		else
		{
			return &internal_nodes[child_idx];
		}
	}
	else
		return nullptr;
}
__device__ BRTreeNode*  get_right_child(BRTreeNode*  leaf_nodes, BRTreeNode*  internal_nodes, BRTreeNode* node)
{
	bool is_leaf = false;
	bool is_null = false;
	int  child_idx = false;
	child_idx = node->getChildB(is_leaf, is_null);
	if (!is_null)
	{
		if (is_leaf)
		{
			return &leaf_nodes[child_idx];
		}
		else
		{
			return &internal_nodes[child_idx];
		}
	}
	else
		return nullptr;
}
__device__ bool  is_leaf(BRTreeNode*  leaf_nodes, BRTreeNode*  internal_nodes, BRTreeNode* node)
{
	bool is_leaf = false;
	bool is_null_a = false;
	bool is_null_b = false;
	node->getChildA(is_leaf, is_null_a);
	node->getChildB(is_leaf, is_null_b);

	if (is_null_a && is_null_b)
		return true;
	return false;
}
__device__ bool  check_overlap(const glm::vec3 point, BRTreeNode* node)
{
	return node->bbox.intersect(point);
}