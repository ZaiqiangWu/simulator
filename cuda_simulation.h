#pragma once
#include "load_obj.h"
#include "parameter.h"
#include "./bvh/bvh.h"
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>


class CUDA_Simulation
{
public:
	CUDA_Simulation();
	~CUDA_Simulation();
	CUDA_Simulation(Obj& cloth);
	void simulate();
	void add_bvh(BVHAccel& bvh);

private:
	void get_vertex_adjface();
	void init_cuda();

	void verlet_cuda();
	void computeGridSize(unsigned int n, unsigned int blockSize, unsigned int &numBlocks, unsigned int &numThreads);
	void swap_buffer();

public:
	int readID, writeID;
	glm::vec4* const_cuda_pos;      //���㵯��ԭ��
	glm::vec4* X[2];
	glm::vec4* X_last[2];
	glm::vec4 * X_in, *X_out;
	glm::vec4 * X_last_in, *X_last_out;

	unsigned int* cuda_vertex_index;    //�������
	unsigned int* cuda_vertex_adjface;       //����ÿ���㷨����ʱ��Ҫ����Χƽ�������
	glm::vec3* cuda_face_normal;        //��ķ�����

	cudaGraphicsResource* cuda_vbo_resource;
	glm::vec4* cuda_p_vertex;           //ָ��OPENGL buffer��vertex�ĵ�ַ
	glm::vec3* cuda_p_normal;           //ָ��OPENGL buffer��normal�ĵ�ַ

	unsigned int* cuda_neigh1;  //��ά����תΪһά����
	unsigned int* cuda_neigh2;

	BRTreeNode*  d_leaf_nodes;   //for bvh tree
	BRTreeNode*  d_internal_nodes;
	Primitive* d_primitives;

private:
	Obj* sim_cloth;
	vector<unsigned int> vertex_adjface;    //ÿ����������20���ڽ��棬��������UINT_MAX��Ϊ������־
	unsigned int NUM_ADJFACE; 
};

