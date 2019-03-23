#pragma once
#include "Mesh.h"
#include "./bvh/bvh.h"
#include "spring.h"
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>


class Simulator
{
public:
	Simulator();
	~Simulator();
	Simulator(Mesh& cloth,Mesh& body);
	void simulate();
	void add_bvh(BVHAccel& bvh);

private:
	void init_cloth(Mesh& cloth);
	void get_vertex_adjface();
	void init_cuda();

	void verlet_cuda();
	void computeGridSize(unsigned int n, unsigned int blockSize, unsigned int &numBlocks, unsigned int &numThreads);
	void swap_buffer();
	void get_primitives(Mesh& body, vector<glm::vec3>& obj_vertices, vector<Primitive>& h_primitives);
	void save(string file_name);

public:
	int readID, writeID;
	glm::vec4* const_cuda_pos;      //���㵯��ԭ��
	glm::vec4* x_cur[2];
	glm::vec4* X_last[2];
	glm::vec4 * X_in, *X_out;
	glm::vec4 * X_last_in, *X_last_out;

	glm::vec3* collision_force;           //��ײ�����������εķ����������û����ײ��Ϊ0
	unsigned int* cuda_vertex_index;    //�������
	unsigned int* cuda_vertex_adjface;       //����ÿ���㷨����ʱ��Ҫ����Χƽ�������
	glm::vec3* cuda_face_normal;        //��ķ�����

	cudaGraphicsResource* cuda_vbo_resource;
	glm::vec4* cuda_p_vertex;           //ָ��OPENGL buffer��vertex�ĵ�ַ
	glm::vec3* cuda_p_normal;           //ָ��OPENGL buffer��normal�ĵ�ַ

	s_spring* cuda_neigh1;  //��ά����תΪһά����
	s_spring* cuda_neigh2;

	BRTreeNode*  d_leaf_nodes;   //for bvh tree
	BRTreeNode*  d_internal_nodes;
	Primitive* d_primitives;
	Springs* cuda_spring;

public:
	Mesh* sim_cloth;
	vector<glm::vec3> obj_vertices;
	vector<Primitive> h_primitives;     // host primitives for cuda_bvh construction
	BVHAccel* cuda_bvh;

	vector<unsigned int> vertex_adjface;    //ÿ����������20���ڽ��棬��������UINT_MAX��Ϊ������־
	unsigned int NUM_ADJFACE; 
};

