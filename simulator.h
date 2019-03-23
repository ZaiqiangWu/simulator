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
	glm::vec4* x_original;      // keep it to compute spring original length
	glm::vec4* x_cur[2];
	glm::vec4* x_last[2];
	glm::vec4 * x_cur_in, *x_cur_out;
	glm::vec4 *x_last_in, *x_last_out;

	glm::vec3* d_collision_force;           // store the normal of the face if collided, or set 0.0 if no collision
	unsigned int* d_adj_face_to_vertex;    // the order like this: f0(v0,v1,v2) -> f1(v0,v1,v2) -> ... ->fn(v0,v1,v2)
	unsigned int* d_adj_vertex_to_face;       //To compute each point's normal, we need the adjacent face indices of each point  
	glm::vec3* d_face_normals;        // face(triangle) normal

	cudaGraphicsResource* cuda_vbo_resource;
	glm::vec4* cuda_p_vertex;           //指向OPENGL buffer中vertex的地址
	glm::vec3* cuda_p_normal;           //指向OPENGL buffer中normal的地址

	s_spring* d_adj_structure_spring;  // adjacent structure springs for each vertex 
	s_spring* d_adj_bend_spring;       // adjacent bend springs for each vertex 

	BRTreeNode*  d_leaf_nodes;   //for bvh tree
	BRTreeNode*  d_internal_nodes;
	Primitive* d_primitives;
	Springs* cuda_spring;

public:
	Mesh* sim_cloth;
	vector<glm::vec3> obj_vertices;
	vector<Primitive> h_primitives;     // host primitives for cuda_bvh construction
	BVHAccel* cuda_bvh;

	vector<unsigned int> vertex_adjface;    //每个点最大包含20个邻近面，不足者以UINT_MAX作为结束标志
	unsigned int NUM_PER_VERTEX_ADJ_FACES; 
};

