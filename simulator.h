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
	void get_vertex_adjface(vector<unsigned int>& vertex_adjface);

	void cuda_get_face_normal(unsigned int* d_adjvertex_to_face);
	void cuda_verlet(glm::vec4* d_vbo_vertex, glm::vec3* d_vbo_normal);
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

	// pre-malloc for nomal computation
	glm::vec3* d_collision_force;           // store the normal of the face if collided, or set 0.0 if no collision
	unsigned int* d_adjface_to_vertex;       //To compute each point's normal, we need the adjacent face indices of each point  
	glm::vec3* d_face_normals;        // face(triangle) normal

	cudaGraphicsResource* d_vbo_array_resource;  //map OPENGL array buffer to cuda
	cudaGraphicsResource* d_vbo_index_resource;  //map OPENGL index buffer to cuda

	// springs
	s_spring* d_adj_structure_spring;  // adjacent structure springs for each vertex 
	s_spring* d_adj_bend_spring;       // adjacent bend springs for each vertex 


	//for bvh tree
	BRTreeNode*  d_leaf_nodes;   
	BRTreeNode*  d_internal_nodes;
	Primitive* d_primitives;

public:
	Mesh* sim_cloth;
	vector<glm::vec3> obj_vertices;
	vector<Primitive> h_primitives;     // host primitives for cuda_bvh construction
	BVHAccel* cuda_bvh;
	unsigned int NUM_PER_VERTEX_ADJ_FACES; 
};

