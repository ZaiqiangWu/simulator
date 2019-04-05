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
	void simulate(Mesh* cloth);
	void cuda_update_vbo(glm::vec4* pos_vbo, glm::vec4* pos_cur, glm::vec3* normals, unsigned int* vertex_adjface, glm::vec3* face_normal, const unsigned int NUM_VERTICES);


private:
	void init_cloth(Mesh& cloth);
	void init_spring(Mesh& cloth);
	void init_bvh(Mesh& body);
	
	void cuda_get_face_normal(Mesh* sim_cloth, unsigned int* d_adjvertex_to_face);
	void cuda_verlet(Mesh* sim_cloth, glm::vec4* d_vbo_vertex, glm::vec3* d_vbo_normal);

	void get_vertex_adjface(Mesh& sim_cloth,vector<unsigned int>& vertex_adjface);
	void computeGridSize(unsigned int n, unsigned int blockSize, unsigned int &numBlocks, unsigned int &numThreads);
	void swap_buffer();
	void get_primitives(Mesh& body, vector<glm::vec3>& obj_vertices, vector<Primitive>& h_primitives);
	void save(string file_name);

private:
	cudaGraphicsResource* d_vbo_array_resource;  //map OPENGL array buffer to cuda
	cudaGraphicsResource* d_vbo_index_resource;  //map OPENGL index buffer to cuda

	// springs
	s_spring* d_adj_structure_spring;  // adjacent structure springs for each vertex 
	s_spring* d_adj_bend_spring;       // adjacent bend springs for each vertex 

	//for bvh tree
	BRTreeNode*  d_leaf_nodes;
	BRTreeNode*  d_internal_nodes;
	Primitive* d_primitives;

	// pre-malloc for vertices computation
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

public:
	vector<glm::vec3> obj_vertices;  // seems redundant
	vector<Primitive> h_primitives;     // host primitives for cuda_bvh construction
};

