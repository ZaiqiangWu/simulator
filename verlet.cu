#include <cuda.h>
#include <device_functions.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <glm/glm.hpp>
#include "parameter.h"

//physics parameter,若要修改参数，还需同时修改parameter.cpp
__device__ float spring_structure = 30;
__device__ float spring_bend = 50;
__device__ float damp = -0.0125f;
__device__ float mass = 0.3;
__device__ float dt = 1.0f /50.0f;
__device__ int NUM_ADJFACE = 20;
__device__ unsigned int NUM_NEIGH1 = 20;
__device__ unsigned int NUM_NEIGH2 = 20;



__global__ void get_face_normal(glm::vec4* g_pos_in, unsigned int* cloth_index, const unsigned int cloth_index_size, glm::vec3* cloth_face)
{
	unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
	unsigned int max_thread = cloth_index_size / 3;
	if (index >= max_thread)
		return;

	unsigned int f_index[3];
	for (int i = 0; i<3; i++)
		f_index[i] = index * 3 + i;

	glm::vec4 vertex[3];
	for (int i = 0; i < 3; i++)
		vertex[i] = g_pos_in[cloth_index[f_index[i]]];  //find the fucking bug!

	glm::vec3 pos[3];
	for (int i = 0; i < 3; i++)
		pos[i] = glm::vec3(vertex[i].x, vertex[i].y, vertex[i].z);

	glm::vec3 side1, side2, normal;
	side1 = pos[1] - pos[0];
	side2 = pos[2] - pos[0];
	normal = glm::normalize(glm::cross(side1, side2));

	cloth_face[index] = normal;

}

__device__ glm::vec3 get_spring_force(int index, glm::vec4* g_pos_in, glm::vec4* g_pos_old_in, glm::vec4* const_pos,
									  unsigned int* neigh,unsigned int NUM_NEIGH,
									  glm::vec3 pos,glm::vec3 vel)
{
	glm::vec3 force(0.0);
	int first_neigh = index*NUM_NEIGH;   //访问一级邻域，UINT_MAX为截至标志
	int time = 0;
	for (int k = first_neigh; neigh[k]< UINT_MAX && time<NUM_NEIGH; k++, time++) //部分点邻域大于MAX_NEIGH(20)
	{
		float ks = spring_structure;
		float kd = 0;

		int index_neigh = neigh[k];
		volatile glm::vec4 pos_neighData = g_pos_in[index_neigh];
		volatile glm::vec4 pos_lastData = g_pos_old_in[index_neigh];
		glm::vec3 p2 = glm::vec3(pos_neighData.x, pos_neighData.y, pos_neighData.z);
		glm::vec3 p2_last = glm::vec3(pos_lastData.x, pos_lastData.y, pos_lastData.z);

		glm::vec3 v2 = (p2 - p2_last) / dt;
		glm::vec3 deltaP = pos - p2;
		if (glm::length(deltaP) == 0) { force += glm::vec3(0.0f); continue; }  //deltaP += glm::vec3(0.0001);	//avoid '0'

		glm::vec3 deltaV = vel - v2;
		float dist = glm::length(deltaP); //avoid '0'


		float original_length = glm::distance(const_pos[index_neigh],const_pos[index]);
		float leftTerm = -ks * (dist - original_length);
		float  rightTerm = kd * (glm::dot(deltaV, deltaP) / dist);
		glm::vec3 springForce = (leftTerm + rightTerm)*glm::normalize(deltaP);
		
		force += springForce;
	}
	return force;

}
__global__ void verlet(glm::vec4* pos_vbo, glm::vec4* g_pos_in, glm::vec4* g_pos_old_in, glm::vec4* g_pos_out, glm::vec4* g_pos_old_out, glm::vec4* const_pos,
					  unsigned int* neigh1, unsigned int* neigh2,
					  glm::vec3* p_normal, unsigned int* vertex_adjface, glm::vec3* face_normal,
					  const unsigned int NUM_VERTICES)
{
	unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= NUM_VERTICES)
		return;

	volatile glm::vec4 posData = g_pos_in[index];
	volatile glm::vec4 posOldData = g_pos_old_in[index];


	glm::vec3 pos = glm::vec3(posData.x, posData.y, posData.z);
	glm::vec3 pos_old = glm::vec3(posOldData.x, posOldData.y, posOldData.z);
	glm::vec3 vel = (pos - pos_old) / dt;

	const glm::vec3 gravity = glm::vec3(0.0f, -0.000981*2.0f, 0.0f); //set gravity
	glm::vec3 force = gravity*mass + vel*damp;
	force += get_spring_force(index, g_pos_in, g_pos_old_in, const_pos, neigh1, NUM_NEIGH1, pos, vel); //计算一级邻域弹簧力
	force += get_spring_force(index, g_pos_in, g_pos_old_in, const_pos, neigh2, NUM_NEIGH2, pos, vel); //计算二级邻域弹簧力
	
	glm::vec3 acc = force / mass;
	glm::vec3 tmp = pos;
	pos = pos + pos - pos_old + acc * dt * dt;
	pos_old = tmp;

	//compute point normal
	glm::vec3 normal(0.0);
	int first_face_index = index * NUM_ADJFACE;
	for (int i = first_face_index, time = 0; vertex_adjface[i] <UINT_MAX && time<NUM_ADJFACE; i++, time++)
	{
		int findex = vertex_adjface[i];
		glm::vec3 fnormal = face_normal[findex]; 
		normal += fnormal;
	}
	normal = glm::normalize(normal);

	pos_vbo[index] = glm::vec4(pos.x, pos.y, pos.z, posData.w);
	p_normal[index] = glm::vec3(normal.x, normal.y, normal.z);

	g_pos_out[index] = glm::vec4(pos.x, pos.y, pos.z, posData.w);
	g_pos_old_out[index] = glm::vec4(pos_old.x, pos_old.y, pos_old.z, posOldData.w);

}