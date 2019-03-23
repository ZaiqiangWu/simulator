
#include "./bvh/bvh.h"
#include "spring.h"
#include <cuda.h>
#include <device_functions.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <glm/glm.hpp>


//physics parameter,若要修改参数，还需同时修改parameter.cpp
__device__ float spring_structure = 100.0;
__device__ float spring_bend = 2.0;
__device__ float damp = -0.02f; //增加damp avoid trembling
__device__ float mass = 0.3;
__device__ float dt =1/40.0f;
__device__ int NUM_PER_VERTEX_ADJ_FACES = 20;        //与simulation中的NUM_PER_VERTEX_ADJ_FACES一致
__device__ unsigned int NUM_PER_VERTEX_SPRING_STRUCT = 20;  //与spring中的NUM_PER_VERTEX_SPRING_STRUCT一致
__device__ unsigned int NUM_PER_VERTEX_SPRING_BEND = 20;  //与spring中的NUM_PER_VERTEX_SPRING_BEND一致 


__device__ bool  intersect(BRTreeNode*  leaf_nodes, BRTreeNode*  internal_nodes, const glm::vec3 point, int& idx);
__device__ BRTreeNode*  get_root(BRTreeNode* leaf_nodes, BRTreeNode* internal_nodes);
__device__ BRTreeNode*  get_left_child(BRTreeNode*  leaf_nodes, BRTreeNode*  internal_nodes, BRTreeNode* node);
__device__ BRTreeNode*  get_right_child(BRTreeNode*  leaf_nodes, BRTreeNode*  internal_nodes, BRTreeNode* node);
__device__ bool  is_leaf(BRTreeNode*  leaf_nodes, BRTreeNode*  internal_nodes, BRTreeNode* node);
__device__ bool  check_overlap(const glm::vec3 point, BRTreeNode* node);


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
__device__ void collision_response(BRTreeNode*  leaf_nodes, BRTreeNode*  internal_nodes, Primitive* primitives,
	glm::vec3& force, glm::vec3& pos, glm::vec3& pos_old)
{
	int idx_pri;
	bool inter = intersect(leaf_nodes, internal_nodes, pos, idx_pri); 
	if (inter)     
	{
		float dist;
		glm::vec3 normal;
		if (primitives[idx_pri].d_intersect(pos, dist, normal))
		{
			dist = 8.0*glm::abs(dist);    //collision response with penalty force
			glm::vec3 temp = dist*normal;
			force = force + temp;
			pos_old = pos;
		}

	}
}
__device__ void collision_response_projection(BRTreeNode*  leaf_nodes, BRTreeNode*  internal_nodes, Primitive* primitives,
	glm::vec3& force, glm::vec3& pos, glm::vec3& pos_old,
	int idx, glm::vec3* collision_force)
{
	int idx_pri;
	bool inter = intersect(leaf_nodes, internal_nodes, pos, idx_pri);
	if (inter)
	{
		float dist;
		glm::vec3 normal;
		if (primitives[idx_pri].d_intersect(pos, dist, normal))  //primitives[idx_pri].d_intersect(pos, dist, normal)
		{
			float k = 1.0;
			dist = k*glm::abs(dist);    // //collision response with penalty force
			pos += dist*normal;
			pos_old = pos;

			collision_force[idx] = normal;
		}
		else
			collision_force[idx] = glm::vec3(0.0);

	}
	else
		collision_force[idx] = glm::vec3(0.0);

}

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
										s_spring* neigh,unsigned int NUM_NEIGH,
									  glm::vec3 pos,glm::vec3 vel,float k_spring)
{
	glm::vec3 force(0.0);
	int first_neigh = index*NUM_NEIGH;   //访问一级邻域，UINT_MAX为截至标志
	int time = 0;
	for (int k = first_neigh; neigh[k].end < UINT_MAX && time<NUM_NEIGH; k++, time++) //部分点邻域大于MAX_NEIGH(20)
	{
		float ks = k_spring;
		float kd = -0.5;

		int index_neigh = neigh[k].end;
		volatile glm::vec4 pos_neighData = g_pos_in[index_neigh];
		volatile glm::vec4 pos_lastData = g_pos_old_in[index_neigh];
		glm::vec3 p2 = glm::vec3(pos_neighData.x, pos_neighData.y, pos_neighData.z);
		glm::vec3 p2_last = glm::vec3(pos_lastData.x, pos_lastData.y, pos_lastData.z);

		glm::vec3 v2 = (p2 - p2_last) / dt;
		glm::vec3 deltaP = pos - p2;
		if (glm::length(deltaP) == 0)
		{
			//force += glm::vec3(0.0f); continue; 
			deltaP += glm::vec3(1.0e-6);	//avoid '0'
		}  

		glm::vec3 deltaV = vel - v2;
		float dist = glm::length(deltaP); //avoid '0'


		float original_length = neigh[k].original;//glm::distance(const_pos[neigh[k].end],const_pos[index]);
		float leftTerm = -ks * (dist - original_length);
		float  rightTerm =  kd * (glm::dot(deltaV, deltaP) / dist);
		glm::vec3 springForce = (leftTerm + rightTerm)*glm::normalize(deltaP);
		
		force += springForce;
	}
	return force;

}




__global__ void verlet(glm::vec4* pos_vbo, glm::vec4* g_pos_in, glm::vec4* g_pos_old_in, glm::vec4* g_pos_out, glm::vec4* g_pos_old_out, glm::vec4* const_pos,
						s_spring* neigh1, s_spring* neigh2,
					  glm::vec3* p_normal, unsigned int* vertex_adjface, glm::vec3* face_normal,
					  const unsigned int NUM_VERTICES,
					BRTreeNode*  leaf_nodes, BRTreeNode*  internal_nodes, Primitive* primitives, glm::vec3* collision_force)
{
	unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= NUM_VERTICES)
		return;

	
	volatile glm::vec4 posData = g_pos_in[index];
	volatile glm::vec4 posOldData = g_pos_old_in[index];


	glm::vec3 pos = glm::vec3(posData.x, posData.y, posData.z);
	glm::vec3 pos_old = glm::vec3(posOldData.x, posOldData.y, posOldData.z);
	glm::vec3 vel = (pos - pos_old) / dt;


	const glm::vec3 gravity = glm::vec3(0.0f, -0.00981f, 0.0f); //set gravity
	
	glm::vec3 force = gravity*mass + vel*damp;
	force += get_spring_force(index, g_pos_in, g_pos_old_in, const_pos, neigh1, NUM_PER_VERTEX_SPRING_STRUCT, pos, vel,spring_structure); //计算一级邻域弹簧力
	force += get_spring_force(index, g_pos_in, g_pos_old_in, const_pos, neigh2, NUM_PER_VERTEX_SPRING_BEND, pos, vel,spring_bend); //计算二级邻域弹簧力
	//if (pos.y > 2.5)
	//	force = glm::vec3(0.0);
	//verlet integration
	//collision_response(leaf_nodes, internal_nodes, primitives, force, pos, pos_old);  //******************?

	glm::vec3 inelastic_force = glm::dot(collision_force[index], force) * collision_force[index];       //collision response force, if intersected, keep tangential
	//inelastic_force *= 1.5;
	force -= inelastic_force;
	glm::vec3 acc = force / mass;
	glm::vec3 tmp = pos;          
	pos = pos + pos - pos_old + acc * dt * dt;   
	pos_old = tmp;
	collision_response_projection(leaf_nodes, internal_nodes, primitives, force, pos, pos_old, index, collision_force);

	//compute point normal
	glm::vec3 normal(0.0);
	int first_face_index = index * NUM_PER_VERTEX_ADJ_FACES;
	for (int i = first_face_index, time = 0; vertex_adjface[i] <UINT_MAX && time<NUM_PER_VERTEX_ADJ_FACES; i++, time++)
	{
		int findex = vertex_adjface[i];
		glm::vec3 fnormal = face_normal[findex]; 
		normal += fnormal;
	}
	normal = glm::normalize(normal);


	//set new vertex and new normal
	pos_vbo[index] = glm::vec4(pos.x, pos.y, pos.z, posData.w);
	p_normal[index] = glm::vec3(normal.x, normal.y, normal.z);

	g_pos_out[index] = glm::vec4(pos.x, pos.y, pos.z, posData.w);
	g_pos_old_out[index] = glm::vec4(pos_old.x, pos_old.y, pos_old.z, posOldData.w);

}