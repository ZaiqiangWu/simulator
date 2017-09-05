// simulation_bvh.cpp : 定义控制台应用程序的入口点。
//

#include "load_obj.h"
#include "scene.h"
#include "spring.h"
#include "cuda_simulation.h"
#include "parameter.h"
#include "./bvh/bvh.h"
#include <iostream>
using namespace std;

extern inline void copyFromCPUtoGPU(void** dst, void* src, int size);

void get_primitives(Obj& body, vector<glm::vec3>& obj_vertices,vector<Primitive>& h_primitives)
{
	//prepare primitives
	obj_vertices.resize(body.uni_vertices.size());
	for (int i = 0; i < body.uni_vertices.size(); i++)
	{
		obj_vertices[i] = glm::vec3(body.uni_vertices[i].x,
			body.uni_vertices[i].y,
			body.uni_vertices[i].z);
	}
	glm::vec3* d_obj_vertices;
	copyFromCPUtoGPU((void**)&d_obj_vertices, &obj_vertices[0], sizeof(glm::vec3)*obj_vertices.size());
	glm::vec3* h_obj_vertices = &obj_vertices[0];

	//create primitives
	h_primitives.resize(body.vertex_index.size() / 3);
	for (int i = 0; i < h_primitives.size(); i++)
	{
		Primitive tem_pri(h_obj_vertices, d_obj_vertices, body.vertex_index[i * 3 + 0],
			body.vertex_index[i * 3 + 1],
			body.vertex_index[i * 3 + 2]);
		h_primitives[i] = tem_pri;
	}
}

int main(int argc, char** argv)
{
	Scene* main_scene = Scene::getInstance(argc, argv); //initialize opengl 

	Obj cloth("./cloth/cloth.obj");  
	cloth.pretreat(0.31, 0, 1.95, 0.02);
	

	Obj body("./pose/pose0.obj");
	body.pretreat(0.30, 0, 1.0, 0);
	
	main_scene->add(cloth);
	main_scene->add(body);

	//Obj bvh_body("./pose/pose0.obj");
	//bvh_body.pretreat(0.295, 0, 1.0, 0);

	vector<glm::vec3> obj_vertices;
	vector<Primitive> h_primitives;
	get_primitives(body, obj_vertices, h_primitives);
	BVHAccel cuda_bvh(h_primitives);


	CUDA_Simulation simulation(cloth);   //add(obj)会初始化gpu端数据，simulation需要用到这些数据
	simulation.add_bvh(cuda_bvh);
	main_scene->add(simulation);
	main_scene->render();
	
	return 0;
}

