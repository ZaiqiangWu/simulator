// simulation_bvh.cpp : �������̨Ӧ�ó������ڵ㡣
//
#include <iostream>
#include <cmdline.h>

#include "scene.h"
#include "spring.h"
#include "cuda_simulation.h"
#include "ObjLoader.h"
#include "./bvh/bvh.h"
#include "Mesh.h"

using namespace std;

extern inline void copyFromCPUtoGPU(void** dst, void* src, int size);


void get_primitives(Mesh& body, vector<glm::vec3>& obj_vertices, vector<Primitive>& h_primitives)
{
	//prepare primitives
	obj_vertices.resize(body.vertices.size());
	for (int i = 0; i < body.vertices.size(); i++)
	{
		obj_vertices[i] = glm::vec3(body.vertices[i].x,
			body.vertices[i].y,
			body.vertices[i].z);
	}
	glm::vec3* d_obj_vertices;
	copyFromCPUtoGPU((void**)&d_obj_vertices, &obj_vertices[0], sizeof(glm::vec3)*obj_vertices.size());
	glm::vec3* h_obj_vertices = &obj_vertices[0];

	//create primitives
	h_primitives.resize(body.vertex_indices.size() / 3);
	for (int i = 0; i < h_primitives.size(); i++)
	{
		Primitive tem_pri(h_obj_vertices, d_obj_vertices, body.vertex_indices[i * 3 + 0],
			body.vertex_indices[i * 3 + 1],
			body.vertex_indices[i * 3 + 2]);
		h_primitives[i] = tem_pri;
	}
}

int main(int argc, char** argv)
{
	/*cmdline::parser a;
	a.add<string>("cloth", 'c', "cloth file", true, "");
	a.parse_check(argc, argv);
	cout << a.get<string>("cloth") << endl;*/

	Scene* main_scene = Scene::getInstance(argc, argv); //initialize opengl 


	ObjLoader obj_cloth("../../cloth/tshirt2/tshirt2.obj", SINGLE_LAYER_NOB);
	//obj_cloth.unified();
	Mesh cloth(obj_cloth, SINGLE_LAYER_NOB);
	cloth.rotation(90, X);   
	cloth.rotation(-4, Z);
	cloth.scale_translate(0.30, 0, 1.98, 0.02);



	Springs cuda_spring(&cloth);

	ObjLoader obj_body("../../pose/female.obj");
	Mesh body(obj_body);
	body.scale_translate(0.31, 0, 1.8, 0);

	main_scene->add(cloth);
	main_scene->add(body);

	Mesh bvh_body = body;   //for bvh consttruction
	bvh_body.vertex_extend(0.003);  


	vector<glm::vec3> obj_vertices;
	vector<Primitive> h_primitives;
	get_primitives(bvh_body, obj_vertices, h_primitives);
	BVHAccel cuda_bvh(h_primitives);

	CUDA_Simulation simulation(cloth, cuda_spring);   //add(obj)���ʼ��gpu�����ݣ�simulation��Ҫ�õ���Щ����
	simulation.add_bvh(cuda_bvh);
	main_scene->add(simulation);
	main_scene->render();

	return 0;
}

