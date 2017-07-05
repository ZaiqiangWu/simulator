// simulation_bvh.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "load_obj.h"
#include "scene.h"
#include "spring.h"
#include "cuda_simulation.h"
#include "parameter.h"
#include <iostream>
using namespace std;


int _tmain(int argc, char** argv)
{
	Scene* main_scene = Scene::getInstance(argc, argv); //initialize opengl 

	Obj cloth("./cloth/cloth.obj");  
	cloth.pretreat(0.31, 0, 1.95, 0.02);
	//test
	//for(auto v : cloth.vertex_object)
	//	cout << v.first << " "<< v.second << endl;

	//cout << "face" << endl;
	//for(auto v : cloth.face_group)
	//	cout << v.first << " "<< v.second << endl;
	//exit(-1);

	//Obj body("./pose/pose0.obj");
	//body.pretreat(0.30, 0, 1.0, 0);
	
	main_scene->add(cloth);
	//main_scene->add(body);

	CUDA_Simulation simulation(cloth);   //add(obj)会初始化gpu端数据，simulation需要用到这些数据
	main_scene->add(simulation);
	main_scene->render();
	
	return 0;
}

