// simulation_bvh.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "load_obj.h"
#include "scene.h"
#include <iostream>

using namespace std;


int _tmain(int argc, char** argv)
{
	Scene* main_scene = Scene::getInstance(argc, argv); //initialize opengl 

	Obj cloth("./cloth/cloth.obj");  
	cloth.pretreat(0.31, 0, 1.95, 0.02);
	Obj body("./pose/pose0.obj");
	body.pretreat(0.30, 0, 1.0, 0);
	
	main_scene->add(cloth);
	main_scene->add(body);
	main_scene->render();

	return 0;
}

