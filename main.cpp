// simulation_bvh.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "load_obj.h"
#include "scene.h"
#include <iostream>

using namespace std;


int _tmain(int argc, char** argv)
{
	Obj cloth("./cloth/cloth.obj");  
	Scene* mainScene = Scene::getInstance(argc, argv); //create an OpenGL render scene 
	/*//test
	cout << cloth.vertices.size() << endl;
	cout << cloth.faces.size() << endl;

	for(auto val:cloth.vertex_object)
		cout << val.first << " " << val.second << endl; 

	for(auto val:cloth.face_group)
		cout << val.first << " " << val.second << endl;*/

	return 0;
}

