#pragma once
#include "vao_buffer.h"
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <vector>
#include <string>

using namespace std;


//just for simulation, no render info
struct Face
{
	unsigned int vertex_index[3];
	unsigned int tex_index[3];
	unsigned int normal_index[3];
};

class ObjLoader
{
public:
	ObjLoader(const string file);
	~ObjLoader();


public:
	GLuint g_textureID;

	string obj_file;                  //load from *.obj
	string mtl_file;
	string texture_file;

	vector<glm::vec4> vertices; 
	vector<glm::vec3> normals;
	vector<glm::vec2> tex;
	vector<Face> faces;

	vector<pair<string,unsigned int>> vertex_object;  //for vertices region division 
	vector<pair<string,unsigned int>> face_group;
};


