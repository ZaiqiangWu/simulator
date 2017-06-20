#pragma once
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


class Obj
{
public:
	Obj();
	~Obj();
	Obj(const string file);
public:
	vector<glm::vec4> uni_vertices; 
	vector<glm::vec2> uni_tex;
	vector<glm::vec3> uni_normals;

	vector<unsigned int> vertex_index;           // unified the index for render

private:
	string obj_file;                  //load from *.obj
	string mtl_file;
	string texture_file;
	GLuint g_textureID;
	vector<glm::vec4> vertices; 
	vector<glm::vec3> normals;
	vector<glm::vec2> tex;
	vector<Face> faces;

	vector<pair<string,unsigned int>> vertex_object;  //for vertices region division 
	vector<pair<string,unsigned int>> face_group;

private:
	void unified();   //one vertices -> one tex -> one normal

};


