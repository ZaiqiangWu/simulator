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



class Obj
{
public:
	Obj();
	Obj(const string file);
	~Obj();
	void pretreat(float S, float x_up, float y_up, float z_up);
	


public:
	vector<glm::vec4> uni_vertices;             //unifieed data
	vector<glm::vec2> uni_tex;
	vector<glm::vec3> uni_normals;
	vector<unsigned int> vertex_index;           // unified the index for render
	GLuint g_textureID;
	VAO_Buffer vbo;
                


	string obj_file;                  //load from *.obj
	string mtl_file;
	string texture_file;

	vector<glm::vec4> vertices; 
	vector<glm::vec3> normals;
	vector<glm::vec2> tex;
	vector<Face> faces;

	vector<pair<string,unsigned int>> vertex_object;  //for vertices region division 
	vector<pair<string,unsigned int>> face_group;

private:
	void change_size(float S, float x_up, float y_up, float z_up);
	void unified();   //one vertices -> one tex -> one normal

};


