#pragma once

#include "ObjLoader.h"


class Mesh
{
public:

	Mesh(ObjLoader& Obj, cloth_type type = SINGLE_LAYER_BOUNDARY);

	void scale_translate(float S, float x_up, float y_up, float z_up);
	void rotation(float angle, direction dir);

	//沿着NORMAL方向扩展点，不同于SCALE
	void vertex_extend(float dist);

	//save
	void save();

private:
	// unify the data, so that one vertex -> one normal -> one texture, 
	// or error acurred while rendering
	void unified(ObjLoader& Obj);

	glm::vec3 get_center();
	cloth_type get_obj_type();

public:
	vector<glm::vec4> vertices;             //unified data
	vector<glm::vec2> tex;
	vector<glm::vec3> normals;
	vector<unsigned int> vertex_indices;           // unified the index for render
	vector<Face> faces;

	GLuint g_textureID;
	VAO_Buffer vbo;

private:
	cloth_type mesh_type;
};