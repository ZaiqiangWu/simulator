#include <iostream>
#include <fstream>

#include "Mesh.h"

Mesh::Mesh(ObjLoader& Obj, cloth_type type):mesh_type(type)
{
	unified(Obj);
}

void Mesh::unified(ObjLoader& Obj)
{
	g_textureID = Obj.g_textureID;
	vertices = Obj.vertices;
	tex.resize(vertices.size());
	normals.resize(vertices.size());
	faces = Obj.faces;
	vertex_indices.resize(faces.size() * 3);

	vertex_object = Obj.vertex_object;  //for vertices region division 
	face_group = Obj.face_group;

	for (int i = 0; i < faces.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			tex[faces[i].vertex_index[j]] = Obj.tex[faces[i].tex_index[j]];
			normals[faces[i].vertex_index[j]] = Obj.normals[faces[i].normal_index[j]];
			vertex_indices[i * 3 + j] = faces[i].vertex_index[j];
		}
	}
}

void Mesh::scale_translate(float S, float x_up, float y_up, float z_up)
{
	//获取模型中心坐标
	glm::vec3 center = get_center();
	//const float up = 1.2;
	int n = vertices.size();
	for (int i = 0; i < n; ++i)
	{
		vertices[i].x = (vertices[i].x - center.x) / S; vertices[i].x += x_up;
		vertices[i].y = (vertices[i].y - center.y) / S; vertices[i].y += y_up;
		vertices[i].z = (vertices[i].z - center.z) / S; vertices[i].z += z_up;
	}
}

void Mesh::rotation(float angle, direction dir)
{
	angle = angle / 180 * 3.1415;
	glm::vec3 center = get_center();
	glm::mat4x4 R_matrix;
	if (dir == X)
		R_matrix = { {1.0, 0.0, 0.0, 0.0},
					{0.0, cos(angle), -sin(angle), 0.0},
					{0.0, sin(angle), cos(angle), 0.0},
					{0.0, 0.0, 0.0, 1.0}
	};
	else if (dir == Y)
	{
		R_matrix = { {cos(angle), 0, -sin(angle), 0},
					{0, 1.0, 0, 0},
					{sin(angle), 0, cos(angle), 0},
					{0, 0, 0, 1.0}
		};
	}
	else
	{
		R_matrix = { {cos(angle), -sin(angle), 0, 0},
					{sin(angle), cos(angle), 0, 0},
					{0, 0, 1, 0},
					{0, 0, 0, 1}
		};
	}

	int n = vertices.size();
	for (auto& vertex : vertices)
	{
		vertex -= glm::vec4(center, 0.0);
		vertex = R_matrix * vertex;
		vertex += glm::vec4(center, 0.0);
	}

}

void Mesh::vertex_extend(float dist)
{
	vector<bool> visited(vertices.size(), false);
	for (int i = 0; i < faces.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			int v_index = faces[i].vertex_index[j];
			if (visited[v_index] == false)
			{
				glm::vec3 n = normals[faces[i].normal_index[j]];
				vertices[v_index] += dist * glm::vec4(n, 0);
				visited[v_index] = true;
			}
		}
	}
}

glm::vec3 Mesh::get_center()
{
	glm::vec3 center;
	int n = vertices.size();
	float sumx = 0, sumy = 0, sumz = 0;
	for (int i = 0; i < n; ++i)
	{
		sumx += vertices[i].x;
		sumy += vertices[i].y;
		sumz += vertices[i].z;
	}
	center.x = sumx / n;
	center.y = sumy / n;
	center.z = sumz / n;
	return center;
}

cloth_type Mesh::get_obj_type()
{
	return mesh_type;
}

void Mesh::save()
{
	ofstream outfile("../tem/body.obj");

	outfile << "# vertices" << endl;
	for (auto ver : vertices)
	{
		outfile << "v " << ver.x << " " << ver.y << " " << ver.z << endl;   //数据写入文件
	}

	outfile << "# faces" << endl;
	for (auto face : faces)
	{
		outfile << "f " << face.vertex_index[0] + 1 << " " << face.vertex_index[1] + 1 << " " << face.vertex_index[2] + 1 << endl;
	}

	outfile.close();
}