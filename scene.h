#pragma once

#define GLEW_STATIC
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <glm/glm.hpp>

#include "vao_buffer.h"
#include "GLSLShader.h"
#include "Mesh.h"
#include "cuda_simulation.h"
#include "./bvh/bvh.h"

#pragma comment(lib, "glew32s.lib")

//singleton
class Scene
{
public:
	static Scene* getInstance(int argc, char** argv);
	~Scene(); //closeFunc() 
	
	void add_cloth(Mesh& object);   // mesh(cloth) to be simulated
	void add_body(Mesh& object);    // mesh(body) to be collided
	void simulate();               // start cloth simulation
	void render();

public:
	vector<VAO_Buffer> obj_vaos;
	
private:
	Scene(int argc, char** argv);  //initial
	void add(Mesh& object);   //add objects,bind VAOs 
	void loadShader();
	void save_obj(string file, vector<glm::vec3> vertices);
	void RenderBuffer(VAO_Buffer vao_buffer);

	vector<glm::vec3> obj_vertices;
	void get_primitives(Mesh& body, vector<glm::vec3>& obj_vertices, vector<Primitive>& h_primitives);

private:
	static Scene* pscene;       //pscene points to the Scene(singleton)
	Mesh* cloth;
	Mesh* body;
	CUDA_Simulation* simulation;
	BVHAccel* cuda_bvh;
	GLSLShader renderShader;
	enum attributes { position, texture, normal };
	vector<Primitive> h_primitives;     // host primitives for cuda_bvh construction

private:
	static void screenshot();
	static void DrawGrid();                  // OPENGL场景的各种函数
	static void RenderGPU_CUDA();
	static void onRender();
	static void OnReshape(int nw, int nh);
	static void OnIdle();
	static void OnMouseMove(int x, int y);
	static void OnMouseDown(int button, int s, int x, int y);
	static void OnKey(unsigned char key, int, int);
	static void OnShutdown();
	inline void check_GL_error();

private:
	static int oldX, oldY;    // OPENGL场景的各种参数declaration
	static float rX, rY;
	static int state;
	static float dist, dy;
	static GLint viewport[4];
	static GLfloat modelview[16];
	static GLfloat projection[16];
	static glm::vec3 Up, Right, viewDir;
	static int selected_index;
	static const int width = 1024, height = 1024;
	static bool start_sim;
};



