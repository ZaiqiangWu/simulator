#pragma once
#include "load_obj.h"
#include "parameter.h"
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>


class CUDA_Simulation
{
public:
	CUDA_Simulation();
	~CUDA_Simulation();
	CUDA_Simulation(Obj& cloth);
	void simulate();

private:
	void get_vertex_adjface();
	void init_cuda();

	void verlet_cuda();
	void computeGridSize(unsigned int n, unsigned int blockSize, unsigned int &numBlocks, unsigned int &numThreads);
	void swap_buffer();

public:
	int readID, writeID;
	glm::vec4* const_cuda_pos;      //计算弹簧原长
	glm::vec4* X[2];
	glm::vec4* X_last[2];
	glm::vec4 * X_in, *X_out;
	glm::vec4 * X_last_in, *X_last_out;

	unsigned int* cuda_vertex_index;    //点的索引
	unsigned int* cuda_vertex_adjface;       //计算每个点法向量时需要其周围平面的索引
	glm::vec3* cuda_face_normal;        //面的法向量

	cudaGraphicsResource* cuda_vbo_resource;
	glm::vec4* cuda_p_vertex;           //指向OPENGL buffer中vertex的地址
	glm::vec3* cuda_p_normal;           //指向OPENGL buffer中normal的地址

	unsigned int* cuda_neigh1;  //二维数组转为一维数组
	unsigned int* cuda_neigh2;

private:
	Obj* sim_cloth;
	vector<unsigned int> vertex_adjface;    //每个点最大包含20个邻近面，不足者以UINT_MAX作为结束标志
	unsigned int NUM_ADJFACE; 
};

