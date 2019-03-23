
#include "cuda_simulation.h"
#include "spring.h"
#include "Mesh.h"
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include <iostream>
#include <fstream>
using namespace std;

extern GLenum GL_MODE;
extern bool SAVE_OBJ;

extern inline void copyFromCPUtoGPU(void** dst, void* src, int size);

__global__ void get_face_normal(glm::vec4* g_pos_in, unsigned int* cloth_index, const unsigned int cloth_index_size, glm::vec3* cloth_face);   //update cloth face normal
__global__ void verlet(glm::vec4* pos_vbo, glm::vec4 * g_pos_in, glm::vec4 * g_pos_old_in, glm::vec4 * g_pos_out, glm::vec4 * g_pos_old_out,glm::vec4* const_pos,
						s_spring* neigh1, s_spring* neigh2,
					  glm::vec3* p_normal, unsigned int* vertex_adjface, glm::vec3* face_normal,
					  const unsigned int NUM_VERTICES,
					  BRTreeNode*  leaf_nodes, BRTreeNode*  internal_nodes, Primitive* primitives,glm::vec3* collision_force, int* collided_vertex,
					glm::vec3* d_force,glm::vec3* d_velocity, float timestep);  //verlet intergration

CUDA_Simulation::CUDA_Simulation()
{
	
}

CUDA_Simulation::~CUDA_Simulation()
{
	delete cuda_spring;

	cudaFree(const_cuda_pos);
	cudaFree(X[0]);
	cudaFree(X[1]);
	cudaFree(X_last[0]);
	cudaFree(X_last[1]);
	cudaFree(collision_force);
	cudaFree(cuda_vertex_index);
	cudaFree(cuda_vertex_adjface);
	cudaFree(cuda_face_normal);
	cudaFree(d_force);
	cudaFree(d_velocity);
}

void CUDA_Simulation::init_cloth(Mesh& cloth)
{
	cuda_spring =  new Springs(&cloth);  

	cudaError_t cudaStatus = cudaGraphicsGLRegisterBuffer(&cuda_vbo_resource, sim_cloth->vbo.array_buffer, cudaGraphicsMapFlagsWriteDiscard);   	//register vbo
	if (cudaStatus != cudaSuccess)
		fprintf(stderr, "register failed\n");

	get_vertex_adjface();     //必须位于init_cuda前面，否则邻域数据为空
	init_cuda();              //将相关数据传送GPU
}

CUDA_Simulation::CUDA_Simulation(Mesh& cloth, Mesh& body) :readID(0), writeID(1), sim_cloth(&cloth), NUM_ADJFACE(20), dt(1 / 20.0)
{
	init_cloth(cloth);  // initial cloth 

	Mesh bvh_body = body;   //for bvh consttruction
	bvh_body.vertex_extend(0.003);

	get_primitives(bvh_body, obj_vertices, h_primitives);
	cuda_bvh = new BVHAccel(h_primitives);

	add_bvh(*cuda_bvh);
}

void CUDA_Simulation::get_primitives(Mesh& body, vector<glm::vec3>& obj_vertices, vector<Primitive>& h_primitives)
{
	//prepare primitives
	obj_vertices.resize(body.vertices.size());
	for (int i = 0; i < body.vertices.size(); i++)
	{
		obj_vertices[i] = glm::vec3(body.vertices[i].x,
			body.vertices[i].y,
			body.vertices[i].z);
	}
	glm::vec3* d_obj_vertices;
	copyFromCPUtoGPU((void**)&d_obj_vertices, &obj_vertices[0], sizeof(glm::vec3)*obj_vertices.size());
	glm::vec3* h_obj_vertices = &obj_vertices[0];

	//create primitives
	h_primitives.resize(body.vertex_indices.size() / 3);
	for (int i = 0; i < h_primitives.size(); i++)
	{
		Primitive tem_pri(h_obj_vertices, d_obj_vertices, body.vertex_indices[i * 3 + 0],
			body.vertex_indices[i * 3 + 1],
			body.vertex_indices[i * 3 + 2]);
		h_primitives[i] = tem_pri;
	}
}

void CUDA_Simulation::simulate()
{
	size_t num_bytes;
	cudaError_t cudaStatus = cudaGraphicsMapResources(1, &cuda_vbo_resource, 0);
	cudaStatus = cudaGraphicsResourceGetMappedPointer((void **)&cuda_p_vertex, &num_bytes, cuda_vbo_resource);
	cuda_p_normal = (glm::vec3*)((float*)cuda_p_vertex + 4 * sim_cloth->vertices.size() + 2 * sim_cloth->tex.size());   // 获取normal位置指针

	//cuda kernel compute .........
	verlet_cuda();
	cudaStatus = cudaGraphicsUnmapResources(1, &cuda_vbo_resource, 0);
	swap_buffer();
}

void CUDA_Simulation::init_cuda()
{
	size_t heap_size = 256 * 1024 * 1024;  //set heap size, the default is 8M
	cudaDeviceSetLimit(cudaLimitMallocHeapSize, heap_size);

	//将sim_cloth的点的坐标发送到GPU
	cudaError_t cudaStatus;      
	const unsigned int vertices_bytes = sizeof(glm::vec4) * sim_cloth->vertices.size();
	cudaStatus = cudaMalloc((void**)&const_cuda_pos, vertices_bytes); // cloth vertices (const)
	cudaStatus = cudaMalloc((void**)&X[0], vertices_bytes);			 // cloth vertices
	cudaStatus = cudaMalloc((void**)&X[1], vertices_bytes);			 // cloth vertices
	cudaStatus = cudaMalloc((void**)&X_last[0], vertices_bytes);	 // cloth old vertices
	cudaStatus = cudaMalloc((void**)&X_last[1], vertices_bytes);	 // cloth old vertices
	cudaStatus = cudaMalloc((void**)&collision_force, sizeof(glm::vec3) * sim_cloth->vertices.size());  //collision response force
	cudaMemset(collision_force, 0, sizeof(glm::vec3) * sim_cloth->vertices.size());    //initilize to 0

	X_in = X[readID];
	X_out = X[writeID];
	X_last_in = X_last[readID];
	X_last_out = X_last[writeID];

	cudaStatus = cudaMemcpy(const_cuda_pos, &sim_cloth->vertices[0], vertices_bytes, cudaMemcpyHostToDevice);
	cudaStatus = cudaMemcpy(X[0], &sim_cloth->vertices[0], vertices_bytes, cudaMemcpyHostToDevice);
	cudaStatus = cudaMemcpy(X_last[0], &sim_cloth->vertices[0], vertices_bytes, cudaMemcpyHostToDevice);

	//计算normal所需的数据：每个点邻接的面的索引 + 每个面的3个点的索引 + 以及所有点的索引（虽然OPENGL有该数据）
	const unsigned int vertices_index_bytes = sizeof(unsigned int) * sim_cloth->vertex_indices.size();       //点的索引
	cudaStatus = cudaMalloc((void**)&cuda_vertex_index, vertices_index_bytes);	
	cudaStatus = cudaMemcpy(cuda_vertex_index, &sim_cloth->vertex_indices[0], vertices_index_bytes, cudaMemcpyHostToDevice);

	const unsigned int face_normal_bytes = sizeof(glm::vec3) * sim_cloth->faces.size();    //面的法向量
	cudaStatus = cudaMalloc((void**)&cuda_face_normal, face_normal_bytes);

	const unsigned int vertex_adjface_bytes = sizeof(unsigned int) * vertex_adjface.size();  //每个点邻接的面的索引
	cudaStatus = cudaMalloc((void**)&cuda_vertex_adjface, vertex_adjface_bytes);
	cudaStatus = cudaMemcpy(cuda_vertex_adjface, &vertex_adjface[0], vertex_adjface_bytes, cudaMemcpyHostToDevice);
	
	//弹簧信息，即两级邻域点信息传送GPU
	cuda_neigh1 = cuda_spring->cuda_neigh1;
	cuda_neigh2 = cuda_spring->cuda_neigh2;

	updated_vertex.resize(sim_cloth->vertices.size());
	cudaStatus = cudaMalloc((void**)&d_force, sizeof(glm::vec3)*sim_cloth->vertices.size());
	cudaStatus = cudaMalloc((void**)&d_velocity, sizeof(glm::vec3)*sim_cloth->vertices.size());

}

void CUDA_Simulation::get_vertex_adjface()
{
	vector<vector<unsigned int>> adjaceny(sim_cloth->vertices.size());
	for(int i=0;i<sim_cloth->faces.size();i++)
	{
		unsigned int f[3];
		for(int j=0;j<3;j++)
		{
			f[j] = sim_cloth->faces[i].vertex_index[j];
			adjaceny[f[j]].push_back(i);
		}
	}

	vertex_adjface.resize(sim_cloth->vertices.size()*NUM_ADJFACE);
	for(int i=0;i<adjaceny.size();i++)
	{
		int j;
		for(j=0;j<adjaceny[i].size() && j<NUM_ADJFACE;j++)
		{
			vertex_adjface[i*NUM_ADJFACE+j] = adjaceny[i][j];
		}
		if(NUM_ADJFACE>adjaceny[i].size())
			vertex_adjface[i*NUM_ADJFACE+j] = UINT_MAX;                  //Sentinel
	}
}

void CUDA_Simulation::verlet_cuda()
{
	cudaError_t cudaStatus;
	unsigned int numThreads0, numBlocks0;
	computeGridSize(sim_cloth->faces.size(), 512, numBlocks0, numThreads0);
	unsigned int cloth_index_size = sim_cloth->vertex_indices.size(); 
	get_face_normal <<<numBlocks0, numThreads0 >>>(X_in, cuda_vertex_index, cloth_index_size, cuda_face_normal);  
	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess)
		fprintf(stderr, "normal cudaDeviceSynchronize returned error code %d after launching addKernel!\n%s\n", cudaStatus, cudaGetErrorString(cudaStatus));

	
	unsigned int numThreads, numBlocks;
	unsigned int numParticles = sim_cloth->vertices.size();
	

	computeGridSize(numParticles, 512, numBlocks, numThreads);
	verlet <<< numBlocks, numThreads >>>(cuda_p_vertex, X_in, X_last_in, X_out, X_last_out,const_cuda_pos,
										cuda_neigh1,cuda_neigh2,
										cuda_p_normal,cuda_vertex_adjface,cuda_face_normal,
										numParticles,
										d_leaf_nodes,d_internal_nodes,d_primitives, collision_force,
										collided_vertex,
										d_force,d_velocity,dt);

	// stop the CPU until the kernel has been executed
	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess)
	{
		fprintf(stderr, "verlet cudaDeviceSynchronize returned error code %d after launching addKernel!\n%s\n",
			cudaStatus, cudaGetErrorString(cudaStatus));
		exit(-1);
	}

	if (SAVE_OBJ)
	{
		SAVE_OBJ = false;
		cudaMemcpy(&updated_vertex[0], cuda_p_vertex, sizeof(glm::vec4)*numParticles, cudaMemcpyDeviceToHost);
		ofstream outfile("../tem/cloth.obj");

		outfile << "# vertices" << endl;
		for (auto ver : updated_vertex)
		{
			outfile << "v " << ver.x << " " << ver.y << " " << ver.z << endl;   //数据写入文件
		}
		
		outfile << "# faces" << endl;
		for (auto face : sim_cloth->faces)
		{
			outfile << "f " << face.vertex_index[0]+1 << " " << face.vertex_index[1]+1 << " " << face.vertex_index[2]+1 << endl;
		}

		outfile.close();
	}
}

void CUDA_Simulation::computeGridSize(unsigned int n, unsigned int blockSize, unsigned int &numBlocks, unsigned int &numThreads)
{
	numThreads = min(blockSize, n);
	numBlocks = (n % numThreads != 0) ? (n / numThreads + 1) : (n / numThreads);
}

void CUDA_Simulation::swap_buffer()
{
	int tmp = readID;
	readID = writeID;
	writeID = tmp;

	X_in = X[readID];
	X_out = X[writeID];
	X_last_in = X_last[readID];
	X_last_out = X_last[writeID];

}

void CUDA_Simulation::add_bvh(BVHAccel& bvh)
{
	d_leaf_nodes = bvh.d_leaf_nodes;
	d_internal_nodes = bvh.d_internal_nodes;
	d_primitives = bvh.d_primitives;
}
