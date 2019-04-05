
#include "simulator.h"
#include "spring.h"
#include "Mesh.h"
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include <iostream>
#include <fstream>

#include "sim_parameter.h"
#include "watch.h"

using namespace std;

__global__ void get_face_normal(glm::vec4* g_pos_in, unsigned int* cloth_index, const unsigned int cloth_index_size, glm::vec3* cloth_face);   //update cloth face normal
__global__ void verlet(glm::vec4 * g_pos_in, glm::vec4 * g_pos_old_in, glm::vec4 * g_pos_out, glm::vec4 * g_pos_old_out,glm::vec4* const_pos,
						s_spring* neigh1, s_spring* neigh2,
					  const unsigned int NUM_VERTICES,
					  BRTreeNode*  leaf_nodes, BRTreeNode*  internal_nodes, Primitive* primitives,glm::vec3* d_collision_force);  //verlet intergration
__global__ void update_vbo_pos(glm::vec4* pos_vbo, glm::vec4* pos_cur, const unsigned int NUM_VERTICES);
__global__ void update_vbo_normal(glm::vec3* normals, unsigned int* vertex_adjface, glm::vec3* face_normal, const unsigned int NUM_VERTICES);

Simulator::Simulator()
{
	
}

Simulator::~Simulator()
{
	cudaFree(x_original);
	cudaFree(x_cur[0]);
	cudaFree(x_cur[1]);
	cudaFree(x_last[0]);
	cudaFree(x_last[1]);
	cudaFree(d_collision_force);
	cudaFree(d_adjface_to_vertex);
	cudaFree(d_face_normals);
}

Simulator::Simulator(Mesh& sim_cloth, Mesh& body) :readID(0), writeID(1)
{
	init_cloth(sim_cloth);
	init_spring(sim_cloth);
	init_bvh(body);
}

void Simulator::init_cloth(Mesh& sim_cloth)
{
	// \d_vbo_array_resource points to cloth's array buffer  
	cudaError_t cudaStatus = cudaGraphicsGLRegisterBuffer(&d_vbo_array_resource, sim_cloth.vbo.array_buffer, cudaGraphicsMapFlagsWriteDiscard);   	//register vbo
	if (cudaStatus != cudaSuccess)
		fprintf(stderr, "register failed\n");

	//set heap size, the default is 8M
	size_t heap_size = 256 * 1024 * 1024;  
	cudaDeviceSetLimit(cudaLimitMallocHeapSize, heap_size);

	// Send the cloth's vertices to GPU
	const unsigned int vertices_bytes = sizeof(glm::vec4) * sim_cloth.vertices.size();
	cudaStatus = cudaMalloc((void**)&x_original, vertices_bytes); // cloth vertices (const)
	cudaStatus = cudaMalloc((void**)&x_cur[0], vertices_bytes);			 // cloth vertices
	cudaStatus = cudaMalloc((void**)&x_cur[1], vertices_bytes);			 // cloth vertices
	cudaStatus = cudaMalloc((void**)&x_last[0], vertices_bytes);	 // cloth old vertices
	cudaStatus = cudaMalloc((void**)&x_last[1], vertices_bytes);	 // cloth old vertices
	cudaStatus = cudaMalloc((void**)&d_collision_force, sizeof(glm::vec3) * sim_cloth.vertices.size());  //collision response force
	cudaMemset(d_collision_force, 0, sizeof(glm::vec3) * sim_cloth.vertices.size());    //initilize to 0

	x_cur_in = x_cur[readID];
	x_cur_out = x_cur[writeID];
	x_last_in = x_last[readID];
	x_last_out = x_last[writeID];

	cudaStatus = cudaMemcpy(x_original, &sim_cloth.vertices[0], vertices_bytes, cudaMemcpyHostToDevice);
	cudaStatus = cudaMemcpy(x_cur[0], &sim_cloth.vertices[0], vertices_bytes, cudaMemcpyHostToDevice);
	cudaStatus = cudaMemcpy(x_last[0], &sim_cloth.vertices[0], vertices_bytes, cudaMemcpyHostToDevice);

	//计算normal所需的数据：每个点邻接的面的索引 + 每个面的3个点的索引
	vector<unsigned int> vertex_adjface;
	get_vertex_adjface(sim_cloth, vertex_adjface);
	const unsigned int vertex_adjface_bytes = sizeof(unsigned int) * vertex_adjface.size();  //每个点邻接的面的索引
	cudaStatus = cudaMalloc((void**)&d_adjface_to_vertex, vertex_adjface_bytes);
	cudaStatus = cudaMemcpy(d_adjface_to_vertex, &vertex_adjface[0], vertex_adjface_bytes, cudaMemcpyHostToDevice);

	cudaStatus = cudaGraphicsGLRegisterBuffer(&d_vbo_index_resource, sim_cloth.vbo.index_buffer, cudaGraphicsMapFlagsWriteDiscard);   	//register vbo
	if (cudaStatus != cudaSuccess)
		fprintf(stderr, "register failed\n");

	const unsigned int face_normal_bytes = sizeof(glm::vec3) * sim_cloth.faces.size();    //face normal
	cudaStatus = cudaMalloc((void**)&d_face_normals, face_normal_bytes);
}

void Simulator::init_spring(Mesh& sim_cloth)
{
	// Construct structure and bend springs in GPU
	Springs springs(&sim_cloth);
	vector<s_spring> cpu_str_spring;
	vector<s_spring> cpu_bend_spring;

	springs.serialize_structure_spring(cpu_str_spring);
	springs.serialize_bend_spring(cpu_bend_spring);

	cudaError_t cudaStatus = cudaMalloc((void**)&d_adj_structure_spring, cpu_str_spring.size() * sizeof(s_spring));
	cudaStatus = cudaMalloc((void**)&d_adj_bend_spring, cpu_bend_spring.size() * sizeof(s_spring));
	cudaStatus = cudaMemcpy(d_adj_structure_spring, &cpu_str_spring[0], cpu_str_spring.size() * sizeof(s_spring), cudaMemcpyHostToDevice);
	cudaStatus = cudaMemcpy(d_adj_bend_spring, &cpu_bend_spring[0], cpu_bend_spring.size() * sizeof(s_spring), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess)
	{
		cout << "error" << cudaGetErrorString(cudaStatus) << endl;
	}
}

void Simulator::init_bvh(Mesh& body)
{
	stop_watch watch;
	watch.start();
	Mesh bvh_body = body;   // for bvh consttruction
	bvh_body.vertex_extend(0.003);

	get_primitives(bvh_body, obj_vertices, h_primitives);
	watch.stop();
	cout << "bvh init done free time elapsed: " << watch.elapsed() << "us" << endl;

	watch.restart();
	BVHAccel* cuda_bvh = new BVHAccel(h_primitives);
	watch.stop();
	cout << "bvh build done free time elapsed: " << watch.elapsed() << "us" << endl;

	d_leaf_nodes = cuda_bvh->d_leaf_nodes;
	d_internal_nodes = cuda_bvh->d_internal_nodes;
	d_primitives = cuda_bvh->d_primitives;
}

void Simulator::get_primitives(Mesh& body, vector<glm::vec3>& obj_vertices, vector<Primitive>& h_primitives)
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
	cudaMalloc((void**)&d_obj_vertices, sizeof(glm::vec3)*obj_vertices.size());
	cudaMemcpy(d_obj_vertices, &obj_vertices[0], sizeof(glm::vec3)*obj_vertices.size(), cudaMemcpyHostToDevice);
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

void Simulator::simulate(Mesh* sim_cloth)
{
	size_t num_bytes;
	glm::vec4* d_vbo_vertex;           //point to vertex address in the OPENGL buffer
	glm::vec3* d_vbo_normal;           //point to normal address in the OPENGL buffer
	unsigned int* d_adjvertex_to_face;    // the order like this: f0(v0,v1,v2) -> f1(v0,v1,v2) -> ... ->fn(v0,v1,v2)
	unsigned int numParticles = sim_cloth->vertices.size();

	cudaError_t cudaStatus = cudaGraphicsMapResources(1, &d_vbo_array_resource);
	cudaStatus = cudaGraphicsMapResources(1, &d_vbo_index_resource);
	cudaStatus = cudaGraphicsResourceGetMappedPointer((void **)&d_vbo_vertex, &num_bytes, d_vbo_array_resource);
	cudaStatus = cudaGraphicsResourceGetMappedPointer((void **)&d_adjvertex_to_face, &num_bytes, d_vbo_index_resource);
	d_vbo_normal = (glm::vec3*)((float*)d_vbo_vertex + 4 * sim_cloth->vertices.size() + 2 * sim_cloth->tex.size());   // 获取normal位置指针

	//cuda kernel compute .........
	
	cuda_verlet(sim_cloth, d_vbo_vertex, d_vbo_normal);

	cuda_get_face_normal(sim_cloth, d_adjvertex_to_face);

	cuda_update_vbo(d_vbo_vertex, x_cur_out, d_vbo_normal, d_adjface_to_vertex, d_face_normals, numParticles);     // update array buffer for opengl

	swap_buffer();

	cudaStatus = cudaGraphicsUnmapResources(1, &d_vbo_index_resource);
	cudaStatus = cudaGraphicsUnmapResources(1, &d_vbo_array_resource);
}

void Simulator::get_vertex_adjface(Mesh& sim_cloth, vector<unsigned int>& vertex_adjface)
{
	vector<vector<unsigned int>> adjaceny(sim_cloth.vertices.size());
	for(int i=0;i<sim_cloth.faces.size();i++)
	{
		unsigned int f[3];
		for(int j=0;j<3;j++)
		{
			f[j] = sim_cloth.faces[i].vertex_index[j];
			adjaceny[f[j]].push_back(i);
		}
	}

	// 每个点最大包含20个邻近面，不足者以SENTINEL作为结束标志
	vertex_adjface.resize(sim_cloth.vertices.size()*NUM_PER_VERTEX_ADJ_FACES);
	for(int i=0;i<adjaceny.size();i++)
	{
		int j;
		for(j=0;j<adjaceny[i].size() && j<NUM_PER_VERTEX_ADJ_FACES;j++)
		{
			vertex_adjface[i*NUM_PER_VERTEX_ADJ_FACES+j] = adjaceny[i][j];
		}
		if(NUM_PER_VERTEX_ADJ_FACES>adjaceny[i].size())
			vertex_adjface[i*NUM_PER_VERTEX_ADJ_FACES+j] = SENTINEL;                  //Sentinel
	}
}

void Simulator::cuda_get_face_normal(Mesh* sim_cloth,unsigned int* d_adjvertex_to_face)
{
	unsigned int numThreads0, numBlocks0;
	computeGridSize(sim_cloth->faces.size(), 512, numBlocks0, numThreads0);
	unsigned int cloth_index_size = sim_cloth->vertex_indices.size();
	get_face_normal << <numBlocks0, numThreads0 >> > (x_cur_in, d_adjvertex_to_face, cloth_index_size, d_face_normals);

	cudaError_t cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess)
		fprintf(stderr, "normal cudaDeviceSynchronize returned error code %d after launching addKernel!\n%s\n", cudaStatus, cudaGetErrorString(cudaStatus));
}

void Simulator::cuda_verlet(Mesh* sim_cloth, glm::vec4* d_vbo_vertex, glm::vec3* d_vbo_normal)
{
	unsigned int numThreads, numBlocks;
	unsigned int numParticles = sim_cloth->vertices.size();
	
	computeGridSize(numParticles, 512, numBlocks, numThreads);
	verlet <<< numBlocks, numThreads >>>(x_cur_in,x_last_in, x_cur_out, x_last_out,x_original,
										d_adj_structure_spring,d_adj_bend_spring,							
										numParticles,
										d_leaf_nodes,d_internal_nodes,d_primitives, d_collision_force);

	// stop the CPU until the kernel has been executed
	cudaError_t cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess)
	{
		fprintf(stderr, "verlet cudaDeviceSynchronize returned error code %d after launching addKernel!\n%s\n",
			cudaStatus, cudaGetErrorString(cudaStatus));
		exit(-1);
	}
}

void Simulator::cuda_update_vbo(glm::vec4* pos_vbo, glm::vec4* pos_cur, glm::vec3* normals, unsigned int* vertex_adjface, glm::vec3* face_normal, const unsigned int numParticles)
{
	// update vertex position
	unsigned int numThreads, numBlocks;

	computeGridSize(numParticles, 512, numBlocks, numThreads);
	update_vbo_pos << < numBlocks, numThreads >> > (pos_vbo, pos_cur, numParticles);

	// stop the CPU until the kernel has been executed
	cudaError_t cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess)
	{
		fprintf(stderr, "update_vbo_pos cudaDeviceSynchronize returned error code %d after launching addKernel!\n%s\n",
			cudaStatus, cudaGetErrorString(cudaStatus));
		exit(-1);
	}

	// update vertex normal

	unsigned int numThreads1, numBlocks1;

	computeGridSize(numParticles, 1024, numBlocks1, numThreads1);
	update_vbo_normal << < numBlocks1, numThreads1 >> > (normals, vertex_adjface, face_normal,numParticles);

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess)
	{
		fprintf(stderr, "update_vbo_normal cudaDeviceSynchronize returned error code %d after launching addKernel!\n%s\n",
			cudaStatus, cudaGetErrorString(cudaStatus));
		exit(-1);
	}
}

void Simulator::save(string file_name)
{
	//vector<glm::vec4> updated_vertex(1);
	//size_t num_bytes;
	//glm::vec4* d_vbo_vertex;           //point to vertex address in the OPENGL buffer
	//unsigned int numParticles = sim_cloth->vertices.size();
	//cudaError_t cudaStatus = cudaGraphicsMapResources(1, &d_vbo_array_resource);

	//cudaStatus = cudaGraphicsResourceGetMappedPointer((void **)&d_vbo_vertex, &num_bytes, d_vbo_array_resource);

	//cudaMemcpy(&updated_vertex[0], d_vbo_vertex, sizeof(glm::vec4)*numParticles, cudaMemcpyDeviceToHost);

	//cudaStatus = cudaGraphicsUnmapResources(1, &d_vbo_array_resource);

	//ofstream outfile(file_name);
	//outfile << "# vertices" << endl;
	//for (auto ver : updated_vertex)
	//{
	//	outfile << "v " << ver.x << " " << ver.y << " " << ver.z << endl;   //数据写入文件
	//}

	//outfile << "# faces" << endl;
	//for (auto face : sim_cloth->faces)
	//{
	//	outfile << "f " << face.vertex_index[0] + 1 << " " << face.vertex_index[1] + 1 << " " << face.vertex_index[2] + 1 << endl;
	//}

	//outfile.close();
}

void Simulator::computeGridSize(unsigned int n, unsigned int blockSize, unsigned int &numBlocks, unsigned int &numThreads)
{
	numThreads = min(blockSize, n);
	numBlocks = (n % numThreads != 0) ? (n / numThreads + 1) : (n / numThreads);
}

void Simulator::swap_buffer()
{
	swap(readID, writeID);

	x_cur_in = x_cur[readID];
	x_cur_out = x_cur[writeID];
	x_last_in = x_last[readID];
	x_last_out = x_last[writeID];
}


