
#include "simulator.h"
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
					  BRTreeNode*  leaf_nodes, BRTreeNode*  internal_nodes, Primitive* primitives,glm::vec3* d_collision_force);  //verlet intergration

Simulator::Simulator()
{
	
}

Simulator::~Simulator()
{
	delete cuda_spring;

	cudaFree(x_original);
	cudaFree(x_cur[0]);
	cudaFree(x_cur[1]);
	cudaFree(x_last[0]);
	cudaFree(x_last[1]);
	cudaFree(d_collision_force);
	cudaFree(d_adj_face_to_vertex);
	cudaFree(d_adj_vertex_to_face);
	cudaFree(d_face_normals);
}

void Simulator::init_cloth(Mesh& cloth)
{
	cuda_spring =  new Springs(&cloth);  

	cudaError_t cudaStatus = cudaGraphicsGLRegisterBuffer(&cuda_vbo_resource, sim_cloth->vbo.array_buffer, cudaGraphicsMapFlagsWriteDiscard);   	//register vbo
	if (cudaStatus != cudaSuccess)
		fprintf(stderr, "register failed\n");

	get_vertex_adjface();     //����λ��init_cudaǰ�棬������������Ϊ��
	init_cuda();              //��������ݴ���GPU
}

Simulator::Simulator(Mesh& cloth, Mesh& body) :readID(0), writeID(1), sim_cloth(&cloth), NUM_PER_VERTEX_ADJ_FACES(20)
{
	init_cloth(cloth);  // initial cloth 

	Mesh bvh_body = body;   //for bvh consttruction
	bvh_body.vertex_extend(0.003);

	get_primitives(bvh_body, obj_vertices, h_primitives);
	cuda_bvh = new BVHAccel(h_primitives);

	add_bvh(*cuda_bvh);
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

void Simulator::simulate()
{
	size_t num_bytes;
	cudaError_t cudaStatus = cudaGraphicsMapResources(1, &cuda_vbo_resource, 0);
	cudaStatus = cudaGraphicsResourceGetMappedPointer((void **)&cuda_p_vertex, &num_bytes, cuda_vbo_resource);
	cuda_p_normal = (glm::vec3*)((float*)cuda_p_vertex + 4 * sim_cloth->vertices.size() + 2 * sim_cloth->tex.size());   // ��ȡnormalλ��ָ��

	//cuda kernel compute .........
	verlet_cuda();
	cudaStatus = cudaGraphicsUnmapResources(1, &cuda_vbo_resource, 0);
	swap_buffer();
}

void Simulator::init_cuda()
{
	size_t heap_size = 256 * 1024 * 1024;  //set heap size, the default is 8M
	cudaDeviceSetLimit(cudaLimitMallocHeapSize, heap_size);

	//��sim_cloth�ĵ�����귢�͵�GPU
	cudaError_t cudaStatus;      
	const unsigned int vertices_bytes = sizeof(glm::vec4) * sim_cloth->vertices.size();
	cudaStatus = cudaMalloc((void**)&x_original, vertices_bytes); // cloth vertices (const)
	cudaStatus = cudaMalloc((void**)&x_cur[0], vertices_bytes);			 // cloth vertices
	cudaStatus = cudaMalloc((void**)&x_cur[1], vertices_bytes);			 // cloth vertices
	cudaStatus = cudaMalloc((void**)&x_last[0], vertices_bytes);	 // cloth old vertices
	cudaStatus = cudaMalloc((void**)&x_last[1], vertices_bytes);	 // cloth old vertices
	cudaStatus = cudaMalloc((void**)&d_collision_force, sizeof(glm::vec3) * sim_cloth->vertices.size());  //collision response force
	cudaMemset(d_collision_force, 0, sizeof(glm::vec3) * sim_cloth->vertices.size());    //initilize to 0

	x_cur_in = x_cur[readID];
	x_cur_out = x_cur[writeID];
	x_last_in = x_last[readID];
	x_last_out = x_last[writeID];

	cudaStatus = cudaMemcpy(x_original, &sim_cloth->vertices[0], vertices_bytes, cudaMemcpyHostToDevice);
	cudaStatus = cudaMemcpy(x_cur[0], &sim_cloth->vertices[0], vertices_bytes, cudaMemcpyHostToDevice);
	cudaStatus = cudaMemcpy(x_last[0], &sim_cloth->vertices[0], vertices_bytes, cudaMemcpyHostToDevice);

	//����normal��������ݣ�ÿ�����ڽӵ�������� + ÿ�����3��������� + �Լ����е����������ȻOPENGL�и����ݣ�
	const unsigned int vertices_index_bytes = sizeof(unsigned int) * sim_cloth->vertex_indices.size();       //�������
	cudaStatus = cudaMalloc((void**)&d_adj_face_to_vertex, vertices_index_bytes);	
	cudaStatus = cudaMemcpy(d_adj_face_to_vertex, &sim_cloth->vertex_indices[0], vertices_index_bytes, cudaMemcpyHostToDevice);

	const unsigned int face_normal_bytes = sizeof(glm::vec3) * sim_cloth->faces.size();    //��ķ�����
	cudaStatus = cudaMalloc((void**)&d_face_normals, face_normal_bytes);

	const unsigned int vertex_adjface_bytes = sizeof(unsigned int) * vertex_adjface.size();  //ÿ�����ڽӵ��������
	cudaStatus = cudaMalloc((void**)&d_adj_vertex_to_face, vertex_adjface_bytes);
	cudaStatus = cudaMemcpy(d_adj_vertex_to_face, &vertex_adjface[0], vertex_adjface_bytes, cudaMemcpyHostToDevice);
	
	//������Ϣ���������������Ϣ����GPU
	d_adj_structure_spring = cuda_spring->d_adj_structure_spring;
	d_adj_bend_spring = cuda_spring->d_adj_bend_spring;
}

void Simulator::get_vertex_adjface()
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

	vertex_adjface.resize(sim_cloth->vertices.size()*NUM_PER_VERTEX_ADJ_FACES);
	for(int i=0;i<adjaceny.size();i++)
	{
		int j;
		for(j=0;j<adjaceny[i].size() && j<NUM_PER_VERTEX_ADJ_FACES;j++)
		{
			vertex_adjface[i*NUM_PER_VERTEX_ADJ_FACES+j] = adjaceny[i][j];
		}
		if(NUM_PER_VERTEX_ADJ_FACES>adjaceny[i].size())
			vertex_adjface[i*NUM_PER_VERTEX_ADJ_FACES+j] = UINT_MAX;                  //Sentinel
	}
}

void Simulator::verlet_cuda()
{
	cudaError_t cudaStatus;
	unsigned int numThreads0, numBlocks0;
	computeGridSize(sim_cloth->faces.size(), 512, numBlocks0, numThreads0);
	unsigned int cloth_index_size = sim_cloth->vertex_indices.size(); 
	get_face_normal <<<numBlocks0, numThreads0 >>>(x_cur_in, d_adj_face_to_vertex, cloth_index_size, d_face_normals);  
	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess)
		fprintf(stderr, "normal cudaDeviceSynchronize returned error code %d after launching addKernel!\n%s\n", cudaStatus, cudaGetErrorString(cudaStatus));

	
	unsigned int numThreads, numBlocks;
	unsigned int numParticles = sim_cloth->vertices.size();
	

	computeGridSize(numParticles, 512, numBlocks, numThreads);
	verlet <<< numBlocks, numThreads >>>(cuda_p_vertex, x_cur_in,x_last_in, x_cur_out, x_last_out,x_original,
										d_adj_structure_spring,d_adj_bend_spring,
										cuda_p_normal,d_adj_vertex_to_face,d_face_normals,
										numParticles,
										d_leaf_nodes,d_internal_nodes,d_primitives, d_collision_force);
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
		save("../tem/cloth.obj");
	}
}

void Simulator::save(string file_name)
{
	vector<glm::vec4> updated_vertex(1);
	unsigned int numParticles = sim_cloth->vertices.size();
	cudaMemcpy(&updated_vertex[0], cuda_p_vertex, sizeof(glm::vec4)*numParticles, cudaMemcpyDeviceToHost);

	ofstream outfile(file_name);
	outfile << "# vertices" << endl;
	for (auto ver : updated_vertex)
	{
		outfile << "v " << ver.x << " " << ver.y << " " << ver.z << endl;   //����д���ļ�
	}

	outfile << "# faces" << endl;
	for (auto face : sim_cloth->faces)
	{
		outfile << "f " << face.vertex_index[0] + 1 << " " << face.vertex_index[1] + 1 << " " << face.vertex_index[2] + 1 << endl;
	}

	outfile.close();
}

void Simulator::computeGridSize(unsigned int n, unsigned int blockSize, unsigned int &numBlocks, unsigned int &numThreads)
{
	numThreads = min(blockSize, n);
	numBlocks = (n % numThreads != 0) ? (n / numThreads + 1) : (n / numThreads);
}

void Simulator::swap_buffer()
{
	int tmp = readID;
	readID = writeID;
	writeID = tmp;

	x_cur_in = x_cur[readID];
	x_cur_out = x_cur[writeID];
	x_last_in = x_last[readID];
	x_last_out = x_last[writeID];
}

void Simulator::add_bvh(BVHAccel& bvh)
{
	d_leaf_nodes = bvh.d_leaf_nodes;
	d_internal_nodes = bvh.d_internal_nodes;
	d_primitives = bvh.d_primitives;
}
