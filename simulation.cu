
#include "cuda_simulation.h"
#include "spring.h"
#include "load_obj.h"
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include <iostream>
#include <fstream>
using namespace std;

extern GLenum GL_MODE;
extern bool SAVE_OBJ;

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
	
}

CUDA_Simulation::CUDA_Simulation(Obj& cloth, Springs& springs):readID(0), writeID(1),sim_cloth(&cloth),NUM_ADJFACE(20),cuda_spring(&springs),dt(1/20.0)
{
	cudaError_t cudaStatus = cudaGraphicsGLRegisterBuffer(&cuda_vbo_resource, sim_cloth->vbo.array_buffer, cudaGraphicsMapFlagsWriteDiscard);   	//register vbo
	if (cudaStatus != cudaSuccess)
		fprintf(stderr, "register failed\n");

	get_vertex_adjface();     //����λ��init_cudaǰ�棬������������Ϊ��
	init_cuda();              //��������ݴ���GPU


		
}

void CUDA_Simulation::simulate()
{
	size_t num_bytes;
	cudaError_t cudaStatus = cudaGraphicsMapResources(1, &cuda_vbo_resource, 0);
	cudaStatus = cudaGraphicsResourceGetMappedPointer((void **)&cuda_p_vertex, &num_bytes, cuda_vbo_resource);
	cuda_p_normal = (glm::vec3*)((float*)cuda_p_vertex + 4 * sim_cloth->uni_vertices.size() + 2 * sim_cloth->uni_tex.size());   // ��ȡnormalλ��ָ��

	//cuda kernel compute .........
	verlet_cuda();
	cudaStatus = cudaGraphicsUnmapResources(1, &cuda_vbo_resource, 0);
	swap_buffer();
}

void CUDA_Simulation::init_cuda()
{
	size_t heap_size = 256 * 1024 * 1024;  //set heap size, the default is 8M
	cudaDeviceSetLimit(cudaLimitMallocHeapSize, heap_size);

	//��sim_cloth�ĵ�����귢�͵�GPU
	cudaError_t cudaStatus;      
	const unsigned int vertices_bytes = sizeof(glm::vec4) * sim_cloth->uni_vertices.size();
	cudaStatus = cudaMalloc((void**)&const_cuda_pos, vertices_bytes); // cloth vertices (const)
	cudaStatus = cudaMalloc((void**)&X[0], vertices_bytes);			 // cloth vertices
	cudaStatus = cudaMalloc((void**)&X[1], vertices_bytes);			 // cloth vertices
	cudaStatus = cudaMalloc((void**)&X_last[0], vertices_bytes);	 // cloth old vertices
	cudaStatus = cudaMalloc((void**)&X_last[1], vertices_bytes);	 // cloth old vertices
	cudaStatus = cudaMalloc((void**)&collision_force, sizeof(glm::vec3) * sim_cloth->uni_vertices.size());  //collision response force
	cudaMemset(collision_force, 0, sizeof(glm::vec3) * sim_cloth->uni_vertices.size());    //initilize to 0

	X_in = X[readID];
	X_out = X[writeID];
	X_last_in = X_last[readID];
	X_last_out = X_last[writeID];

	cudaStatus = cudaMemcpy(const_cuda_pos, &sim_cloth->uni_vertices[0], vertices_bytes, cudaMemcpyHostToDevice);
	cudaStatus = cudaMemcpy(X[0], &sim_cloth->uni_vertices[0], vertices_bytes, cudaMemcpyHostToDevice);
	cudaStatus = cudaMemcpy(X_last[0], &sim_cloth->uni_vertices[0], vertices_bytes, cudaMemcpyHostToDevice);

	//����normal��������ݣ�ÿ�����ڽӵ�������� + ÿ�����3��������� + �Լ����е����������ȻOPENGL�и����ݣ�
	const unsigned int vertices_index_bytes = sizeof(unsigned int) * sim_cloth->vertex_index.size();       //�������
	cudaStatus = cudaMalloc((void**)&cuda_vertex_index, vertices_index_bytes);	
	cudaStatus = cudaMemcpy(cuda_vertex_index, &sim_cloth->vertex_index[0], vertices_index_bytes, cudaMemcpyHostToDevice);

	const unsigned int face_normal_bytes = sizeof(glm::vec3) * sim_cloth->faces.size();    //��ķ�����
	cudaStatus = cudaMalloc((void**)&cuda_face_normal, face_normal_bytes);

	const unsigned int vertex_adjface_bytes = sizeof(unsigned int) * vertex_adjface.size();  //ÿ�����ڽӵ��������
	cudaStatus = cudaMalloc((void**)&cuda_vertex_adjface, vertex_adjface_bytes);
	cudaStatus = cudaMemcpy(cuda_vertex_adjface, &vertex_adjface[0], vertex_adjface_bytes, cudaMemcpyHostToDevice);
	
	//������Ϣ���������������Ϣ����GPU
	cuda_neigh1 = cuda_spring->cuda_neigh1;
	cuda_neigh2 = cuda_spring->cuda_neigh2;

#ifdef _DEBUG
	cudaMalloc((void**)&collided_vertex, sizeof(int)*sim_cloth->uni_vertices.size());
	cudaMemset(collided_vertex, 0, sizeof(int)*sim_cloth->uni_vertices.size());
	cpu_collided_veretx.resize(sim_cloth->uni_vertices.size());
	updated_vertex.resize(sim_cloth->uni_vertices.size());
#endif
	updated_vertex.resize(sim_cloth->uni_vertices.size());
	cudaStatus = cudaMalloc((void**)&d_force, sizeof(glm::vec3)*sim_cloth->uni_vertices.size());
	cudaStatus = cudaMalloc((void**)&d_velocity, sizeof(glm::vec3)*sim_cloth->uni_vertices.size());

}

void CUDA_Simulation::get_vertex_adjface()
{
	vector<vector<unsigned int>> adjaceny(sim_cloth->uni_vertices.size());
	for(int i=0;i<sim_cloth->faces.size();i++)
	{
		unsigned int f[3];
		for(int j=0;j<3;j++)
		{
			f[j] = sim_cloth->faces[i].vertex_index[j];
			adjaceny[f[j]].push_back(i);
		}
	}

	//test
	/*for(int i=0;i<10;i++)
	{
		for(int j=0;j<adjaceny[i].size();j++)
			cout << adjaceny[i][j] << "  ";
		cout << endl;
		
	}
*/
	vertex_adjface.resize(sim_cloth->uni_vertices.size()*NUM_ADJFACE);
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
	unsigned int cloth_index_size = sim_cloth->vertex_index.size(); 
	get_face_normal <<<numBlocks0, numThreads0 >>>(X_in, cuda_vertex_index, cloth_index_size, cuda_face_normal);  
	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess)
		fprintf(stderr, "normal cudaDeviceSynchronize returned error code %d after launching addKernel!\n%s\n", cudaStatus, cudaGetErrorString(cudaStatus));

	
	unsigned int numThreads, numBlocks;
	unsigned int numParticles = sim_cloth->uni_vertices.size();
	

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
			outfile << "v " << ver.x << " " << ver.y << " " << ver.z << endl;   //����д���ļ�
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

void CUDA_Simulation::draw_collided_vertex()
{

	//draw outline first
		for (int i = 0; i < sim_cloth->faces.size(); i++)
		{
			glm::vec4 ver[3];
			glm::vec3 normal[3];
			for (int j = 0; j < 3; j++)
			{
				ver[j] = updated_vertex[sim_cloth->faces[i].vertex_index[j]];
			}
			glPointSize(1.0);
			glBegin(GL_MODE);
			glColor3f(1.0, 1.0,1.0);
			for (int j = 0; j < 3; j++)
			{
				glVertex3f(ver[j].x, ver[j].y, ver[j].z);
			}
				
			glEnd();
		}


	for (int i = 0; i < cpu_collided_veretx.size(); i++)
	{
		glm::vec4 v = updated_vertex[i];
		if (cpu_collided_veretx[i] == 1)
		{
			//draw it
			glPointSize(10.0);
			glBegin(GL_POINTS);
				glColor3f(1.0, 0, 0);
				glVertex3f(v.x, v.y, v.z);
			glEnd();
		}
	}
}