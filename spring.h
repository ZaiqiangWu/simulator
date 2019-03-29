#pragma once
#include "Mesh.h"
#include <set>
#include <vector>
#include <map>

//simplified spring
struct s_spring      
{
	int end;  //һ�˵ĵ������
	float original;   //����ԭ��
};

class Springs
{
public:
	~Springs();
	Springs(Mesh* spring_obj);  //������������boundary and cloth���ɾ���ϵ��һ��
	void draw(Mesh* spring_obj);
	void serialize_structure_spring(vector<s_spring>& cpu_neigh1);  // 2D->1D
	void serialize_bend_spring(vector<s_spring>& cpu_neigh2);  // 2D->1D

private:
	vector<pair<unsigned int,unsigned int>> cloth_boundary_springs;   //ֻ����pair(1,2)
	vector<pair<unsigned int,unsigned int>> boundary_boundary_springs;   //Ӧ���Ѿ�����pair(1,2) && pair(2,1)
	set<pair<unsigned int,unsigned int>> boundary;
	vector<vector<unsigned int>> neigh1;   //�洢ÿ���������һ��������Ϣ(�洢�������),�� structure spring
	vector<vector<unsigned int>> neigh2;   //�洢ÿ��������ж���������Ϣ(�洢�������),�� bend spring

	vector<vector<s_spring>> neigh1_spring;   //�洢ÿ���������һ��������Ϣ(�洢�������)+ ԭ��,�� structure spring
	vector<vector<s_spring>> neigh2_spring;   //�洢ÿ��������ж���������Ϣ(�洢�������)+ + ԭ��,�� bend spring
	map<pair<unsigned int, unsigned int>,float> bend_spring_length;  //�洢�������������ζԽǶ�������+�����ľ���

private:
	//void ad spring(float stiffness,vector<glm::vec4>& vertices,unsigned int p1,unsigned int p2);
	void create_neigh(Mesh* spring_obj);
	void create_neigh_spring(Mesh* spring_obj);
	bool exist(const vector<unsigned int>& array, const unsigned int val);
	void get_cloth_boundary_spring(Mesh* spring_obj);
	void get_boundary_boundary_spring(Mesh* spring_obj);
};