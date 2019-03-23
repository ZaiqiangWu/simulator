#pragma once
#include "Mesh.h"
#include <set>
#include <vector>
#include <map>

class Matrix
{
public:
	Matrix() {};
	Matrix(Mesh* _obj):obj(_obj) {};
	~Matrix();
	void Insert_Matrix(unsigned int i, unsigned int j, unsigned int k, 
		vector<pair<unsigned int,unsigned int>> &value_inline, map<pair<unsigned int, unsigned int>, float>& spring_length);

private:
	map<pair<unsigned int,unsigned int>,unsigned int>  mat;
	Mesh* obj;

};

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
	Springs(Mesh* cloth);  //������������boundary and cloth���ɾ���ϵ��һ��
	void draw();


public:
	s_spring* cuda_neigh1;  //��ά����תΪһά����
	s_spring* cuda_neigh2;
	unsigned int NUM_NEIGH1 ;    //һ������������Ŀ����С��NUM_NEIGH1����MAX_INT��β
	unsigned int NUM_NEIGH2 ;    //��������������Ŀ����С��NUM_NEIGH2����MAX_INT��β

private:
	vector<pair<unsigned int,unsigned int>> cloth_boundary_springs;   //ֻ����pair(1,2)
	vector<pair<unsigned int,unsigned int>> boundary_boundary_springs;   //Ӧ���Ѿ�����pair(1,2) && pair(2,1)
	set<pair<unsigned int,unsigned int>> boundary;
	vector<vector<unsigned int>> neigh1;   //�洢ÿ���������һ��������Ϣ(�洢�������),�� structure spring
	vector<vector<unsigned int>> neigh2;   //�洢ÿ��������ж���������Ϣ(�洢�������),�� bend spring

	vector<vector<s_spring>> neigh1_spring;   //�洢ÿ���������һ��������Ϣ(�洢�������)+ ԭ��,�� structure spring
	vector<vector<s_spring>> neigh2_spring;   //�洢ÿ��������ж���������Ϣ(�洢�������)+ + ԭ��,�� bend spring
	Mesh* spring_obj;
	map<pair<unsigned int, unsigned int>,float> bend_spring_length;  //�洢�������������ζԽǶ�������+�����ľ���

private:
	//void ad spring(float stiffness,vector<glm::vec4>& vertices,unsigned int p1,unsigned int p2);
	void create_neigh();
	void create_neigh_spring();
	bool exist(const vector<unsigned int>& array, const unsigned int val);
	bool cuda_neigh();
	void get_cloth_boundary_spring();
	void get_boundary_boundary_spring();
};