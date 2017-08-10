#pragma once
#include "load_obj.h"
#include <set>
#include <vector>
#include <map>
#include "parameter.h"


class Matrix
{
public:
	Matrix() {};
	~Matrix();
	void Insert_Matrix(unsigned int i, unsigned int j, unsigned int k, vector<pair<unsigned int,unsigned int>> &value_inline);

private:
	map<pair<unsigned int,unsigned int>,unsigned int>  mat;

};


class Springs
{
public:
	Springs();
	~Springs();
	Springs(Obj* cloth);  //������������boundary and cloth���ɾ���ϵ��һ��


public:
	unsigned int* cuda_neigh1;  //��ά����תΪһά����
	unsigned int* cuda_neigh2;
	unsigned int NUM_NEIGH1;    //һ������������Ŀ����С��NUM_NEIGH1����MAX_INT��β
	unsigned int NUM_NEIGH2;    //��������������Ŀ����С��NUM_NEIGH2����MAX_INT��β

private:
	vector<pair<unsigned int,unsigned int>> cloth_boundary_springs;   //ֻ����pair(1,2)
	vector<pair<unsigned int,unsigned int>> boundary_boundary_springs;   //Ӧ���Ѿ�����pair(1,2) && pair(2,1)
	set<pair<unsigned int,unsigned int>> boundary;
	vector<vector<unsigned int>> neigh1;   //�洢ÿ���������һ��������Ϣ(�洢�������),�� structure spring
	vector<vector<unsigned int>> neigh2;   //�洢ÿ��������ж���������Ϣ(�洢�������),�� bend spring
	Obj* spring_obj;

private:
	//void ad spring(float stiffness,vector<glm::vec4>& vertices,unsigned int p1,unsigned int p2);
	bool exist(const vector<unsigned int>& array, const unsigned int val);
	bool cuda_neigh();
	void get_cloth_boundary_spring();
	void get_boundary_boundary_spring();
};