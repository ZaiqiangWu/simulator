#include "load_obj.h"
#include <vector>
#include <map>

class Matrix
{
public:
	Matrix() {};
	~Matrix();
	void Insert_Matrix(unsigned int i, unsigned int j, unsigned int k, vector<pair<unsigned int,unsigned int>> &value_inline);

private:
	map<pair<unsigned int,unsigned int>,unsigned int>  mat;

};

//struct Spring
//{
//	unsigned int p1,p2;
//	float origial_length;
//	float stiffness;
//};

class Springs
{
public:
	Springs();
	~Springs();
	Springs(Obj& cloth);  //������������boundary and cloth���ɾ���ϵ��һ��


public:
	//vector<Spring> springs;
	vector<vector<unsigned int>> neigh1;   //�洢ÿ���������һ��������Ϣ(�洢�������),�� structure spring
	vector<vector<unsigned int>> neigh2;   //�洢ÿ��������ж���������Ϣ(�洢�������),�� bend spring

private:
	//void add_spring(float stiffness,vector<glm::vec4>& vertices,unsigned int p1,unsigned int p2);
	bool exist(const vector<unsigned int>& array, const unsigned int val);  

};