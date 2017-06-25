#include "stdafx.h"
#include "spring.h"
#include <string>
#include <algorithm>
#include <iostream>
using namespace std;

float spring_structure = 30.0;
float spring_bend = 50.0;

Matrix::~Matrix()
{
}


void Matrix::Insert_Matrix(
	unsigned int i,
	unsigned int j,
	unsigned int k,
	vector<pair<unsigned int,unsigned int>> &value_inline)
{
	map<pair<unsigned int,unsigned int>,unsigned int>::iterator ite1 = mat.find(make_pair(i,j));
	map<pair<unsigned int,unsigned int>,unsigned int>::iterator ite2 = mat.find(make_pair(j,i));
	if(mat.end()!=ite1)
		{value_inline.push_back(make_pair(ite1->second,k));return;}
	if(mat.end()!=ite2)
		{value_inline.push_back(make_pair(ite2->second,k));return;}

	mat.insert(make_pair(make_pair(i,j),k));

}

Springs::~Springs()
{}

bool Springs::exist(const vector<unsigned int>& array, const unsigned int val)
{
	if(array.end() == find(array.begin(),array.end(),val))
		return false;
	else 
		return true;
}

Springs::Springs(Obj& cloth)
{
	cout << "build springs" << endl;
	//create neigh1 for each vertex
	neigh1.resize(cloth.uni_vertices.size());
	for(int i=0;i<cloth.faces.size();i++)
	{
		unsigned int f[3];
		for(int j=0;j<3;j++)
		{
			f[j] = cloth.faces[i].vertex_index[j];
		}
		if(!exist(neigh1[f[0]],f[1]))   //去掉neighbour中重复的邻接点
			neigh1[f[0]].push_back(f[1]);
		if(!exist(neigh1[f[0]],f[2]))
			neigh1[f[0]].push_back(f[2]);

		if(!exist(neigh1[f[1]],f[0]))
			neigh1[f[1]].push_back(f[0]);
		if(!exist(neigh1[f[1]],f[2]))
			neigh1[f[1]].push_back(f[2]);

		if(!exist(neigh1[f[2]],f[0]))
			neigh1[f[2]].push_back(f[0]);
		if(!exist(neigh1[f[2]],f[1]))
			neigh1[f[2]].push_back(f[1]);
	}

	//create neigh2 for each vertex
	neigh2.resize(cloth.uni_vertices.size());
	Matrix NR;   //Neighbour Relation
	vector<pair<unsigned int,unsigned int>> point_inline;  //存储两个共边三角形对角顶点索引

	for(int i=0;i<cloth.faces.size();i++)
	{
		unsigned int f[3];
		for(int j=0;j<3;j++)
		{
			f[j] = cloth.faces[i].vertex_index[j];
		}

		NR.Insert_Matrix(f[0],f[1],f[2],point_inline);
		NR.Insert_Matrix(f[0],f[2],f[1],point_inline);
		NR.Insert_Matrix(f[1],f[2],f[0],point_inline);
	}

	for (int i = 0; i<point_inline.size(); i++) 
	{
		unsigned int fir = point_inline[i].first;
		unsigned int sec = point_inline[i].second;

		neigh2[fir].push_back(sec);
		neigh2[sec].push_back(fir);
	}
	cout << "springs build successfully!" << endl;

}