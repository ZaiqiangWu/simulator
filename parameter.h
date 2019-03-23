#pragma once


class Parameter
{
public:
	Parameter();
	~Parameter();
	Parameter(unsigned int _NUM_ADJFACE,
	unsigned int _NUM_NEIGH1,
	unsigned int _NUM_NEIGH2,
	float _damp,
	float _mass,
	float _dt ,
	float _spring_structure,
	float _spring_bend);


public:
	unsigned int NUM_PER_VERTEX_ADJ_FACES;
	unsigned int NUM_PER_VERTEX_SPRING_STRUCT;
	unsigned int NUM_PER_VERTEX_SPRING_BEND;
	float damp;
	float mass;
	float dt ;
	float spring_structure;
	float spring_bend;
};

extern Parameter sim_parameter;


