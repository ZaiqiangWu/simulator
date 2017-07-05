#include "stdafx.h"
#include "parameter.h"

Parameter sim_parameter(    //��Ҫ�޸Ĳ�����ͬʱ��Ҫ�޸�GPU�˲�����verlet.cu��,��ʱδͬ��
	20,                    //NUM_ADJFACE
	20,                    //NUM_NEIGH1
	20,                    //NUM_NEIGH2
	-0.0125f,              //damp
	0.3f,                  //mass
	1.0f /50.0f,           //dt
	30.0,                  //pring_structure
	50.0                   //spring_bend
	);

Parameter::~Parameter()
{}
Parameter::Parameter(unsigned int _NUM_ADJFACE,
	unsigned int _NUM_NEIGH1,
	unsigned int _NUM_NEIGH2,
	float _damp,
	float _mass,
	float _dt ,
	float _spring_structure,
	float _spring_bend):NUM_ADJFACE(_NUM_ADJFACE),
						NUM_NEIGH1(_NUM_NEIGH1),
						NUM_NEIGH2(_NUM_NEIGH2),
						damp(_damp),
						mass(_mass),
						dt(_dt),
						spring_structure(_spring_structure),
						spring_bend(_spring_bend)
{
}


