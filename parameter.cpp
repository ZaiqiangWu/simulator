
#include "parameter.h"

Parameter sim_parameter(    //若要修改参数，同时需要修改GPU端参数（verlet.cu）,暂时未同步
	20,                    //NUM_PER_VERTEX_ADJ_FACES
	20,                    //NUM_PER_VERTEX_SPRING_STRUCT
	20,                    //NUM_PER_VERTEX_SPRING_BEND
	-0.0125f,              //damp
	0.3f,                  //mass
	1.0f /50.0f,           //dt
	30.0,                  //pring_structure
	1.0                  //spring_bend
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
	float _spring_bend):NUM_PER_VERTEX_ADJ_FACES(_NUM_ADJFACE),
						NUM_PER_VERTEX_SPRING_STRUCT(_NUM_NEIGH1),
						NUM_PER_VERTEX_SPRING_BEND(_NUM_NEIGH2),
						damp(_damp),
						mass(_mass),
						dt(_dt),
						spring_structure(_spring_structure),
						spring_bend(_spring_bend)
{
}


