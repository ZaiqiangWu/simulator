#include "primitive.h"

BBox Primitive::get_bbox() const
{
	BBox bbox(vertices[v0]);
	bbox.expand(vertices[v1]);
	bbox.expand(vertices[v2]);
	return bbox;
}

bool Primitive::intersect(const glm::vec3& point) const
{
	//use normal or barycentric coordinates
	glm::vec3 side1, side2, normalface;
	side1 = vertices[v1] - vertices[v0];
	side2 = vertices[v2] - vertices[v0];
	normalface = glm::cross(side1, side2);
	

	glm::vec3 tem = point - vertices[v0];
	
	if (glm::dot(tem, normalface) > 0)
		return true;

	return false;
}
