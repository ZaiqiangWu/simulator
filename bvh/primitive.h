#pragma once
#include "BBox.h"
#include <glm/glm.hpp>
#define uint unsigned int

//here primitive refer to triangle
class Primitive      
{
public:
	Primitive() {}
	Primitive(const glm::vec3* _vertices, const glm::vec3* _d_vertices, const uint _v0, const uint _v1, const uint _v2)
		:vertices(_vertices),  d_vertices(_d_vertices), v0(_v0), v1(_v1), v2(_v2) {}
	/**
	* Get the world space bounding box of the primitive.
	* \return world space bounding box of the primitive
	*/
	BBox get_bbox() const ;
	__device__ BBox d_get_bbox() const
	{
		BBox bbox(d_vertices[v0]);
		bbox.expand(d_vertices[v1]);
		bbox.expand(d_vertices[v2]);

		//���ŷ��߷����ʵ���չ������������Ƭ,���ڸ�Ϊ��ĸ��Է��߷���
		float depth = 0.01;
		glm::vec3 n = d_get_normal();
		bbox.expand(d_vertices[v0] - depth*n);
		bbox.expand(d_vertices[v1] - depth*n);
		bbox.expand(d_vertices[v2] - depth*n);

		return bbox;
	}

	/**
	* Check if the given point intersects with the primitive, no intersection
	* information is stored
	* \return true if the given point intersects with the primitive,
	false otherwise
	*/
	bool intersect(const glm::vec3& point) const;

	glm::vec3 get_normal() const;
	__device__ glm::vec3 d_get_normal() const
	{
		glm::vec3 side1, side2, normalface;
		side1 = d_vertices[v1] - d_vertices[v0];
		side2 = d_vertices[v2] - d_vertices[v0];
		normalface = glm::cross(side1, side2);
		return glm::normalize(normalface);
	}

 __device__
		bool d_intersect(const glm::vec3& point, float &dist, glm::vec3 &normal) const
		{
			//use normal or barycentric coordinates
			glm::vec3 side1, side2, normalface;
			side1 = d_vertices[v1] - d_vertices[v0];
			side2 = d_vertices[v2] - d_vertices[v0];
			normalface = glm::cross(side1, side2);
			normal = glm::normalize(normalface);

			glm::vec3 tem = point - d_vertices[v0];
			dist = glm::dot(tem, normal);
			if (dist > 0)
				return false;

			return true;
		}
public:
	const glm::vec3* vertices;  //for device, ptr to body vertices
	const glm::vec3*  d_vertices;
	unsigned int morton_code;
	uint v0;
	uint v1;
	uint v2;
};
