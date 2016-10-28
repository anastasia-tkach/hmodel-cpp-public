#pragma once
#include "cudax/cuda_glm.h"
#include <vector>
#include <iostream>

class Model;

struct Outline {
	glm::vec2 t1;
	glm::vec2 t2;
	glm::ivec2 indices;
	glm::vec2 start;
	glm::vec2 end;
	size_t block;
};

struct Outline3D {
	glm::vec3 start;
	glm::vec3 end;
	glm::ivec2 indices;
	size_t block;
};

class OutlineFinder {
public:
	std::vector<Outline3D> outline3D;
	Model * const model;

	OutlineFinder(Model * _model);

	void print_outline(const std::vector<Outline> & outline);

	void print_outline3D(const std::vector<Outline3D> & outline);

	Outline crop_outline_segment(const glm::dvec3 & c1, const glm::dvec3 & c2, double r1, double r2, const glm::dvec2 & t, Outline outline);

	void adjust_fingers_outline(std::vector<Outline> & outline);
		
	std::vector<Outline3D> find_3D_outline(const std::vector<Outline> & outline);
	
	void find_outline();

	void write_outline(); 

	void compute_projections_outline(const std::vector<glm::vec3> & centers, const std::vector<float> & radii, const std::vector<glm::vec3> & points, const glm::vec3 & camera_ray);
};
