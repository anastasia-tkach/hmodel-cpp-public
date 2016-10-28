#pragma once
#include <fstream>
#include <vector>
#include "cudax/cuda_glm.h"

void read_float_matrix(std::string data_path, std::string name, std::vector<glm::vec3> & input) {
	FILE *fp = fopen((data_path + name + ".txt").c_str(), "r");
	int N;
	fscanf(fp, "%d", &N);
	for (int j = 0; j < N; ++j) {
		glm::vec3 v;
		fscanf(fp, "%f%f%f", &v[0], &v[1], &v[2]);
		input.push_back(v);
	}
	fclose(fp);
}

void read_float_vector(std::string data_path, std::string name, std::vector<float> & input) {
	FILE *fp = fopen((data_path + name + ".txt").c_str(), "r");
	int N;
	fscanf(fp, "%d", &N);
	for (int i = 0; i < N; ++i) {
		float a;
		fscanf(fp, "%f", &a);
		input.push_back(a);
	}
	fclose(fp);
}

void read_int_matrix(std::string data_path, std::string name, std::vector<glm::ivec3> & input) {
	FILE *fp = fopen((data_path + name + ".txt").c_str(), "r");
	int N;
	fscanf(fp, "%d", &N);
	for (int j = 0; j < N; ++j) {
		glm::ivec3 v;
		fscanf(fp, "%d%d%d", &v[0], &v[1], &v[2]);
		input.push_back(v);
	}
	fclose(fp);
}

/*void write_float_vector(std::string name, thrust::host_vector<float> output) {
	std::ofstream output_file;
	output_file.open(data_path + name + ".txt");
	for (size_t i = 0; i < output.size(); i++) {
		output_file << output[i] << " ";
	}
	output_file.close();
}*/

void write_floats(std::string data_path, std::string name, float * output, size_t size) {
	std::ofstream output_file;
	output_file.open(data_path + name + ".txt");
	for (size_t i = 0; i < size; i++) {
		output_file << output[i] << " ";
	}
	output_file.close();
}

