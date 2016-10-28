#pragma once
#include <vector>
#include <iostream>
#include "cudax/cuda_glm.h"
#include "util/MathUtils.h"
#include "tracker/ForwardDeclarations.h"
#include "tracker/DataStructure/CustomJointInfo.h"
#include "OutlineFinder.h"
#include "ModelSerializer.h"
#include "ModelSemantics.h"

#include "opencv2/core/core.hpp"       ///< cv::Mat
#include "opencv2/highgui/highgui.hpp" ///< cv::imShow
#include "opencv2/imgproc/imgproc.hpp"

enum AxisType {
	ROTATION_AXIS = 0,
	TRANSLATION_AXIS = 1,
};

struct Phalange {
	string name;
	int parent_id;
	std::vector<size_t> children_ids;
	std::vector<size_t> kinematic_chain;
	Mat4f init_local;
	Mat4f local;
	Mat4f global;
	float length;
	float radius1;
	float radius2;

	size_t center_id;
	std::vector<size_t> attachments;
	std::vector<Vec3d> offsets;

	size_t segment_id; //temporary
};

struct Dof {
	AxisType type;
	Eigen::Vector3f axis;
	size_t phalange_id;
	float min;
	float max;

	size_t joint_id; //temporary
};

struct Tangent {
	glm::vec3 v1; glm::vec3 v2; glm::vec3 v3; glm::vec3 n;
	glm::vec3 u1; glm::vec3 u2; glm::vec3 u3; glm::vec3 m;
	Tangent() {
		v1 = glm::vec3(0, 0, 0); v2 = glm::vec3(0, 0, 0); v3 = glm::vec3(0, 0, 0);
		u1 = glm::vec3(0, 0, 0); u2 = glm::vec3(0, 0, 0); u3 = glm::vec3(0, 0, 0);
	}
};

class Model {
public:
	int num_tangent_fields = 8;
	int num_outline_fields = 3;
	int max_num_outlines = 200;

	std::vector<glm::vec3> centers;
	std::vector<float> radii;
	std::vector<glm::ivec3> blocks;
	std::vector<Tangent> tangent_points;

	float * host_pointer_centers;
	float * host_pointer_radii;
	int * host_pointer_blocks;
	float * host_pointer_tangent_points;
	float * host_pointer_outline;
	int * host_pointer_blockid_to_jointid_map;

	glm::vec3 camera_ray;

	std::vector<int> palm_block_indices;
	std::vector<std::vector<int>> fingers_block_indices;
	std::vector<size_t> fingers_base_centers;
	std::vector<glm::ivec2> crop_indices_thumb;
	std::vector<glm::ivec2> limit_indices_thumb;
	std::vector<glm::ivec2> crop_indices_fingers;
	std::vector<glm::ivec2> limit_indices_fingers;
	std::vector<int> adjuct_block_indices_fingers;

	OutlineFinder outline_finder;
	ModelSerializer serializer;
	ModelSemantics semantics;
	KinematicChain kinematic_chain;
	JointTransformations transformations;
	int * rendered_pixels;
	float * rendered_points;
	int * rendered_block_ids;
	int num_rendered_points;

	std::vector<float> theta;
	std::vector<Phalange> phalanges;
	std::vector<Dof> dofs;
	std::map<std::string, size_t> phalanges_name_to_id_map;
	std::map<std::string, size_t> centers_name_to_id_map;
	std::map<size_t, size_t> jointid_to_centerid_map;
	std::map<size_t, size_t> jointid_to_phalangeid_map;
	std::vector<int> blockid_to_jointid_map;

	cv::Mat silhouette_texture;
	cv::Mat real_color;

	enum Type { HTRACK, HMODEL_OLD, HMODEL} model_type;
	enum UserName { ANASTASIA, ANDRII, ANONYMOUS } user_name;
	std::string data_path;
	
	Model();
	~Model();

	void init(int user_name, std::string data_path);

	void initialize_offsets();

	void reindex();

	void compute_outline();

	void compute_tangent_point(const glm::vec3 & camera_ray, glm::vec3 & c1, const glm::vec3 & c2, const glm::vec3 & c3, float r1, float r2, float r3,
		glm::vec3 & v1, glm::vec3 & v2, glm::vec3 & v3, glm::vec3 & u1, glm::vec3 & u2, glm::vec3 & u3, glm::vec3 & n, glm::vec3 & m);

	void compute_tangent_points();

	void print_model();

	void render_outline();

	int compute_rendered_indicator_helper(int row, int col, Vector3 p, int block, int num_rendered_points, bool display,
		cv::Mat & image, const cv::Mat & sensor_silhouette, Camera * camera);

	void compute_rendered_indicator(const cv::Mat & sensor_silhouette, Camera * camera);

	void write_model(std::string data_path, int frame_number = 0);

	void load_model_from_file();

	void resize_model(float uniform_scaling_factor, float width_scaling_factor, float thickness_scaling_factor);

	void update_centers();

	Matrix_3xN jacobian(const Vector3 & s, size_t id);

	void move(const std::vector<float> & theta);

	void transform_joints(const std::vector<float> & theta);

	void Model::update(Phalange & phalange);

	void Model::translate(Phalange & phalange, const Vec3f & t);

	void Model::rotate(Phalange & phalange, const Vec3f &axis, float angle);

	const std::vector<float>& Model::get_theta();

	std::vector<float> Model::get_updated_parameters(const vector<float> & theta, const vector<float> &delta_theta);

	Vec3f get_palm_center();

	float get_palm_length();

	Mat3f build_rotation_matrix(Vec3f euler_angles);

	void manually_adjust_initial_transformations();
};