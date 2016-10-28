#include "Model.h"
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <Eigen/Geometry>
#include "DataLoader.h"
#include "ModelSemantics.h";
#include "tracker/OpenGL/DebugRenderer/DebugRenderer.h"
#include "tracker/Data/Camera.h"

Model::Model() :outline_finder(this), serializer(this), semantics(this) {
	centers = std::vector<glm::vec3>();
	radii = std::vector<float>();
	blocks = std::vector<glm::ivec3>();

	theta = std::vector<float>(num_thetas, 0);
	phalanges = std::vector<Phalange>(num_phalanges + 1, Phalange());
	dofs = std::vector<Dof>(num_thetas, Dof());

	host_pointer_centers = NULL;
	host_pointer_radii = NULL;
	host_pointer_blocks = NULL;
	host_pointer_tangent_points = NULL;
	host_pointer_outline = NULL;
	host_pointer_blockid_to_jointid_map = NULL;

	camera_ray = glm::vec3(0, 0, 1);

	rendered_pixels = new int[upper_bound_num_rendered_outline_points];
	rendered_points = new float[3 * upper_bound_num_rendered_outline_points];
	rendered_block_ids = new int[upper_bound_num_rendered_outline_points];
}

Model::~Model() {
	if (host_pointer_centers) delete[] host_pointer_centers;
	if (host_pointer_radii) delete[] host_pointer_radii;
	if (host_pointer_blocks) delete[] host_pointer_blocks;
	if (host_pointer_tangent_points) delete[] host_pointer_tangent_points;
	if (host_pointer_outline) delete[] host_pointer_outline;
	if (host_pointer_blockid_to_jointid_map) delete[] host_pointer_blockid_to_jointid_map;
	delete[] rendered_pixels;
	delete[] rendered_points;
	delete[] rendered_block_ids;
}

void Model::init(int user_name, std::string data_path) {
	model_type = HMODEL;
	this->user_name = (UserName)user_name;
	this->data_path = data_path;	

	if (model_type == HMODEL_OLD || model_type == HMODEL) {
		load_model_from_file();
		semantics.setup_topology();
		semantics.setup_outline();
		semantics.setup_dofs();
		semantics.setup_phalanges();
		move(std::vector<float>(num_thetas, 0));
		initialize_offsets();
	}
	semantics.setup_kinematic_chain();

	for (size_t i = 0; i < num_phalanges - 1; i++) jointid_to_phalangeid_map[phalanges[i].segment_id] = i;
	cout << "minus one in jointid_to_phalangeid_map" << endl;
	jointid_to_phalangeid_map[7] = 3;
	jointid_to_phalangeid_map[11] = 6;
	jointid_to_phalangeid_map[15] = 9;
	jointid_to_phalangeid_map[19] = 12;
	jointid_to_phalangeid_map[23] = 15;

	/*for (size_t i = 0; i < num_phalanges; i++) {
		Phalange phalange = phalanges[i];
		cout << "id = " << i << endl;
		cout << "init_local: " << endl << phalange.init_local << endl;
		cout << endl << endl;
		}*/
	//print_model();
}

void Model::initialize_offsets() {

	for (size_t i = 0; i < num_phalanges; i++) {
		//cout << endl << i << endl;
		if (phalanges[i].attachments.empty()) continue;
		phalanges[i].offsets.resize(phalanges[i].attachments.size());
		for (size_t j = 0; j < phalanges[i].attachments.size(); j++) {
			Vec3f c = Eigen::Vector3f(centers[phalanges[i].attachments[j]][0], centers[phalanges[i].attachments[j]][1], centers[phalanges[i].attachments[j]][2]);
			Mat3d inverse = (phalanges[i].global.block(0, 0, 3, 3).cast<double>()).inverse();
			Vec3d offset = inverse *  (c - phalanges[i].global.block(0, 3, 3, 1)).cast<double>();
			phalanges[i].offsets[j] = offset;
			//phalanges[i].offsets[j] = phalanges[i].global.block(0, 0, 3, 3).inverse() * (c - phalanges[i].global.block(0, 3, 3, 1));

			//cout << "attachment_id = " << phalanges[i].attachments[j] << endl;
			//cout << " center = " << centers[phalanges[i].center_id][0] << ", " << centers[phalanges[i].center_id][1] << ", " << centers[phalanges[i].center_id][2] << endl;
			//cout << "R = " << phalanges[i].global.block(0, 0, 3, 3) << endl;
			//cout << " offset = " << phalanges[i].offsets[j][0] << ", " << phalanges[i].offsets[j][1] << ", " << phalanges[i].offsets[j][2] << endl;
		}
	}
}

void Model::reindex() {
	for (size_t i = 0; i < blocks.size(); i++) {
		if (blocks[i][2] == RAND_MAX) {
			if (radii[blocks[i][0]] < radii[blocks[i][1]]) {
				std::swap(blocks[i][0], blocks[i][1]);
			}
		}
		else {
			if (radii[blocks[i][0]] <= radii[blocks[i][1]] && radii[blocks[i][1]] <= radii[blocks[i][2]]) {
				blocks[i] = glm::ivec3(blocks[i][2], blocks[i][1], blocks[i][0]);
			}
			if (radii[blocks[i][0]] <= radii[blocks[i][2]] && radii[blocks[i][2]] <= radii[blocks[i][1]]) {
				blocks[i] = glm::ivec3(blocks[i][1], blocks[i][2], blocks[i][0]);
			}
			if (radii[blocks[i][1]] <= radii[blocks[i][0]] && radii[blocks[i][0]] <= radii[blocks[i][2]]) {
				blocks[i] = glm::ivec3(blocks[i][2], blocks[i][0], blocks[i][1]);
			}
			if (radii[blocks[i][1]] <= radii[blocks[i][2]] && radii[blocks[i][2]] <= radii[blocks[i][0]]) {
				blocks[i] = glm::ivec3(blocks[i][0], blocks[i][2], blocks[i][1]);
			}
			if (radii[blocks[i][2]] <= radii[blocks[i][0]] && radii[blocks[i][0]] <= radii[blocks[i][1]]) {
				blocks[i] = glm::ivec3(blocks[i][1], blocks[i][0], blocks[i][2]);
			}
		}
	}
}

void Model::compute_outline() {
	outline_finder.find_outline();
	//outline_finder.write_outline();
	//outline_finder.compute_projections_outline(centers, radii, data_points, camera_ray);
}

void Model::compute_tangent_point(const glm::vec3 & camera_ray, glm::vec3 & c1, const glm::vec3 & c2, const glm::vec3 & c3, float r1, float r2, float r3,
	glm::vec3 & v1, glm::vec3 & v2, glm::vec3 & v3, glm::vec3 & u1, glm::vec3 & u2, glm::vec3 & u3, glm::vec3 & n, glm::vec3 & m) {

	/*std::cout << "c1 = (" << c1[0] << ", " << c1[1] << ", " << c1[2] << ")" << std::endl;
	std::cout << "c2 = (" << c2[0] << ", " << c2[1] << ", " << c2[2] << ")" << std::endl;
	std::cout << "c3 = (" << c3[0] << ", " << c3[1] << ", " << c3[2] << ")" << std::endl;
	std::cout << "r1 = " << r1 << std::endl;
	std::cout << "r2 = " << r2 << std::endl;
	std::cout << "r3 = " << r3 << std::endl;*/

	float epsilon = 1e-2;
	if (r1 - r2 < epsilon && r1 - r3 < epsilon) {
		n = cross(c1 - c2, c1 - c3); n = n / length(n);
		if (dot(camera_ray, n) < 0) {
			m = -n;
		}
		else {
			m = n;
			n = -m;
		}
		v1 = c1 + r1 * n;
		v2 = c2 + r2 * n;
		v3 = c3 + r3 * n;
		u1 = c1 + r1 * m;
		u2 = c2 + r2 * m;
		u3 = c3 + r3 * m;
		/*std::cout << "n = (" << n[0] << ", " << n[1] << ", " << n[2] << ")" << std::endl;
		std::cout << "v1 = (" << v1[0] << ", " << v1[1] << ", " << v1[2] << ")" << std::endl;
		std::cout << "v2 = (" << v2[0] << ", " << v2[1] << ", " << v2[2] << ")" << std::endl;
		std::cout << "v3 = (" << v3[0] << ", " << v3[1] << ", " << v3[2] << ")" << std::endl;
		std::cout << std::endl << std::endl;*/
		return;
	}

	glm::vec3 z12 = c1 + (c2 - c1) * r1 / (r1 - r2);
	glm::vec3 z13 = c1 + (c3 - c1) * r1 / (r1 - r3);

	glm::vec3 l = (z12 - z13) / length(z12 - z13);
	float projection = dot(c1 - z12, l);
	glm::vec3 z = z12 + projection * l;

	float eta = length(c1 - z);
	float sin_beta = r1 / eta;
	float nu = sqrt(eta * eta - r1 * r1);
	float cos_beta = nu / eta;

	glm::vec3 f = (c1 - z) / eta;
	glm::vec3 h = cross(l, f);
	normalize(h);

	glm::vec3 g;

	g = sin_beta * h + cos_beta * f;
	v1 = z + nu * g;
	n = (v1 - c1) / length(v1 - c1);
	v2 = c2 + r2 * n;
	v3 = c3 + r3 * n;

	g = -sin_beta * h + cos_beta * f;
	u1 = z + nu * g;
	m = (u1 - c1) / length(u1 - c1);
	u2 = c2 + r2 * m;
	u3 = c3 + r3 * m;

	if (dot(camera_ray, n) > 0) {
		std::swap(v1, u1);
		std::swap(v2, u2);
		std::swap(v3, u3);
		std::swap(n, m);
	}
}

void Model::compute_tangent_points() {
	//cout << "centers.size() = " << centers.size() << endl;
	//cout << "radii.size() = " << radii.size() << endl;
	//cout << "blocks.size() = " << blocks.size() << endl;
	tangent_points = std::vector<Tangent>(blocks.size(), Tangent());
	for (size_t i = 0; i < blocks.size(); i++) {
		if (blocks[i][2] > centers.size()) continue;
		//cout << "c1 = " << centers[blocks[i][0]][0] << endl;
		//cout << "c2 = " << centers[blocks[i][1]][0] << endl;
		//cout << "c3 = " << centers[blocks[i][2]][0] << endl;
		compute_tangent_point(camera_ray, centers[blocks[i][0]], centers[blocks[i][1]], centers[blocks[i][2]],
			radii[blocks[i][0]], radii[blocks[i][1]], radii[blocks[i][2]],
			tangent_points[i].v1, tangent_points[i].v2, tangent_points[i].v3,
			tangent_points[i].u1, tangent_points[i].u2, tangent_points[i].u3,
			tangent_points[i].n, tangent_points[i].m);
		//std::cout << "in model: (" << tangent_points[i].v1[0] << ", " << tangent_points[i].v1[1] << ", " << tangent_points[i].v1[2] << " ); " << std::endl;
	}
}

void Model::print_model() {
	std::cout << "CENTERS" << std::endl;
	for (size_t i = 0; i < centers.size(); i++) {
		for (size_t j = 0; j < d; j++) {
			std::cout << centers[i][j] << " ";
		}
		std::cout << std::endl;
	}
	std::cout << "RADII" << std::endl;
	for (size_t i = 0; i < radii.size(); i++) {
		std::cout << radii[i] << std::endl;
	}
	std::cout << "BLOCKS" << std::endl;
	for (size_t i = 0; i < blocks.size(); i++) {
		for (size_t j = 0; j < d; j++) {
			if (blocks[i][j] < centers.size())
				std::cout << blocks[i][j] << " ";
		}
		std::cout << std::endl;
	}
	std::cout << "TANGENT POINTS" << std::endl;
	for (size_t i = 0; i < tangent_points.size(); i++) {
		if (blocks[i][2] > centers.size()) continue;
		std::cout << "block " << i << std::endl;
		std::cout << "	v1: "; for (size_t j = 0; j < d; j++) std::cout << tangent_points[i].v1[j] << " "; std::cout << std::endl;
		std::cout << "	v2: "; for (size_t j = 0; j < d; j++) std::cout << tangent_points[i].v2[j] << " "; std::cout << std::endl;
		std::cout << "	v3: "; for (size_t j = 0; j < d; j++) std::cout << tangent_points[i].v3[j] << " "; std::cout << std::endl;
		std::cout << "	u1: "; for (size_t j = 0; j < d; j++) std::cout << tangent_points[i].u1[j] << " "; std::cout << std::endl;
		std::cout << "	u2: "; for (size_t j = 0; j < d; j++) std::cout << tangent_points[i].u2[j] << " "; std::cout << std::endl;
		std::cout << "	u3: "; for (size_t j = 0; j < d; j++) std::cout << tangent_points[i].u3[j] << " "; std::cout << std::endl;
	}
}

void Model::render_outline() {
	DebugRenderer::instance().clear();
	std::vector<std::pair<Vector3, Vector3>> segments;
	std::vector<std::pair<Vector3, Vector3>> arc_endpoints;
	std::vector<Vector3> arc_centers;
	std::vector<float> arc_radii;
	for (size_t i = 0; i < outline_finder.outline3D.size(); i++) {
		glm::vec3 s = outline_finder.outline3D[i].start;
		glm::vec3 e = outline_finder.outline3D[i].end;
		if (outline_finder.outline3D[i].indices[1] != RAND_MAX) {
			segments.push_back(std::pair<Vector3, Vector3>(Vector3(s[0], s[1], s[2]), Vector3(e[0], e[1], e[2])));
		}
		else {
			glm::vec3 c = centers[outline_finder.outline3D[i].indices[0]];
			arc_centers.push_back(Vector3(c[0], c[1], c[2]));
			arc_radii.push_back(radii[outline_finder.outline3D[i].indices[0]]);
			arc_endpoints.push_back(std::pair<Vector3, Vector3>(Vector3(s[0], s[1], s[2]), Vector3(e[0], e[1], e[2])));
		}
	}
	DebugRenderer::instance().add_segments(segments, Vector3(0.673, 0.286, 0.406));
	DebugRenderer::instance().add_arcs(arc_endpoints, arc_centers, arc_radii, Vector3(0.673, 0.286, 0.406));
}

int Model::compute_rendered_indicator_helper(int row, int col, Vector3 p, int block, int num_rendered_points, bool display,
	cv::Mat & image, const cv::Mat & sensor_silhouette, Camera * camera) {

	row = camera->height() - row - 1;
	if (col < 1 || col >= camera->width() || row < 1 || row >= camera->height()) return num_rendered_points;

	if (sensor_silhouette.at<uchar>(row, col) == 255) return num_rendered_points;

	rendered_pixels[num_rendered_points] = row * camera->width() + col;
	rendered_points[3 * num_rendered_points] = p[0];
	rendered_points[3 * num_rendered_points + 1] = p[1];
	rendered_points[3 * num_rendered_points + 2] = p[2];
	rendered_block_ids[num_rendered_points] = block;

	num_rendered_points++;

	if (display) {
		image.at<cv::Vec3b>(cv::Point(2 * col - 1, 2 * row - 1)) = cv::Vec3b(200, 200, 0);
		image.at<cv::Vec3b>(cv::Point(2 * col - 1, 2 * row)) = cv::Vec3b(200, 200, 0);
		image.at<cv::Vec3b>(cv::Point(2 * col, 2 * row - 1)) = cv::Vec3b(200, 200, 0);
		image.at<cv::Vec3b>(cv::Point(2 * col, 2 * row)) = cv::Vec3b(200, 200, 0);
	}

	return num_rendered_points;
}

void Model::compute_rendered_indicator(const cv::Mat & sensor_silhouette, Camera * camera) {
	bool display = false;
	cv::Mat image;
	if (display) {
		cv::Scalar color = cv::Scalar(100, 0, 230);
		cv::Mat in[] = { sensor_silhouette, sensor_silhouette, sensor_silhouette };
		cv::merge(in, 3, image);
		cv::resize(image, image, cv::Size(640, 480));
	}

	num_rendered_points = 0;
	for (size_t i = 0; i < outline_finder.outline3D.size(); i++) {
		glm::vec3 s_glm = outline_finder.outline3D[i].start;
		glm::vec3 e_glm = outline_finder.outline3D[i].end;
		Vector3 s = Vector3(s_glm[0], s_glm[1], s_glm[2]);
		Vector3 e = Vector3(e_glm[0], e_glm[1], e_glm[2]);
		int block = outline_finder.outline3D[i].block;
		if (outline_finder.outline3D[i].indices[1] != RAND_MAX) {
			Vector2 s_image = camera->world_to_image(s);
			Vector2 e_image = camera->world_to_image(e);

			float x1 = s_image[0]; float y1 = s_image[1]; 
			float x2 = e_image[0]; float y2 = e_image[1];
			float X1 = s[0]; float Y1 = s[1]; float Z1 = s[2];
			float X2 = e[0]; float Y2 = e[1]; float Z2 = e[2];
			int num_samples = 1.2 * (e_image - s_image).norm();
			for (int n = 0; n < num_samples; n++) {
				float t = (float)n / (num_samples - 1);
				float x = x1 + t * (x2 - x1);
				float y = y1 + t * (y2 - y1);
				Vector3 p = Vector3(X1 + t * (X2 - X1), Y1 + t * (Y2 - Y1), Z1 + t * (Z2 - Z1));
				int col = (int)x;
				int row = (int)y;
				num_rendered_points = compute_rendered_indicator_helper(row, col, p, block, num_rendered_points, display, image, sensor_silhouette, camera);
			}
		}
		else {
			glm::vec3 c_glm = centers[outline_finder.outline3D[i].indices[0]];
			float r = radii[outline_finder.outline3D[i].indices[0]];
			Vector3 c = Vector3(c_glm[0], c_glm[1], c_glm[2]);
			Vector2 c_image = camera->world_to_image(c);
			Vector2 s_image = camera->world_to_image(s);
			float r_image = (c_image - s_image).norm();
			Vector3 v1 = s - c;
			Vector3 v2 = e - c;

			float phi;
			Vector2 u = Vector2(1, 0); Vector2 v = Vector2(0, 1);
			Vector3 U = Vector3(1, 0, 0); Vector3 V = Vector3(0, 1, 0);

			float alpha = atan2(v1[0], v1[1]);
			float beta = atan2(v2[0], v2[1]);
			if (beta > alpha) alpha = alpha + 2 * M_PI;
			int num_samples = 1.2 * (alpha - beta) * r_image;
			for (int n = 0; n < num_samples; n++) {
				phi = alpha + n * (beta - alpha) / (num_samples - 1);
				Vector2 x = c_image + r_image * (u * sin(phi) + v * cos(phi));
				Vector3 p = c + r * (U * sin(phi) + V * cos(phi));
				int col = (int)x[0];
				int row = (int)x[1];
				num_rendered_points = compute_rendered_indicator_helper(row, col, p, block, num_rendered_points, display, image, sensor_silhouette, camera);
			}
		}
	}
	if (display) cv::imshow("sensor_silhouette", image);	
	/*num_rendered_points = 0;
	for (int row = 0; row < rendered_silhouette.rows; ++row) {
		for (int col = 0; col < rendered_silhouette.cols; ++col) {
			if (rendered_silhouette.at<uchar>(row, col) != 255 && sensor_silhouette.at<uchar>(row, col) != 255) {
				rendered_indicator[num_rendered_points] = row * camera->width() + col;
				num_rendered_points++;
			}
		}
	}*/

}

void Model::write_model(std::string data_path, int frame_number) {

	std::ofstream centers_file;
	centers_file.open(data_path + "C-" + std::to_string(frame_number) + ".txt");
	centers_file << centers.size() << " ";
	for (size_t i = 0; i < centers.size(); i++) {
		for (size_t j = 0; j < 3; j++) {
			centers_file << centers[i][j] << " ";
		}
	}
	centers_file.close();

	std::ofstream radii_file;
	radii_file.open(data_path + "R-" + std::to_string(frame_number) + ".txt");
	radii_file << radii.size() << " ";
	for (size_t i = 0; i < radii.size(); i++) {
		radii_file << radii[i] << " ";
	}
	radii_file.close();

	std::ofstream blocks_file;
	blocks_file.open(data_path + "B-" + std::to_string(frame_number) + ".txt");
	blocks_file << blocks.size() << " ";
	for (size_t i = 0; i < blocks.size(); i++) {
		for (size_t j = 0; j < 3; j++) {
			blocks_file << blocks[i][j] << " ";
		}
	}
	blocks_file.close();

	std::ofstream theta_file;
	theta_file.open(data_path + "T-" + std::to_string(frame_number) + ".txt");
	theta_file << theta.size() << " ";
	for (size_t i = 0; i < theta.size(); i++) {
		theta_file << theta[i] << " ";
	}
	theta_file.close();

	std::ofstream transformations_file;
	transformations_file.open(data_path + "I-" + std::to_string(frame_number) + ".txt");
	transformations_file << num_phalanges << endl;
	for (size_t i = 0; i < num_phalanges; i++) {
		for (size_t u = 0; u < 4; u++) {
			for (size_t v = 0; v < 4; v++) {
				//transformations_file << phalanges[i].global(u, v) << " ";
				transformations_file << phalanges[i].init_local(u, v) << " ";
			}
		}
	}
	transformations_file.close();
}

void Model::load_model_from_file() {
	blocks.clear();

	std::string model_folder_path = "models/anastasia/";
	read_float_matrix(data_path + model_folder_path, "C", centers);
	read_float_vector(data_path + model_folder_path, "R", radii);
	read_int_matrix(data_path + model_folder_path, "B", blocks);

	// Read initial transformations

	FILE *fp = fopen((data_path + model_folder_path + "I.txt").c_str(), "r");
	int N;
	fscanf(fp, "%d", &N);
	for (int i = 0; i < N; ++i) {
		phalanges[i].init_local = Mat4f::Zero(d + 1, d + 1);
		for (size_t u = 0; u < d + 1; u++) {
			for (size_t v = 0; v < d + 1; v++) {
				fscanf(fp, "%f", &phalanges[i].init_local(v, u));
			}
		}
	}
	fclose(fp);

	/*for (size_t i = 0; i < centers.size(); i++) {
		centers[i] += glm::vec3(0, -70, 400);
		}*/
}

// Inverse kinematics

Matrix_3xN Model::jacobian(const Vector3 & s, size_t id) {
	Matrix_3xN J = Matrix_3xN::Zero(d, num_thetas);
	for (size_t i = 0; i < phalanges[id].kinematic_chain.size(); i++) {
		size_t dof_id = phalanges[id].kinematic_chain[i];
		size_t phalange_id = dofs[dof_id].phalange_id;
		Vector3 u = dofs[dof_id].axis;
		Vector3 p = phalanges[phalange_id].global.block(0, 3, 3, 1);
		Transform3f T = Transform3f(phalanges[phalange_id].global);
		Vector3 v = T * u - p; // rotated axis

		switch (dofs[dof_id].type) {
		case TRANSLATION_AXIS:
			//J.col(dof_id) = T * u;
			J.col(dof_id) = u;
			break;
		case ROTATION_AXIS:
			J.col(dof_id) = v.cross(s - p);
			break;
		}
		/*std::cout << endl << "i = " << i << std::endl;
		std::cout << "axis = " << u.transpose() << std::endl;
		std::cout << "p = " << p.transpose() << std::endl;
		std::cout << "s = " << s.transpose() << std::endl;
		std::cout << "T = " << endl << phalanges[phalange_id].global << std::endl;
		std::cout << "v = " << v.transpose() << std::endl;*/
	}
	return J;
}

void Model::update(Phalange & phalange) {
	if (phalange.parent_id >= 0)
		phalange.global = phalanges[phalange.parent_id].global * phalange.local;
	else  phalange.global = phalange.local;
	for (size_t i = 0; i < phalange.children_ids.size(); i++)  update(phalanges[phalange.children_ids[i]]);
}

void Model::translate(Phalange & phalange, const Vec3f & t) {
	phalange.local(0, 3) += t[0];
	phalange.local(1, 3) += t[1];
	phalange.local(2, 3) += t[2];
	update(phalange);
	//cout << phalange.local << endl;
}

void Model::rotate(Phalange & phalange, const Vec3f &axis, float angle) {
	phalange.local = phalange.local * Transform3f(Eigen::AngleAxisf(angle, axis)).matrix();
	update(phalange);
}

void Model::transform_joints(const std::vector<float> & theta) {
	//cout << "transform_joints" << endl;	

	for (size_t i = 0; i < dofs.size(); ++i) {
		if (dofs[i].phalange_id == -1) continue;
		switch (dofs[i].type) {
		case TRANSLATION_AXIS:
			translate(phalanges[dofs[i].phalange_id], dofs[i].axis * theta[i]);
			break;
		case ROTATION_AXIS:
			rotate(phalanges[dofs[i].phalange_id], dofs[i].axis, theta[i]);
			break;
		}
	}
}

void Model::move(const std::vector<float> & theta) {
	for (size_t i = 0; i < num_thetas; i++) {
		this->theta[i] = theta[i];
	}
	for (size_t i = 0; i < num_phalanges + 1; i++) {
		phalanges[i].local = phalanges[i].init_local;
	}

	//cout << "move" << endl;
	vector<float> rotateX(num_thetas, 0); // flexion
	vector<float> rotateZ(num_thetas, 0); // abduction
	vector<float> rotateY(num_thetas, 0); // twist
	vector<float> globals(num_thetas, 0); // pose

	for (size_t i = 0; i < num_thetas; ++i) {
		if (dofs[i].phalange_id < num_phalanges && dofs[i].type == ROTATION_AXIS) {
			if (dofs[i].axis == Vec3f(1, 0, 0))
				rotateX[i] = theta[i];
			else if (dofs[i].axis == Vec3f(0, 1, 0))
				rotateY[i] = theta[i];
			else if (dofs[i].axis == Vec3f(0, 0, 1))
				rotateZ[i] = theta[i];
			else
				cout << "wrong axis" << endl;

		}
		else
			globals[i] = theta[i];
	}

	//transform joints separately
	transform_joints(globals); // pose	
	transform_joints(rotateX); // flexion
	transform_joints(rotateZ); // abduction
	transform_joints(rotateY); // twist
}

void Model::update_centers() {
	for (size_t i = 0; i < num_phalanges; i++) {
		Vec3f p = phalanges[i].global.block(0, 3, 3, 1);
		centers[phalanges[i].center_id] = glm::vec3(p[0], p[1], p[2]);

		/*//cout << endl << phalanges[i].global.block(0, 3, 3, 1).transpose() << endl;
		cout << endl << endl << endl << "i = " << i << endl;
		cout << "name = " << phalanges[i].name << endl;
		cout << "center_id = " << phalanges[i].center_id << endl;
		cout << " center = " << centers[phalanges[i].center_id][0] << ", " << centers[phalanges[i].center_id][1] << ", " << centers[phalanges[i].center_id][2] << endl;
		//cout << phalanges[i].global << endl << endl;*/

		for (size_t j = 0; j < phalanges[i].attachments.size(); j++) {
			Vec3d t = phalanges[i].global.block(0, 0, 3, 3).cast<double>() * phalanges[i].offsets[j];
			centers[phalanges[i].attachments[j]] = glm::vec3(p[0], p[1], p[2]) + glm::vec3(t[0], t[1], t[2]);

			/*cout << endl << "attachment_id = " << phalanges[i].attachments[j] << endl;
			cout << "offset = " << phalanges[i].offsets[j][0] << ", " << phalanges[i].offsets[j][1] << ", " << phalanges[i].offsets[j][2] << endl;
			cout << " center = " << centers[phalanges[i].attachments[j]][0] << ", " << centers[phalanges[i].attachments[j]][1] << ", " << centers[phalanges[i].attachments[j]][2] << endl;
			//cout << "R = " << phalanges[i].global.block(0, 0, 3, 3) << endl;
			//cout << "p = " << p[0] << ", " << p[1] << ", " << p[2] << endl;
			cout << "t = " << t[0] << ", " << t[1] << ", " << t[2] << endl;*/

		}
	}
	reindex();
	compute_tangent_points();
}

std::vector<float> Model::get_updated_parameters(const vector<float> & theta, const vector<float> &delta_theta) {
	size_t rx = 3;
	size_t ry = 4;
	size_t rz = 5;

	std::vector<float> updated(num_thetas);
	for (size_t i = 0; i < num_thetas; ++i)
		updated[i] = theta[i] + (i < delta_theta.size() ? delta_theta[i] : 0);

	Vec3f axisX(1, 0, 0);
	Vec3f axisY(0, 1, 0);
	Vec3f axisZ(0, 0, 1);

	Transform3f rX(Eigen::Quaternionf(Eigen::AngleAxisf(delta_theta[rx], axisX)));
	Transform3f rY(Eigen::Quaternionf(Eigen::AngleAxisf(delta_theta[ry], axisY)));
	Transform3f rZ(Eigen::Quaternionf(Eigen::AngleAxisf(delta_theta[rz], axisZ)));

	Mat3f r = (rZ * rX * rY).rotation();

	r = phalanges[17].global.block(0, 0, 3, 3) * r;

	Vec3f e = r.eulerAngles(0, 1, 2);

	updated[rx] = e[0];
	updated[ry] = e[1];
	updated[rz] = e[2];

	return updated;
}

const std::vector<float>& Model::get_theta() {
	return theta;
}

Vec3f Model::get_palm_center() {
	glm::vec3 palm_center = glm::vec3(0, 0, 0);
	if (model_type == HTRACK) {
		palm_center += centers[centers_name_to_id_map["palm_top_left"]];
		palm_center += centers[centers_name_to_id_map["palm_top_right"]];
		palm_center += centers[centers_name_to_id_map["palm_bottom_left"]];
		palm_center += centers[centers_name_to_id_map["palm_bottom_right"]];
		palm_center = palm_center / 4.0f;
	}
	if (model_type == HMODEL || model_type == HMODEL_OLD) {
		palm_center += centers[centers_name_to_id_map["palm_right"]];
		palm_center += centers[centers_name_to_id_map["palm_left"]];
		palm_center += centers[centers_name_to_id_map["palm_pinky"]];
		palm_center += centers[centers_name_to_id_map["palm_ring"]];
		palm_center += centers[centers_name_to_id_map["palm_middle"]];
		palm_center += centers[centers_name_to_id_map["palm_index"]];
		palm_center = palm_center / 6.0f;
	}
	return Vec3f(palm_center[0], palm_center[1], palm_center[2]);
}

float Model::get_palm_length() {
	float length = 0;
	if (model_type == HTRACK) {
		length = glm::length(centers[centers_name_to_id_map["palm_top_left"]] - centers[centers_name_to_id_map["palm_bottom_left"]]);
	}
	if (model_type == HMODEL || model_type == HMODEL_OLD) {
		glm::vec3 c = (centers[centers_name_to_id_map["ring_membrane"]] + centers[centers_name_to_id_map["middle_membrane"]]) / 2.0f;
		length = glm::length(centers[centers_name_to_id_map["palm_back"]] - c);
	}
	return length;
}

Mat3f Model::build_rotation_matrix(Vec3f euler_angles) {
	float alpha = euler_angles(0);
	float beta = euler_angles(1);
	float gamma = euler_angles(2);
	Mat3f Rx, Ry, Rz;
	Rx << 1, 0, 0, 0, cos(alpha), -sin(alpha), 0, sin(alpha), cos(alpha);
	Ry << cos(beta), 0, sin(beta), 0, 1, 0, -sin(beta), 0, cos(beta);
	Rz << cos(gamma), -sin(gamma), 0, sin(gamma), cos(gamma), 0, 0, 0, 1;
	return Rz * Ry * Rx;
}

void Model::manually_adjust_initial_transformations() {
	Mat3f R;
	// thumb
	R = build_rotation_matrix(Vec3f(-1.45, 0.6, -1.95));
	phalanges[1].init_local.block(0, 0, 3, 3) = R;

	//R = build_rotation_matrix(Vec3f(-1.45, 0.6, -1));
	//phalanges[1].init_local.block(0, 0, 3, 3) = R;

	// index	
	R = build_rotation_matrix(Vec3f(0, 0, -0.08));
	phalanges[15].init_local.block(0, 0, 3, 3) = R;

	//R = build_rotation_matrix(Vec3f(3.1126, 0, 3.6));
	//phalanges[13].init_local.block(0, 0, 3, 3) = R;

	//middle
	R = build_rotation_matrix(Vec3f(3.1067, -0.12, -3.1215));
	phalanges[10].init_local.block(0, 0, 3, 3) = R;
	// ring
	R = build_rotation_matrix(Vec3f(-3.054, 0.0, -2.9823));
	phalanges[7].init_local.block(0, 0, 3, 3) = R;

	std::vector<float> theta = std::vector<float>(num_thetas, 0);
	theta[1] = -70; theta[2] = 400;
	theta[9] = 0.7; theta[10] = 0.6;
	move(theta);
	update_centers();
	initialize_offsets();
}

void Model::resize_model(float uniform_scaling_factor, float width_scaling_factor, float thickness_scaling_factor) {
	Mat3d scaling_matrix = Mat3d::Identity();
	scaling_matrix(0, 0) = uniform_scaling_factor * width_scaling_factor;
	scaling_matrix(1, 1) = uniform_scaling_factor;
	scaling_matrix(2, 2) = uniform_scaling_factor;

	for (size_t i = 0; i < radii.size(); i++) {
		radii[i] *= uniform_scaling_factor * thickness_scaling_factor;
	}

	for (size_t i = 0; i < num_phalanges; i++) {
		phalanges[i].init_local.col(3).segment(0, 3) = scaling_matrix.cast <float>() * phalanges[i].init_local.col(3).segment(0, 3);
		for (size_t j = 0; j < phalanges[i].attachments.size(); j++) {
			phalanges[i].offsets[j] = scaling_matrix * phalanges[i].offsets[j];
		}
	}
}