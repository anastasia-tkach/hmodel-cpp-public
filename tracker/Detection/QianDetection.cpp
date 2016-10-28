#include <fstream>
#include <math.h>
#include "util/gl_wrapper.h"
#include "util/opencv_wrapper.h"

#include "tracker/Worker.h"
#include "tracker/OpenGL/DebugRenderer/DebugRenderer.h"
#include "tracker/HandFinder/HandFinder.h"
#include "tracker/TwSettings.h"
#include "tracker/Hmodel/Model.h"

///--- Locals
#include "DetectionStream.h"
#include "QianDetection.h"
#include "FindFingers.h"

QianDetection::QianDetection(Worker *worker) : worker(worker) {

    detection = new DetectionStream(worker->model);
    find_fingers = new FindFingers(worker, detection);

    tw_settings->tw_add(settings->display, "Detect. SHOW?","group=Tracker");
}

QianDetection::~QianDetection(){
    delete detection;
    delete find_fingers;
}

bool QianDetection::can_reinitialize(){
    return find_fingers->find_fingers_main(settings->display);
}

void QianDetection::reinitialize(){
    this->detection_iterate();
}

bool verbose = false;


void QianDetection::draw_detected_features(bool verbose){

	if (!verbose) return;

	//Draw fingers
    DebugRenderer::instance().clear();
	std::vector<Vector3> points;
	std::vector<std::pair<Vector3, Vector3>> lines;

	std::vector<size_t> indices;
	indices = detection->get_point_targets_indices();
	for (int i = 0; i < indices.size(); ++i) {
		size_t index = indices[i];
		Vector3 target = detection->get_point_target(index);
		points.push_back(target);
	}
	indices = detection->get_line_targets_indices();
	for (int i = 0; i < indices.size(); ++i) {
		size_t index = indices[i];
		Vector3 start = detection->get_line_target_start(index);
		Vector3 end = detection->get_line_target_end(index);
		lines.push_back(std::pair<Vector3, Vector3>(start, end));
	}

	//Draw palm
	/*Vector3 t = detection->get_plane_target_origin();
	Vector3 n = detection->get_plane_target_normal();
	Vector3 d1 = Vector3(n(0), -n(1), 0);
	d1.normalize();
	Vector3 d2 = n.cross(d1);
	d2.normalize();
	Scalar factor = 35;
	lines.push_back(std::pair<Vector3, Vector3>(t, t + factor * d1));
	lines.push_back(std::pair<Vector3, Vector3>(t, t - factor * d1));
	lines.push_back(std::pair<Vector3, Vector3>(t, t + factor * d2));
	lines.push_back(std::pair<Vector3, Vector3>(t, t - factor * d2));*/

    DebugRenderer::instance().add_points(points, Vector3(0, 1, 0));
    DebugRenderer::instance().add_segments(lines, Vector3(1, 0.5, 0));
    worker->updateGL();
}

void QianDetection::draw_point_constraints(size_t permutation, size_t index, bool verbose) {
	if (!verbose) return;

    //DebugRenderer::instance().clear();
	std::vector<std::pair<Vector3, Vector3>> lines;
	std::vector<size_t> ids = detection->get_ids(permutation, index);
	Vector3 s = detection->get_point_source(ids[0]);
	Vector3 t = detection->get_point_target(index);
	lines.push_back(std::pair<Vector3, Vector3>(s, t));
    DebugRenderer::instance().add_segments(lines, Vector3(0, 0.8, 0.5));
    worker->updateGL();
}

void QianDetection::draw_line_constraints(size_t permutation, size_t index, size_t j, Vector3 p, bool verbose) {
	if (!verbose) return;

    //DebugRenderer::instance().clear();
	std::vector<std::pair<Vector3, Vector3>> lines;

	std::vector<size_t> ids = detection->get_ids(permutation, index);
	Vector3 s = detection->get_line_source(ids[j]);
	lines.push_back(std::pair<Vector3, Vector3>(s, p));
    DebugRenderer::instance().add_segments(lines, Vector3(1, 0, 0));
    worker->updateGL();
}

void QianDetection::draw_direction_constraints(size_t permutation, size_t direction, size_t index, size_t i, bool verbose) {
	if (!verbose) return;

    //DebugRenderer::instance().clear();
	std::vector<std::pair<Vector3, Vector3>> lines;
	std::vector<size_t> ids = detection->get_ids(permutation, index);
	Vector3 s = detection->get_point_source(ids[0]);
	Vector3 t = detection->get_direction_target(direction, index, i);
	lines.push_back(std::pair<Vector3, Vector3>(s, t));
    DebugRenderer::instance().add_segments(lines, Vector3(0.7, 0, 0.8));
    worker->updateGL();
}


void QianDetection::detect_compute_points_update(LinearSystem& system_detection, size_t permutation, bool verbose) {
	if (call_stack) cout << "[Worker_detection::detect_compute_points_update()]" << endl;

	const int k = detection->get_num_point_targets();

	Scalar c = 3;

	Eigen::Matrix<Scalar, Eigen::Dynamic, num_thetas> J = Eigen::Matrix<Scalar, Eigen::Dynamic, num_thetas>::Zero(c * k, num_thetas);
	VectorN f = VectorN::Zero(c * k);

	std::vector<size_t> indices = detection->get_point_targets_indices();
	for (int i = 0; i < k; ++i) {
		size_t index = indices[i];
		Vector3 t = detection->get_point_target(index);
		size_t id = detection->get_ids(permutation, index)[0];
		Vector3 s = detection->get_point_source(id);

		Eigen::Matrix<Scalar, 3, num_thetas> js;
		js = worker->model->jacobian(s, worker->model->jointid_to_phalangeid_map[id]);

		J.block(c * i, 0, c, num_thetas) += js;
		f.block(c * i, 0, c, 1) += factor *(t - s);

		draw_point_constraints(permutation, index, verbose);
	}
	system_detection.lhs.selfadjointView<Eigen::Upper>().rankUpdate(J.transpose(), 3.0);
	system_detection.rhs.noalias() += 3 * J.transpose() * f;
}

void QianDetection::detect_compute_lines_update(LinearSystem& system_detection, size_t permutation, bool verbose) {
	if (call_stack) cout << "[Worker_detection::detect_compute_lines_update()]" << endl;

	const int k = detection->get_num_line_targets();
	Scalar c = 3;

	Eigen::Matrix<Scalar, Eigen::Dynamic, num_thetas> J = Eigen::Matrix<Scalar, Eigen::Dynamic, num_thetas>::Zero(2 * c * k, num_thetas);
	VectorN f = VectorN::Zero(2 * c * k);

	std::vector<size_t> indices = detection->get_line_targets_indices();

	for (int i = 0; i < k; ++i) {
		size_t index = indices[i];

		std::vector<size_t> ids = detection->get_ids(permutation, index);

		for (int j = 0; j < 3; ++j) {
			size_t id = ids[j];
			Vector3 t = detection->get_line_target_start(index);
			Vector3 r = detection->get_line_target_end(index);
			Vector3 s = detection->get_line_source(id);
			Vector3 n = detection->get_line_target_normal(index);
			Scalar indicator = (r - t).dot(s - t) / (r - t).norm() / (r - t).norm();

			Eigen::Matrix<Scalar, 3, num_thetas> js;
			js = worker->model->jacobian(s, worker->model->jointid_to_phalangeid_map[id]);			

			Vector3 p;
			if (indicator > 0 && indicator < 1) {
				//cout << "case 1" << endl;
				J.block(2 * c * i + j, 0, c, num_thetas) += js - n * n.transpose() * js;
				f.block(2 * c * i + j, 0, c, 1) += factor * ((t - s) - n * n.transpose() * (t - s));
				p = t + n * n.transpose() * (s - t);
			}
			else if (indicator <= 0) {
				//cout << "case 2" << endl;
				J.block(2 * c * i + j, 0, c, num_thetas) += js;
				f.block(2 * c * i + j, 0, c, 1) += factor * (t - s);
				p = t;
			}
			else if (indicator >= 1) {
				//cout << "case 3" << endl;
				J.block(2 * c * i + j, 0, c, num_thetas) += js;
				f.block(2 * c * i + j, 0, c, 1) += factor * (r - s);
				p = r;
			}
			draw_line_constraints(permutation, index, j, p, verbose);
		}
	}
	system_detection.lhs.selfadjointView<Eigen::Upper>().rankUpdate(J.transpose(), 1.0);
	system_detection.rhs.noalias() += J.transpose() * f;
}

void QianDetection::detect_compute_direction_update(LinearSystem& system_detection, size_t permutation, size_t direction, bool verbose) {
	if (call_stack) cout << "[detect_compute_directions_update()]" << endl;

	const int k = detection->get_num_direction_targets();
	Scalar c = 3;

	Eigen::Matrix<Scalar, Eigen::Dynamic, num_thetas> J = Eigen::Matrix<Scalar, Eigen::Dynamic, num_thetas>::Zero(c * k, num_thetas);

	VectorN f = VectorN::Zero(c * k);

	std::vector<size_t> indices = detection->get_direction_targets_indices();
	for (int i = 0; i < k; ++i) {
		size_t index = indices[i];
		Vector3 t = detection->get_direction_target(direction, index, i);
		size_t id = detection->get_ids(permutation, index)[0];
		Vector3 s = detection->get_point_source(id);

		Eigen::Matrix<Scalar, 3, num_thetas> js; 
		js = worker->model->jacobian(s, worker->model->jointid_to_phalangeid_map[id]);		

		J.block(c * i, 0, c, num_thetas) += js;
		f.block(c * i, 0, c, 1) += factor *(t - s);

		draw_direction_constraints(permutation, direction, index, i, verbose);
	}
	system_detection.lhs.selfadjointView<Eigen::Upper>().rankUpdate(J.transpose(), 1.0);
	system_detection.rhs.noalias() += J.transpose() * f;
}

Scalar QianDetection::compute_detection_energy(size_t permutation, size_t direction) {
	//cout << "[compute_detection_energy()]" << endl;

	Scalar energy = 0;

	// Point constraints energy
	std::vector<size_t> point_targets_indices = detection->get_point_targets_indices();
	for (int i = 0; i < detection->get_num_point_targets(); ++i) {
		size_t index = point_targets_indices[i];
		size_t id = detection->get_ids(permutation, index)[0];
		Vector3 t = detection->get_point_target(index);
		Vector3 s = detection->get_point_source(id);
		energy += (t - s).transpose() * (t - s);
	}

	// Line constraints energy
	std::vector<size_t> line_targets_indices = detection->get_line_targets_indices();
	for (int i = 0; i < detection->get_num_line_targets(); ++i) {
		size_t index = line_targets_indices[i];
		std::vector<size_t> ids = detection->get_ids(permutation, index);
		for (int j = 0; j < 2; ++j) {
			size_t id = ids[j];
			Vector3 t = detection->get_line_target_start(index);
			Vector3 r = detection->get_line_target_end(index);
			Vector3 s = detection->get_line_source(id);
			Vector3 n = detection->get_line_target_normal(index);
			Scalar indicator = (r - t).dot(s - t) / (r - t).norm() / (r - t).norm();
			if (indicator > 0 && indicator < 1) {
				Vector3 e = (t - s) - n * n.transpose() * (t - s);
				energy += e.transpose() * e;
			}
			else if (indicator <= 0) {
				energy += (t - s).transpose() * (t - s);
			}
			else if (indicator >= 1) {
				energy += (r - s).transpose() * (r - s);
			}
		}
	}

	// Direction constraints energy
	std::vector<size_t> direction_targers_indices = detection->get_direction_targets_indices();
	for (int i = 0; i < detection->get_num_direction_targets(); ++i) {
		size_t index = direction_targers_indices[i];
		size_t id = detection->get_ids(permutation, index)[0];
		Vector3 t = detection->get_direction_target(direction, index, i);
		Vector3 s = detection->get_point_source(id);
		energy += (t - s).transpose() * (t - s);
	}
	return energy;
}

void QianDetection::my_damping_track(LinearSystem& system) {
	Scalar lambda_rotation = 300;
	Scalar lambda_translation = 1;

	Eigen::Matrix<Scalar, num_thetas, num_thetas> D = Eigen::Matrix<Scalar, num_thetas, num_thetas>::Identity(system.lhs.rows(), system.lhs.cols());
	Eigen::Matrix<Scalar, num_thetas, 1>  d = Eigen::Matrix<Scalar, num_thetas, 1>::Ones(system.lhs.rows(), 1);

	for (int i = 0; i < num_thetas; ++i) {
		if (worker->model->dofs[i].type == TRANSLATION_AXIS) d(i) = lambda_translation;
		else d(i) = lambda_rotation;
		if (i >= 3 && i < 6) d(i) *= 1000;
	}
	
	D.diagonal() = d;
	system.lhs = system.lhs + D;
}

void QianDetection::optimize_current_permutation(size_t permutation, size_t direction, const Eigen::Matrix<Scalar, num_thetas, 1> & theta_0, bool verbose, std::vector<float> & theta_std) {
	if (call_stack) cout << "[Worker_detection::optimize_current_permutation()]" << endl;

	for (size_t i = 0; i < num_thetas; i++) theta_std[i] = theta_0(i);

	size_t num_iterations = 10;
	for (size_t optimization_iteration = 0; optimization_iteration < num_iterations; optimization_iteration++) {

		worker->model->move(theta_std);
		worker->model->update_centers();

		if (optimization_iteration == num_iterations - 1) break;

		LinearSystem system_detection(num_thetas);
		system_detection.lhs.setZero();
		system_detection.rhs.setZero();
		draw_detected_features(verbose);

		detect_compute_points_update(system_detection, permutation, verbose);
		detect_compute_lines_update(system_detection, permutation, verbose);
		detect_compute_direction_update(system_detection, permutation, direction, verbose);

        worker->E_limits.track(system_detection, theta_std);
        my_damping_track(system_detection);

		VectorN delta_theta = system_detection.lhs.colPivHouseholderQr().solve(system_detection.rhs);
        std::vector<Scalar> delta_theta_std;
        for(int t = 0; t < delta_theta.size(); ++t)
            delta_theta_std.push_back(delta_theta(t));
		theta_std = worker->model->get_updated_parameters(theta_std, delta_theta_std);
	}
}

void QianDetection::rigid_aligment(const Eigen::Matrix<Scalar, num_thetas, 1> & theta_0, Eigen::Matrix<Scalar, num_thetas, 1> & theta, size_t permutation_index) {
	Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Vs =
		Eigen::Matrix<Scalar, 3, Eigen::Dynamic>::Zero(3, detection->get_num_point_targets() + 1);
	Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Vt =
		Eigen::Matrix<Scalar, 3, Eigen::Dynamic>::Zero(3, detection->get_num_point_targets() + 1);
	
	// Set joint angles to initial pose
	std::vector<float>theta_std;
	theta_std = std::vector<float>(num_thetas, 0);
	for (size_t i = 0; i < num_thetas; i++) theta_std[i] = theta_0(i);
	worker->model->move(theta_std);
	worker->model->update_centers();
	
	// Get palm point for right alignment
	Vector3 palm_target = detection->get_point_target(0);
	Vector3 palm_source = worker->model->get_palm_center();

	Vs.col(detection->get_num_point_targets()) = palm_source.transpose();
	Vt.col(detection->get_num_point_targets()) = palm_target.transpose();

	// Get finger tips for rigid alignment
	std::vector<size_t> indices = detection->get_point_targets_indices();
	for (int i = 0; i < indices.size(); ++i) {
		size_t index = indices[i];
		Vector3 t = detection->get_point_target(index);
		size_t id = detection->get_ids(permutation_index, index)[0];
		Vector3 s = detection->get_point_source(id);
		Vs.col(i) = s.transpose();
		Vt.col(i) = t.transpose();
	}

	Mat4f T = Eigen::umeyama(Vs, Vt, false);	
	Mat3f R = T.block(0, 0, 3, 3);
	Vec3f euler_angles = R.eulerAngles(0, 1, 2);
	theta = theta_0;
	theta.segment(3, 3) = euler_angles;
	worker->model->move(theta_std);
	worker->model->update_centers();

	// Display
	/*std::vector<Vector3> result_points;
	for (int i = 0; i < num_fingers + 2; ++i) result_points.push_back(Vs.col(i));
    DebugRenderer::instance().add_points(result_points, Vector3(0, 1, 0));
	std::vector<Vector3> target_points;
	for (int i = 0; i < num_fingers + 2; ++i) target_points.push_back(Vt.col(i));
    DebugRenderer::instance().add_points(target_points, Vector3(1, 0, 0));
	std::vector<std::pair<Vector3, Vector3>> lines;
	for (int i = 0; i < num_fingers + 2; ++i) lines.push_back(std::pair<Vector3, Vector3>(Vt.col(i), Vs.col(i)));
    DebugRenderer::instance().add_segments(lines, Vector3(0, 0.8, 0.5));
	::glarea->updateGL();*/

}

void QianDetection::detection_iterate() {
	if (call_stack) cout << "[Worker_detection::detection_iterate()]" << endl;

	// Set palm position
	Eigen::Matrix<Scalar, num_thetas, 1> theta_0 = Eigen::Matrix<Scalar, num_thetas, 1>::Zero(num_thetas, 1);
	float palm_length = worker->model->get_palm_length();
	Vector3 shift = 0.5 * palm_length * Vector3(0, 1, 0);
	Vector3 target = detection->get_plane_target_origin() - shift;
	theta_0.head(3) = target;

	if (detection->get_num_targets() < num_fingers + 1 || detection->get_thumb_index() < 0) {
		return;
	}

	// Find min rigid error pose
	size_t min_energy_permutation_index;
	size_t min_energy_direction_index;

	Scalar min_energy = std::numeric_limits<Scalar>::max();

	std::vector<float> theta_min_energy = std::vector<float>(num_thetas, 0);
	std::vector<float> theta_fitted = std::vector<float>(num_thetas, 0);
	Eigen::Matrix<Scalar, num_thetas, 1> theta;


	// Fit the fingers
	for (int direction = 0; direction < max(detection->get_num_directions(), (size_t)1); direction++) {
		for (int permutation_index = 0; permutation_index < detection->get_num_permutations(); permutation_index++) {
			//cout << "permutation [" << permutation_index << "] = " << detection->get_permutation(permutation_index).transpose() << endl;
			
			rigid_aligment(theta_0, theta, permutation_index);

			optimize_current_permutation(permutation_index, direction, theta, false, theta_fitted);			
			Scalar energy = compute_detection_energy(permutation_index, direction);
			if (energy < min_energy) {
				min_energy = energy;
				min_energy_permutation_index = permutation_index;
				min_energy_direction_index = direction;
				theta_min_energy = theta_fitted;			
			}
		}
		break;
	}	
	worker->model->move(theta_min_energy);
	worker->model->update_centers();
}




