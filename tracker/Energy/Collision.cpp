#include "Collision.h"

#include "util/mylogger.h"
#include "tracker/HModel/Model.h"

#include "tracker/Worker.h"
#include "tracker/TwSettings.h"
#include "tracker/Data/DataFrame.h"

namespace geometry {

	/* Copyright 2001 softSurfer, 2012 Dan Sunday
	   This code may be freely used and modified for any purpose
	   providing that this copyright notice is included with it.
	   SoftSurfer makes no warranty for this code, and cannot be held
	   liable for any real or imagined damage resulting from its use.
	   Users of this code must verify correctness for their application.
	   */
	std::pair<Vector3, Vector3>  segment_to_segment_distance(std::pair<Vector3, Vector3> S1, std::pair<Vector3, Vector3> S2, bool display) {
		double epsilon = 0.00000001;
		Vector3   u = S1.second - S1.first;
		Vector3   v = S2.second - S2.first;
		Vector3   w = S1.first - S2.first;
		double a = u.dot(u);       // always >= 0
		double b = u.dot(v);
		double c = v.dot(v);         // always >= 0
		double d = u.dot(w);
		double e = v.dot(w);
		double D = a*c - b*b;        // always >= 0
		double sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
		double tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0

		// compute the line parameters of the two closest points
		if (D < epsilon) { // the lines are almost parallel
			sN = 0.0;         // force using point P0 on segment S1
			sD = 1.0;         // to prevent possible division by 0.0 later
			tN = e;
			tD = c;
		}
		else {                 // get the closest points on the infinite lines
			sN = (b*e - c*d);
			tN = (a*e - b*d);
			if (sN < 0.0) {        // sc < 0 => the s=0 edge is visible
				sN = 0.0;
				tN = e;
				tD = c;
			}
			else if (sN > sD) {  // sc > 1  => the s=1 edge is visible
				sN = sD;
				tN = e + b;
				tD = c;
			}
		}

		if (tN < 0.0) {            // tc < 0 => the t=0 edge is visible
			tN = 0.0;
			// recompute sc for this edge
			if (-d < 0.0)
				sN = 0.0;
			else if (-d > a)
				sN = sD;
			else {
				sN = -d;
				sD = a;
			}
		}
		else if (tN > tD) {      // tc > 1  => the t=1 edge is visible
			tN = tD;
			// recompute sc for this edge
			if ((-d + b) < 0.0)
				sN = 0;
			else if ((-d + b) > a)
				sN = sD;
			else {
				sN = (-d + b);
				sD = a;
			}
		}
		// finally do the division to get sc and tc
		sc = (abs(sN) < epsilon ? 0.0 : sN / sD);
		tc = (abs(tN) < epsilon ? 0.0 : tN / tD);

		// get the difference of the two closest points
		Vector3 dP = w + (sc * u) - (tc * v);  // =  S1(sc) - S2(tc)

		Vector3 P1 = S1.first + sc * u;
		Vector3 P2 = S2.first + tc * v;
		std::pair<Vector3, Vector3> closest_distance = std::pair<Vector3, Vector3>(P1, P2);

#if 0
		if (display == true){
			std::vector<Vector3> points;
			std::vector<std::pair<Vector3, Vector3>> segments;
			points.push_back(S1.first);
			points.push_back(S1.second);
			points.push_back(S2.first);
			points.push_back(S2.second);
			points.push_back(P1);
			points.push_back(P2);
			segments.push_back(S1);
			segments.push_back(S2);
			segments.push_back(std::pair<Vector3, Vector3>(P1, P2));
			DebugRenderer::instance().add_points(points, Vector3(1, 0, 0));
			DebugRenderer::instance().add_segments(segments, Vector3(0, 0, 1));
		}
#endif

		//return dP.norm();
		return closest_distance;
	}

	bool ray_triangle_intersection(Vector3 O, Vector3 D, Vector3 P1, Vector3 P2, Vector3 P3, bool display) {
		double epsilon = 0.00000001;

#if 0
		if (display == true){
			std::vector<Vector3> points;
			std::vector<std::pair<Vector3, Vector3>> segments;
			points.push_back(P1);
			points.push_back(P2);
			points.push_back(P3);
			points.push_back(O);
			float offset = 1000;
			segments.push_back(std::pair<Vector3, Vector3>(P1, P2));
			segments.push_back(std::pair<Vector3, Vector3>(P2, P3));
			segments.push_back(std::pair<Vector3, Vector3>(P1, P3));
			segments.push_back(std::pair<Vector3, Vector3>(O, offset * D));
			DebugRenderer::instance().add_points(points, Vector3(1, 0, 0));
			DebugRenderer::instance().add_segments(segments, Vector3(0, 0, 1));
		}
#endif

		Vector3 e1 = P2 - P1;
		Vector3 e2 = P3 - P1;
		Vector3 q = D.cross(e2);
		float a = e1.dot(q); // determinant of the matrix M

		if (a > -epsilon && a < epsilon) {
			LOG(INFO) << "The vector is parallel to the plane (the intersection is at infinity).";
			return false;
		}

		float f = 1 / a;
		Vector3 s = O - P1;
		float u = f * s.dot(q);

		if (u < 0.0) {
			LOG(INFO) << "The intersection is outside of the triangle, case 1.";
			return false;
		}

		Vector3 r = s.cross(e1);
		float v = f * D.dot(r);

		if (v < 0.0 || u + v > 1.0) {
			LOG(INFO) << "The intersection is outside of the triangle, case 2.";
			return false;
		}

		float t = f * e2.dot(r);
		if (t < 0.0) {
			LOG(INFO) << "The intersection is in the continuation of the ray.";
			return false;
		}

#if 0
		if (display == true){
			std::vector<Vector3> points;
			points.push_back(O + t * D);
			DebugRenderer::instance().add_points(points, Vector3(1, 0, 0));
		}
#endif

		return true;
	}

} ///< geometry::

// Apply transformation to the given point
Vector3 transform_point_position(Vector3 point, Mat4f transformation) {
	Eigen::VectorXf homogen_point(4);
	homogen_point << point, 1.0;
	Eigen::VectorXf transformed_homogen_point = transformation * homogen_point;
	return (transformed_homogen_point / transformed_homogen_point[3]).head(3);
}

// Return axis endpoint of the segment
std::pair<Vector3, Vector3> get_axis_endpoints(Phalange phalange) {
	Vector3 axis_start = Vector3(0, 0, 0);
	Vector3 axis_end = Vector3(0, phalange.length, 0);
	Vector3 transformed_axis_start = transform_point_position(axis_start, phalange.global.cast<float>());
	Vector3 transformed_axis_end = transform_point_position(axis_end, phalange.global.cast<float>());
	return std::pair<Vector3, Vector3>(transformed_axis_start, transformed_axis_end);
}

// Display collision constraints
void debug_display_constraints(std::pair<Vector3, Vector3> shortest_path, Vector3 current_intersection, Vector3 other_intersection) {
	std::vector<std::pair<Vector3, Vector3>> lines;
	std::vector<Vector3> colors, points;
	lines.push_back(std::pair<Vector3, Vector3>(shortest_path.first, current_intersection));
	points.push_back(current_intersection);
	colors.push_back(Vector3(1, 0.5, 0));
	lines.push_back(std::pair<Vector3, Vector3>(shortest_path.second, other_intersection));
	points.push_back(other_intersection);
	colors.push_back(Vector3(0, 0.5, 1));
	//DebugRenderer::instance().add_segments(lines, colors);
	//DebugRenderer::instance().add_points(points, colors);
	//::glarea->updateGL();
}

// A(i, j) = 1, if i is adjacent to j
// A(i, j) = 2, if i should not be compared to j
// A(i, j) = 0, if we should test the joint i and j for collision
Matrix_MxN create_adjacency_matrix(std::vector<Phalange> phalanges) {
	Matrix_MxN adjacency_matrix = Matrix_MxN::Zero(num_phalanges - 1, num_phalanges - 1);
	for (int i = 0; i < num_phalanges - 1; i++) {
		if (phalanges[i].parent_id >= 0 && phalanges[i].parent_id < num_phalanges - 1)
			adjacency_matrix(i, phalanges[i].parent_id) = 1;
		for (size_t j = 0; j < phalanges[i].children_ids.size(); j++) {
			adjacency_matrix(i, phalanges[i].children_ids[j]) = 1;
		}
	}
	return adjacency_matrix;
}

std::vector<std::vector<std::pair<Vector3, Vector3>>> create_distance_matrix(std::vector<Phalange> phalanges, Matrix_MxN adjacency_matrix) {

	std::vector<std::vector<std::pair<Vector3, Vector3>>> distance_matrix;
	std::pair<Vector3, Vector3> big_distance(Vector3(0, 0, 0), Vector3(RAND_MAX, RAND_MAX, RAND_MAX));
	std::vector<std::pair<Vector3, Vector3>> current_row(num_phalanges - 1, big_distance);
	distance_matrix.push_back(current_row);
	for (int i = 1; i < num_phalanges - 1; i++) {
		distance_matrix.push_back(current_row);
		for (int j = 1; j < i; j++) {
			if (adjacency_matrix(i, j) != 0) continue;
			std::pair<Vector3, Vector3> shortest_path =
				geometry::segment_to_segment_distance(get_axis_endpoints(phalanges[i]), get_axis_endpoints(phalanges[j]), false);
			distance_matrix[i][j] = shortest_path;
		}
	}
	/*for (size_t i = 0; i < num_phalanges; i++) {
		for (int j = 1; j < i; j++) {
		if (adjacency_matrix(i, j) == 0) {
		cout << i << ", " << j << ":" << distance_matrix[i][j].first.transpose() << ",  " << distance_matrix[i][j].second.transpose() << endl;
		}
		}
		}*/
	return distance_matrix;
}

namespace energy {

	void Collision::init(Model * model) {
		this->model = model;
	}
	
	Scalar Collision::create_collision_constraints() {
		int discard_palm = 1;
		Scalar E = 0;

		// TIMED_SCOPE(timer, "Worker::create_collision_constraints");
		s.clear();
		t.clear();

		Matrix_MxN adjacency_matrix = create_adjacency_matrix(model->phalanges);
		
		std::vector<std::vector<std::pair<Vector3, Vector3>>> distance_matrix = create_distance_matrix(model->phalanges, adjacency_matrix);

		for (int i = discard_palm; i < num_phalanges - 1; i++) {
			for (int j = discard_palm; j < i; j++) {
				std::pair<Vector3, Vector3> shortest_path = distance_matrix[i][j];
				float distance = (shortest_path.first - shortest_path.second).norm();
				if (distance > FLT_MIN && distance < (model->phalanges[i].radius2 + model->phalanges[j].radius2)) {

					Vector3 normal = (shortest_path.second - shortest_path.first) / distance;
					Vector3 surface_i = shortest_path.first + normal * model->phalanges[i].radius2;
					Vector3 surface_j = shortest_path.second - normal * model->phalanges[j].radius2;

					s.push_back(Point(surface_i, i));
					t.push_back(Point(surface_j, j));
					n.push_back(normal);

					Scalar e = normal.transpose() * (surface_i - surface_j);
					E = E + e * e;
					//debug_display_constraints(shortest_path, surface_i, surface_j);
				}
			}
		}
		return E;
	}

	void Collision::collision_compute_update(LinearSystem &system, Scalar omega) {
		// TIMED_SCOPE(timer,"Worker::collision_compute_update");		
		const int k = max(s.size(), t.size());
		Matrix_MxN J; VectorN rhs;

		Scalar c = 2;
		Scalar fraction = settings->collision_fraction;
		J = Matrix_MxN::Zero(c * k, num_thetas);
		rhs = VectorN::Zero(c * k);
		for (int i = 0; i < k; ++i) {
			J.block(c * i, 0, 1, num_thetas) = n[i].transpose() * model->jacobian(s[i].p, s[i].id);
			rhs.block(c * i, 0, 1, 1) = fraction * n[i].transpose() * (t[i].p - s[i].p);
			J.block(c * i + 1, 0, 1, num_thetas) = n[i].transpose() * model->jacobian(t[i].p, t[i].id);
			rhs.block(c * i + 1, 0, 1, 1) = fraction *  n[i].transpose() * (s[i].p - t[i].p);
		}
		J.block(0, 0, c * k, num_thetas_rigid_motion) = Matrix_MxN::Zero(c * k, num_thetas_rigid_motion);

		system.lhs += omega * J.transpose() * J;
		system.rhs += omega * J.transpose() * rhs;
	}

	void Collision::track(LinearSystem& system) {
		// TIMED_SCOPE(timer,"Worker::collision_track");
		if (!settings->collision_enable) return;

		create_collision_constraints();
		collision_compute_update(system, settings->collision_weight);		
	}

} ///< energy::
