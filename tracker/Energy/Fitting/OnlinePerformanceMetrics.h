#include "nanoflann/nanoflann.hpp"

#include "opencv2/core/core.hpp"       
#include "opencv2/highgui/highgui.hpp" 
#include "opencv2/imgproc/imgproc.hpp"

#include "cudax/cuda_glm.h"
#include "util/MathUtils.h"

#include <iomanip>

struct PointCloud {
	struct Point { float  x, y, z; };
	std::vector<Point>  points;
	inline size_t kdtree_get_point_count() const { return points.size(); }
	inline float kdtree_distance(const float *query_point, const size_t index, size_t) const {
		float d0 = query_point[0] - points[index].x;
		float d1 = query_point[1] - points[index].y;
		float d2 = query_point[2] - points[index].z;
		return d0*d0 + d1*d1 + d2*d2;
	}
	inline float kdtree_get_pt(const size_t idx, int dim) const {
		if (dim == 0) return points[idx].x;
		else if (dim == 1) return points[idx].y;
		else return points[idx].z;
	}
	template <class BBOX> bool kdtree_get_bbox(BBOX&) const { return false; }
};

class OnlinePeformanceMetrics {

	void write_rastorized_model() {
		//imshow("model", rastorized_model);
		//imshow("normals", rastorized_normals);

		/*std::ostringstream stringstream;
		stringstream << std::setfill('0') << std::setw(7) << current_frame;
		string path = "...";
		string filename = path + "model-" + stringstream.str() + ".png";
		cv::imwrite(filename, rastorized_model);
		filename = path + "mask-" + stringstream.str() + ".png";
		cv::imwrite(filename, worker->handfinder->sensor_silhouette);
		filename = path + "depth-" + stringstream.str() + ".png";
		cv::imwrite(filename, worker->current_frame.depth);*/

		//string input_path = "...";

		//cv::Mat data_image = cv::imread(input_path + "depth-0000050.png", CV_LOAD_IMAGE_UNCHANGED);
		//cv::Mat model_image = cv::imread(input_path + "model-0000050.png", CV_LOAD_IMAGE_UNCHANGED);

		//cv::Mat mask = cv::imread(input_path + "mask-0000020.png", CV_LOAD_IMAGE_UNCHANGED);
		//cv::Mat rastorized_model = cv::imread(input_path + "model-0000021.png", CV_LOAD_IMAGE_UNCHANGED);
		//cv::Mat data_image = cv::imread(input_path + "depth-0000020.png", CV_LOAD_IMAGE_UNCHANGED);
		//imshow("mask", mask); cv::waitKey(0);
	}

	void find_rendered_pixels_outside_sensor_silhouette(vector<int> & rendered_pixels, const cv::Mat & rendered_model, const cv::Mat & sensor_silhouette) {
		int BACKGROUND = 5000;
					
		for (int row = 0; row < sensor_silhouette.rows; ++row) {
			for (int col = 0; col < sensor_silhouette.cols; ++col) {
				if (rendered_model.at<short>(row, col) < BACKGROUND && sensor_silhouette.at<uchar>(row, col) != 255) {
					rendered_pixels.push_back(row * sensor_silhouette.cols + col);
				}
			}
		}
		return;

		// Display
		cv::Mat rendered_indicator_image;
		rendered_indicator_image = cv::Mat(sensor_silhouette.rows, sensor_silhouette.cols, CV_8UC1, cv::Scalar(0));
		for (size_t i = 0; i < rendered_pixels.size(); i++) {
			int row = rendered_pixels[i] / sensor_silhouette.cols;
			int col = rendered_pixels[i] - sensor_silhouette.cols * row;
			rendered_indicator_image.at<uchar>(row, col) = 255;
		}		
		cv::imshow("rendered_silhouette", rendered_model); cv::waitKey(1);
		cv::imshow("sensor_silhouette", sensor_silhouette); cv::waitKey(1);
		cv::imshow("rendered_indicator_image", rendered_indicator_image);
		
	}

public:

	float compute_rastorized_2D_metric(const cv::Mat & rendered_model, const cv::Mat & sensor_silhouette, const cv::Mat & sensor_dtform_idxs) {

		vector <int> rendered_pixels;
		find_rendered_pixels_outside_sensor_silhouette(rendered_pixels, rendered_model, sensor_silhouette);

		float E = 0;

		for (size_t index = 0; index < rendered_pixels.size(); index++) {
			int linear_index = rendered_pixels[index];

			int offset_y = linear_index / sensor_silhouette.cols;
			int offset_x = linear_index - sensor_silhouette.cols * offset_y;

			glm::vec2 p_rend(offset_x, sensor_silhouette.rows - offset_y - 1);

			int closest_idx = sensor_dtform_idxs.at<int>(sensor_silhouette.rows - offset_y - 1, offset_x); // or in reverse;			
			int row = closest_idx / sensor_silhouette.cols;
			int col = closest_idx - sensor_silhouette.cols * row;
			glm::vec2 p_sens(col, row);

			glm::vec2 p_diff = p_sens - p_rend;

			E += abs(p_diff.x);
			E += abs(p_diff.y);
		}
		float E2D = E / 2 / rendered_pixels.size();
		//cout << "E2D = " << E2D << endl;
		return E2D;
	}


	float compute_rastorized_3D_metric(const cv::Mat & rendered_model, const cv::Mat & sensor_depth, const cv::Mat & sensor_silhouette, const Matrix3& inv_projection_matrix) {
		//write_rastorized_model();

		PointCloud point_cloud;

		Eigen::Vector3f xyz, uvd, p, q, n; float d, w, weight, depth;
		PointCloud::Point point;
		//std::vector<Eigen::Vector3f> model_points;
		for (size_t row = 0; row < rendered_model.rows; row++) {
			for (size_t col = 0; col < rendered_model.cols; col++) {

				depth = rendered_model.at<ushort>(rendered_model.rows - row - 1, col);

				if (depth == 5000) continue;
				uvd = Eigen::Vector3f(col * depth, row * depth, depth);
				q = inv_projection_matrix * uvd;
				//model_points.push_back(q);

				point.x = q[0]; point.y = q[1]; point.z = q[2];
				point_cloud.points.push_back(point);

				//d = rastorized_model.at<ushort>(i, j);
				//xyz = Pinv * Eigen::Vector3f(j + 1, i + 1, 1);
				//q = Eigen::Vector3f(xyz[0] * d, -xyz[1] * d, d);
			}
		}

		nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, PointCloud>, PointCloud, 3>
			kd_tree(3, point_cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
		kd_tree.buildIndex();

		float E = 0;
		int num_data_points = 0;
		size_t num_results = 1;

		for (size_t row = 0; row < sensor_depth.rows; row++) {
			for (size_t col = 0; col < sensor_depth.cols; col++) {

				if (sensor_silhouette.at<uchar>(sensor_depth.rows - row - 1, col) == 0) continue;

				depth = sensor_depth.at<ushort>(sensor_depth.rows - row - 1, col);

				uvd = Eigen::Vector3f(col * depth, row * depth, depth);
				p = inv_projection_matrix * uvd;

				float min_distance = std::numeric_limits<float>::max();
				size_t min_index = -1;

				//knn search
				float min_distance2;
				float query_point[3] = { p[0], p[1], p[2] };
				nanoflann::KNNResultSet<float> resultSet(num_results);
				resultSet.init(&min_index, &min_distance2);
				kd_tree.findNeighbors(resultSet, &query_point[0], nanoflann::SearchParams());

				d = sqrt(min_distance2);
				w = 1 / sqrt(d + 1e-3);
				weight = 1;
				if (d > 1e-3) weight = w * 3.5;
				E += weight * d;
				num_data_points++;
			}
		}
		float E3D = E / num_data_points;
		//cout << "E3D = " << E3D << endl;
		return  E3D;
	}



};