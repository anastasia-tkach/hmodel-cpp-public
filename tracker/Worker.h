#pragma once
#include "util/gl_wrapper.h"
#include "tracker/ForwardDeclarations.h"
#include "tracker/Types.h"
#include "OpenGL/OffscreenRenderer.h"
#include "Data/DataFrame.h"
#include "Energy/JointLimits.h"
#include "Energy/Damping.h"
#include "Energy/Collision.h"
#include "Energy/PoseSpace.h"
#include "Energy/Fitting.h"
#include "Energy/Fitting/TrackingMonitor.h"
#include "Energy/Temporal.h"

#include "opencv2/core/core.hpp"       ///< cv::Mat
#include "opencv2/highgui/highgui.hpp" ///< cv::imShow

/// @note do not construct more than one instance of this class
class Worker {

public:
	struct Settings {
		int termination_max_iters = 6;
		int termination_max_rigid_iters = 1;
	} _settings;
	Settings*const settings = &_settings;

public:
	QGLWidget* glarea = NULL;
public:
	void bind_glwidget(QGLWidget* glarea) { this->glarea = glarea; }
	void updateGL();

public:
	bool test;
	bool benchmark;
	bool save_rastorized_model;
	int user_name;
	std::string data_path;

	Camera* camera = NULL;
	Model * model;
	DataFrame current_frame = DataFrame(-1);
	TrackingError tracking_error;
	//std::vector<TrackingError> tracking_error_optimization;

	DepthTexture16UC1* sensor_depth_texture = NULL;
	ColorTexture8UC3* sensor_color_texture = NULL;

	energy::Fitting E_fitting;
	energy::Temporal E_temporal;
	energy::Damping E_damping;
	energy::JointLimits E_limits;
	energy::Collision E_collision;
	energy::PoseSpace E_pose;

	HandFinder* handfinder = NULL;
	TrivialDetector* trivial_detector = NULL;
	OffscreenRenderer offscreen_renderer;
	OffscreenRenderer rastorizer;
	TrackingMonitor monitor;

public:
	Worker(Camera *camera, bool test, bool benchmark, bool save_rasotrized_model, int user_name, string data_path);
	~Worker();
	void init_graphic_resources(); ///< not in constructor as needs valid OpenGL context
	void cleanup_graphic_resources();

public:
	void track(int iter);
	bool track_till_convergence();
};
