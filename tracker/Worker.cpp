#include "Worker.h"
#include "util/gl_wrapper.h"
#include "util/tictoc.h"

#include <QElapsedTimer>
#include <QGLWidget>

#include "tracker/ForwardDeclarations.h"
#include "tracker/Sensor/Sensor.h"
#include "tracker/Data/TextureColor8UC3.h"
#include "tracker/Data/TextureDepth16UC1.h"
#include "tracker/HandFinder/HandFinder.h"
#include "tracker/Energy/Energy.h"
#include "tracker/TwSettings.h"
#include "tracker/HModel/Model.h"

#include <ctime>

void Worker::updateGL() { if (glarea != NULL) glarea->updateGL(); }

Worker::Worker(Camera *camera, bool test, bool benchmark, bool save_rasotrized_model, int user_name, std::string data_path) {

	this->camera = camera;
	this->benchmark = benchmark;
	this->test = test;
	this->save_rastorized_model = save_rasotrized_model;
	this->user_name = user_name;
	this->data_path = data_path;

	this->model = new Model();
	this->model->init(user_name, data_path);
	std::vector<float> theta_initial = std::vector<float>(num_thetas, 0);
	theta_initial[1] = -70; theta_initial[2] = 400;
	model->move(theta_initial);

	model->update_centers();
	model->compute_outline();

	if (user_name == 0) model->manually_adjust_initial_transformations();
}

/// @note any initialization that has to be done once GL context is active
void Worker::init_graphic_resources() {
	offscreen_renderer.init(camera, model, data_path, true);
	if (save_rastorized_model) rastorizer.init(camera, model, data_path, false);
	sensor_color_texture = new ColorTexture8UC3(camera->width(), camera->height());
	sensor_depth_texture = new DepthTexture16UC1(camera->width(), camera->height());

	tw_settings->tw_add(settings->termination_max_iters, "#iters", "group=Tracker");
	tw_settings->tw_add(settings->termination_max_rigid_iters, "#iters (rigid)", "group=Tracker");

	///--- Initialize the energies modules
	using namespace energy;
	trivial_detector = new TrivialDetector(camera, &offscreen_renderer);
	handfinder = new HandFinder(camera);
	E_fitting.init(this);

	E_limits.init(model);
	E_collision.init(model);
	E_pose.init(this);
	E_temporal.init(model);
	E_damping.init(model);
}

void Worker::cleanup_graphic_resources() {
	delete sensor_color_texture;
	delete sensor_depth_texture;
	E_fitting.cleanup();
}

Worker::~Worker() {
	delete trivial_detector;
	delete handfinder;
	delete model;
}

bool Worker::track_till_convergence() {
	
	for (int i = 0; i < settings->termination_max_iters; ++i) {
		track(i);
		//tracking_error_optimization[i] = tracking_error;
	}
	
	return monitor.is_failure_frame(tracking_error.pull_error, tracking_error.push_error, E_fitting.settings->fit2D_enable);
}

void Worker::track(int iter) {
	bool eval_error = (iter == settings->termination_max_iters - 1);
	bool rigid_only = (iter < settings->termination_max_rigid_iters);

	std::vector<float> _thetas = model->get_theta();

	///--- Serialize matrices for jacobian computation
	model->serializer.serialize_model();
	//model->compute_rendered_indicator(handfinder->sensor_silhouette, camera);

	///--- Optimization phases	
	LinearSystem system(num_thetas);
	//eval_error = true;
	E_fitting.track(current_frame, system, rigid_only, eval_error, tracking_error.push_error, tracking_error.pull_error, iter); ///<!!! MUST BE FIRST CALL		
	E_collision.track(system);
	E_temporal.track(system, current_frame);
	E_limits.track(system, _thetas);
	E_damping.track(system);
	if (rigid_only) 
		energy::Energy::rigid_only(system);
	else
		E_pose.track(system, _thetas); ///<!!! MUST BE LAST CALL	

	///--- Solve 
	VectorN delta_thetas = energy::Energy::solve(system);	

	///--- Update
	const vector<float> dt(delta_thetas.data(), delta_thetas.data() + num_thetas);
	_thetas = model->get_updated_parameters(_thetas, dt);
	model->move(_thetas);
	model->update_centers();
	model->compute_outline();
	E_temporal.update(current_frame.id, _thetas);
}
