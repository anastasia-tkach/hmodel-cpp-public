
#include "util/gl_wrapper.h"
#include "util/OpenGL32Format.h" 
#include "apps/hmodel_atb/AntTweakBarEventFilter.h"

#include "TwSettings.h"
#include "GLWidget.h"
#include "tracker/Data/Camera.h"
#include "tracker/Data/DataStream.h"
#include "tracker/Worker.h"
#include "tracker/Data/SolutionStream.h"
#include "tracker/Data/TextureColor8UC3.h"
#include "tracker/Data/TextureDepth16UC1.h"
#include "tracker/HandFinder/HandFinder.h"

#include "tracker/OpenGL/DebugRenderer/DebugRenderer.h"

#define M_PI 3.14159265358979323846

GLWidget::GLWidget(Worker* worker, DataStream * datastream, SolutionStream * solutions, bool playback, bool real_color, std::string data_path) :
QGLWidget(OpenGL32Format()),
worker(worker),
datastream(datastream),
solutions(solutions),
_camera(worker->camera),
convolution_renderer(worker->model, real_color, data_path) {
	this->playback = playback;
	this->data_path = data_path;
	this->resize(640 * 2, 480 * 2);
	this->move(1250, 375);
	convolution_renderer.window_width = this->width();
	convolution_renderer.window_height = this->height();

	std::cout << "Started OpenGL " << this->format().majorVersion() << "." << this->format().minorVersion() << std::endl;
	this->installEventFilter(new AntTweakBarEventFilter(this)); ///< all actions pass through filter
}

GLWidget::~GLWidget() {
	worker->cleanup_graphic_resources();
	tw_settings->tw_cleanup();
}

void GLWidget::initializeGL() {
	std::cout << "GLWidget::initializeGL()" << std::endl;
	initialize_glew();
	tw_settings->tw_init(this->width(), this->height()); ///< FIRST!!

	glEnable(GL_DEPTH_TEST);

	kinect_renderer.init(_camera);

	///--- Initialize other graphic resources
	this->makeCurrent();
	worker->init_graphic_resources();

	///--- Setup with data from worker
	kinect_renderer.setup(worker->sensor_color_texture->texid(), worker->sensor_depth_texture->texid());

	convolution_renderer.projection = _camera->view_projection_matrix();
	convolution_renderer.init(ConvolutionRenderer::NORMAL);
}

void GLWidget::paintGL() {
	glViewport(0, 0, this->width(), this->height());
	glClearColor(1, 1, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	///--- Rendering
	Eigen::Matrix4f view_projection = _camera->view_projection_matrix() * view;
	if (worker->handfinder->wristband_found()) {
		kinect_renderer.enable_colormap(true);
		kinect_renderer.set_zNear(worker->handfinder->wristband_center()[2] - 150);
		kinect_renderer.set_zFar(worker->handfinder->wristband_center()[2] + 150);
	}
	kinect_renderer.set_uniform("view_projection", view_projection);
	kinect_renderer.render();

	glDisable(GL_BLEND);
	convolution_renderer.render();

	//worker->model->render_outline();
	//DebugRenderer::instance().set_uniform("view_projection", view_projection);
	//DebugRenderer::instance().render();

	//tw_settings->tw_draw();
}

void GLWidget::process_mouse_movement(GLfloat cursor_x, GLfloat cursor_y) {
	glm::vec3 image_center_glm = worker->model->centers[worker->model->centers_name_to_id_map["palm_back"]] +
		worker->model->centers[worker->model->centers_name_to_id_map["palm_middle"]];
	image_center = Eigen::Vector3f(image_center_glm[0] / 2, image_center_glm[1] / 2 + 30, image_center_glm[2] / 2);
	float d = (camera_center - image_center).norm();

	float delta_x = cursor_x - cursor_position[0];
	float delta_y = cursor_y - cursor_position[1];

	float theta = initial_euler_angles[0] + cursor_sensitivity * delta_x;
	float phi = initial_euler_angles[1] + cursor_sensitivity * delta_y;

	Eigen::Vector3f x = sin(theta) * sin(phi) * Eigen::Vector3f::UnitX();
	Eigen::Vector3f y = cos(phi) * Eigen::Vector3f::UnitY();
	Eigen::Vector3f z = cos(theta) * sin(phi) * Eigen::Vector3f::UnitZ();

	camera_center = image_center + d * (x + y + z);
	euler_angles = Eigen::Vector2f(theta, phi);

	Vector3 f, u, s;
	f = (image_center - camera_center).normalized();
	u = camera_up.normalized();
	s = u.cross(f).normalized();
	u = f.cross(s);
	view.block(0, 0, 1, 3) = s.transpose();
	view(0, 3) = -s.dot(camera_center);
	view.block(1, 0, 1, 3) = u.transpose();
	view(1, 3) = -u.dot(camera_center);
	view.block(2, 0, 1, 3) = f.transpose();
	view(2, 3) = -f.dot(camera_center);

	// set view matrix 
	convolution_renderer.camera.view = view;
	convolution_renderer.camera.camera_center = camera_center;

	worker->offscreen_renderer.convolution_renderer->camera.view = view;
	worker->offscreen_renderer.convolution_renderer->camera.camera_center = camera_center;
}

void GLWidget::process_mouse_button_pressed(GLfloat cursor_x, GLfloat cursor_y) {
	mouse_button_pressed = true;
	cursor_position = Eigen::Vector2f(cursor_x, cursor_y);
}

void GLWidget::process_mouse_button_released() {
	initial_euler_angles = euler_angles;
}

void GLWidget::mouseMoveEvent(QMouseEvent *event) {
	if (event->buttons() == Qt::LeftButton) {
		process_mouse_movement(event->x(), event->y());
	}
	else {
		if (mouse_button_pressed == true) {
			process_mouse_button_released();
			mouse_button_pressed = false;
		}
	}
}

void GLWidget::mousePressEvent(QMouseEvent *event) {
	process_mouse_button_pressed(event->x(), event->y());
}

void GLWidget::wheelEvent(QWheelEvent * event) {}

void GLWidget::keyPressEvent(QKeyEvent *event) {
	GLWidget* qglviewer = this;
	switch (event->key()) {
	case Qt::Key_Escape: {
		this->close();
	}
	break;
	case Qt::Key_S: {
		cout << "set up path for saving images" << std::endl;
		//datastream->save_as_images(data_path);
	}
	break;
	case Qt::Key_1: {
		cout << "uniform scaling up" << endl;
		worker->model->resize_model(1.05, 1.0, 1.0);
	}
	break;
	case Qt::Key_2: {
		cout << "uniform scaling down" << endl;
		worker->model->resize_model(0.95, 1.0, 1.0);
	}
	break;
	case Qt::Key_3: {
		cout << "width scaling up" << endl;
		worker->model->resize_model(1.0, 1.05, 1.0);
	}
	break;
	case Qt::Key_4: {
		cout << "width scaling down" << endl;
		worker->model->resize_model(1.0, 0.95, 1.0);
	}
	break;
	case Qt::Key_5: {
		cout << "thickness scaling up" << endl;
		worker->model->resize_model(1.0, 1.0, 1.05);
	}
	break;
	case Qt::Key_6: {
		cout << "thickness scaling down" << endl;
		worker->model->resize_model(1.0, 1.0, 0.95);
	}
	break;
	}
}

