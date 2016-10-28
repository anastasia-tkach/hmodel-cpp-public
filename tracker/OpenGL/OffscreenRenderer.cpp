#include "OffscreenRenderer.h"

#include "util/tictoc.h"
#include "util/gl_wrapper.h"
#include "util/mylogger.h"
#include "tracker/Data/Camera.h"
#include "tracker/OpenGL/ConvolutionRenderer/ConvolutionRenderer.h"
#include "tracker/OpenGL/CustomFrameBuffer.h"

void OffscreenRenderer::init(Camera* camera, Model * model, std::string data_path, bool render_block_id) {
	this->camera = camera;
	this->model = model;
	
	if (render_block_id) {
		frame_buffer = new CustomFrameBuffer(camera->width(), camera->height(), render_block_id);
		convolution_renderer = new ConvolutionRenderer(model, ConvolutionRenderer::FRAMEBUFFER, camera->view_projection_matrix(), data_path);
	}
	else { // render_depth
		frame_buffer = new CustomFrameBuffer(camera->width(), camera->height(), render_block_id);
		convolution_renderer = new ConvolutionRenderer(model, ConvolutionRenderer::RASTORIZER, camera->view_projection_matrix(), data_path);
	}
}

OffscreenRenderer::~OffscreenRenderer() {
	delete frame_buffer;
	delete convolution_renderer;
}

void OffscreenRenderer::render_offscreen(bool last_iter, bool fingers_only, bool reinit) {
	
	glViewport(0, 0, camera->width(), camera->height());
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glDisable(GL_BLEND); ///< just in case
	glEnable(GL_DEPTH_TEST);

	//frame_buffer->display_color_attachment();

	frame_buffer->bind(true);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	convolution_renderer->render_offscreen(fingers_only);
	frame_buffer->unbind();

	if (last_iter) frame_buffer->fetch_color_attachment(model->silhouette_texture);

	glFinish();
}

void OffscreenRenderer::rastorize_model(cv::Mat & rastorized_model) {

	glViewport(0, 0, camera->width(), camera->height());
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glDisable(GL_BLEND); ///< just in case
	glEnable(GL_DEPTH_TEST);

	frame_buffer->bind(false);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	convolution_renderer->render_offscreen(false);
	frame_buffer->unbind();

	//frame_buffer->display_depth_attachment();	
	frame_buffer->fetch_depth_attachment(rastorized_model);
	cv::flip(rastorized_model, rastorized_model, 0);
	//frame_buffer->fetch_normals_attachment(rastorized_normals);
	//cv::flip(rastorized_normals, rastorized_normals, 0);

	glFinish();	
}

