#pragma once
#include "ConvolutionRenderer.h"
#include <iostream>
#include <vector>
#include <cstdlib>
#include <Eigen/Dense>
#include <GL/glew.h> 
#include <OpenGP/GL/EigenOpenGLSupport3.h>
#include <QGLBuffer>
#include <QGLShaderProgram>
#include <QOpenGLVertexArrayObject>

#include <iostream>
#include <fstream>

glm::vec3 ConvolutionRenderer::world_to_window_coordinates(glm::vec3 point) {
	Eigen::Matrix4f view_projection = projection * camera.view;
	glm::mat4 MVP_glm = glm::mat4(0);
	for (size_t i = 0; i < 4; i++) {
		for (size_t j = 0; j < 4; j++) {
			MVP_glm[j][i] = projection(i, j);
		}
	}

	glm::vec4 point_gl = MVP_glm * glm::vec4(point, 1.0);
	glm::vec3 point_clip = glm::vec3(point_gl[0], point_gl[1], point_gl[2]) / point_gl[3];
	float f = camera.zFar;
	float n = camera.zNear;

	float ox = window_left + window_width / 2;
	float oy = window_bottom + window_height / 2;

	float xd = point_clip[0];
	float yd = point_clip[1];
	float zd = point_clip[2];

	glm::vec3 point_window = glm::vec3(0, 0, 0);
	point_window[0] = xd * window_width / 2 + ox;
	point_window[1] = yd * window_height / 2 + oy;
	point_window[2] = zd * (f - n) / 2 + (n + f) / 2;

	return point_window;
}

ConvolutionRenderer::ConvolutionRenderer(Model *model, bool real_color, std::string data_path) {
	this->data_path = data_path;
	this->model = model;
	this->real_color = real_color;	
}

ConvolutionRenderer::ConvolutionRenderer(Model *model, ConvolutionRenderer::SHADERMODE mode, const Eigen::Matrix4f& projection, std::string data_path) {
	this->data_path = data_path;
	this->model = model;
	this->init(mode);
	this->projection = projection;
}

void ConvolutionRenderer::send_vertices_to_shader(std::string vertices_name) {

	bool success = vertexbuffer.create(); assert(success);
	vertexbuffer.setUsagePattern(QGLBuffer::StaticDraw);
	success = vertexbuffer.bind(); assert(success);
	vertexbuffer.allocate(points.data(), sizeof(points[0]) * points.size());
	program.setAttributeBuffer(vertices_name.c_str(), GL_FLOAT, 0, 3);
	program.enableAttributeArray(vertices_name.c_str());
}

void ConvolutionRenderer::setup_canvas() {

	points = std::vector<Eigen::Vector3f>(4, Eigen::Vector3f::Zero());
	points[0] = Eigen::Vector3f(-1, -1, 0); points[1] = Eigen::Vector3f(1, -1, 0);
	points[2] = Eigen::Vector3f(-1, 1, 0); points[3] = Eigen::Vector3f(1, 1, 0);
	send_vertices_to_shader("position");

	/// Specify window bounds
	glUniform1f(glGetUniformLocation(program.programId(), "window_left"), window_left);
	glUniform1f(glGetUniformLocation(program.programId(), "window_bottom"), window_bottom);
	glUniform1f(glGetUniformLocation(program.programId(), "window_height"), window_height);
	glUniform1f(glGetUniformLocation(program.programId(), "window_width"), window_width);
}

void ConvolutionRenderer::pass_model_to_shader(bool fingers_only) {

	if (mode == FRAMEBUFFER) {
		glm::vec3 min_x_world = glm::vec3(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());
		glm::vec3 min_y_world = glm::vec3(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());
		glm::vec3 max_x_world = -glm::vec3(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());
		glm::vec3 max_y_world = -glm::vec3(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());

		int num_centers = model->centers.size();
		if (fingers_only) num_centers = 34;
		for (size_t i = 0; i < num_centers; i++) {
			if (model->centers[i][0] - model->radii[i] < min_x_world[0]) min_x_world = model->centers[i] - model->radii[i];
			if (model->centers[i][1] - model->radii[i] < min_y_world[1]) min_y_world = model->centers[i] - model->radii[i];
			if (model->centers[i][0] + model->radii[i] > max_x_world[0]) max_x_world = model->centers[i] + model->radii[i];
			if (model->centers[i][1] + model->radii[i] > max_y_world[1]) max_y_world = model->centers[i] + model->radii[i];
		}
		glm::vec3 min_x_window = world_to_window_coordinates(min_x_world);
		glm::vec3 min_y_window = world_to_window_coordinates(min_y_world);
		glm::vec3 max_x_window = world_to_window_coordinates(max_x_world);
		glm::vec3 max_y_window = world_to_window_coordinates(max_y_world);

		glUniform1f(glGetUniformLocation(program.programId(), "min_x"), min_x_window[0]);
		glUniform1f(glGetUniformLocation(program.programId(), "min_y"), min_y_window[1]);
		glUniform1f(glGetUniformLocation(program.programId(), "max_x"), max_x_window[0]);
		glUniform1f(glGetUniformLocation(program.programId(), "max_y"), max_y_window[1]);

		glUniform1i(glGetUniformLocation(program.programId(), "fingers_only"), fingers_only);
		/*for (size_t i = 0; i < 4; i++) {
			for (size_t j= 0; j< 4; j++) {
			cout << camera.MVP_glm[i][j] << " ";
			}
			cout << endl;
			}
			cout << endl << endl;

			cout << "min_x_world = " << min_x_world[0] << endl;
			cout << "min_y_world = " << min_y_world[1] << endl;
			cout << "max_x_world = " << max_x_world[0] << endl;
			cout << "max_y_world = " << max_y_world[1] << endl;

			cout << "min_x = " << min_x_window[0] << endl;
			cout << "min_y = " << min_y_window[1] << endl;
			cout << "max_x = " << max_x_window[0] << endl;
			cout << "max_y = " << max_y_window[1] << endl;*/
	}

	glUniform1f(glGetUniformLocation(program.programId(), "num_blocks"), blocks.size());
	glUniform3fv(glGetUniformLocation(program.programId(), "centers"), model->centers.size(), (GLfloat *)model->centers.data());
	glUniform1fv(glGetUniformLocation(program.programId(), "radii"), model->radii.size(), (GLfloat *)model->radii.data());
	glUniform3iv(glGetUniformLocation(program.programId(), "blocks"), model->blocks.size(), (GLint *)model->blocks.data());

	tangents_v1 = std::vector<Eigen::Vector3f>(model->tangent_points.size(), Eigen::Vector3f());
	tangents_v2 = std::vector<Eigen::Vector3f>(model->tangent_points.size(), Eigen::Vector3f());
	tangents_v3 = std::vector<Eigen::Vector3f>(model->tangent_points.size(), Eigen::Vector3f());
	tangents_u1 = std::vector<Eigen::Vector3f>(model->tangent_points.size(), Eigen::Vector3f());
	tangents_u2 = std::vector<Eigen::Vector3f>(model->tangent_points.size(), Eigen::Vector3f());
	tangents_u3 = std::vector<Eigen::Vector3f>(model->tangent_points.size(), Eigen::Vector3f());
	for (size_t i = 0; i < model->tangent_points.size(); i++) {
		tangents_v1[i] = Eigen::Vector3f(model->tangent_points[i].v1[0], model->tangent_points[i].v1[1], model->tangent_points[i].v1[2]);
		tangents_v2[i] = Eigen::Vector3f(model->tangent_points[i].v2[0], model->tangent_points[i].v2[1], model->tangent_points[i].v2[2]);
		tangents_v3[i] = Eigen::Vector3f(model->tangent_points[i].v3[0], model->tangent_points[i].v3[1], model->tangent_points[i].v3[2]);
		tangents_u1[i] = Eigen::Vector3f(model->tangent_points[i].u1[0], model->tangent_points[i].u1[1], model->tangent_points[i].u1[2]);
		tangents_u2[i] = Eigen::Vector3f(model->tangent_points[i].u2[0], model->tangent_points[i].u2[1], model->tangent_points[i].u2[2]);
		tangents_u3[i] = Eigen::Vector3f(model->tangent_points[i].u3[0], model->tangent_points[i].u3[1], model->tangent_points[i].u3[2]);
	}

	glUniform3fv(glGetUniformLocation(program.programId(), "tangents_v1"), tangents_v1.size(), (GLfloat *)tangents_v1.data());
	glUniform3fv(glGetUniformLocation(program.programId(), "tangents_v2"), tangents_v2.size(), (GLfloat *)tangents_v2.data());
	glUniform3fv(glGetUniformLocation(program.programId(), "tangents_v3"), tangents_v3.size(), (GLfloat *)tangents_v3.data());
	glUniform3fv(glGetUniformLocation(program.programId(), "tangents_u1"), tangents_u1.size(), (GLfloat *)tangents_u1.data());
	glUniform3fv(glGetUniformLocation(program.programId(), "tangents_u2"), tangents_u2.size(), (GLfloat *)tangents_u2.data());
	glUniform3fv(glGetUniformLocation(program.programId(), "tangents_u3"), tangents_u3.size(), (GLfloat *)tangents_u3.data());
}

void ConvolutionRenderer::setup_texture() {
	QImage texture_image;
	if (!texture_image.load(QString::fromUtf8(data_path.c_str()) + "shaders//skin_texture.png")) std::cerr << "error loading" << std::endl;
	QImage formatted_image = QGLWidget::convertToGLFormat(texture_image);
	if (formatted_image.isNull()) std::cerr << "error formatting" << std::endl;

	const GLfloat vtexcoord[] = { 0, 0, 1, 0, 0, 1, 1, 1 };

	glGenTextures(1, &synthetic_texture_id);
	glBindTexture(GL_TEXTURE_2D, synthetic_texture_id);

	bool success = texturebuffer.create(); assert(success);
	texturebuffer.setUsagePattern(QGLBuffer::StaticDraw);
	success = texturebuffer.bind(); assert(success);
	texturebuffer.allocate(vtexcoord, sizeof(vtexcoord));
	program.setAttributeBuffer("vtexcoord", GL_FLOAT, 0, 2);
	program.enableAttributeArray("vtexcoord");

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, formatted_image.width(), formatted_image.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, formatted_image.bits());
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glUniform1i(glGetUniformLocation(program.programId(), "synthetic_texture"), 0);
}

void ConvolutionRenderer::setup_texture(cv::Mat & image) {

	glGenTextures(1, &real_texture_id);
	glBindTexture(GL_TEXTURE_2D, real_texture_id);

	cv::flip(image, image, 0);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.cols, image.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, image.ptr());

	glUniform1i(glGetUniformLocation(program.programId(), "real_texture"), 2);

}

void ConvolutionRenderer::setup_silhoeutte() {
	glGenTextures(1, &silhouette_texture_id);
	glBindTexture(GL_TEXTURE_2D, silhouette_texture_id);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, model->silhouette_texture.cols, model->silhouette_texture.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, model->silhouette_texture.ptr());
	glUniform1i(glGetUniformLocation(program.programId(), "silhouette"), 1);

}

void ConvolutionRenderer::init(ConvolutionRenderer::SHADERMODE mode) {
	this->mode = mode;
	if (!vao.isCreated()) {
		bool success = vao.create();
		assert(success);
		vao.bind();
	}

	switch (mode) {
	case NORMAL:
		vertex_shader_name = QString::fromUtf8(data_path.c_str()) + "shaders//" + "model_vshader.glsl";
		fragment_shader_name = QString::fromUtf8(data_path.c_str()) + "shaders//" + "model_fshader.glsl";
		break;
	case FRAMEBUFFER:
		vertex_shader_name = QString::fromUtf8(data_path.c_str()) + "shaders//" + "model_vshader.glsl";
		fragment_shader_name = QString::fromUtf8(data_path.c_str()) + "shaders//" + "model_FB_fshader.glsl";
		window_width = 320; window_height = 240;
		break;
	case RASTORIZER:
		vertex_shader_name = QString::fromUtf8(data_path.c_str()) + "shaders//" + "model_vshader.glsl";
		fragment_shader_name = QString::fromUtf8(data_path.c_str()) + "shaders//" + "model_rastorizer_fshader.glsl";
		window_width = 320; window_height = 240;
		break;
	}

	bool vok = program.addShaderFromSourceFile(QGLShader::Vertex, vertex_shader_name);
	bool fok = program.addShaderFromSourceFile(QGLShader::Fragment, fragment_shader_name);
	bool lok = program.link();
	if (!(lok && vok && fok)) {
		std::cout << "shader compile error: " << std::endl;
		std::cout << "vshader: " << vertex_shader_name.toStdString() << std::endl;
		std::cout << "fshader: " << fragment_shader_name.toStdString() << std::endl;
		std::cout << "shaders log: " << program.log().toStdString() << std::endl;
		exit(EXIT_FAILURE);
	}
	bool success = program.bind();
	assert(success);

	setup_canvas();
	setup_texture();
	setup_silhoeutte();

	material.setup(program.programId());
	light.setup(program.programId());

	camera.setup(program.programId(), projection);

	program.release();
	vao.release();
}

void ConvolutionRenderer::render() {
	vao.bind();
	program.bind();

	//cout << "render" << endl;
	camera.setup(program.programId(), projection);
	if (real_color) setup_texture(model->real_color);
	pass_model_to_shader(false);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, synthetic_texture_id);

	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, silhouette_texture_id);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, model->silhouette_texture.cols, model->silhouette_texture.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, model->silhouette_texture.ptr());

	if (real_color) {
		glActiveTexture(GL_TEXTURE2);
		glBindTexture(GL_TEXTURE_2D, real_texture_id);
	}

	glDrawArrays(GL_TRIANGLE_STRIP, 0, points.size());

	program.release();
	vao.release();
}

void ConvolutionRenderer::render_offscreen(bool fingers_only) {
	vao.bind();
	program.bind();

	//cout << "render_offscreen" << endl;
	camera.setup(program.programId(), projection);
	pass_model_to_shader(fingers_only);

	glDrawArrays(GL_TRIANGLE_STRIP, 0, points.size());

	program.release();
	vao.release();
}