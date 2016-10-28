#pragma once
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

#include "tracker/HModel/Model.h"

#include "opencv2/core/core.hpp"       ///< cv::Mat
#include "opencv2/highgui/highgui.hpp" ///< cv::imShow

class ConvolutionRenderer {	
	static const int ONE = 1;

	struct Light {
		Eigen::Vector3f Ia = Eigen::Vector3f(1.0f, 1.0f, 1.0f);
		Eigen::Vector3f Id = Eigen::Vector3f(1.0f, 1.0f, 1.0f);
		Eigen::Vector3f Is = Eigen::Vector3f(1, 1, 1);
		Eigen::Vector3f light_pos = Eigen::Vector3f(0.0f, 0.0f, 0.01f);

		void setup(GLuint program_id) {
			glUseProgram(program_id);
			GLuint light_pos_id = glGetUniformLocation(program_id, "light_pos"); //Given in camera space
			GLuint Ia_id = glGetUniformLocation(program_id, "Ia");
			GLuint Id_id = glGetUniformLocation(program_id, "Id");
			GLuint Is_id = glGetUniformLocation(program_id, "Is");
			glUniform3fv(light_pos_id, ONE, light_pos.data());
			glUniform3fv(Ia_id, ONE, Ia.data());
			glUniform3fv(Id_id, ONE, Id.data());
			glUniform3fv(Is_id, ONE, Is.data());
		}
	};

	struct Material {
		Eigen::Vector3f ka = 0.65 * Eigen::Vector3f(0.9176, 0.7412, 0.6157);
		Eigen::Vector3f kd = 0.75 * Eigen::Vector3f(0.9176, 0.7412, 0.6157);
		//Eigen::Vector3f ka = 0.5 * Eigen::Vector3f(0.9373, 0.7490, 0.6627);
		//Eigen::Vector3f kd = 0.7 * Eigen::Vector3f(0.9373, 0.7490, 0.6627);
		Eigen::Vector3f ks = Eigen::Vector3f(0, 0, 0);
		float p = 60.0f;

		void setup(GLuint program_id) {
			glUseProgram(program_id);
			GLuint ka_id = glGetUniformLocation(program_id, "ka");
			GLuint kd_id = glGetUniformLocation(program_id, "kd");
			GLuint ks_id = glGetUniformLocation(program_id, "ks");
			GLuint p_id = glGetUniformLocation(program_id, "p");
			glUniform3fv(ka_id, ONE, ka.data());
			glUniform3fv(kd_id, ONE, kd.data());
			glUniform3fv(ks_id, ONE, ks.data());
			glUniform1f(p_id, p);
		}
	};

	struct Camera {

		float fovy = 45.0f;
		float zNear = 0.01f;
		float zFar = 5000.0f;
		Eigen::Vector3f camera_center = Eigen::Vector3f(0, 0, 0);		
		Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f projection;
		Eigen::Matrix4f MVP;
		Eigen::Matrix4f invMVP;

		void setup(GLuint program_id, Eigen::Matrix4f projection) {
			//View_projection matrix is equal to the bellow plus small correction terms
			/*float F = 287.26;
			float w = 320;
			float h = 240;
			zFar = 1000;
			zNear = 300;
			projection = Eigen::Matrix4f::Zero();
			projection << 2 * F / w, 0, 0, 0,
			0, 2 * F / h, 0, 0,
			0, 0, -(zFar + zNear) / (zNear - zFar), (2 * zFar * zNear) / (zNear - zFar),
			0, 0, 1, 0;*/
			//float aspect = window_width / (float)window_height;
			//projection = Eigen::perspective(fovy, aspect, zNear, zFar);
		
			//We need to modify Eigen::lookAt, because Htrack uses a non-standard left camera frame
			//view = Eigen::lookAt(camera_center, image_center, camera_up);


			MVP = projection * view * model;
			invMVP = MVP.inverse();
			
			glUniform3fv(glGetUniformLocation(program_id, "camera_center"), 1, camera_center.data());
			glUniformMatrix4fv(glGetUniformLocation(program_id, "MVP"), 1, GL_FALSE, MVP.data());
			glUniformMatrix4fv(glGetUniformLocation(program_id, "invMVP"), 1, GL_FALSE, invMVP.data());

			glUniformMatrix4fv(glGetUniformLocation(program_id, "model"), ONE, false, model.data());
			glUniformMatrix4fv(glGetUniformLocation(program_id, "view"), ONE, false, view.data());
			glUniformMatrix4fv(glGetUniformLocation(program_id, "projection"), ONE, false, projection.data());
		}
	};

public:
	enum SHADERMODE { NORMAL, FRAMEBUFFER, RASTORIZER };

	int window_left = 0;
	int window_bottom = 0;
	int window_width;
	int window_height;

	Light light;
	Material material;
	Camera camera;

	GLuint synthetic_texture_id;
	GLuint silhouette_texture_id;
	GLuint real_texture_id;
	QString vertex_shader_name;
	QString fragment_shader_name;
	std::vector<Eigen::Vector3f> points;

	QGLShaderProgram program;
	QOpenGLVertexArrayObject vao;

	QGLBuffer vertexbuffer = QGLBuffer(QGLBuffer::VertexBuffer);
	QGLBuffer texturebuffer = QGLBuffer(QGLBuffer::VertexBuffer);

	std::vector<Eigen::Vector3f> centers;
	std::vector<float> radii;
	std::vector<Eigen::Vector3i> blocks;
	std::vector<Eigen::Vector3f> tangents_v1;
	std::vector<Eigen::Vector3f> tangents_v2;
	std::vector<Eigen::Vector3f> tangents_v3;
	std::vector<Eigen::Vector3f> tangents_u1;
	std::vector<Eigen::Vector3f> tangents_u2;
	std::vector<Eigen::Vector3f> tangents_u3;

	Cylinders *cylinders;
	Eigen::Matrix4f projection;
	Model * model;
	SHADERMODE mode;
	bool real_color;
	std::string data_path;
	
	ConvolutionRenderer(Model *model, bool real_color, std::string data_path);

	ConvolutionRenderer(Model *model, ConvolutionRenderer::SHADERMODE mode, const Eigen::Matrix4f& view_projection, std::string data_path);

	void send_vertices_to_shader(std::string vertices_name);

	void setup_canvas();

	void pass_model_to_shader(bool fingers_only);

	void setup_texture(cv::Mat & color_frame);

	void setup_texture();

	void setup_silhoeutte();
	
	void init(ConvolutionRenderer::SHADERMODE);

	void render();

	void render_offscreen(bool fingers_only);

	glm::vec3 world_to_window_coordinates(glm::vec3 point);
};

