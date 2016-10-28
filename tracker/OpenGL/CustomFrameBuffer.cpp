#include "tracker/OpenGL/CustomFrameBuffer.h"

#include "opencv2/highgui/highgui.hpp" ///< cv::imShow
#include "opencv2/contrib/contrib.hpp" ///< cvtConvert
#include "opencv2/imgproc/types_c.h"   ///< CV_BGRA2RGBA

#include "util/mylogger.h"
#include <cassert>

CustomFrameBuffer::CustomFrameBuffer() :needs_cleanup(false) {}

CustomFrameBuffer::CustomFrameBuffer(int image_width, int image_height, bool render_block_id) {
	needs_cleanup = false;
	init(image_width, image_height, render_block_id);
}

CustomFrameBuffer::~CustomFrameBuffer() { 
	if (needs_cleanup) cleanup(); 
}

bool CustomFrameBuffer::ready() {
	return needs_cleanup;
}

void CustomFrameBuffer::bind(bool render_block_id) {
	glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

	/*if (render_block_id == false) {
		const GLenum buffers[] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1 };
		glDrawBuffers(2, buffers);
	}*/
}

void CustomFrameBuffer::unbind() {
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void CustomFrameBuffer::init(int image_width, int image_height, bool render_block_id) {
	std::cout << "init" << std::endl;
	CHECK(needs_cleanup == false);
	this->image_width = image_width;
	this->image_height = image_height;
	if (render_block_id) {
		color_tex = create_color_attachment(image_width, image_height);
	}
	else { // render depth
		color_tex = create_depth_attachment(image_width, image_height);
		//normals_tex = create_normals_attachment(image_width, image_height);
	}
	framebuffer = create_framebuffer(render_block_id);
	needs_cleanup = true;
}

void CustomFrameBuffer::cleanup() {
	assert(needs_cleanup == true);
	//Delete resources
	glDeleteTextures(1, &color_tex);

	//Bind 0, which means render to back buffer, as a result, fb is unbound
	glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0);
	glDeleteFramebuffers(1, &framebuffer);
	needs_cleanup = false;
}

GLuint CustomFrameBuffer::create_color_attachment(int image_width, int image_height) {
	GLuint tex_screen;
	glGenTextures(1, &tex_screen);
	glBindTexture(GL_TEXTURE_2D, tex_screen);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_R8UI, image_width, image_height, 0, GL_RED_INTEGER, GL_UNSIGNED_BYTE, NULL);
	return tex_screen;
}

GLuint CustomFrameBuffer::create_depth_attachment(int image_width, int image_height) {
	GLuint texture;
	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_R16UI, image_width, image_height, 0, GL_RED_INTEGER, GL_UNSIGNED_SHORT, NULL);
	return texture;
}

GLuint CustomFrameBuffer::create_normals_attachment(int image_width, int image_height) {
	GLuint texture;
	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, image_width, image_height, 0, GL_RGB, GL_FLOAT, NULL);	
	return texture;
}

void CustomFrameBuffer::fetch_color_attachment(cv::Mat& image) {
	static cv::Mat gray;
	if (gray.empty()) gray = cv::Mat(image_height, image_width, CV_8UC1, cv::Scalar(0));
	if (image.empty()) image = cv::Mat(image_height, image_width, CV_8UC3, cv::Scalar(0));
	glBindTexture(GL_TEXTURE_2D, color_tex);
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RED_INTEGER, GL_UNSIGNED_BYTE, gray.data);
	glBindTexture(GL_TEXTURE_2D, 0);
	int from_to[] = { 0, 0 };
	cv::mixChannels(&gray, 1, &image, 1, from_to, 1);
}

void CustomFrameBuffer::fetch_depth_attachment(cv::Mat& image) {
	if (image.empty()) image = cv::Mat(image_height, image_width, CV_16UC1, cv::Scalar(0));
	glBindTexture(GL_TEXTURE_2D, color_tex);
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RED_INTEGER, GL_UNSIGNED_SHORT, image.data);
	glBindTexture(GL_TEXTURE_2D, 0);
}

void CustomFrameBuffer::fetch_normals_attachment(cv::Mat& image) {
	if (image.empty()) image = cv::Mat(image_height, image_width, CV_32FC3, cv::Scalar(0));
	glBindTexture(GL_TEXTURE_2D, normals_tex);
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_FLOAT, image.data);
	glBindTexture(GL_TEXTURE_2D, 0);
}

void CustomFrameBuffer::display_color_attachment() {
	static cv::Mat image;
	fetch_color_attachment(image);
	cv::flip(image, image, 0 /*flip rows*/);
	cv::imshow("color_channel", image);
}

void CustomFrameBuffer::display_depth_attachment() {
	static cv::Mat image;
	fetch_depth_attachment(image);
	cv::flip(image, image, 0 /*flip rows*/);
	cv::imshow("depth_channel", image);
}

void CustomFrameBuffer::display_normals_attachment() {
	static cv::Mat image;
	fetch_normals_attachment(image);
	cv::flip(image, image, 0 /*flip rows*/);
	cv::imshow("normals_channel", image);
}

GLuint CustomFrameBuffer::create_framebuffer(bool render_block_id) {
	GLuint fbo;
	// create and bind a framebuffer
	glGenFramebuffers(1, &fbo);
	glBindFramebuffer(GL_FRAMEBUFFER_EXT, fbo);

	if (render_block_id) {
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 /*location = 0*/, GL_TEXTURE_2D, color_tex, 0 /*level*/);
	}
	else {
		//glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1 /*location = 0*/, GL_TEXTURE_2D, normals_tex, 0 /*level*/);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 /*location = 1*/, GL_TEXTURE_2D, color_tex, 0 /*level*/);
	}

	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
		std::cerr << "Framebuffer not OK." << std::endl;
	glBindFramebuffer(GL_FRAMEBUFFER, 0); ///< avoid pollution
	return fbo;
}

