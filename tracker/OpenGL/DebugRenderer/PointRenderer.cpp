#include "PointRenderer.h"

PointRenderer::PointRenderer(Matrix_3xN& points) {
	this->init(); ///< compile shaders
	setup(&points, NULL);
}

PointRenderer::PointRenderer(const std::vector<Vector3>& points, const std::vector<Vector3>& colors) {
	this->init();
	Matrix_3xN _points(3, points.size());
	for (int i = 0; i < points.size(); i++)
		_points.col(i) = points[i];
	Matrix_3xN _colors(3, colors.size());
	for (int i = 0; i < colors.size(); i++)
		_colors.col(i) = colors[i];
	setup(&_points, &_colors);
}

PointRenderer::PointRenderer(const std::vector<Vector3>& points, Vector3 color) {
	this->init(); ///< compile shaders
	Matrix_3xN data(3, points.size());
	for (int i = 0; i < points.size(); i++)
		data.col(i) = points[i];
	Matrix_3xN colors(3, points.size());
	for (int i = 0; i < 3; i++)
		colors.row(i).array().setConstant(color(i));
	setup(&data, &colors);
}

void PointRenderer::render() {
	if (num_primitives == 0) return;
	vao.bind();
	program.bind();
	glEnable(GL_PROGRAM_POINT_SIZE);
	glDrawArrays(GL_POINTS, 0, num_primitives);
	glDisable(GL_PROGRAM_POINT_SIZE);
	program.release();
	vao.release();
}

