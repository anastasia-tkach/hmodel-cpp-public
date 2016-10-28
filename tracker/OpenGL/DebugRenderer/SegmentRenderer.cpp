#include "SegmentRenderer.h"

SegmentRenderer::SegmentRenderer(const std::vector<pair<Vector3, Vector3>>& segments, const std::vector<Vector3>& colors) {
	this->init();
	Matrix_3xN _segments(3, segments.size() * 2);
	for (int i = 0; i < segments.size(); i++) {
		_segments.col(i * 2) = segments[i].first;
		_segments.col(i * 2 + 1) = segments[i].second;
	}
	Matrix_3xN _colors(3, colors.size() * 2);
	for (int i = 0; i < colors.size(); i++) {
		_colors.col(i * 2) = colors[i];
		_colors.col(i * 2 + 1) = colors[i];
	}
	setup(&_segments, &_colors);
}

SegmentRenderer::SegmentRenderer(const std::vector<pair<Vector3, Vector3>>& segments, Vector3 color) {
	this->init(); ///< compile shaders
	Matrix_3xN _segments(3, segments.size() * 2);
	for (int i = 0; i < segments.size(); i++) {
		_segments.col(i * 2) = segments[i].first;
		_segments.col(i * 2 + 1) = segments[i].second;
	}
	Matrix_3xN _colors(3, segments.size() * 2);
	for (int i = 0; i < 3; i++)
		_colors.row(i).array().setConstant(color(i));
	setup(&_segments, &_colors);
}

void SegmentRenderer::render() {
	if (num_primitives == 0) return;
	vao.bind();
	program.bind();
	glEnable(GL_LINE_SMOOTH);
	glDrawArrays(GL_LINES, 0, num_primitives);
	glDisable(GL_LINE_SMOOTH);
	program.release();
	vao.release();
}
