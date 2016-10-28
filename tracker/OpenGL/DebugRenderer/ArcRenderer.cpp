#include "ArcRenderer.h"
#define M_PI 3.14159265358979323846

ArcRenderer::ArcRenderer(const std::vector<pair<Vector3, Vector3>>& endpoints, std::vector<Vector3> centers, std::vector<float> radii, Vector3 color) {
	this->init(); ///< compile shaders

	int num_samples = 10;
	Vector3 u = Vector3(1, 0, 0);
	Vector3 v = Vector3(0, 1, 0);
	Matrix_3xN _segments(3, 2 * endpoints.size() * num_samples);
	int count = 0;
	float phi;
	for (size_t i = 0; i < endpoints.size(); i++) {
		Vector3 v1 = endpoints[i].first - centers[i];
		Vector3 v2 = endpoints[i].second - centers[i];
		float alpha = atan2(v1[0], v1[1]);
		float beta = atan2(v2[0], v2[1]);
		if (beta > alpha) alpha = alpha + 2 * M_PI; 
		for (size_t j = 0; j < num_samples; j++) {
			phi = alpha + j * (beta - alpha) / (num_samples - 1);
			//phi = j * 2 * M_PI / num_samples;
			_segments.col(count++) = centers[i] + radii[i] * (u * sin(phi) + v * cos(phi));
			phi = alpha + (j + 1) * (beta - alpha) / (num_samples - 1);
			//phi = (j + 1) * 2 * M_PI / num_samples;
			_segments.col(count++) = centers[i] + radii[i] * (u * sin(phi) + v * cos(phi));
		}
	}

	Matrix_3xN _colors(3, 2 * endpoints.size() * num_samples);
	for (int i = 0; i < 3; i++)
		_colors.row(i).array().setConstant(color(i));
	setup(&_segments, &_colors);
}

void ArcRenderer::render() {
	if (num_primitives == 0) return;
	vao.bind();
	program.bind();
	glEnable(GL_LINE_SMOOTH);
	glDrawArrays(GL_LINES, 0, num_primitives);
	glDisable(GL_LINE_SMOOTH);
	program.release();
	vao.release();
}
