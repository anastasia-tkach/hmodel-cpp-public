#pragma once
#include "tracker/Types.h"
#include "tracker/OpenGL/GeometricPrimitiveRenderer.h"

class ArcRenderer : public GeometricPrimitiveRenderer {

public:
	ArcRenderer(const std::vector<pair<Vector3, Vector3>>& endpoints, std::vector<Vector3> centers, std::vector<float> radii, Vector3 color);

	void render();
};
