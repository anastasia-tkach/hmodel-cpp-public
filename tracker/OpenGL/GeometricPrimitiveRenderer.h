#pragma once
#include "tracker/Types.h"
#include "tracker/OpenGL/ObjectRenderer.h"

class GeometricPrimitiveRenderer :public ObjectRenderer {
protected:
	int num_primitives = 0;
	QGLBuffer vertexbuffer = QGLBuffer(QGLBuffer::VertexBuffer);
	QGLBuffer vsize_buf = QGLBuffer(QGLBuffer::VertexBuffer);
	QGLBuffer vcolor_buf = QGLBuffer(QGLBuffer::VertexBuffer);

	void setup(Matrix_3xN* primitives, Matrix_3xN* colors);
	void init();
};