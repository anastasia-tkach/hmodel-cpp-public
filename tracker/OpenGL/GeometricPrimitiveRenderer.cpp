#include "tracker/OpenGL/GeometricPrimitiveRenderer.h"
#include "util/mylogger.h"

void GeometricPrimitiveRenderer::setup(Matrix_3xN* primitives, Matrix_3xN* colors) {
	CHECK_NOTNULL(primitives);
	if (colors) CHECK(primitives->cols() == colors->cols());
	this->num_primitives = primitives->cols();

	vao.bind();
	program.bind();

	///--- Create vertex buffer/attributes "position"
	{
		bool success = vertexbuffer.create();
		assert(success);
		vertexbuffer.setUsagePattern(QGLBuffer::StaticDraw);
		success = vertexbuffer.bind();
		assert(success);
		vertexbuffer.allocate(primitives->data(), sizeof(Scalar) * primitives->size());
		program.setAttributeBuffer("vpoint", GL_FLOAT, 0, 3);
		program.enableAttributeArray("vpoint");
	}

	///--- Create vertex buffer/attributes "colors"
	if (!colors) {
		static Matrix_3xN _colors(primitives->rows(), primitives->cols());
		_colors.row(0).array().setConstant(1); ///< force red
		colors = &_colors;
	}
	{
		bool success = vcolor_buf.create();
		assert(success);
		vcolor_buf.setUsagePattern(QGLBuffer::StaticDraw);
		success = vcolor_buf.bind();
		assert(success);
		vcolor_buf.allocate(colors->data(), sizeof(Scalar) * primitives->size());
		program.setAttributeBuffer("vcolor", GL_FLOAT, 0, 3);
		program.enableAttributeArray("vcolor");
	}

	///--- Create vertex buffer/attributes "sizes"
	{
		static VectorN _sizes(primitives->cols());
		VectorN* sizes = &_sizes;
		bool success = vsize_buf.create(); assert(success);
		vsize_buf.setUsagePattern(QGLBuffer::StaticDraw);
		success = vsize_buf.bind(); assert(success);
		vsize_buf.allocate(colors->data(), sizeof(Scalar) * sizes->size());
		program.setAttributeBuffer("vsize", GL_FLOAT, 0, 1);
		program.enableAttributeArray("vsize");
	}

	program.release();
	vao.release();
}

void GeometricPrimitiveRenderer::init() {
	///--- Load/compile shaders
	if (!program.isLinked()) {
		const char* vshader = "C://Developer//hmodel-build-vs//CloudRenderer_vshader.glsl";
		const char* fshader = "C://Developer//hmodel-build-vs//CloudRenderer_fshader.glsl";
		bool vok = program.addShaderFromSourceFile(QGLShader::Vertex, vshader);
		bool fok = program.addShaderFromSourceFile(QGLShader::Fragment, fshader);
		bool lok = program.link();
		assert(lok && vok && fok);
		bool success = program.bind();
		assert(success);
	}

	///--- Create vertex array object
	if (!vao.isCreated()) {
		bool success = vao.create();
		assert(success);
	}

	///--- Avoid pollution
	program.release();
	vao.release();
}