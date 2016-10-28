#version 330 core
out vec3 color;
in vec2 uv;
uniform sampler2D tex;
uniform sampler2D tex_mirror;

void main() {
	color = vec3(0.75, 0.75, 0.75);

	gl_FragDepth = gl_FragCoord.z * 0.99;
}





