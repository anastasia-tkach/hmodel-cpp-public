#version 330 core
out vec3 color;
in vec2 uv;
uniform sampler2D tex;
uniform sampler2D tex_mirror;

void main() {
	vec2 coord = gl_PointCoord - vec2(0.5); 
	if(length(coord) > 0.5) discard;
	color = vec3(1, 1, 0);
	gl_FragDepth = gl_FragCoord.z * 0.99;
}





