#version 330 core
out vec3 color;
in vec2 uv;
uniform sampler2D tex;
uniform sampler2D tex_mirror;

void main() {
	vec2 coord = gl_PointCoord - vec2(0.5); 
	if(length(coord) > 0.5) discard;
	color = vec3(0, 0.7, 1);

	/*float window_width = textureSize(tex_mirror, 0).x;
    float window_height = textureSize(tex_mirror, 0).y;   
    float u =  gl_FragCoord.x/window_width;
    float v = 1 - gl_FragCoord.y/window_height;    
    color = texture(tex, uv).rgb;*/
	//gl_FragDepth =  0;
	gl_FragDepth = gl_FragCoord.z * 0.99;
}





