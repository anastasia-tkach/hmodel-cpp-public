#version 330 core

layout(location=0) out uint out_color;

uniform float window_left;
uniform float window_bottom;
uniform float window_height;
uniform float window_width;
uniform vec3 camera_center;
uniform mat4 MVP;
uniform mat4 invMVP;

//HModel
//const uint num_blocks = uint(28);
//const uint num_centers = uint(34);

//HModel with wrist
const uint num_blocks = uint(30);
const uint num_centers = uint(38);

//HTrack
//const uint num_blocks = uint(17);
//const uint num_centers = uint(24); 

uniform vec3 centers[num_centers];
uniform float radii[num_centers];
uniform ivec3 blocks[num_blocks];
uniform vec3 tangents_v1[num_blocks];
uniform vec3 tangents_v2[num_blocks];
uniform vec3 tangents_v3[num_blocks];
uniform vec3 tangents_u1[num_blocks];
uniform vec3 tangents_u2[num_blocks];
uniform vec3 tangents_u3[num_blocks];

uniform float min_x;
uniform float min_y;
uniform float max_x;
uniform float max_y;

uniform int fingers_only;

// Texture
in vec2 uv;
uniform sampler2D tex;

const int RAND_MAX = 32767;
const float epsilon = 0.00001;

vec3 project(vec3 point){
	vec4 point_gl =  MVP * vec4(point, 1.0);
    vec3 point_clip = vec3(point_gl[0], point_gl[1], point_gl[2]) / point_gl[3];
	float f = gl_DepthRange.far; 
	float n = gl_DepthRange.near;
	//int n = 0; int f = 1; 
	float ox = window_left + window_width/2;
	float oy = window_bottom + window_height/2;
	
    float xd = point_clip[0];
    float yd = point_clip[1];
    float zd = point_clip[2];
	vec3 point_window = vec3(0, 0, 0);
    point_window[0] = xd * window_width / 2 + ox;
    point_window[1] = yd * window_height / 2 + oy;
    point_window[2] = zd * (f - n) / 2 + (n + f) / 2;

	return point_window;
}


vec3 unproject(float winx, float winy, float winz){
    vec4 a = vec4(0, 0, 0, 0);
    a[0] = (winx - window_left) / window_width * 2.0 - 1.0;
    a[1] = (winy - window_bottom) / window_height * 2.0 - 1.0;
    a[2] = 2.0 * winz - 1.0;
    a[3] = 1.0;
    vec4 b  = invMVP * a;
    if (b[3] == 0.0)
		return vec3(0, 0, 0);
    b[3] = 1.0 / b[3];
    vec3 world_point = vec3(0, 0, 0);
    world_point[0] = b[0] * b[3];
    world_point[1] = b[1] * b[3];
    world_point[2] = b[2] * b[3];
    return world_point;
}

vec3 ray_sphere_intersection(vec3 c, float r, vec3 p, vec3 v, inout vec3 normal) {
    float A = dot(v, v);
    float B = -2 * dot(c - p, v);
    float C = dot(c - p, c - p) - r*r;
    float D = B*B - 4 * A*C;
    float t1 = RAND_MAX;
    float t2 = RAND_MAX;
    vec3 i1, i2;
    if (D >= 0) {
        t1 = (-B - sqrt(D)) / 2 / A;
        t2 = (-B + sqrt(D)) / 2 / A;
        i1 = p + t1 * v;
        i2 = p + t2 * v;
    }    
    vec3 i = vec3(RAND_MAX, RAND_MAX, RAND_MAX);
    if (abs(t1) < abs(t2)) {
        i = i1;
    }
    if (abs(t1) > abs(t2)) {
        i = i2;
    }	
	normal = normalize(i - c);
    return i;
}

vec3 ray_cylinder_intersection(vec3 pa, vec3 va, float r, vec3 p, vec3 v, inout vec3 normal) {
    vec3 i = vec3(RAND_MAX, RAND_MAX, RAND_MAX);
    vec3 delta_p = p - pa;
    vec3 e = v - dot(v, va) * va;
    float f = dot(v, va);
    vec3 g = delta_p - dot(delta_p, va) * va;
    float h = dot(delta_p, va);
    float A = dot(e, e);
    float B = 2 * dot(e, g);
    float C = dot(g, g) - r * r;
    float D = B*B - 4 * A*C;
    float t1 = RAND_MAX;
    float t2 = RAND_MAX;
    vec3 i1 = vec3(RAND_MAX, RAND_MAX, RAND_MAX);
    vec3 i2 = vec3(RAND_MAX, RAND_MAX, RAND_MAX);
    if (D >= 0) {
        t1 = (-B - sqrt(D)) / 2 / A;
        t2 = (-B + sqrt(D)) / 2 / A;
        i1 = p + t1 * v;
        i2 = p + t2 * v;
        if (dot(va, i1 - pa) > 0) t1 = RAND_MAX;
        if (dot(va, i2 - pa) > 0) t2 = RAND_MAX;
    }
    
    if (length(p - i1) < length(p - i2)) {
        i = i1;
    }

    if (length(p - i2) < length(p - i1)) {
        i = i2;
    }
	// Find normal
	vec3 c = pa + dot(i - pa, va) * va;
	normal = normalize(i - c); 
    return i;
}

vec3 ray_cone_intersection(vec3 pa, vec3 va, float alpha, vec3 p, vec3 v, inout vec3 normal) {
    vec3 i = vec3(RAND_MAX, RAND_MAX, RAND_MAX);
    float cos2 = cos(alpha) * cos(alpha);
    float sin2 = sin(alpha) * sin(alpha);
    vec3 delta_p = p - pa;
    vec3 e = v - dot(v, va) * va;
    float f = dot(v, va);
    vec3 g = delta_p - dot(delta_p, va) * va;
    float h = dot(delta_p, va);
    float A = cos2 * dot(e, e) - sin2 * f * f;
    float B = 2 * cos2 * dot(e, g) - 2 * sin2 * f * h;
    float C = cos2 * dot(g, g) - sin2 * h * h;
    float D = B*B - 4 * A*C;
    float t1 = RAND_MAX;
    float t2 = RAND_MAX;
    vec3 i1 = vec3(RAND_MAX, RAND_MAX, RAND_MAX);
    vec3 i2 = vec3(RAND_MAX, RAND_MAX, RAND_MAX);
    if (D >= 0) {
        t1 = (-B - sqrt(D)) / 2 / A;
        t2 = (-B + sqrt(D)) / 2 / A;
        i1 = p + t1 * v;
        i2 = p + t2 * v;
        if (dot(va, i1 - pa) > 0) t1 = RAND_MAX;
        if (dot(va, i2 - pa) > 0) t2 = RAND_MAX;
    }
    
    if (length(p - i1) < length(p - i2)) {
        i = i1;
    }

    if (length(p - i2) < length(p - i1)) {
        i = i2;
    }

	// Find normal - fix this
	float l = length(pa - i);
	vec3 c = pa - l * va;
	normal = normalize(i - c); 

    return i;
}

vec3 ray_triangle_intersection(vec3 p0, vec3 p1, vec3 p2, vec3 o, vec3 d, inout vec3 normal) {
    vec3 i = vec3(RAND_MAX, RAND_MAX, RAND_MAX);  
    vec3 e1 = p1 - p0;
    vec3 e2 = p2 - p0;
    vec3 q = cross(d, e2);
    float a = dot(e1, q);     

	// the vector is parallel to the plane(the intersection is at infinity)
    if (a > -epsilon && a < epsilon) {  
        return i;		
    }    
    float f = 1 / a;
    vec3 s = o - p0;
    float u = f * dot(s, q);   
	 
	// the intersection is outside of the triangle
    if (u < 0.0) {        
        return i;
    }    
    vec3 r = cross(s, e1);
    float v = f * dot(d, r); 
	   
	// the intersection is outside of the triangle
    if ( v <0.0 || u + v > 1.0) {        
        return i;
    }    
    float t = f * dot(e2, r); 
    i = o + t * d;
	normal = normalize(cross(e1, e2));
    return i;
}

vec3 ray_convsegment_intersection(vec3 c1, vec3 c2, float r1, float r2, vec3 p, vec3 v, inout vec3 normal) {

	// Ray - sphere intersection
	vec3 n1 = vec3(0, 0, 0);
    vec3 i1 = ray_sphere_intersection(c1, r1, p, v, n1);
    
    // Ray - sphere intersection
	vec3 n2 = vec3(0, 0, 0);
    vec3 i2 = ray_sphere_intersection(c2, r2, p, v, n2);
    
	// Ray - cone intersections
    vec3 n = (c2 - c1) / length(c2 - c1);
	vec3 i = vec3(RAND_MAX , RAND_MAX, RAND_MAX);

	vec3 n12 = vec3(0, 0, 0);
	vec3 i12, s1, s2;
	if (r1 - r2 < epsilon) {
		i12 = ray_cylinder_intersection(c2, n, r1, p, v, n12);
		s1 = c1;
		s2 = c2;
	}
	else {	
		float beta = asin((r1 - r2) / length(c1 - c2));
		float eta1 = r1 * sin(beta);
		s1 = c1 + eta1 * n;
		float eta2 = r2 * sin(beta);
		s2 = c2 + eta2 * n;
		vec3 z = c1 + (c2 - c1) * r1 / (r1 - r2);
		float r = r1 * cos(beta);
		float h = length(z - s1);
		float alpha = atan(r / h); 
		i12 = ray_cone_intersection(z, n, alpha, p, v, n12);		
	}   

	if (dot(n, i12 - s1) >= 0 && dot(n, i12 - s2) <= 0 && length(i12) < RAND_MAX) {
		i = i12;
		normal = n12;
	}       
	if (dot(n, i1 - s1) < 0 && length(i1) < RAND_MAX) {
		i = i1;
		normal = n1;
	}        
	if (dot(n, i2 - s2) > 0 && length(i2) < RAND_MAX) {
		i = i2;
		normal = n2;
	}    	

    return i;
}

vec3 ray_convtriangle_intersection(vec3 c1, vec3 c2, vec3 c3, vec3 v1, vec3 v2, vec3 v3,
        vec3 u1, vec3 u2, vec3 u3, float r1, float r2, float r3, vec3 p, vec3 v, inout vec3 normal) {
    
    vec3 n1 = vec3(0, 0, 0);
	vec3 n2 = vec3(0, 0, 0);
	vec3 n3 = vec3(0, 0, 0); 
	vec3 n4 = vec3(0, 0, 0); 
	vec3 n5 = vec3(0, 0, 0);
    vec3 i1 = ray_convsegment_intersection(c1, c2, r1, r2, p, v, n1);
    vec3 i2 = ray_convsegment_intersection(c1, c3, r1, r3, p, v, n2);
    vec3 i3 = ray_convsegment_intersection(c2, c3, r2, r3, p, v, n3);
    vec3 i4 = ray_triangle_intersection(v1, v2, v3, p, v, n4);
    vec3 i5 = ray_triangle_intersection(u1, u2, u3, p, v, n5);
    
    float min_value = distance(p, i1);  
	vec3 i = i1;
	normal = n1;
	if (distance(p, i2) < min_value) {
		min_value = distance(p, i2);
		i = i2;
		normal = n2; 		
	}
	if (distance(p, i3) < min_value) {
		min_value = distance(p, i3);
		i = i3;
		normal = n3; 		
	}
	if (distance(p, i4) < min_value) {
		min_value = distance(p, i4);
		i = i4;
		normal = n4; 		
	}
	if (distance(p, i5) < min_value) {
		min_value = distance(p, i5);
		i = i5;
		normal = n5; 		
	}	
	if (dot(normal, i - c1) < 0) normal = -normal;
	
    return i;
}


vec3 ray_model_intersection(vec3 p, vec3 d, inout vec3 min_normal, inout uint min_b) {
    const int RAND_MAX = 32767;
    vec3 i; vec3 normal = vec3(0, 0, 0);
    vec3 min_i = vec3(RAND_MAX, RAND_MAX, RAND_MAX);
    float min_distance = RAND_MAX;
    vec3 c1, c2, c3, v1, v2, v3, u1, u2, u3;
    float r1, r2, r3;
	for (uint j = uint(0); j < num_blocks; j++) {

		//if (fingers_only == uint(1) && j > uint(14) && j < uint(27)) continue;
		if (fingers_only == 1 && j > uint(14) && j < uint(27)) continue;

        ivec3 block = blocks[j];       
        if (block[2] < RAND_MAX) {
            c1 = centers[block[0]]; c2 = centers[block[1]]; c3 = centers[block[2]];
            r1 = radii[block[0]]; r2 = radii[block[1]]; r3 = radii[block[2]];
            v1 = tangents_v1[j]; v2 = tangents_v2[j]; v3 = tangents_v3[j];
            u1 = tangents_u1[j]; u2 = tangents_u2[j]; u3 = tangents_u3[j];
            i = ray_convtriangle_intersection(c1, c2, c3, v1, v2, v3, u1, u2, u3, r1, r2, r3, p, d, normal);			
            if (length(p - i) < min_distance) {				
                min_distance = length(p - i);
                min_i = i;
				min_normal = normal;
				min_b = j;
				break;
            }
        }
        if (block[2] >= RAND_MAX) {						
            c1 = centers[block[0]]; c2 = centers[block[1]];
            r1 = radii[block[0]]; r2 = radii[block[1]];
            i = ray_convsegment_intersection(c1, c2, r1, r2, p, d, normal);			
            if (length(p - i) < min_distance ) {				
                min_distance = length(p - i);
                min_i = i;
				min_normal = normal;
				min_b = j;
				break;
            }
        }
    }
    return min_i;
}

void main() {

	/*int offset = 5;
	if (gl_FragCoord.x < min_x - offset || gl_FragCoord.x > max_x + offset|| gl_FragCoord.y < min_y - offset|| gl_FragCoord.y > max_y + offset) {	
		out_color = uint(255); 	
		return;	
	}*/

    const int RAND_MAX = 32767;
	vec2 pixel = vec2(gl_FragCoord.x, gl_FragCoord.y);
    vec3 p1 = unproject(pixel[0], pixel[1], 0);
    vec3 p2 = unproject(pixel[0], pixel[1], 1);
    vec3 ray_direction = normalize(p2 - p1);
    vec3 normal = vec3(0, 0, 0);	
	out_color =  uint(255);

	/*float tried = camera_center[2];
	float truth = 0;	 
	float a = tried - truth;
	float b = tried - truth + 1;
	out_extra = vec4(a, b, 0, 1);*/

	vec3 i = ray_model_intersection(camera_center, ray_direction, normal, out_color);

	//vec3 i = ray_sphere_intersection(vec3(0, 0, 10), 4, camera_center, ray_direction, normal);
	//if (i == RAND_MAX) out_color = uint(0);
	//out_color = uint(distance(camera_center, i)); 
}



