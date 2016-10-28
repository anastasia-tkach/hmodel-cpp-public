#define _USE_MATH_DEFINES 
#include <math.h>   
#include <limits>
#include <algorithm>
#include "cudax/cuda_glm.h"

double eps = std::numeric_limits<double>::min();

void print_dvec3(glm::dvec3 v) {
	std::cout << v[0] << " " << v[1] << " " << v[2] << std::endl;
}
void print_dvec2(glm::dvec2 v) {
	std::cout << v[0] << " " << v[1] << std::endl;
}

int block_size(glm::ivec3 block) {
	if (block[2] == RAND_MAX) return 2;
	else return 3;
}

double myatan2(glm::dvec2 v) {
	double alpha = atan2(v[1], v[0]);
	if (alpha < 0) alpha = alpha + 2 * M_PI;
	return alpha;
}

bool is_point_on_segment(const glm::dvec2 & p, const glm::dvec2 & a, const glm::dvec2 & b) {
	double alpha = dot(b - a, p - a);
	if (alpha < 0) return false;
	if (alpha > dot(b - a, b - a)) return false;
	return true;
}

bool is_point_on_arc(const glm::dvec2 & c, const glm::dvec2 & p, const glm::dvec2 & q, const glm::dvec2 & t) {
	double alpha = myatan2(p - c);
	double beta = myatan2(q - c);
	double gamma = myatan2(t - c);

	if (beta < alpha) beta = beta + 2 * M_PI; 
	if (gamma < alpha) gamma = gamma + 2 * M_PI; 
	if (gamma < beta) return true;
	else return false;
}

double distance_point_segment(const glm::dvec2 & p, const glm::dvec2 & c1, const glm::dvec2 & c2) {
	glm::dvec2 u = c2 - c1;
	glm::dvec2 v = p - c1;
	double  q = dot(u, v) / dot(u, u);
	if (q <= 0) return  length(p - c1);
	if (q > 0 && q < 1) return length(p - c1 - q * (c2 - c1));
	if (q >= 1) return length(p - c2);
}

bool quadratic_roots(double a, double b, double c, double & x1, double & x2) {
	//long long b2 = b * b;
	//long long _4ac = 4 * a * c;
	//long long D = b2 - _4ac;
	//std::cout << b2 << std::endl;
	//std::cout << _4ac << std::endl;
	//std::cout << D << std::endl;
	//if (D == 0) {
	//	x1 = (-b) / (2 * a);
	//	x2 = x1;		
	//	return true;
	//}
	double D = b * b - 4 * a * c;
	if (D > 0) {
		double sqrt_D = sqrt(D);
		x1 = - (b - sqrt_D) / (2 * a);
		x2 = - (b + sqrt_D) / (2 * a);
		return true;
	}
	return false;
}

bool intersect_circle_circle(const glm::dvec2 & c1, const glm::dvec2 & c2, double r1, double r2, glm::dvec2 & t1, glm::dvec2 & t2) {

	double x1 = c1[0];
	double y1 = c1[1];
	double x2 = c2[0];
	double y2 = c2[1];
	
	// cartesian separation of the two circle centers
	double r3 = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
	// too far apart to intersect
	if (r3 > r1 + r2) return false;
	// circle one completely inside circle two
	if (r2 > r3 + r1) return false;
	// circle two completely inside circle one
	if (r1 > r3 + r2) return false;
	// circles identical
	if ((r3 < 10 * eps) & (abs(r1 - r2) < 10 * eps)) return false;

	double anought = atan2((y2 - y1), (x2 - x1));

	// law of cosines
	double aone = acos(-((r2 * r2 - r1 * r1 - r3 * r3) / (2 * r1 * r3)));
	double alpha1 = anought + aone;
	double alpha2 = anought - aone;
	t1 = glm::dvec2(x1 + r1 * cos(alpha1), y1 + r1 * sin(alpha1));
	t2 = glm::dvec2(x1 + r1 * cos(alpha2), y1 + r1 * sin(alpha2));	

	return true;
}

bool intersect_circle_line(const glm::dvec2 & c, double r, double slope, double intercept, glm::dvec2 & t1, glm::dvec2 & t2) {
	double centerx = c[0];
	double centery = c[1];
	if (abs(slope) < 100) {
		// from the law of cosines
		double a = 1 + slope * slope;
		double b = 2 * (slope * (intercept - centery) - centerx);
		double d = centery * centery + centerx * centerx + intercept * intercept - 2 * centery * intercept - r * r;
		double x1, x2;
		if (!quadratic_roots(a, b, d, x1, x2)) return false;
		else {
			t1 = glm::dvec2(x1, intercept + slope * x1);
			t2 = glm::dvec2(x2, intercept + slope * x2);
			return true;
		}
	}
	// vertical slope case
	else {
		// they don't intercept 
		if (abs(centerx - intercept) > r) return false;
		else {
			double step = sqrt(r * r - (intercept - centerx) * (intercept - centerx));
			t1 = glm::dvec2(intercept, centery + step);			
			t2 = glm::dvec2(intercept, centery - step);
		}
	}
}

void intersect_circle_segment(const glm::dvec2 & c, double r, const glm::dvec2 & p, const glm::dvec2 & q, glm::dvec2 & t1, glm::dvec2 & t2, bool & i1, bool & i2) {
	double k = (p[1] - q[1]) / (p[0] - q[0]);
	double b = p[1] - k * p[0];
	if (!intersect_circle_line(c, r, k, b, t1, t2)) {
		i1 = false;
		i2 = false;
		return;
	}
	i1 = true;
	i2 = true;
	if (!is_point_on_segment(t1, p, q)) i1 = false;
	if (!is_point_on_segment(t2, p, q)) i2 = false;
}

bool intersect_segment_segment(const glm::dvec2 & a, const glm::dvec2 & b, const glm::dvec2 & c, const glm::dvec2 & d, glm::dvec2 & t) {	
	double x1 = a[0]; double y1 = a[1];
	double x2 = b[0]; double y2 = b[1];
	double x3 = c[0]; double y3 = c[1];
	double x4 = d[0]; double y4 = d[1];
	double denom = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4);
	if (abs(denom) < eps) return false;
	double xi = ((x3 - x4) * (x1 * y2 - y1 * x2) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom;
	double yi = ((y3 - y4) * (x1 * y2 - y1 * x2) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom;
	t = glm::dvec2(xi, yi);
	if (xi < std::min(x1, x2) || xi > std::max(x1, x2)) return false;
	if (xi < std::min(x3, x4) || xi > std::max(x3, x4)) return false;
}

std::vector<std::pair<glm::dvec2, glm::dvec2>> intersect_segment_segment_same_circle(const glm::dvec2 & c, double r, const glm::dvec2 & s1, const glm::dvec2 & e1, const glm::dvec2 & s2, const glm::dvec2 & e2) {
	glm::dvec2 v1 = s1 - c;
	glm::dvec2 u1 = e1 - c;
	double alpha1 = atan2(v1[1], v1[0]);
	double beta1 = atan2(u1[1], u1[0]);
	if (alpha1 < 0) alpha1 = alpha1 + 2 * M_PI;
	if (beta1 < 0) beta1 = beta1 + 2 * M_PI;

	glm::dvec2 v2 = s2 - c;
	glm::dvec2 u2 = e2 - c;
	double alpha2 = atan2(v2[1], v2[0]);
	double beta2 = atan2(u2[1], u2[0]);
	if (alpha2 < 0) alpha2 = alpha2 + 2 * M_PI; 
	if (beta2 < 0) beta2 = beta2 + 2 * M_PI; 

	std::vector<std::pair<double, double>> arcs1;
	if (alpha1 < beta1)
		arcs1.push_back(std::pair<double, double>(alpha1, beta1));
	else {
		arcs1.push_back(std::pair<double, double>(0, beta1));
		arcs1.push_back(std::pair<double, double>(alpha1, 2 * M_PI));
	}
	std::vector<std::pair<double, double>> arcs2;
	if (alpha2 < beta2)
		arcs2.push_back(std::pair<double, double>(alpha2, beta2));
	else {
		arcs2.push_back(std::pair<double, double>(0, beta2));
		arcs2.push_back(std::pair<double, double>(alpha2, 2 * M_PI));
	}
	std::vector<std::pair<glm::dvec2, glm::dvec2>> intersections;
	double gamma, delta; glm::dvec2 a; glm::dvec2 b;
	for (size_t i = 0; i < arcs1.size(); i++) {
		for (size_t j = 0; j < arcs2.size(); j++) {
			if (std::max(arcs1[i].first, arcs2[j].first) < std::min(arcs1[i].second, arcs2[j].second)) {
				gamma = std::max(arcs1[i].first, arcs2[j].first);
				delta = std::min(arcs1[i].second, arcs2[j].second);
				a = c + r * glm::dvec2(cos(gamma), sin(gamma));
				b = c + r * glm::dvec2(cos(delta), sin(delta));
				intersections.push_back(std::pair<glm::dvec2, glm::dvec2>(a, b));
			}
		}
	}	
	return intersections;
}

glm::dvec3 project_point_on_plane(const glm::dvec3 & p, const glm::dvec3 & p0, const glm::dvec3 & n) {
	double distance = dot(p - p0, n);
	return p - n * distance;
}

glm::dvec3 project_point_on_segment(const glm::dvec3 & p, const glm::dvec3 & c1, const glm::dvec3 & c2) {
	glm::dvec3 u = c2 - c1;
	glm::dvec3 v = p - c1;
	double alpha = dot(u, v) / dot(u, u);
	if (alpha <= 0) return c1;
	if (alpha > 0 && alpha < 1) return c1 + alpha * u;
	if (alpha >= 1) return c2;
}

glm::dvec3 project_point_on_arc(const glm::dvec3 & p, const glm::dvec3 & c, double r, const glm::dvec3 & n, glm::dvec3 & t1, glm::dvec3 & t2) {
	glm::dvec3 s = project_point_on_plane(p, c, n);
	glm::dvec3 q = c + r * (s - c) / length(s - c);

	// asume that arc is in xy plane(this is always the case in our system)
	if (!is_point_on_arc(glm::dvec2(c[0], c[1]), glm::dvec2(t1[0], t1[1]), glm::dvec2(t2[0], t2[1]), glm::dvec2(q[0], q[1]))) {
		double d1 = length(p - t1);
		double d2 = length(p - t2);
		if (d1 < d2)q = t1;
		else q = t2; 
	}
	return q; 
}

bool get_tangents(const glm::dvec3 & c1, const glm::dvec3 & c2, double r1, double r2,
	glm::dvec2 & lt1, glm::dvec2 & lt2, glm::dvec2 & rt1, glm::dvec2 & rt2) {

	/*std::cout << "c1 = (" << c1[0] << ", " << c1[1] << ", " << c1[2] << ")" << std::endl;
	std::cout << "c2 = (" << c2[0] << ", " << c2[1] << ", " << c2[2] << ")" << std::endl;
	std::cout << "r1 = " << r1 << std::endl;
	std::cout << "r2 = " << r2 << std::endl;*/

	double a1 = c1[0];
	double b1 = c1[1];
	double a2 = c2[0];
	double b2 = c2[1];
	double a21 = a2 - a1;
	double b21 = b2 - b1;
	double d2 = a21 * a21 + b21 * b21;
	double delta_r = r2 - r1;
	double delta_r2 = delta_r * delta_r;
	double r21 = delta_r / d2;
	if (d2 - delta_r2 < 0) return false;
	double s21 = sqrt(d2 - delta_r2) / d2;
	glm::dvec2 u1 = glm::dvec2(-a21 * r21 - b21 * s21, -b21 * r21 + a21 * s21); // Left unit vector
	glm::dvec2 u2 = glm::dvec2(-a21 * r21 + b21 * s21, -b21 * r21 - a21 * s21); // Right unit vector
	lt1 = glm::dvec2(a1, b1) + r1 * u1;
	lt2 = glm::dvec2(a2, b2) + r2 * u1; // Left line tangency points
	rt1 = glm::dvec2(a1, b1) + r1 * u2;
	rt2 = glm::dvec2(a2, b2) + r2 * u2; //Right line tangency points

	/*std::cout << "lt1 = (" << lt1[0] << ", " << lt1[1] << ", " << lt1[2] << ")" << std::endl;
	std::cout << "lt2 = (" << lt2[0] << ", " << lt2[1] << ", " << lt2[2] << ")" << std::endl;
	std::cout << "rt1 = (" << rt1[0] << ", " << rt1[1] << ", " << rt1[2] << ")" << std::endl;
	std::cout << "rt2 = (" << rt2[0] << ", " << rt2[1] << ", " << rt2[2] << ")" << std::endl;*/

	return true;
}


