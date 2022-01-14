#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>
#include <iostream>
using namespace std;

class Triangle: public Object3D {

public:
	Triangle() = delete;

    // a b c are three vertex positions of the triangle
	Triangle(const Vector3f& a, const Vector3f& b, const Vector3f& c, Material* m) : Object3D(m) {
		vertices[0] = a; vertices[1] = b, vertices[2] = c;
		normal = Vector3f::cross(b - a, c - a).normalized();
		useVT = false; useVN = false;
	}

	void setVT(const Vector2f& at, const Vector2f& bt, const Vector2f& ct) {
		vt[0] = at; vt[1] = bt; vt[2] = ct;
		useVT = true;
	}

	void setVNormal(const Vector3f& an, const Vector3f& bn, const Vector3f& cn) {
		vn[0] = an; vn[1] = bn; vn[2] = cn;
		useVN = true;
	}

	Vector3f min() const {
		return ::min(vertices[0], ::min(vertices[1], vertices[2]));
	}

	Vector3f max() const {
        return ::max(vertices[0], ::max(vertices[1], vertices[2]));
    }

	bool intersect(const Ray& ray,  Hit& hit , double tmin) override {
		Vector3f origin = ray.getOrigin(), dir = ray.getDirection();
		Matrix3f M(dir, vertices[0]-vertices[1], vertices[0]-vertices[2]);
		double det_m = M.determinant();
		if (det_m == 0) return false;
		Matrix3f M_t(vertices[0]-origin, vertices[0]-vertices[1], vertices[0]-vertices[2]);
		Matrix3f M_b(dir, vertices[0]-origin, vertices[0]-vertices[2]);
		Matrix3f M_g(dir, vertices[0]-vertices[1], vertices[0]-origin);
		double t = M_t.determinant() / det_m;
		double b = M_b.determinant() / det_m;
		double g = M_g.determinant() / det_m;
		if (t < tmin || t >= hit.getT() || b < 0 || b > 1 || g < 0 || g > 1 || b + g > 1)
		//if (t < tmin || t >= hit.getT() || b < -feps || b > 1 + feps || g < -feps || g > 1 + feps || b + g > 1 + feps)
			return false;
		
		double u = b, v = g;
		if (useVT) {
			Vector2f uv = ((1 - b - g) * vt[0] + b * vt[1] + g * vt[2]);
            u = uv.x(), v = 1 - uv.y();
        }
		Vector3f n = normal;
		if (useVN) 
			n = ((1 - b - g) * vn[0] + b * vn[1] + g * vn[2]).normalized();
		bool outf = Vector3f::dot(dir, n) < 0 ? true : false;
		n *= outf ? 1 : -1;
		hit.set(t, material, n, material->getColor(u, v), outf, getIndex());
		return true;
	}

protected:
	Vector3f normal;
	Vector3f vertices[3];
	Vector2f vt[3];
	Vector3f vn[3];
	bool useVT, useVN;
};

#endif //TRIANGLE_H
