#ifndef PLANE_H
#define PLANE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>

// function: ax+by+cz=d

class Plane : public Object3D {
public:
    Plane() {

    }

    Plane(const Vector3f &normal, double d, Material *m, double w=100, double h=100, bool isfinite=false) : Object3D(m) {
        normal_ = normal.normalized();
        d_ = d;
        w_ = w; h_ = h;
        center_ = normal_ * d_;
        if(Vector3f::cross(Vector3f::UP, normal_).length() > feps){
            xaxis_ = Vector3f::cross(Vector3f::UP, normal_).normalized();
            yaxis_ = Vector3f::cross(normal_, xaxis_).normalized();  
        }
        else{
            xaxis_ = Vector3f::cross(Vector3f::FORWARD, normal_).normalized();
            yaxis_ = Vector3f::cross(normal_, xaxis_).normalized();
        }
        isFinite = isfinite;
    }

    ~Plane() override = default;

    bool intersect(const Ray &r, Hit &h, double tmin) override {
        Vector3f origin = r.getOrigin(), dir = r.getDirection();
        double n_dot_d = Vector3f::dot(normal_, dir);
        if (n_dot_d <= feps && n_dot_d >= -feps)
            return false;
        double t = -(-d_ + Vector3f::dot(normal_, origin)) / n_dot_d;
        if (t >= h.getT() || t < tmin || t <= 0)
            return false;
        double u, v;
        getUV(r.pointAtParameter(t), u, v);
        if (isFinite && (u > 1 + feps || v > 1 + feps || u < -feps || v < -feps))
		    return false;
        Vector3f n = getNormal(u, v);
        n *= n_dot_d > 0 ? -1 : 1;
        h.set(t, material, n, material->getColor(u, v), true, getIndex());
        return true;
    }

    void getUV(const Vector3f &p, double &u, double &v) const {
        u = Vector3f::dot(p - center_, xaxis_) / w_ + 0.5;
        v = 0.5 - Vector3f::dot(p - center_, yaxis_) / h_;
    }

    Vector3f getNormal(double u, double v) const {
        if (!material->getBump()) return normal_;
        Vector2f grad = Vector2f::ZERO;
        double f = material->getBump()->getDisturb(u, v, grad);
        if (fabs(f) < DBL_EPSILON) return normal_;
        return Vector3f::cross(xaxis_ + normal_ * grad[0], yaxis_ + normal_ * grad[1]).normalized();
    }
    
    Vector3f normal_;
    double d_, w_, h_;
    Vector3f center_, xaxis_, yaxis_;
    bool isFinite;

protected:

};

#endif //PLANE_H
		

