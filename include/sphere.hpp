#ifndef SPHERE_H
#define SPHERE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>
#include "bound.hpp"


class Sphere : public Object3D {
public:
    Sphere() {
        // unit ball at the center
        center_ = Vector3f::ZERO;
        radius_ = 1.0;
    }

    Sphere(const Vector3f &center, double radius, Material *material) : Object3D(material) {
        center_ = center;
        radius_ = radius;
    }

    ~Sphere() override = default;

    bool intersect(const Ray &r, Hit &h, double tmin) override {
        Vector3f origin = r.getOrigin(), dir = r.getDirection();
        Vector3f l = center_ - origin;
        bool in_sphere = (l.length() < radius_ - feps);
        bool out_sphere = (l.length() > radius_ + feps);
        double t_p = Vector3f::dot(l, dir.normalized());
        if (in_sphere || out_sphere){
            if (out_sphere and t_p < 0)
                return false;
            double dd = l.squaredLength() - pow(t_p, 2);
            if (dd > pow(radius_, 2))
                return false;
            double dt = sqrt(pow(radius_, 2) - dd);
            double t = in_sphere ? t_p + dt : t_p - dt;
            if (t < h.getT() && t >= tmin) {
                Vector3f n = (r.pointAtParameter(t) - center_).normalized();
                double u = 0.5 + atan2(n.x(), n.z()) / (2 * M_PI);
                double v = 0.5 - asin(n.y()) / M_PI;
                n = getNormal(n, r.pointAtParameter(t) - center_, u, v);
                n *= out_sphere ? 1 : -1;
                h.set(t, material, n, material->getColor(u, v), out_sphere, getIndex());
                return true;
            }
        }
        else{
            if (t_p <= 0) return false;
            double t = 2 * t_p;
            if (t < h.getT() && t >= tmin) {
                Vector3f n = (r.pointAtParameter(t) - center_).normalized();
                double u = 0.5 + atan2(n.x(), n.z()) / (2 * M_PI);
                double v = 0.5 - asin(n.y()) / M_PI;
                n = getNormal(n, r.pointAtParameter(t) - center_, u, v);
                h.set(t, material, -n, material->getColor(u, v), false, getIndex());
                return true;
            }
        }
        return false;
    }

    Vector3f getNormal(const Vector3f& n, const Vector3f& p, double u, double v){
        if (!material->getBump()) return n;
        Vector2f grad = Vector2f::ZERO;
        double f = material->getBump()->getDisturb(u, v, grad);
        if (fabs(f) < DBL_EPSILON) return n;
        double phi = u * 2 * M_PI, theta = M_PI * v;
        Vector3f uaxis(-p.z(), 0, p.x());
        Vector3f vaxis(p.y() * cos(phi), -radius_ * sin(theta), p.y() * sin(phi));
        if (uaxis.squaredLength() < DBL_EPSILON) return n;
        return Vector3f::cross(uaxis + n * grad[0] / (2 * M_PI), vaxis + n * grad[1] / M_PI).normalized();
    }

    Vector3f center_;
    double radius_;

protected:

};


#endif
