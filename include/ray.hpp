#ifndef RAY_H
#define RAY_H

#include <cassert>
#include <iostream>
#include <cmath>
#include <Vector3f.h>
#define feps 1e-8

// Ray class mostly copied from Peter Shirley and Keith Morley
class Ray {
public:

    Ray() = delete;
    Ray(const Vector3f &orig, const Vector3f &dir) {
        origin = orig;
        direction = dir;
    }

    Ray(const Ray &r) {
        origin = r.origin;
        direction = r.direction;
    }

    const Vector3f &getOrigin() const {
        return origin;
    }

    const Vector3f &getDirection() const {
        return direction;
    }

    Vector3f pointAtParameter(double t) const {
        return origin + direction * t;
    }

    Ray getReflectionRay(const double t, const Vector3f& normal) const {
        Vector3f orig = pointAtParameter(t) + normal * 2 * feps;
        Vector3f dir = 2 * Vector3f::dot(normal, -direction) * normal + direction;
        return Ray(orig, dir);
    }

    Ray getRefractionRay(const double t, const Vector3f& normal, const double refr_iI, const double refr_iT) const {
        double cosI = Vector3f::dot(normal, -direction);
        double nIT = refr_iI / refr_iT;
        double cosT2 = 1 - (nIT * nIT) * (1 - cosI * cosI);
        if (cosT2 > feps){
            Vector3f orig = pointAtParameter(t) - normal * 2 * feps;
            Vector3f dir = direction * nIT + normal * (nIT * cosI - sqrt(cosT2));
            return Ray(orig, dir);
        }
        else return getReflectionRay(t, normal);
    }

private:

    Vector3f origin;
    Vector3f direction;

};

inline std::ostream &operator<<(std::ostream &os, const Ray &r) {
    os << "Ray <" << r.getOrigin() << ", " << r.getDirection() << ">";
    return os;
}

#endif // RAY_H
