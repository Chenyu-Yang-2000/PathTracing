#ifndef HIT_H
#define HIT_H

#include <vecmath.h>
#include "ray.hpp"

class Material;

class Hit {
public:

    // constructors
    Hit() {
        material = nullptr;
        t = 1e38;
        color = Vector3f::ZERO;
        outerface = true;
        hash = 0;
    }

    Hit(double _t, Material *m, const Vector3f &n, const Vector3f &col, bool outf=true, unsigned _hash=0) {
        t = _t;
        material = m;
        normal = n;
        color = col;
        outerface = outf;
        hash = _hash;
    }

    Hit(const Hit &h) {
        t = h.t;
        material = h.material;
        normal = h.normal;
        color = h.color;
        outerface = h.outerface;
        hash = h.hash;
    }

    // destructor
    ~Hit() = default;

    double getT() const {
        return t;
    }

    Material *getMaterial() const {
        return material;
    }

    const Vector3f &getNormal() const {
        return normal;
    }

    const Vector3f& getColor() const {
        return color;
    }

    bool isOuterFace() const {
        return outerface;
    }

    unsigned getHash() const {
        return hash;
    }

    void set(double _t, Material *m, const Vector3f &n, const Vector3f& col=Vector3f::ZERO, bool outf=true, unsigned _hash=0) {
        t = _t;
        material = m;
        normal = n;
        color = col;
        outerface = outf;
        hash = _hash;
    }

private:
    double t;
    Material *material;
    Vector3f normal;
    Vector3f color;
    bool outerface;
    unsigned hash;
};

inline std::ostream &operator<<(std::ostream &os, const Hit &h) {
    os << "Hit <" << h.getT() << ", " << h.getNormal() << ">";
    return os;
}

#endif // HIT_H
