#ifndef OBJECT3D_H
#define OBJECT3D_H
#define feps 1e-8

#include <random>
#include "ray.hpp"
#include "hit.hpp"
#include "material.hpp"

// Base class for all 3d entities.
class Object3D {
public:
    Object3D() : material(nullptr), index(rand()) {}

    virtual ~Object3D() = default;

    explicit Object3D(Material *material) {
        this->material = material;
        //srand((int)time(0));
        this->index = rand();
    }

    unsigned getIndex() const {
        return index;
    }

    void setIndex(unsigned idx) {
        index = idx;
    }

    // Intersect Ray with this object. If hit, store information in hit structure.
    virtual bool intersect(const Ray &r, Hit &h, double tmin) = 0;
protected:

    Material *material;
    unsigned index;
};

#endif

