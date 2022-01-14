#ifndef GROUP_H
#define GROUP_H


#include "object3d.hpp"
#include "ray.hpp"
#include "hit.hpp"
#include <iostream>
#include <vector>

class Group : public Object3D {

public:
    Group() {}

    explicit Group (int num_objects) {}

    ~Group() override = default;

    bool intersect(const Ray &r, Hit &h, double tmin) override {
        bool isIntersect = false;
        for (auto obj : objects)
            if (obj->intersect(r, h, tmin))
                isIntersect = true;
        return isIntersect;
    }

    void addObject(int index, Object3D *obj) {
        objects.push_back(obj);
    }

    int getGroupSize() {
        return objects.size();
    }

private:
    std::vector<Object3D*> objects;

};

#endif
	
