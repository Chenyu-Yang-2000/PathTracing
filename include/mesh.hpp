#ifndef MESH_H
#define MESH_H

#include <vector>
#include "object3d.hpp"
#include "triangle.hpp"
#include "Vector2f.h"
#include "Vector3f.h"
#include "bound.hpp"
#include "kdtree.hpp"

struct TriangleIndex {
        TriangleIndex() {
            x[0] = -1; x[1] = -1; x[2] = -1;
        }
        TriangleIndex(int id_0, int id_1, int id_2) {
            x[0] = id_0; x[1] = id_1; x[2] = id_2;
        }
        int &operator[](const int i) { return x[i]; }
        // By Computer Graphics convention, counterclockwise winding is front face
        int x[3]{};
    };

class Mesh : public Object3D {

public:
    std::vector<Vector3f> v, vn;
    std::vector<TriangleIndex> t, vtIndex, vnIndex;
    std::vector<Vector2f> vt;
    std::vector<Triangle*> faces;
    Vector3f minCoord, maxCoord;
    Bound bound;
    KDTree *kdtree;

    Mesh(const char *filename, Material *m);
    Mesh(const std::vector<Vector3f>& VV, const std::vector<Vector3f>& VN, 
         const std::vector<TriangleIndex>& VF, Material *m);
    
    bool intersect(const Ray &r, Hit &h, double tmin) override;
    const Vector3f& min() {
        return minCoord;
    };
    const Vector3f& max() {
        return maxCoord;
    };

private:

};

#endif
